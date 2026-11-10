//! Contraction hierarchies (Geisberger et al. 2008).
//!
//! Preprocessing contracts nodes one by one in order of importance, adding a
//! shortcut `u -> w` whenever removing `v` would destroy the only shortest
//! `u -> v -> w` path. Queries then run a bidirectional Dijkstra that only ever
//! moves *up* the hierarchy, which touches a tiny fraction of the graph.

use crate::graph::{GraphEdge, RoadGraph};
use crate::planner::bidir_common::{
    BWD, BidirCore, FWD, LazyQueue, finish, still_running, still_searching,
};
use crate::planner::cache::Deadline;
use crate::planner::state::{CostMode, PlannerState};
use ordered_float::NotNan;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

/// Settled-node budget of a witness search. Smaller is faster to build but
/// adds more (harmless) shortcuts.
const WITNESS_SETTLE_LIMIT: usize = 64;

pub struct ContractionHierarchy {
    /// up[FWD][v]: edges v -> w with rank(w) > rank(v).
    /// up[BWD][v]: edges u -> v with rank(u) > rank(v), stored as (u, cost).
    up: [Vec<Vec<(usize, f64)>>; 2],
    /// Middle node of each shortcut edge, for unpacking.
    shortcut_mid: HashMap<(usize, usize), usize>,
}

/// Mutable overlay graph during contraction: cheapest edge per node pair among
/// the nodes not yet contracted.
struct Overlay {
    out: Vec<HashMap<usize, f64>>,
    inn: Vec<HashMap<usize, f64>>,
}

impl Overlay {
    fn new(n: usize) -> Self {
        Self {
            out: vec![HashMap::new(); n],
            inn: vec![HashMap::new(); n],
        }
    }

    fn add_edge(&mut self, graph: &RoadGraph, edge: &GraphEdge, mode: CostMode) {
        if edge.from == edge.to {
            return;
        }
        let c = PlannerState::edge_cost(graph, edge, mode);
        let e = self.out[edge.from].entry(edge.to).or_insert(f64::INFINITY);
        if c < *e {
            *e = c;
            self.inn[edge.to].insert(edge.from, c);
        }
    }

    /// Shortcuts `(u, w, cost)` needed to contract `v`.
    fn shortcuts(&self, v: usize) -> Vec<(usize, usize, f64)> {
        let mut result = Vec::new();
        for (&u, &c_uv) in &self.inn[v] {
            let targets: Vec<(usize, f64)> = self.out[v]
                .iter()
                .filter(|&(&w, _)| w != u)
                .map(|(&w, &c_vw)| (w, c_uv + c_vw))
                .collect();
            let Some(limit) = targets.iter().map(|&(_, c)| c).reduce(f64::max) else {
                continue;
            };
            let witness = self.witness_search(u, v, limit);
            for (w, via) in targets {
                if witness.get(&w).is_none_or(|&d| d > via) {
                    result.push((u, w, via));
                }
            }
        }
        result
    }

    /// Bounded Dijkstra from `source` that ignores `skip`.
    fn witness_search(&self, source: usize, skip: usize, limit: f64) -> HashMap<usize, f64> {
        let mut dist = HashMap::from([(source, 0.0)]);
        let mut heap = BinaryHeap::from([(Reverse(NotNan::new(0.0).unwrap()), source)]);
        let mut settled = 0;
        while let Some((Reverse(d), node)) = heap.pop() {
            let d = d.into_inner();
            if d > dist[&node] {
                continue;
            }
            settled += 1;
            if d > limit || settled > WITNESS_SETTLE_LIMIT {
                break;
            }
            for (&next, &c) in &self.out[node] {
                let nd = d + c;
                if next != skip && dist.get(&next).is_none_or(|&old| nd < old) {
                    dist.insert(next, nd);
                    heap.push((Reverse(NotNan::new(nd).unwrap()), next));
                }
            }
        }
        dist
    }
}

/// Resumable hierarchy construction, so the UI can stay responsive: loads
/// the edges, computes every node's initial priority, then contracts nodes one
/// by one.
pub(crate) struct ChBuilder {
    mode: CostMode,
    next_edge: usize,
    overlay: Overlay,
    up: [Vec<Vec<(usize, f64)>>; 2],
    shortcut_mid: HashMap<(usize, usize), usize>,
    contracted_neighbors: Vec<i64>,
    queue: BinaryHeap<Reverse<(i64, usize)>>,
    next_init: usize,
    contracted: usize,
}

impl ChBuilder {
    pub(crate) fn new(graph: &RoadGraph, mode: CostMode) -> Self {
        let n = graph.nodes.len();
        Self {
            mode,
            next_edge: 0,
            overlay: Overlay::new(n),
            up: [vec![Vec::new(); n], vec![Vec::new(); n]],
            shortcut_mid: HashMap::new(),
            contracted_neighbors: vec![0; n],
            queue: BinaryHeap::with_capacity(n),
            next_init: 0,
            contracted: 0,
        }
    }

    /// Edge difference plus a penalty for already-contracted neighbors
    /// (spreads contraction evenly over the map).
    fn priority(&self, v: usize, shortcuts: &[(usize, usize, f64)]) -> i64 {
        let degree = self.overlay.out[v].len() + self.overlay.inn[v].len();
        shortcuts.len() as i64 - degree as i64 + self.contracted_neighbors[v]
    }

    pub(crate) fn progress(&self) -> f32 {
        let n = self.up[FWD].len().max(1);
        (self.next_init + self.contracted) as f32 / (2 * n) as f32
    }

    /// Works until `deadline`; returns the hierarchy once complete.
    pub(crate) fn advance(
        &mut self,
        graph: &RoadGraph,
        deadline: &Deadline,
    ) -> Option<ContractionHierarchy> {
        while let Some(edge) = graph.edges.get(self.next_edge) {
            self.overlay.add_edge(graph, edge, self.mode);
            self.next_edge += 1;
            if self.next_edge.is_multiple_of(4096) && deadline.passed() {
                return None;
            }
        }

        let n = self.up[FWD].len();
        while self.next_init < n {
            let v = self.next_init;
            let p = self.priority(v, &self.overlay.shortcuts(v));
            self.queue.push(Reverse((p, v)));
            self.next_init += 1;
            if v.is_multiple_of(16) && deadline.passed() {
                return None;
            }
        }

        // Late nodes are hubs with many neighbors, so check the clock every node.
        while let Some(Reverse((_, v))) = self.queue.pop() {
            // Lazy update: re-evaluate and defer if no longer the cheapest.
            let shortcuts = self.overlay.shortcuts(v);
            let p = self.priority(v, &shortcuts);
            if let Some(&Reverse((next, _))) = self.queue.peek()
                && p > next
            {
                self.queue.push(Reverse((p, v)));
            } else {
                self.contract(v, shortcuts);
            }
            if deadline.passed() {
                return None;
            }
        }

        Some(ContractionHierarchy {
            up: std::mem::take(&mut self.up),
            shortcut_mid: std::mem::take(&mut self.shortcut_mid),
        })
    }

    fn contract(&mut self, v: usize, shortcuts: Vec<(usize, usize, f64)>) {
        let overlay = &mut self.overlay;
        self.up[FWD][v] = overlay.out[v].iter().map(|(&w, &c)| (w, c)).collect();
        self.up[BWD][v] = overlay.inn[v].iter().map(|(&u, &c)| (u, c)).collect();
        for &(w, _) in &self.up[FWD][v] {
            overlay.inn[w].remove(&v);
            self.contracted_neighbors[w] += 1;
        }
        for &(u, _) in &self.up[BWD][v] {
            overlay.out[u].remove(&v);
            self.contracted_neighbors[u] += 1;
        }
        for (u, w, c) in shortcuts {
            if overlay.out[u].get(&w).is_none_or(|&old| c < old) {
                overlay.out[u].insert(w, c);
                overlay.inn[w].insert(u, c);
                self.shortcut_mid.insert((u, w), v);
            }
        }
        self.contracted += 1;
    }
}

impl ContractionHierarchy {
    /// Builds the whole hierarchy in one go (blocking).
    pub(crate) fn build(graph: &RoadGraph, mode: CostMode) -> Self {
        ChBuilder::new(graph, mode)
            .advance(graph, &Deadline::never())
            .expect("unbounded build completes")
    }

    /// Expands shortcut edges in a hierarchy path back into original edges.
    fn unpack(&self, path: &[usize]) -> Vec<usize> {
        let mut result = vec![path[0]];
        for pair in path.windows(2) {
            let mut stack = vec![(pair[0], pair[1])];
            while let Some((a, b)) = stack.pop() {
                match self.shortcut_mid.get(&(a, b)) {
                    Some(&m) => {
                        stack.push((m, b));
                        stack.push((a, m));
                    }
                    None => result.push(b),
                }
            }
        }
        result
    }
}

/// Per-query state: an upward bidirectional Dijkstra over the hierarchy.
pub struct ChState {
    core: BidirCore,
    queues: [LazyQueue; 2],
}

/// Contraction hierarchy query step. The first call builds the hierarchy for
/// the current cost mode (cached on the graph); later queries reuse it.
pub fn step_ch(state: &mut PlannerState, graph: &RoadGraph, n_steps: usize) -> bool {
    let (Some(start), Some(goal)) = (state.start, state.goal) else {
        return false;
    };
    let ch = graph.contraction_hierarchy(state.config.cost_mode);

    let mut s = state.ch.take().unwrap_or_else(|| {
        let mut queues = [LazyQueue::default(), LazyQueue::default()];
        for (dir, root) in [(FWD, start), (BWD, goal)] {
            queues[dir].push(0.0, root, 0.0);
            state.frontier.insert(root);
        }
        ChState {
            core: BidirCore::new(start, goal),
            queues,
        }
    });

    for _ in 0..n_steps {
        if !still_running(state) {
            break;
        }
        // Each side may stop once its smallest g reaches mu; the meeting node
        // with the best total is the highest-ranked node on the optimal path.
        let tops = [FWD, BWD].map(|dir| {
            let side = &s.core.sides[dir];
            let k = s.queues[dir].min_key(|n, g| side.is_current(n, g));
            if k < s.core.mu { k } else { f64::INFINITY }
        });
        if tops.iter().all(|k| k.is_infinite()) {
            let path = s.core.path().map(|p| ch.unpack(&p));
            finish(state, graph, path);
            break;
        }

        let dir = if tops[FWD] <= tops[BWD] { FWD } else { BWD };
        let side = &s.core.sides[dir];
        let (_, node) = s.queues[dir].pop(|n, g| side.is_current(n, g)).unwrap();

        let mut improved = Vec::new();
        let neighbors = ch.up[dir][node].iter().copied();
        s.core
            .expand(state, dir, node, neighbors, |v, g| improved.push((v, g)));
        for (v, g) in improved {
            s.queues[dir].push(g, v, g);
        }
    }

    state.ch = Some(s);
    still_searching(state)
}

#[cfg(test)]
mod tests {
    use crate::planner::heuristic::Heuristic;
    use crate::planner::state::{Algorithm, PlannerStatus};
    use crate::planner::test_graphs::{assert_optimal_all_pairs, make_graph, one_way_grid, run};

    #[test]
    fn matches_dijkstra_on_grid() {
        let graph = one_way_grid(7);
        assert_optimal_all_pairs(&graph, Algorithm::ContractionHierarchy, Heuristic::Zero);
    }

    #[test]
    fn unreachable_goal_finishes_without_path() {
        let graph = make_graph(vec![[0.0, 0.0], [100.0, 0.0]], vec![(1, 0, 100.0)]);
        let state = run(
            &graph,
            Algorithm::ContractionHierarchy,
            Heuristic::Zero,
            0,
            1,
        );
        assert_eq!(state.status, PlannerStatus::Done);
        assert!(state.locked_path.is_none());
    }
}
