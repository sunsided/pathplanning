//! Shared machinery for the bidirectional searches (bidirectional A*, MM, NBS,
//! contraction hierarchies): per-direction labels, the best meeting point, and
//! path assembly. Each algorithm brings its own queues and stopping rule.

use crate::graph::RoadGraph;
use crate::planner::state::{PlannerState, PlannerStatus};
use ordered_float::NotNan;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};

pub(crate) const FWD: usize = 0;
pub(crate) const BWD: usize = 1;

/// Labels of one search direction. The backward side walks edges in reverse,
/// so its `g` is the cost *to* the goal and `parent` points towards the goal.
#[derive(Default)]
pub(crate) struct Side {
    pub g: HashMap<usize, f64>,
    pub parent: HashMap<usize, usize>,
    pub open: HashSet<usize>,
    pub closed: HashSet<usize>,
}

pub(crate) struct BidirCore {
    pub start: usize,
    pub goal: usize,
    pub sides: [Side; 2],
    /// Cost of the best start -> goal path found so far.
    pub mu: f64,
    pub meet: Option<usize>,
}

impl BidirCore {
    pub fn new(start: usize, goal: usize) -> Self {
        let mut sides = [Side::default(), Side::default()];
        for (side, root) in sides.iter_mut().zip([start, goal]) {
            side.g.insert(root, 0.0);
            side.open.insert(root);
        }
        let same = start == goal;
        Self {
            start,
            goal,
            sides,
            mu: if same { 0.0 } else { f64::INFINITY },
            meet: same.then_some(start),
        }
    }

    pub fn g(&self, dir: usize, node: usize) -> f64 {
        *self.sides[dir].g.get(&node).unwrap_or(&f64::INFINITY)
    }

    /// Closes `node` on side `dir` and relaxes the given `(neighbor, cost)`
    /// pairs, calling `on_improve(neighbor, new_g)` for every improved label so
    /// the caller can queue it. Returns false if `node` was already closed.
    pub fn expand(
        &mut self,
        state: &mut PlannerState,
        dir: usize,
        node: usize,
        neighbors: impl Iterator<Item = (usize, f64)>,
        mut on_improve: impl FnMut(usize, f64),
    ) -> bool {
        let side = &mut self.sides[dir];
        side.open.remove(&node);
        if !side.closed.insert(node) {
            return false;
        }
        state.explored.insert(node);
        state.frontier.remove(&node);
        state.expanded_count += 1;

        let g = self.g(dir, node);
        for (neighbor, cost) in neighbors {
            let side = &mut self.sides[dir];
            if side.closed.contains(&neighbor) {
                continue;
            }
            let new_g = g + cost;
            if new_g >= *side.g.get(&neighbor).unwrap_or(&f64::INFINITY) {
                continue;
            }
            side.g.insert(neighbor, new_g);
            side.parent.insert(neighbor, node);
            side.open.insert(neighbor);
            state.frontier.insert(neighbor);
            on_improve(neighbor, new_g);

            let through = new_g + self.g(1 - dir, neighbor);
            if through < self.mu {
                self.mu = through;
                self.meet = Some(neighbor);
            }
        }
        true
    }

    /// The best start -> goal node sequence found, if any.
    pub fn path(&self) -> Option<Vec<usize>> {
        let meet = self.meet?;
        let walk = |dir: usize| {
            let mut nodes = vec![meet];
            while let Some(&p) = self.sides[dir].parent.get(nodes.last().unwrap()) {
                nodes.push(p);
                if nodes.len() > 1_000_000 {
                    break;
                }
            }
            nodes
        };
        let mut path = walk(FWD);
        path.reverse();
        path.extend(walk(BWD).into_iter().skip(1));
        Some(path)
    }
}

/// Ends the search, publishing `path` as the result if there is one.
pub(crate) fn finish(state: &mut PlannerState, graph: &RoadGraph, path: Option<Vec<usize>>) {
    match path {
        Some(path) => {
            state.fill_path_metrics(graph, &path);
            state.best_path = Some(path.clone());
            state.locked_path = Some(path);
            state.status = PlannerStatus::FirstMatchFound;
        }
        None => state.status = PlannerStatus::Done,
    }
}

/// Handles the shared status bookkeeping at the top of each step: returns
/// false once the previous step reported a result.
pub(crate) fn still_running(state: &mut PlannerState) -> bool {
    match state.status {
        PlannerStatus::Searching => true,
        PlannerStatus::FirstMatchFound => {
            state.status = PlannerStatus::Done;
            false
        }
        _ => false,
    }
}

pub(crate) fn still_searching(state: &PlannerState) -> bool {
    state.status == PlannerStatus::Searching
}

/// Heuristic estimate for side `dir`: to the goal going forward, from the start
/// going backward.
pub(crate) fn directed_h(
    state: &PlannerState,
    graph: &RoadGraph,
    core: &BidirCore,
    dir: usize,
    v: usize,
) -> f64 {
    let (h, mode) = (state.config.heuristic, state.config.cost_mode);
    if dir == FWD {
        h.eval(graph, v, core.goal, mode)
    } else {
        h.eval(graph, core.start, v, mode)
    }
}

/// Min-priority queue with lazy deletion: entries remember the `g` they were
/// pushed with, and `peek`/`pop` silently drop entries the caller's validity
/// check rejects (node closed, label improved since, moved to another queue).
#[derive(Default)]
pub(crate) struct LazyQueue {
    heap: BinaryHeap<(Reverse<NotNan<f64>>, usize, NotNan<f64>)>,
}

impl LazyQueue {
    pub fn push(&mut self, key: f64, node: usize, g: f64) {
        if let (Ok(key), Ok(g)) = (NotNan::new(key), NotNan::new(g)) {
            self.heap.push((Reverse(key), node, g));
        }
    }

    /// Smallest valid `(key, node)`; `valid(node, g)` decides validity.
    pub fn peek(&mut self, valid: impl Fn(usize, f64) -> bool) -> Option<(f64, usize)> {
        while let Some(&(Reverse(key), node, g)) = self.heap.peek() {
            if valid(node, g.into_inner()) {
                return Some((key.into_inner(), node));
            }
            self.heap.pop();
        }
        None
    }

    pub fn pop(&mut self, valid: impl Fn(usize, f64) -> bool) -> Option<(f64, usize)> {
        let top = self.peek(valid);
        if top.is_some() {
            self.heap.pop();
        }
        top
    }

    /// Smallest valid key, `INFINITY` if empty.
    pub fn min_key(&mut self, valid: impl Fn(usize, f64) -> bool) -> f64 {
        self.peek(valid).map_or(f64::INFINITY, |(k, _)| k)
    }
}

impl Side {
    /// Validity check for queue entries: node still open with the queued label.
    pub fn is_current(&self, node: usize, g: f64) -> bool {
        self.open.contains(&node) && self.g.get(&node) == Some(&g)
    }
}
