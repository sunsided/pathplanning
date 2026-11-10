use crate::graph::RoadGraph;
use crate::planner::bidir_common::{
    BWD, BidirCore, FWD, LazyQueue, directed_h, finish, still_running, still_searching,
};
use crate::planner::state::PlannerState;
use std::collections::HashSet;

/// Per-search state for NBS ("near-optimal bidirectional search", Chen et al.
/// 2017). Each side's open list is split into a *waiting* queue ordered by f
/// and a *ready* queue ordered by g; ready holds the nodes with f <= clb.
pub struct NbsState {
    core: BidirCore,
    waiting: [LazyQueue; 2],
    ready: [LazyQueue; 2],
    in_ready: [HashSet<usize>; 2],
    /// Current lower bound on the cost of any path not yet found.
    clb: f64,
}

impl NbsState {
    fn push_waiting(
        &mut self,
        state: &PlannerState,
        graph: &RoadGraph,
        dir: usize,
        v: usize,
        g: f64,
    ) {
        self.in_ready[dir].remove(&v);
        let f = g + directed_h(state, graph, &self.core, dir, v);
        self.waiting[dir].push(f, v, g);
    }

    /// Moves every waiting node with f <= clb into ready.
    fn promote(&mut self, dir: usize) {
        let (side, in_ready) = (&self.core.sides[dir], &mut self.in_ready[dir]);
        loop {
            let valid = |n: usize, g: f64| side.is_current(n, g) && !in_ready.contains(&n);
            match self.waiting[dir].peek(valid) {
                Some((f, v)) if f <= self.clb => {
                    let g = side.g[&v];
                    self.waiting[dir].pop(|_, _| true);
                    in_ready.insert(v);
                    self.ready[dir].push(g, v, g);
                }
                _ => break,
            }
        }
    }

    fn ready_min(&mut self, dir: usize) -> f64 {
        let (side, in_ready) = (&self.core.sides[dir], &self.in_ready[dir]);
        self.ready[dir].min_key(|n, g| side.is_current(n, g) && in_ready.contains(&n))
    }

    fn waiting_min(&mut self, dir: usize) -> f64 {
        let (side, in_ready) = (&self.core.sides[dir], &self.in_ready[dir]);
        self.waiting[dir].min_key(|n, g| side.is_current(n, g) && !in_ready.contains(&n))
    }

    /// Raises clb until a ready pair (u, v) with g_f(u) + g_b(v) <= clb exists.
    /// Returns false if the open lists ran dry first.
    fn prepare_best(&mut self) -> bool {
        loop {
            self.promote(FWD);
            self.promote(BWD);
            let pair = self.ready_min(FWD) + self.ready_min(BWD);
            if pair <= self.clb {
                return true;
            }
            let next = pair.min(self.waiting_min(FWD)).min(self.waiting_min(BWD));
            if next.is_infinite() {
                return false;
            }
            self.clb = next;
            if self.core.mu <= self.clb {
                return true;
            }
        }
    }
}

/// NBS step: always expands a *pair* of nodes, one per side, minimising
/// max(f_f(u), f_b(v), g_f(u) + g_b(v)). This expands at most twice as many
/// nodes as the best possible front-to-end bidirectional search.
pub fn step_nbs(state: &mut PlannerState, graph: &RoadGraph, n_steps: usize) -> bool {
    let (Some(start), Some(goal)) = (state.start, state.goal) else {
        return false;
    };

    let mut s = state.nbs.take().unwrap_or_else(|| {
        let mut s = NbsState {
            core: BidirCore::new(start, goal),
            waiting: Default::default(),
            ready: Default::default(),
            in_ready: Default::default(),
            clb: 0.0,
        };
        for (dir, root) in [(FWD, start), (BWD, goal)] {
            s.push_waiting(state, graph, dir, root, 0.0);
            state.frontier.insert(root);
        }
        s
    });

    for _ in 0..n_steps {
        if !still_running(state) {
            break;
        }
        if s.core.mu <= s.clb || !s.prepare_best() || s.core.mu <= s.clb {
            finish(state, graph, s.core.path());
            break;
        }

        for dir in [FWD, BWD] {
            let (side, in_ready) = (&s.core.sides[dir], &s.in_ready[dir]);
            let valid = |n: usize, g: f64| side.is_current(n, g) && in_ready.contains(&n);
            let (_, node) = s.ready[dir].pop(valid).unwrap();
            s.in_ready[dir].remove(&node);

            let mut improved = Vec::new();
            let neighbors = graph.neighbors(node, dir == BWD, state.config.cost_mode);
            s.core
                .expand(state, dir, node, neighbors, |v, g| improved.push((v, g)));
            for (v, g) in improved {
                s.push_waiting(state, graph, dir, v, g);
            }
        }
    }

    state.nbs = Some(s);
    still_searching(state)
}

#[cfg(test)]
mod tests {
    use crate::planner::heuristic::Heuristic;
    use crate::planner::state::{Algorithm, PlannerStatus};
    use crate::planner::test_graphs::{assert_optimal_all_pairs, make_graph, one_way_grid, run};

    #[test]
    fn matches_dijkstra_on_grid() {
        let graph = one_way_grid(6);
        for h in [Heuristic::Euclidean, Heuristic::Alt, Heuristic::Zero] {
            assert_optimal_all_pairs(&graph, Algorithm::Nbs, h);
        }
    }

    #[test]
    fn unreachable_goal_finishes_without_path() {
        let graph = make_graph(vec![[0.0, 0.0], [100.0, 0.0]], vec![(1, 0, 100.0)]);
        let state = run(&graph, Algorithm::Nbs, Heuristic::Euclidean, 0, 1);
        assert_eq!(state.status, PlannerStatus::Done);
        assert!(state.locked_path.is_none());
    }
}
