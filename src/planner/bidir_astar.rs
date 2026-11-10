use crate::graph::RoadGraph;
use crate::planner::bidir_common::{
    BWD, BidirCore, FWD, LazyQueue, finish, still_running, still_searching,
};
use crate::planner::state::PlannerState;

/// Per-search state for bidirectional A*.
pub struct BidirAStarState {
    core: BidirCore,
    /// Keyed by g + p_f on the forward side and g - p_f on the backward side.
    queues: [LazyQueue; 2],
}

/// Forward potential with the "average" construction
/// p_f(v) = (h(v, goal) - h(start, v)) / 2 and p_b = -p_f. Both directions then
/// see the same non-negative reduced edge costs (for a consistent heuristic), so
/// the search may stop once top_f + top_b >= mu.
fn potential_fwd(state: &PlannerState, graph: &RoadGraph, core: &BidirCore, v: usize) -> f64 {
    let h = state.config.heuristic;
    let mode = state.config.cost_mode;
    0.5 * (h.eval(graph, v, core.goal, mode) - h.eval(graph, core.start, v, mode))
}

fn key(
    state: &PlannerState,
    graph: &RoadGraph,
    core: &BidirCore,
    dir: usize,
    v: usize,
    g: f64,
) -> f64 {
    let p = potential_fwd(state, graph, core, v);
    if dir == FWD { g + p } else { g - p }
}

/// Bidirectional A* step: expands the side with fewer open nodes (Pohl's
/// cardinality rule; forward from the start over edges, backward from the goal
/// over reversed edges) until the frontiers prove the best meeting point
/// optimal.
pub fn step_bidir_astar(state: &mut PlannerState, graph: &RoadGraph, n_steps: usize) -> bool {
    let (Some(start), Some(goal)) = (state.start, state.goal) else {
        return false;
    };

    let mut s = state.bidir_astar.take().unwrap_or_else(|| {
        let core = BidirCore::new(start, goal);
        let mut queues = [LazyQueue::default(), LazyQueue::default()];
        for (dir, root) in [(FWD, start), (BWD, goal)] {
            queues[dir].push(key(state, graph, &core, dir, root, 0.0), root, 0.0);
            state.frontier.insert(root);
        }
        BidirAStarState { core, queues }
    });

    for _ in 0..n_steps {
        if !still_running(state) {
            break;
        }
        let tops = [FWD, BWD].map(|dir| {
            let side = &s.core.sides[dir];
            s.queues[dir].min_key(|n, g| side.is_current(n, g))
        });
        // Stop when either side is exhausted or no better meeting can exist.
        if tops.iter().any(|k| k.is_infinite()) || tops[FWD] + tops[BWD] >= s.core.mu {
            finish(state, graph, s.core.path());
            break;
        }

        let sides = &s.core.sides;
        let dir = if sides[BWD].open.len() < sides[FWD].open.len() {
            BWD
        } else {
            FWD
        };
        let side = &s.core.sides[dir];
        let (_, node) = s.queues[dir].pop(|n, g| side.is_current(n, g)).unwrap();

        let mut improved = Vec::new();
        let neighbors = graph.neighbors(node, dir == BWD, state.config.cost_mode);
        s.core
            .expand(state, dir, node, neighbors, |v, g| improved.push((v, g)));
        for (v, g) in improved {
            let k = key(state, graph, &s.core, dir, v, g);
            s.queues[dir].push(k, v, g);
        }
    }

    state.bidir_astar = Some(s);
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
        for h in [Heuristic::Euclidean, Heuristic::Alt] {
            assert_optimal_all_pairs(&graph, Algorithm::BidirAStar, h);
        }
    }

    #[test]
    fn unreachable_goal_finishes_without_path() {
        let graph = make_graph(vec![[0.0, 0.0], [100.0, 0.0]], vec![(1, 0, 100.0)]);
        let state = run(&graph, Algorithm::BidirAStar, Heuristic::Euclidean, 0, 1);
        assert_eq!(state.status, PlannerStatus::Done);
        assert!(state.locked_path.is_none());
    }
}
