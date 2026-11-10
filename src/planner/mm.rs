use crate::graph::RoadGraph;
use crate::planner::bidir_common::{
    BWD, BidirCore, FWD, LazyQueue, directed_h, finish, still_running, still_searching,
};
use crate::planner::state::PlannerState;

/// Per-direction open-list views: MM needs the minimum priority, f and g.
#[derive(Default)]
struct Queues {
    pr: LazyQueue,
    f: LazyQueue,
    g: LazyQueue,
}

/// Per-search state for MM ("meet in the middle", Holte et al. 2016).
pub struct MmState {
    core: BidirCore,
    queues: [Queues; 2],
}

fn push(state: &PlannerState, graph: &RoadGraph, s: &mut MmState, dir: usize, v: usize, g: f64) {
    let f = g + directed_h(state, graph, &s.core, dir, v);
    let q = &mut s.queues[dir];
    q.pr.push(f.max(2.0 * g), v, g);
    q.f.push(f, v, g);
    q.g.push(g, v, g);
}

/// MM step: each side orders its open list by pr(n) = max(g + h, 2g), so
/// neither search expands a node past the midpoint of the optimal cost; the
/// side holding the global minimum priority expands next.
pub fn step_mm(state: &mut PlannerState, graph: &RoadGraph, n_steps: usize) -> bool {
    let (Some(start), Some(goal)) = (state.start, state.goal) else {
        return false;
    };

    let mut s = state.mm.take().unwrap_or_else(|| {
        let mut s = MmState {
            core: BidirCore::new(start, goal),
            queues: Default::default(),
        };
        for (dir, root) in [(FWD, start), (BWD, goal)] {
            push(state, graph, &mut s, dir, root, 0.0);
            state.frontier.insert(root);
        }
        s
    });

    for _ in 0..n_steps {
        if !still_running(state) {
            break;
        }
        let [pr, f, g] = [0, 1, 2].map(|which| {
            [FWD, BWD].map(|dir| {
                let side = &s.core.sides[dir];
                let q = &mut s.queues[dir];
                let q = match which {
                    0 => &mut q.pr,
                    1 => &mut q.f,
                    _ => &mut q.g,
                };
                q.min_key(|n, g| side.is_current(n, g))
            })
        });
        let c = pr[FWD].min(pr[BWD]);
        // MM's stopping rule (with epsilon = 0): mu cannot be beaten once it
        // is no larger than any of these lower bounds.
        let lower = c.max(f[FWD]).max(f[BWD]).max(g[FWD] + g[BWD]);
        if pr.iter().any(|k| k.is_infinite()) || s.core.mu <= lower {
            finish(state, graph, s.core.path());
            break;
        }

        let dir = if pr[FWD] <= pr[BWD] { FWD } else { BWD };
        let side = &s.core.sides[dir];
        let (_, node) = s.queues[dir].pr.pop(|n, g| side.is_current(n, g)).unwrap();

        let mut improved = Vec::new();
        let neighbors = graph.neighbors(node, dir == BWD, state.config.cost_mode);
        s.core
            .expand(state, dir, node, neighbors, |v, g| improved.push((v, g)));
        for (v, g) in improved {
            push(state, graph, &mut s, dir, v, g);
        }
    }

    state.mm = Some(s);
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
            assert_optimal_all_pairs(&graph, Algorithm::Mm, h);
        }
    }

    #[test]
    fn unreachable_goal_finishes_without_path() {
        let graph = make_graph(vec![[0.0, 0.0], [100.0, 0.0]], vec![(1, 0, 100.0)]);
        let state = run(&graph, Algorithm::Mm, Heuristic::Euclidean, 0, 1);
        assert_eq!(state.status, PlannerStatus::Done);
        assert!(state.locked_path.is_none());
    }
}
