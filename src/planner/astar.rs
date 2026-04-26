use crate::graph::RoadGraph;
use crate::planner::state::{HeapEntry, PlannerState, PlannerStatus};
use ordered_float::NotNan;
use std::cmp::Reverse;

/// A* step: f = g + h, with relaxation.
pub fn step_astar(state: &mut PlannerState, graph: &RoadGraph, n_steps: usize) -> bool {
    let goal = match state.goal {
        Some(g) => g,
        None => return false,
    };

    for _ in 0..n_steps {
        if state.status == PlannerStatus::Done {
            return false;
        }
        if state.status == PlannerStatus::FirstMatchFound {
            state.status = PlannerStatus::Done;
            return false;
        }

        let entry = match state.open_set.pop() {
            Some(e) => e,
            None => {
                state.status = PlannerStatus::Done;
                return false;
            }
        };

        let current = entry.node_id;

        if state.explored.contains(&current) {
            continue;
        }
        state.explored.insert(current);
        state.frontier.remove(&current);
        state.expanded_count += 1;

        if current == goal {
            let path = state.reconstruct_path(goal);
            state.best_path = Some(path.clone());
            state.locked_path = Some(path.clone());
            state.fill_path_metrics(graph, &path);
            if state.status == PlannerStatus::Searching {
                state.status = PlannerStatus::FirstMatchFound;
            }
            return false;
        }

        let g = *state.g_score.get(&current).unwrap_or(&f64::INFINITY);

        if current >= graph.adjacency.len() {
            continue;
        }

        for &edge_idx in &graph.adjacency[current] {
            let edge = &graph.edges[edge_idx];
            let neighbor = edge.to;

            if state.explored.contains(&neighbor) {
                continue;
            }

            let new_g = g + PlannerState::edge_cost(graph, edge, state.config.cost_mode);
            let old_g = *state.g_score.get(&neighbor).unwrap_or(&f64::INFINITY);

            if new_g < old_g {
                state.came_from.insert(neighbor, current);
                state.g_score.insert(neighbor, new_g);

                let h = state
                    .config
                    .heuristic
                    .eval(graph, neighbor, goal, state.config.cost_mode);
                let f = new_g + h;

                if let Ok(f_ord) = NotNan::new(f) {
                    state.open_set.push(HeapEntry {
                        cost: Reverse(f_ord),
                        node_id: neighbor,
                    });
                    state.frontier.insert(neighbor);
                }
            }
        }
    }

    state.status == PlannerStatus::Searching
}
