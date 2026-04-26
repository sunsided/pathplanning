use crate::graph::RoadGraph;
use crate::planner::state::{HeapEntry, PlannerState, PlannerStatus};
use ordered_float::NotNan;
use std::cmp::Reverse;

/// Dijkstra step: f = g only, with relaxation (same as A* with h=0).
pub fn step_dijkstra(state: &mut PlannerState, graph: &RoadGraph, n_steps: usize) -> bool {
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
            let path = reconstruct_path(state, goal);
            let dist = *state.g_score.get(&goal).unwrap_or(&0.0);
            state.best_path = Some(path.clone());
            state.locked_path = Some(path);
            state.locked_path_dist = dist;
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

            let new_g = g + edge.weight_meters;
            let old_g = *state.g_score.get(&neighbor).unwrap_or(&f64::INFINITY);

            if new_g < old_g {
                state.came_from.insert(neighbor, current);
                state.g_score.insert(neighbor, new_g);

                let f = new_g;

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

fn reconstruct_path(state: &PlannerState, goal: usize) -> Vec<usize> {
    let mut path = vec![goal];
    let mut current = goal;
    let mut steps = 0;
    while let Some(&prev) = state.came_from.get(&current) {
        path.push(prev);
        current = prev;
        steps += 1;
        if steps > 1_000_000 {
            break;
        }
    }
    path.reverse();
    path
}
