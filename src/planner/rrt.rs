#[cfg(test)]
#[cfg(test)]
use rand::SeedableRng;
use rand::rngs::SmallRng;
use rstar::RTree;
use std::collections::{HashMap, HashSet};

use crate::graph::RoadGraph;
use crate::planner::rrt_common::{TreePoint, clamp_bbox, ensure_bbox_margin, sample_goal};
use crate::planner::state::{PlannerState, PlannerStatus};

const GOAL_BIAS: f64 = 0.05;
const BBOX_MARGIN_FACTOR: f64 = 0.25;
const BBOX_MARGIN_MIN: f64 = 500.0;
const STALL_THRESHOLD: usize = 256;
const BBOX_EXPAND_FACTOR: f64 = 1.5;
const MAX_ITERATION_MULTIPLIER: usize = 10;

pub(crate) struct RrtState {
    pub(crate) rtree: RTree<TreePoint>,
    pub(crate) rng: SmallRng,
    pub(crate) bbox_min: [f64; 2],
    pub(crate) bbox_max: [f64; 2],
    pub(crate) graph_bbox_min: [f64; 2],
    pub(crate) graph_bbox_max: [f64; 2],
    pub(crate) stall_counter: usize,
    pub(crate) exhausted: HashSet<usize>,
    pub(crate) goal_bias: f64,
    pub(crate) iterations: usize,
}

impl RrtState {
    pub(crate) fn new(start: usize, goal: usize, graph: &RoadGraph) -> Self {
        let start_pos = graph.nodes[start].world_pos;
        let goal_pos = graph.nodes[goal].world_pos;

        let (gb_min, gb_max) = graph.bounding_box().unwrap_or((
            [start_pos[0] - 10000.0, start_pos[1] - 10000.0],
            [start_pos[0] + 10000.0, start_pos[1] + 10000.0],
        ));

        let dx = (goal_pos[0] - start_pos[0]).abs();
        let dy = (goal_pos[1] - start_pos[1]).abs();
        let diag = (dx * dx + dy * dy).sqrt();
        let margin = (BBOX_MARGIN_FACTOR * diag).max(BBOX_MARGIN_MIN);

        let bbox_min = [
            start_pos[0].min(goal_pos[0]) - margin,
            start_pos[1].min(goal_pos[1]) - margin,
        ];
        let bbox_max = [
            start_pos[0].max(goal_pos[0]) + margin,
            start_pos[1].max(goal_pos[1]) + margin,
        ];

        let (clamped_min, clamped_max) = (
            clamp_bbox(bbox_min, gb_min, gb_max),
            clamp_bbox(bbox_max, gb_min, gb_max),
        );
        let (bbox_min, bbox_max) = ensure_bbox_margin(clamped_min, clamped_max, margin.min(100.0));

        let mut rtree = RTree::new();
        rtree.insert(TreePoint {
            pos: start_pos,
            node_id: start,
        });

        Self {
            rtree,
            rng: rand::make_rng::<SmallRng>(),
            bbox_min,
            bbox_max,
            graph_bbox_min: gb_min,
            graph_bbox_max: gb_max,
            stall_counter: 0,
            exhausted: HashSet::new(),
            goal_bias: GOAL_BIAS,
            iterations: 0,
        }
    }

    #[cfg(test)]
    pub(crate) fn new_with_seed(start: usize, goal: usize, graph: &RoadGraph, seed: u64) -> Self {
        let start_pos = graph.nodes[start].world_pos;
        let goal_pos = graph.nodes[goal].world_pos;

        let (gb_min, gb_max) = graph.bounding_box().unwrap_or((
            [start_pos[0] - 10000.0, start_pos[1] - 10000.0],
            [start_pos[0] + 10000.0, start_pos[1] + 10000.0],
        ));

        let dx = (goal_pos[0] - start_pos[0]).abs();
        let dy = (goal_pos[1] - start_pos[1]).abs();
        let diag = (dx * dx + dy * dy).sqrt();
        let margin = (BBOX_MARGIN_FACTOR * diag).max(BBOX_MARGIN_MIN);

        let bbox_min = [
            start_pos[0].min(goal_pos[0]) - margin,
            start_pos[1].min(goal_pos[1]) - margin,
        ];
        let bbox_max = [
            start_pos[0].max(goal_pos[0]) + margin,
            start_pos[1].max(goal_pos[1]) + margin,
        ];

        let (clamped_min, clamped_max) = (
            clamp_bbox(bbox_min, gb_min, gb_max),
            clamp_bbox(bbox_max, gb_min, gb_max),
        );
        let (bbox_min, bbox_max) = ensure_bbox_margin(clamped_min, clamped_max, margin.min(100.0));

        let mut rtree = RTree::new();
        rtree.insert(TreePoint {
            pos: start_pos,
            node_id: start,
        });

        Self {
            rtree,
            rng: SmallRng::seed_from_u64(seed),
            bbox_min,
            bbox_max,
            graph_bbox_min: gb_min,
            graph_bbox_max: gb_max,
            stall_counter: 0,
            exhausted: HashSet::new(),
            goal_bias: GOAL_BIAS,
            iterations: 0,
        }
    }

    #[cfg(test)]
    pub(crate) fn with_goal_bias(mut self, goal_bias: f64) -> Self {
        self.goal_bias = goal_bias;
        self
    }
}

pub fn step_rrt(state: &mut PlannerState, graph: &RoadGraph, n_steps: usize) -> bool {
    let goal = match state.goal {
        Some(g) => g,
        None => return false,
    };

    if state.rrt.is_none() {
        state.open_set.clear();
        let start = state.frontier.iter().next().copied().unwrap_or(0);
        state.explored.insert(start);
        state.rrt = Some(RrtState::new(start, goal, graph));
    }

    let max_iterations = MAX_ITERATION_MULTIPLIER * graph.nodes.len();

    for _ in 0..n_steps {
        if state.status == PlannerStatus::Done {
            return false;
        }
        if state.status == PlannerStatus::FirstMatchFound {
            state.status = PlannerStatus::Done;
            return false;
        }

        let rrt = state.rrt.as_mut().unwrap();
        if rrt.rtree.size() == 0 {
            state.status = PlannerStatus::Done;
            return false;
        }
        if rrt.iterations >= max_iterations {
            state.status = PlannerStatus::Done;
            return false;
        }

        rrt.iterations += 1;

        let q_rand = sample_goal(
            &mut rrt.rng,
            &rrt.bbox_min,
            &rrt.bbox_max,
            goal,
            graph,
            rrt.goal_bias,
        );

        let q_near = match rrt.rtree.nearest_neighbor(&q_rand) {
            Some(tp) => tp.node_id,
            None => {
                state.status = PlannerStatus::Done;
                return false;
            }
        };

        let q_new = extend(
            graph,
            q_near,
            &q_rand,
            &mut state.explored,
            &mut state.came_from,
            &mut state.g_score,
            &mut state.frontier,
            &mut state.expanded_count,
            state.config.cost_mode,
            &mut rrt.exhausted,
            &mut rrt.rtree,
        );

        if let Some(new_node) = q_new {
            rrt.stall_counter = 0;

            if new_node == goal {
                let path = state.reconstruct_path(goal);
                state.best_path = Some(path.clone());
                state.locked_path = Some(path.clone());
                state.fill_path_metrics(graph, &path);
                if state.status == PlannerStatus::Searching {
                    state.status = PlannerStatus::FirstMatchFound;
                }
                return false;
            }
        } else {
            rrt.stall_counter += 1;
            if rrt.stall_counter >= STALL_THRESHOLD {
                let dx = rrt.bbox_max[0] - rrt.bbox_min[0];
                let dy = rrt.bbox_max[1] - rrt.bbox_min[1];
                let expand_x = dx * (BBOX_EXPAND_FACTOR - 1.0) / 2.0;
                let expand_y = dy * (BBOX_EXPAND_FACTOR - 1.0) / 2.0;
                rrt.bbox_min[0] = (rrt.bbox_min[0] - expand_x).max(rrt.graph_bbox_min[0]);
                rrt.bbox_min[1] = (rrt.bbox_min[1] - expand_y).max(rrt.graph_bbox_min[1]);
                rrt.bbox_max[0] = (rrt.bbox_max[0] + expand_x).min(rrt.graph_bbox_max[0]);
                rrt.bbox_max[1] = (rrt.bbox_max[1] + expand_y).min(rrt.graph_bbox_max[1]);
                rrt.stall_counter = 0;
            }
        }
    }

    state.status == PlannerStatus::Searching
}

#[allow(clippy::too_many_arguments)]
fn extend(
    graph: &RoadGraph,
    q_near: usize,
    q_rand: &[f64; 2],
    explored: &mut HashSet<usize>,
    came_from: &mut HashMap<usize, usize>,
    g_score: &mut HashMap<usize, f64>,
    frontier: &mut HashSet<usize>,
    expanded_count: &mut usize,
    cost_mode: crate::planner::state::CostMode,
    exhausted: &mut HashSet<usize>,
    rtree: &mut RTree<TreePoint>,
) -> Option<usize> {
    if q_near >= graph.adjacency.len() {
        mark_exhausted(q_near, graph.nodes[q_near].world_pos, exhausted, rtree);
        return None;
    }

    let mut best_neighbor: Option<usize> = None;
    let mut best_edge_idx: Option<usize> = None;
    let mut best_dist_sq = f64::INFINITY;

    for &edge_idx in &graph.adjacency[q_near] {
        let edge = &graph.edges[edge_idx];
        let neighbor = edge.to;

        if explored.contains(&neighbor) {
            continue;
        }

        let neighbor_pos = graph.nodes[neighbor].world_pos;
        let dx = neighbor_pos[0] - q_rand[0];
        let dy = neighbor_pos[1] - q_rand[1];
        let dist_sq = dx * dx + dy * dy;

        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best_neighbor = Some(neighbor);
            best_edge_idx = Some(edge_idx);
        }
    }

    match best_neighbor {
        Some(q_new) => {
            let g_near = *g_score.get(&q_near).unwrap_or(&0.0);
            let edge = &graph.edges[best_edge_idx.unwrap()];
            let new_g = g_near + PlannerState::edge_cost(graph, edge, cost_mode);

            came_from.insert(q_new, q_near);
            g_score.insert(q_new, new_g);
            explored.insert(q_new);
            frontier.insert(q_new);
            frontier.remove(&q_near);
            *expanded_count += 1;

            rtree.insert(TreePoint {
                pos: graph.nodes[q_new].world_pos,
                node_id: q_new,
            });

            if graph.adjacency[q_near].iter().all(|&ei| {
                let n = graph.edges[ei].to;
                explored.contains(&n)
            }) {
                mark_exhausted(q_near, graph.nodes[q_near].world_pos, exhausted, rtree);
            }

            Some(q_new)
        }
        None => {
            mark_exhausted(q_near, graph.nodes[q_near].world_pos, exhausted, rtree);
            None
        }
    }
}

fn mark_exhausted(
    node_id: usize,
    pos: [f64; 2],
    exhausted: &mut HashSet<usize>,
    rtree: &mut RTree<TreePoint>,
) {
    if exhausted.insert(node_id)
        && let Some(tp) = rtree.remove_at_point(&pos)
    {
        let _ = tp;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{GraphEdge, GraphNode, RoadClass};
    use crate::planner::state::{CostMode, PlannerConfig};

    fn make_test_graph(nodes: Vec<[f64; 2]>, edges: Vec<(usize, usize, f64)>) -> RoadGraph {
        let mut graph = RoadGraph::new();
        graph.nodes = nodes
            .into_iter()
            .enumerate()
            .map(|(id, pos)| GraphNode {
                id,
                world_pos: pos,
                lat_lon: [0.0, 0.0],
            })
            .collect();
        graph.adjacency = vec![Vec::new(); graph.nodes.len()];

        for (from, to, weight) in edges {
            let edge_idx = graph.edges.len();
            graph.edges.push(GraphEdge {
                from,
                to,
                weight_meters: weight,
                travel_time_s: weight / 10.0,
                polyline_world: vec![graph.nodes[from].world_pos, graph.nodes[to].world_pos],
                road_class: RoadClass::Residential,
                one_way: true,
            });
            graph.adjacency[from].push(edge_idx);
        }

        graph
    }

    #[test]
    fn test_rrt_trivial_two_node() {
        let graph = make_test_graph(vec![[0.0, 0.0], [100.0, 0.0]], vec![(0, 1, 100.0)]);

        let mut state = PlannerState::new();
        state.config = PlannerConfig {
            algorithm: crate::planner::state::Algorithm::Rrt,
            heuristic: crate::planner::heuristic::Heuristic::Euclidean,
            cost_mode: CostMode::ShortestPath,
        };
        state.start_search(0, 1);
        state.rrt = Some(RrtState::new_with_seed(0, 1, &graph, 42));
        state.open_set.clear();

        for _ in 0..100 {
            let still_searching = step_rrt(&mut state, &graph, 10);
            if state.status == PlannerStatus::FirstMatchFound || state.status == PlannerStatus::Done
            {
                break;
            }
            if !still_searching {
                break;
            }
        }

        assert!(
            state.status == PlannerStatus::FirstMatchFound || state.status == PlannerStatus::Done,
            "RRT should have finished, but status is {:?}",
            state.status
        );

        if state.status == PlannerStatus::FirstMatchFound {
            assert_eq!(state.locked_path.as_ref().unwrap().len(), 2);
            assert_eq!(state.locked_path.as_ref().unwrap()[0], 0);
            assert_eq!(state.locked_path.as_ref().unwrap()[1], 1);
        }
    }

    #[test]
    fn test_rrt_linear_chain() {
        let mut nodes = Vec::new();
        let mut edges = Vec::new();
        for i in 0..10 {
            nodes.push([i as f64 * 100.0, 0.0]);
            if i < 9 {
                edges.push((i, i + 1, 100.0));
            }
        }
        let graph = make_test_graph(nodes, edges);

        let mut state = PlannerState::new();
        state.config = PlannerConfig {
            algorithm: crate::planner::state::Algorithm::Rrt,
            heuristic: crate::planner::heuristic::Heuristic::Euclidean,
            cost_mode: CostMode::ShortestPath,
        };
        state.start_search(0, 9);
        state.rrt = Some(RrtState::new_with_seed(0, 9, &graph, 42));
        state.open_set.clear();

        for _ in 0..200 {
            let still_searching = step_rrt(&mut state, &graph, 10);
            if state.status == PlannerStatus::FirstMatchFound || state.status == PlannerStatus::Done
            {
                break;
            }
            if !still_searching {
                break;
            }
        }

        assert!(
            state.status == PlannerStatus::FirstMatchFound,
            "RRT should have found the path, but status is {:?}",
            state.status
        );
        assert_eq!(state.locked_path.as_ref().unwrap().len(), 10);
    }

    #[test]
    fn test_rrt_unreachable_goal() {
        let graph = make_test_graph(vec![[0.0, 0.0], [100.0, 0.0]], vec![]);

        let mut state = PlannerState::new();
        state.config = PlannerConfig {
            algorithm: crate::planner::state::Algorithm::Rrt,
            heuristic: crate::planner::heuristic::Heuristic::Euclidean,
            cost_mode: CostMode::ShortestPath,
        };
        state.start_search(0, 1);
        state.rrt = Some(RrtState::new_with_seed(0, 1, &graph, 42));
        state.open_set.clear();

        for _ in 0..50 {
            let still_searching = step_rrt(&mut state, &graph, 10);
            if state.status == PlannerStatus::Done {
                break;
            }
            if !still_searching {
                break;
            }
        }

        assert_eq!(
            state.status,
            PlannerStatus::Done,
            "RRT should terminate on unreachable goal"
        );
    }

    #[test]
    fn test_rrt_goal_bias_extremes() {
        let mut nodes = Vec::new();
        let mut edges = Vec::new();
        for i in 0..5 {
            nodes.push([i as f64 * 100.0, 0.0]);
            if i < 4 {
                edges.push((i, i + 1, 100.0));
            }
        }
        let graph = make_test_graph(nodes, edges);

        let mut state = PlannerState::new();
        state.config = PlannerConfig {
            algorithm: crate::planner::state::Algorithm::Rrt,
            heuristic: crate::planner::heuristic::Heuristic::Euclidean,
            cost_mode: CostMode::ShortestPath,
        };
        state.start_search(0, 4);
        state.rrt = Some(RrtState::new_with_seed(0, 4, &graph, 42).with_goal_bias(1.0));
        state.open_set.clear();

        for _ in 0..20 {
            let still_searching = step_rrt(&mut state, &graph, 5);
            if state.status == PlannerStatus::FirstMatchFound || state.status == PlannerStatus::Done
            {
                break;
            }
            if !still_searching {
                break;
            }
        }

        assert!(
            state.status == PlannerStatus::FirstMatchFound,
            "With goal_bias=1.0, RRT should march directly toward goal"
        );
    }
}
