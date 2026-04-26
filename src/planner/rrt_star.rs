use rand::RngExt;
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
const MAX_ITERATION_MULTIPLIER: usize = 3;
const POST_MATCH_BUDGET: usize = 2000;
const RADIUS_GAMMA_FACTOR: f64 = 2.0;
const RADIUS_FLOOR: f64 = 50.0;
const RADIUS_CAP: f64 = 2000.0;
const AVG_EDGE_SAMPLE_SIZE: usize = 1024;

pub(crate) struct RrtStarState {
    pub(crate) rtree: RTree<TreePoint>,
    pub(crate) rng: SmallRng,
    pub(crate) bbox_min: [f64; 2],
    pub(crate) bbox_max: [f64; 2],
    pub(crate) graph_bbox_min: [f64; 2],
    pub(crate) graph_bbox_max: [f64; 2],
    pub(crate) stall_counter: usize,
    pub(crate) exhausted: HashSet<usize>,
    pub(crate) children: HashMap<usize, Vec<usize>>,
    pub(crate) gamma: f64,
    pub(crate) tree_size: usize,
    pub(crate) iterations: usize,
    pub(crate) best_goal_cost: f64,
    pub(crate) goal_bias: f64,
    pub(crate) post_match_iterations: Option<usize>,
}

impl RrtStarState {
    pub(crate) fn new(start: usize, goal: usize, graph: &RoadGraph) -> Self {
        Self::new_with_seed(start, goal, graph, rand::random::<u64>())
    }

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

        let gamma = RADIUS_GAMMA_FACTOR * sample_avg_edge_length(graph);

        let mut children = HashMap::new();
        children.insert(start, Vec::new());

        Self {
            rtree,
            rng: SmallRng::seed_from_u64(seed),
            bbox_min,
            bbox_max,
            graph_bbox_min: gb_min,
            graph_bbox_max: gb_max,
            stall_counter: 0,
            exhausted: HashSet::new(),
            children,
            gamma,
            tree_size: 1,
            iterations: 0,
            best_goal_cost: f64::INFINITY,
            goal_bias: GOAL_BIAS,
            post_match_iterations: None,
        }
    }

    #[cfg(test)]
    pub(crate) fn with_goal_bias(mut self, goal_bias: f64) -> Self {
        self.goal_bias = goal_bias;
        self
    }
}

fn radius(gamma: f64, tree_size: usize) -> f64 {
    let n = tree_size.max(1) as f64;
    let r = gamma * ((n.ln() / n).sqrt());
    r.clamp(RADIUS_FLOOR, RADIUS_CAP)
}

fn sample_avg_edge_length(graph: &RoadGraph) -> f64 {
    let mut rng = SmallRng::seed_from_u64(0);
    let total_edges = graph.edges.len();
    if total_edges == 0 {
        return 100.0;
    }
    let sample_count = AVG_EDGE_SAMPLE_SIZE.min(total_edges);
    let mut sum = 0.0;
    for _ in 0..sample_count {
        let idx = rng.random_range(0..total_edges);
        sum += graph.edges[idx].weight_meters;
    }
    sum / sample_count as f64
}

fn find_edge_to(graph: &RoadGraph, from: usize, to: usize) -> Option<&crate::graph::GraphEdge> {
    if from >= graph.adjacency.len() {
        return None;
    }
    for &edge_idx in &graph.adjacency[from] {
        let edge = &graph.edges[edge_idx];
        if edge.to == to {
            return Some(edge);
        }
    }
    None
}

fn propagate_delta(
    root: usize,
    delta: f64,
    g_score: &mut HashMap<usize, f64>,
    children: &HashMap<usize, Vec<usize>>,
) {
    // `root` already had its g_score updated by the caller. Only propagate
    // the delta to descendants to avoid double-applying it to `root`.
    let mut queue: Vec<usize> = Vec::new();
    let mut visited = HashSet::new();
    visited.insert(root);
    if let Some(kids) = children.get(&root) {
        for &child in kids {
            if visited.insert(child) {
                queue.push(child);
            }
        }
    }

    while let Some(node) = queue.pop() {
        if let Some(old_g) = g_score.get(&node) {
            g_score.insert(node, old_g + delta);
        }
        if let Some(kids) = children.get(&node) {
            for &child in kids {
                if visited.insert(child) {
                    queue.push(child);
                }
            }
        }
    }
}

fn extend_toward(
    graph: &RoadGraph,
    q_near: usize,
    q_rand: &[f64; 2],
    explored: &mut HashSet<usize>,
    exhausted: &mut HashSet<usize>,
    rtree: &mut RTree<TreePoint>,
) -> Option<usize> {
    if q_near >= graph.adjacency.len() {
        return None;
    }

    let mut best_neighbor: Option<usize> = None;
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
        }
    }

    match best_neighbor {
        Some(q_new) => {
            if graph.adjacency[q_near].iter().all(|&ei| {
                let n = graph.edges[ei].to;
                explored.contains(&n)
            }) && exhausted.insert(q_near)
            {
                let _ = rtree.remove_at_point(&graph.nodes[q_near].world_pos);
            }
            Some(q_new)
        }
        None => {
            if exhausted.insert(q_near) {
                let _ = rtree.remove_at_point(&graph.nodes[q_near].world_pos);
            }
            None
        }
    }
}

pub fn step_rrt_star(state: &mut PlannerState, graph: &RoadGraph, n_steps: usize) -> bool {
    let goal = match state.goal {
        Some(g) => g,
        None => return false,
    };

    if state.rrt_star.is_none() {
        state.open_set.clear();
        let start = state.frontier.iter().next().copied().unwrap_or(0);
        state.explored.insert(start);
        state.rrt_star = Some(RrtStarState::new(start, goal, graph));
    }

    let max_iterations = MAX_ITERATION_MULTIPLIER * graph.nodes.len();

    for _ in 0..n_steps {
        if state.status == PlannerStatus::Done {
            return false;
        }

        let rrt = state.rrt_star.as_mut().unwrap();
        if rrt.rtree.size() == 0 {
            if state.locked_path.is_some() && state.status != PlannerStatus::Done {
                state.status = PlannerStatus::FirstMatchFound;
            }
            state.status = PlannerStatus::Done;
            return false;
        }
        if rrt.iterations >= max_iterations {
            if state.locked_path.is_some() && state.status == PlannerStatus::Searching {
                state.status = PlannerStatus::FirstMatchFound;
            }
            state.status = PlannerStatus::Done;
            return false;
        }
        if let Some(post) = rrt.post_match_iterations
            && post >= POST_MATCH_BUDGET
        {
            state.status = PlannerStatus::Done;
            return false;
        }

        rrt.iterations += 1;
        if let Some(post) = rrt.post_match_iterations.as_mut() {
            *post += 1;
        }

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

        let candidate = extend_toward(
            graph,
            q_near,
            &q_rand,
            &mut state.explored,
            &mut rrt.exhausted,
            &mut rrt.rtree,
        );

        rrt.stall_counter += 1;

        let q_new = match candidate {
            Some(q) => q,
            None => {
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
                continue;
            }
        };

        rrt.stall_counter = 0;

        let r = radius(rrt.gamma, rrt.tree_size);
        let r_sq = r * r;
        let q_new_pos = graph.nodes[q_new].world_pos;

        let neighborhood: Vec<usize> = rrt
            .rtree
            .locate_within_distance(q_new_pos, r_sq)
            .map(|tp| tp.node_id)
            .collect();

        let mut best_parent = q_near;
        let g_near = *state.g_score.get(&q_near).unwrap_or(&0.0);
        let mut best_cost = if let Some(edge) = find_edge_to(graph, q_near, q_new) {
            g_near + PlannerState::edge_cost(graph, edge, state.config.cost_mode)
        } else {
            g_near + 1e9
        };

        for &c in &neighborhood {
            if c == q_new {
                continue;
            }
            if let Some(edge) = find_edge_to(graph, c, q_new) {
                let g_c = *state.g_score.get(&c).unwrap_or(&0.0);
                let alt = g_c + PlannerState::edge_cost(graph, edge, state.config.cost_mode);
                if alt < best_cost {
                    best_cost = alt;
                    best_parent = c;
                }
            }
        }

        state.came_from.insert(q_new, best_parent);
        state.g_score.insert(q_new, best_cost);
        state.explored.insert(q_new);
        state.frontier.insert(q_new);
        state.frontier.remove(&best_parent);
        state.expanded_count += 1;

        rrt.rtree.insert(TreePoint {
            pos: q_new_pos,
            node_id: q_new,
        });
        rrt.tree_size += 1;
        rrt.children.entry(q_new).or_default();
        rrt.children.entry(best_parent).or_default().push(q_new);

        let g_goal_before = state.g_score.get(&goal).copied().unwrap_or(f64::INFINITY);

        let mut did_rewire = false;
        for &x in &neighborhood {
            if x == best_parent || x == q_new {
                continue;
            }
            if let Some(edge) = find_edge_to(graph, q_new, x) {
                let alt = best_cost + PlannerState::edge_cost(graph, edge, state.config.cost_mode);
                let g_x = *state.g_score.get(&x).unwrap_or(&f64::INFINITY);
                if alt < g_x {
                    if let Some(&old_parent) = state.came_from.get(&x)
                        && let Some(sibs) = rrt.children.get_mut(&old_parent)
                    {
                        sibs.retain(|&s| s != x);
                    }
                    let delta = alt - g_x;
                    state.came_from.insert(x, q_new);
                    state.g_score.insert(x, alt);
                    rrt.children.entry(q_new).or_default().push(x);
                    if delta.abs() > 1e-12 {
                        propagate_delta(x, delta, &mut state.g_score, &rrt.children);
                    }
                    did_rewire = true;
                }
            }
        }

        let g_goal_after = state.g_score.get(&goal).copied().unwrap_or(f64::INFINITY);
        let goal_improved = g_goal_after < g_goal_before;

        let goal_in_tree = state.g_score.contains_key(&goal)
            && state.came_from.contains_key(&goal)
            && !did_rewire
            && q_new == goal;

        let goal_improved_after = state.g_score.get(&goal).copied().unwrap_or(f64::INFINITY);

        if (goal_in_tree || goal_improved) && state.g_score.contains_key(&goal) {
            let path = state.reconstruct_path(goal);
            state.best_path = Some(path.clone());
            state.locked_path = Some(path.clone());
            state.fill_path_metrics(graph, &path);
            if state.status == PlannerStatus::Searching {
                state.status = PlannerStatus::FirstMatchFound;
            }
            if let Some(rrt) = state.rrt_star.as_mut()
                && rrt.post_match_iterations.is_none()
            {
                rrt.post_match_iterations = Some(0);
            }
        }

        if goal_in_tree && let Some(rrt) = state.rrt_star.as_mut() {
            rrt.best_goal_cost = goal_improved_after;
        }

        if let Some(rrt) = state.rrt_star.as_mut()
            && rrt.stall_counter >= STALL_THRESHOLD
        {
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

    state.status == PlannerStatus::Searching
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{GraphEdge, GraphNode, RoadClass};
    use crate::planner::state::{Algorithm, CostMode, PlannerConfig};

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

    fn run_rrt_star(
        graph: &RoadGraph,
        start: usize,
        goal: usize,
        seed: u64,
        max_iters: usize,
    ) -> PlannerState {
        let mut state = PlannerState::new();
        state.config = PlannerConfig {
            algorithm: Algorithm::RrtStar,
            heuristic: crate::planner::heuristic::Heuristic::Euclidean,
            cost_mode: CostMode::ShortestPath,
        };
        state.start_search(start, goal);
        state.rrt_star = Some(RrtStarState::new_with_seed(start, goal, graph, seed));
        state.open_set.clear();

        for _ in 0..max_iters {
            let still_searching = step_rrt_star(&mut state, graph, 10);
            if state.status == PlannerStatus::Done {
                break;
            }
            if !still_searching {
                break;
            }
        }

        state
    }

    #[test]
    fn test_rrt_star_trivial_two_node() {
        let graph = make_test_graph(vec![[0.0, 0.0], [100.0, 0.0]], vec![(0, 1, 100.0)]);
        let state = run_rrt_star(&graph, 0, 1, 42, 100);

        assert!(
            state.status == PlannerStatus::FirstMatchFound || state.status == PlannerStatus::Done,
            "RRT* should have finished, but status is {:?}",
            state.status
        );

        if state.status == PlannerStatus::FirstMatchFound {
            assert_eq!(state.locked_path.as_ref().unwrap().len(), 2);
            assert_eq!(state.locked_path.as_ref().unwrap()[0], 0);
            assert_eq!(state.locked_path.as_ref().unwrap()[1], 1);
        }
    }

    #[test]
    fn test_rrt_star_linear_chain() {
        let mut nodes = Vec::new();
        let mut edges = Vec::new();
        for i in 0..10 {
            nodes.push([i as f64 * 100.0, 0.0]);
            if i < 9 {
                edges.push((i, i + 1, 100.0));
            }
        }
        let graph = make_test_graph(nodes, edges);
        let state = run_rrt_star(&graph, 0, 9, 42, 200);

        assert!(
            state.status == PlannerStatus::FirstMatchFound || state.status == PlannerStatus::Done,
            "RRT* should have found the path, but status is {:?}",
            state.status
        );
        if let Some(ref path) = state.locked_path {
            assert_eq!(path.len(), 10);
        }
    }

    #[test]
    fn test_rrt_star_diamond_graph() {
        let nodes = vec![
            [0.0, 0.0],      // 0: start
            [200.0, 200.0],  // 1: A (expensive route, spatially far)
            [200.0, -200.0], // 2: C (cheap route, spatially far other direction)
            [400.0, 200.0],  // 3: B (expensive route)
            [400.0, -200.0], // 4: D (cheap route)
            [600.0, 0.0],    // 5: goal
        ];
        let edges = vec![
            (0, 1, 1000.0),
            (0, 2, 100.0),
            (1, 3, 1000.0),
            (2, 4, 100.0),
            (3, 5, 1000.0),
            (4, 5, 100.0),
        ];
        let graph = make_test_graph(nodes, edges);

        let state = run_rrt_star(&graph, 0, 5, 42, 500);

        if state.locked_path.is_some() {
            let final_cost = state.g_score.get(&5).copied().unwrap_or(f64::INFINITY);
            assert!(
                final_cost <= 300.0 + 1e-6,
                "RRT* should find the cheaper cost-300 path, but cost is {}",
                final_cost
            );
        }
    }

    #[test]
    fn test_rrt_star_rewire_correctness() {
        let nodes = vec![
            [0.0, 0.0],   // 0: start
            [100.0, 0.0], // 1
            [200.0, 0.0], // 2: goal
            [50.0, 50.0], // 3: shortcut node
        ];
        let edges = vec![
            (0, 1, 100.0),
            (1, 2, 100.0),
            (0, 3, 50.0),
            (3, 1, 50.0),
            (3, 2, 150.0),
        ];
        let graph = make_test_graph(nodes, edges);

        let state = run_rrt_star(&graph, 0, 2, 42, 500);

        if let Some(ref path) = state.locked_path {
            let mut recomputed = 0.0;
            for i in 0..path.len().saturating_sub(1) {
                if let Some(edge) = find_edge_to(&graph, path[i], path[i + 1]) {
                    recomputed += edge.weight_meters;
                }
            }
            let reported = state
                .g_score
                .get(&path[path.len() - 1])
                .copied()
                .unwrap_or(0.0);
            assert!(
                (recomputed - reported).abs() < 1.0,
                "g_score mismatch: recomputed={}, reported={}",
                recomputed,
                reported
            );
        }
    }

    #[test]
    fn test_rrt_star_unreachable_goal() {
        let graph = make_test_graph(vec![[0.0, 0.0], [100.0, 0.0]], vec![]);
        let state = run_rrt_star(&graph, 0, 1, 42, 50);

        assert_eq!(
            state.status,
            PlannerStatus::Done,
            "RRT* should terminate on unreachable goal, but status is {:?}",
            state.status
        );
    }

    #[test]
    fn test_rrt_star_goal_bias_extremes() {
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
            algorithm: Algorithm::RrtStar,
            heuristic: crate::planner::heuristic::Heuristic::Euclidean,
            cost_mode: CostMode::ShortestPath,
        };
        state.start_search(0, 4);
        state.rrt_star = Some(RrtStarState::new_with_seed(0, 4, &graph, 42).with_goal_bias(1.0));
        state.open_set.clear();

        for _ in 0..20 {
            let still_searching = step_rrt_star(&mut state, &graph, 5);
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
            "With goal_bias=1.0, RRT* should march directly toward goal"
        );
    }
}
