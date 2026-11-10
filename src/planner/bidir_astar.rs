use crate::graph::RoadGraph;
use crate::planner::state::{HeapEntry, PlannerState, PlannerStatus};
use ordered_float::NotNan;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};

/// Per-search state for bidirectional A*. The forward search reuses the
/// `open_set`/`g_score`/`came_from` fields of [`PlannerState`]; the backward
/// search lives here.
pub struct BidirAStarState {
    start: usize,
    /// rev_adjacency[node] = edge indices with edge.to == node.
    // shortcut: rebuilt per search (O(E)); move into RoadGraph if searches get frequent.
    rev_adjacency: Vec<Vec<usize>>,
    closed_fwd: HashSet<usize>,
    closed_bwd: HashSet<usize>,
    open_bwd: BinaryHeap<HeapEntry>,
    g_bwd: HashMap<usize, f64>,
    /// came_to[v] = successor of v on the best known v -> goal path.
    came_to: HashMap<usize, usize>,
    /// Cost of the best start -> goal path found so far.
    mu: f64,
    meet: Option<usize>,
    expand_backward: bool,
}

impl BidirAStarState {
    fn new(start: usize, goal: usize, graph: &RoadGraph) -> Self {
        let mut rev_adjacency = vec![Vec::new(); graph.nodes.len()];
        for (idx, edge) in graph.edges.iter().enumerate() {
            rev_adjacency[edge.to].push(idx);
        }
        let mut open_bwd = BinaryHeap::new();
        open_bwd.push(HeapEntry {
            cost: Reverse(NotNan::new(0.0).unwrap()),
            node_id: goal,
        });
        Self {
            start,
            rev_adjacency,
            closed_fwd: HashSet::new(),
            closed_bwd: HashSet::new(),
            open_bwd,
            g_bwd: HashMap::from([(goal, 0.0)]),
            came_to: HashMap::new(),
            mu: if start == goal { 0.0 } else { f64::INFINITY },
            meet: if start == goal { Some(start) } else { None },
            expand_backward: false,
        }
    }
}

/// Forward potential with the "average" construction
/// p_f(v) = (h(v, goal) - h(start, v)) / 2 and p_b = -p_f. Both directions then
/// see the same non-negative reduced edge costs (for a consistent heuristic), so
/// the search may stop once top_f + top_b >= mu.
fn potential_fwd(
    state: &PlannerState,
    graph: &RoadGraph,
    start: usize,
    goal: usize,
    v: usize,
) -> f64 {
    let h = state.config.heuristic;
    let mode = state.config.cost_mode;
    0.5 * (h.eval(graph, v, goal, mode) - h.eval(graph, start, v, mode))
}

fn top_key(heap: &BinaryHeap<HeapEntry>) -> f64 {
    heap.peek().map_or(f64::INFINITY, |e| e.cost.0.into_inner())
}

fn push(heap: &mut BinaryHeap<HeapEntry>, key: f64, node_id: usize) -> bool {
    match NotNan::new(key) {
        Ok(k) => {
            heap.push(HeapEntry {
                cost: Reverse(k),
                node_id,
            });
            true
        }
        Err(_) => false,
    }
}

fn finish(state: &mut PlannerState, graph: &RoadGraph) {
    let bidir = state.bidir_astar.as_ref().unwrap();
    if let Some(meet) = bidir.meet {
        let mut path = state.reconstruct_path(meet);
        let mut current = meet;
        while let Some(&next) = bidir.came_to.get(&current) {
            path.push(next);
            current = next;
            if path.len() > 1_000_000 {
                break;
            }
        }
        state.best_path = Some(path.clone());
        state.locked_path = Some(path.clone());
        state.fill_path_metrics(graph, &path);
        state.status = PlannerStatus::FirstMatchFound;
    } else {
        state.status = PlannerStatus::Done;
    }
}

/// Bidirectional A* step: alternates one expansion from the start (forward
/// edges) and one from the goal (reversed edges) until the frontiers prove the
/// best meeting point optimal.
pub fn step_bidir_astar(state: &mut PlannerState, graph: &RoadGraph, n_steps: usize) -> bool {
    let goal = match state.goal {
        Some(g) => g,
        None => return false,
    };

    if state.bidir_astar.is_none() {
        let start = state.frontier.iter().next().copied().unwrap_or(0);
        state.bidir_astar = Some(BidirAStarState::new(start, goal, graph));
        state.frontier.insert(goal);
    }

    for _ in 0..n_steps {
        if state.status == PlannerStatus::Done {
            return false;
        }
        if state.status == PlannerStatus::FirstMatchFound {
            state.status = PlannerStatus::Done;
            return false;
        }

        let bidir = state.bidir_astar.as_ref().unwrap();
        let (top_f, top_b) = (top_key(&state.open_set), top_key(&bidir.open_bwd));
        // Stop when either side is exhausted or no better meeting can exist.
        if top_f.is_infinite() || top_b.is_infinite() || top_f + top_b >= bidir.mu {
            finish(state, graph);
            return false;
        }

        let backward = bidir.expand_backward;
        state.bidir_astar.as_mut().unwrap().expand_backward = !backward;
        if backward {
            expand_backward(state, graph, goal);
        } else {
            expand_forward(state, graph, goal);
        }
    }

    state.status == PlannerStatus::Searching
}

fn expand_forward(state: &mut PlannerState, graph: &RoadGraph, goal: usize) {
    let current = state.open_set.pop().unwrap().node_id;
    let start = state.bidir_astar.as_ref().unwrap().start;
    if !state
        .bidir_astar
        .as_mut()
        .unwrap()
        .closed_fwd
        .insert(current)
    {
        return;
    }
    state.explored.insert(current);
    state.frontier.remove(&current);
    state.expanded_count += 1;

    let g = state.g_score[&current];
    for &edge_idx in graph.adjacency.get(current).into_iter().flatten() {
        let edge = &graph.edges[edge_idx];
        let neighbor = edge.to;
        if state
            .bidir_astar
            .as_ref()
            .unwrap()
            .closed_fwd
            .contains(&neighbor)
        {
            continue;
        }
        let new_g = g + PlannerState::edge_cost(graph, edge, state.config.cost_mode);
        if new_g >= *state.g_score.get(&neighbor).unwrap_or(&f64::INFINITY) {
            continue;
        }
        state.g_score.insert(neighbor, new_g);
        state.came_from.insert(neighbor, current);
        let key = new_g + potential_fwd(state, graph, start, goal, neighbor);
        if push(&mut state.open_set, key, neighbor) {
            state.frontier.insert(neighbor);
        }
        let bidir = state.bidir_astar.as_mut().unwrap();
        if let Some(&g_b) = bidir.g_bwd.get(&neighbor)
            && new_g + g_b < bidir.mu
        {
            bidir.mu = new_g + g_b;
            bidir.meet = Some(neighbor);
        }
    }
}

fn expand_backward(state: &mut PlannerState, graph: &RoadGraph, goal: usize) {
    let mut bidir = state.bidir_astar.take().unwrap();
    let current = bidir.open_bwd.pop().unwrap().node_id;
    if bidir.closed_bwd.insert(current) {
        state.explored.insert(current);
        state.frontier.remove(&current);
        state.expanded_count += 1;

        let g = bidir.g_bwd[&current];
        for &edge_idx in bidir.rev_adjacency.get(current).into_iter().flatten() {
            let edge = &graph.edges[edge_idx];
            let neighbor = edge.from;
            if bidir.closed_bwd.contains(&neighbor) {
                continue;
            }
            let new_g = g + PlannerState::edge_cost(graph, edge, state.config.cost_mode);
            if new_g >= *bidir.g_bwd.get(&neighbor).unwrap_or(&f64::INFINITY) {
                continue;
            }
            bidir.g_bwd.insert(neighbor, new_g);
            bidir.came_to.insert(neighbor, current);
            let key = new_g - potential_fwd(state, graph, bidir.start, goal, neighbor);
            if push(&mut bidir.open_bwd, key, neighbor) {
                state.frontier.insert(neighbor);
            }
            if let Some(&g_f) = state.g_score.get(&neighbor)
                && g_f + new_g < bidir.mu
            {
                bidir.mu = g_f + new_g;
                bidir.meet = Some(neighbor);
            }
        }
    }
    state.bidir_astar = Some(bidir);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{GraphEdge, GraphNode, RoadClass};
    use crate::planner::heuristic::Heuristic;
    use crate::planner::state::{Algorithm, CostMode, PlannerConfig};

    fn make_graph(nodes: Vec<[f64; 2]>, edges: Vec<(usize, usize, f64)>) -> RoadGraph {
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
            graph.adjacency[from].push(graph.edges.len());
            graph.edges.push(GraphEdge {
                from,
                to,
                weight_meters: weight,
                travel_time_s: weight / 10.0,
                polyline_world: vec![graph.nodes[from].world_pos, graph.nodes[to].world_pos],
                road_class: RoadClass::Residential,
                one_way: true,
            });
        }
        graph
    }

    fn run(graph: &RoadGraph, algorithm: Algorithm, start: usize, goal: usize) -> PlannerState {
        let mut state = PlannerState::new();
        state.config = PlannerConfig {
            algorithm,
            heuristic: Heuristic::Euclidean,
            cost_mode: CostMode::ShortestPath,
        };
        state.start_search(start, goal);
        while state.step(graph, 100) {}
        state
    }

    /// Grid with one-way edges and a few detours; bidirectional A* must match
    /// Dijkstra's optimal cost on every start/goal pair.
    #[test]
    fn matches_dijkstra_on_grid() {
        let n = 6;
        let mut nodes = Vec::new();
        let mut edges = Vec::new();
        for y in 0..n {
            for x in 0..n {
                nodes.push([x as f64 * 100.0, y as f64 * 100.0]);
            }
        }
        for y in 0..n {
            for x in 0..n {
                let i = y * n + x;
                // Pseudo-random extra weight keeps edge costs >= straight-line length.
                let w = |a: usize, b: usize| 100.0 + ((a * 7 + b * 13) % 5) as f64 * 40.0;
                if x + 1 < n {
                    edges.push((i, i + 1, w(i, i + 1)));
                    if (x + y) % 3 != 0 {
                        edges.push((i + 1, i, w(i + 1, i)));
                    }
                }
                if y + 1 < n {
                    edges.push((i, i + n, w(i, i + n)));
                    if (x * y) % 4 != 1 {
                        edges.push((i + n, i, w(i + n, i)));
                    }
                }
            }
        }
        let graph = make_graph(nodes, edges);

        for start in 0..n * n {
            for goal in 0..n * n {
                if start == goal {
                    continue;
                }
                let dij = run(&graph, Algorithm::Dijkstra, start, goal);
                let bi = run(&graph, Algorithm::BidirAStar, start, goal);
                assert_eq!(dij.locked_path.is_some(), bi.locked_path.is_some());
                if let Some(path) = &bi.locked_path {
                    assert_eq!(path.first(), Some(&start));
                    assert_eq!(path.last(), Some(&goal));
                    assert!(
                        (dij.locked_path_dist_m - bi.locked_path_dist_m).abs() < 1e-6,
                        "{start}->{goal}: dijkstra {} vs bidir {}",
                        dij.locked_path_dist_m,
                        bi.locked_path_dist_m
                    );
                }
            }
        }
    }

    #[test]
    fn unreachable_goal_finishes_without_path() {
        let graph = make_graph(vec![[0.0, 0.0], [100.0, 0.0]], vec![(1, 0, 100.0)]);
        let state = run(&graph, Algorithm::BidirAStar, 0, 1);
        assert_eq!(state.status, PlannerStatus::Done);
        assert!(state.locked_path.is_none());
    }
}
