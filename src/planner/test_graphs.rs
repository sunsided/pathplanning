//! Small graphs shared by planner tests.

use crate::graph::{GraphEdge, GraphNode, RoadClass, RoadGraph};
use crate::planner::heuristic::Heuristic;
use crate::planner::state::{Algorithm, CostMode, PlannerConfig, PlannerState};

pub(crate) fn make_graph(nodes: Vec<[f64; 2]>, edges: Vec<(usize, usize, f64)>) -> RoadGraph {
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

/// `n x n` grid, 100 m spacing, with pseudo-random detour weights (always at
/// least the straight-line length) and a sprinkling of one-way streets.
pub(crate) fn one_way_grid(n: usize) -> RoadGraph {
    let mut nodes = Vec::new();
    let mut edges = Vec::new();
    for y in 0..n {
        for x in 0..n {
            nodes.push([x as f64 * 100.0, y as f64 * 100.0]);
        }
    }
    let w = |a: usize, b: usize| 100.0 + ((a * 7 + b * 13) % 5) as f64 * 40.0;
    for y in 0..n {
        for x in 0..n {
            let i = y * n + x;
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
    make_graph(nodes, edges)
}

pub(crate) fn run(
    graph: &RoadGraph,
    algorithm: Algorithm,
    heuristic: Heuristic,
    start: usize,
    goal: usize,
) -> PlannerState {
    let mut state = PlannerState::new();
    state.config = PlannerConfig {
        algorithm,
        heuristic,
        cost_mode: CostMode::ShortestPath,
    };
    state.start_search(start, goal);
    while state.step(graph, 100) {}
    state
}

/// Asserts `algorithm` finds a path exactly as cheap as Dijkstra's (or none
/// when Dijkstra finds none) for every ordered node pair of `graph`.
pub(crate) fn assert_optimal_all_pairs(
    graph: &RoadGraph,
    algorithm: Algorithm,
    heuristic: Heuristic,
) {
    let n = graph.nodes.len();
    for start in 0..n {
        for goal in 0..n {
            if start == goal {
                continue;
            }
            let dij = run(graph, Algorithm::Dijkstra, Heuristic::Zero, start, goal);
            let got = run(graph, algorithm, heuristic, start, goal);
            assert_eq!(
                dij.locked_path.is_some(),
                got.locked_path.is_some(),
                "{algorithm:?}/{heuristic:?} {start}->{goal}: reachability differs"
            );
            if let Some(path) = &got.locked_path {
                assert_eq!(path.first(), Some(&start));
                assert_eq!(path.last(), Some(&goal));
                for pair in path.windows(2) {
                    assert!(
                        graph.adjacency[pair[0]]
                            .iter()
                            .any(|&e| graph.edges[e].to == pair[1]),
                        "{algorithm:?} {start}->{goal}: no edge {} -> {}",
                        pair[0],
                        pair[1]
                    );
                }
                assert!(
                    (dij.locked_path_dist_m - got.locked_path_dist_m).abs() < 1e-6,
                    "{algorithm:?}/{heuristic:?} {start}->{goal}: dijkstra {} vs {}",
                    dij.locked_path_dist_m,
                    got.locked_path_dist_m
                );
            }
        }
    }
}
