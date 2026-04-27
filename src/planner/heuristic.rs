use crate::graph::RoadGraph;
use crate::planner::state::CostMode;

const MAX_SPEED_MPS: f64 = 25.0; // 90 km/h motorway

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Heuristic {
    Zero,
    Euclidean,
    Manhattan,
    Octile,
}

impl Heuristic {
    pub fn name(&self) -> &'static str {
        match self {
            Heuristic::Zero => "Zero",
            Heuristic::Euclidean => "Euclidean",
            Heuristic::Manhattan => "Manhattan",
            Heuristic::Octile => "Octile",
        }
    }

    pub fn short_label(&self) -> &'static str {
        match self {
            Heuristic::Zero => "Zero",
            Heuristic::Euclidean => "Euclid",
            Heuristic::Manhattan => "Manhat",
            Heuristic::Octile => "Octile",
        }
    }

    pub fn all() -> &'static [Heuristic] {
        &[
            Heuristic::Euclidean,
            Heuristic::Manhattan,
            Heuristic::Octile,
            Heuristic::Zero,
        ]
    }

    pub fn eval(&self, graph: &RoadGraph, from: usize, to: usize, mode: CostMode) -> f64 {
        let n = graph.nodes.len();
        if from >= n || to >= n {
            return 0.0;
        }
        let p1 = graph.nodes[from].world_pos;
        let p2 = graph.nodes[to].world_pos;
        // world_pos is Web Mercator, which stretches distances by 1/cos(lat).
        // Scale back by cos(mean_lat) to get ground meters — otherwise the
        // heuristic overestimates and A* returns suboptimal paths.
        let mean_lat_rad =
            0.5 * (graph.nodes[from].lat_lon[0] + graph.nodes[to].lat_lon[0]).to_radians();
        let scale = mean_lat_rad.cos();
        let dx = (p1[0] - p2[0]).abs() * scale;
        let dy = (p1[1] - p2[1]).abs() * scale;

        let dist_m = match self {
            Heuristic::Zero => 0.0,
            Heuristic::Euclidean => (dx * dx + dy * dy).sqrt(),
            Heuristic::Manhattan => dx + dy,
            Heuristic::Octile => {
                let dmin = dx.min(dy);
                let dmax = dx.max(dy);
                (dmax - dmin) + dmin * 2.0_f64.sqrt()
            }
        };

        match mode {
            CostMode::ShortestPath => dist_m,
            CostMode::ShortestTime => dist_m / MAX_SPEED_MPS,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::GraphNode;

    fn make_graph(nodes: Vec<([f64; 2], [f64; 2])>) -> RoadGraph {
        let mut graph = RoadGraph::new();
        graph.nodes = nodes
            .into_iter()
            .enumerate()
            .map(|(id, (world, lat_lon))| GraphNode {
                id,
                world_pos: world,
                lat_lon,
            })
            .collect();
        graph.adjacency = vec![Vec::new(); graph.nodes.len()];
        graph
    }

    fn equator_graph() -> RoadGraph {
        make_graph(vec![([0.0, 0.0], [0.0, 0.0]), ([300.0, 400.0], [0.0, 0.0])])
    }

    #[test]
    fn test_name_zero() {
        assert_eq!(Heuristic::Zero.name(), "Zero");
    }

    #[test]
    fn test_name_euclidean() {
        assert_eq!(Heuristic::Euclidean.name(), "Euclidean");
    }

    #[test]
    fn test_name_manhattan() {
        assert_eq!(Heuristic::Manhattan.name(), "Manhattan");
    }

    #[test]
    fn test_name_octile() {
        assert_eq!(Heuristic::Octile.name(), "Octile");
    }

    #[test]
    fn test_short_label_zero() {
        assert_eq!(Heuristic::Zero.short_label(), "Zero");
    }

    #[test]
    fn test_short_label_euclidean() {
        assert_eq!(Heuristic::Euclidean.short_label(), "Euclid");
    }

    #[test]
    fn test_short_label_manhattan() {
        assert_eq!(Heuristic::Manhattan.short_label(), "Manhat");
    }

    #[test]
    fn test_short_label_octile() {
        assert_eq!(Heuristic::Octile.short_label(), "Octile");
    }

    #[test]
    fn test_all_variants() {
        let all = Heuristic::all();
        assert_eq!(all.len(), 4);
        assert!(all.contains(&Heuristic::Zero));
        assert!(all.contains(&Heuristic::Euclidean));
        assert!(all.contains(&Heuristic::Manhattan));
        assert!(all.contains(&Heuristic::Octile));
    }

    #[test]
    fn test_eval_same_node_returns_zero() {
        let graph = make_graph(vec![([1000.0, 2000.0], [52.0, 10.0])]);
        for h in Heuristic::all() {
            let val = h.eval(&graph, 0, 0, CostMode::ShortestPath);
            assert_eq!(val, 0.0, "{:?} should return 0 for same node", h);
        }
    }

    #[test]
    fn test_eval_out_of_bounds_from() {
        let graph = make_graph(vec![([0.0, 0.0], [0.0, 0.0])]);
        for h in Heuristic::all() {
            let val = h.eval(&graph, 99, 0, CostMode::ShortestPath);
            assert_eq!(val, 0.0);
        }
    }

    #[test]
    fn test_eval_out_of_bounds_to() {
        let graph = make_graph(vec![([0.0, 0.0], [0.0, 0.0])]);
        for h in Heuristic::all() {
            let val = h.eval(&graph, 0, 99, CostMode::ShortestPath);
            assert_eq!(val, 0.0);
        }
    }

    #[test]
    fn test_eval_empty_graph() {
        let graph = RoadGraph::new();
        for h in Heuristic::all() {
            let val = h.eval(&graph, 0, 0, CostMode::ShortestPath);
            assert_eq!(val, 0.0);
        }
    }

    #[test]
    fn test_zero_at_equator() {
        let graph = equator_graph();
        assert_eq!(
            Heuristic::Zero.eval(&graph, 0, 1, CostMode::ShortestPath),
            0.0
        );
    }

    #[test]
    fn test_euclidean_at_equator() {
        let graph = equator_graph();
        let h = Heuristic::Euclidean.eval(&graph, 0, 1, CostMode::ShortestPath);
        let expected = (300.0_f64.powi(2) + 400.0_f64.powi(2)).sqrt();
        assert!(
            (h - expected).abs() < 1e-9,
            "expected {}, got {}",
            expected,
            h
        );
    }

    #[test]
    fn test_manhattan_at_equator() {
        let graph = equator_graph();
        let h = Heuristic::Manhattan.eval(&graph, 0, 1, CostMode::ShortestPath);
        let expected = 300.0 + 400.0;
        assert!(
            (h - expected).abs() < 1e-9,
            "expected {}, got {}",
            expected,
            h
        );
    }

    #[test]
    fn test_octile_at_equator() {
        let graph = equator_graph();
        let h = Heuristic::Octile.eval(&graph, 0, 1, CostMode::ShortestPath);
        let dmin = 300.0_f64.min(400.0);
        let dmax = 300.0_f64.max(400.0);
        let expected = (dmax - dmin) + dmin * 2.0_f64.sqrt();
        assert!(
            (h - expected).abs() < 1e-9,
            "expected {}, got {}",
            expected,
            h
        );
    }

    #[test]
    fn test_euclidean_axis_aligned_x() {
        let graph = make_graph(vec![([0.0, 0.0], [0.0, 0.0]), ([100.0, 0.0], [0.0, 0.0])]);
        let h = Heuristic::Euclidean.eval(&graph, 0, 1, CostMode::ShortestPath);
        assert!((h - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_euclidean_axis_aligned_y() {
        let graph = make_graph(vec![([0.0, 0.0], [0.0, 0.0]), ([0.0, 100.0], [0.0, 0.0])]);
        let h = Heuristic::Euclidean.eval(&graph, 0, 1, CostMode::ShortestPath);
        assert!((h - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_manhattan_axis_aligned() {
        let graph = make_graph(vec![([0.0, 0.0], [0.0, 0.0]), ([30.0, 40.0], [0.0, 0.0])]);
        let h = Heuristic::Manhattan.eval(&graph, 0, 1, CostMode::ShortestPath);
        assert!((h - 70.0).abs() < 1e-9);
    }

    #[test]
    fn test_octile_diagonal_equals_euclidean() {
        let graph = make_graph(vec![([0.0, 0.0], [0.0, 0.0]), ([100.0, 100.0], [0.0, 0.0])]);
        let octile = Heuristic::Octile.eval(&graph, 0, 1, CostMode::ShortestPath);
        let euclid = Heuristic::Euclidean.eval(&graph, 0, 1, CostMode::ShortestPath);
        assert!(
            (octile - euclid).abs() < 1e-9,
            "octile={}, euclid={}",
            octile,
            euclid
        );
    }

    #[test]
    fn test_octile_axis_aligned_equals_manhattan() {
        let graph = make_graph(vec![([0.0, 0.0], [0.0, 0.0]), ([100.0, 0.0], [0.0, 0.0])]);
        let octile = Heuristic::Octile.eval(&graph, 0, 1, CostMode::ShortestPath);
        let manhattan = Heuristic::Manhattan.eval(&graph, 0, 1, CostMode::ShortestPath);
        assert!(
            (octile - manhattan).abs() < 1e-9,
            "octile={}, manhattan={}",
            octile,
            manhattan
        );
    }

    #[test]
    fn test_latitude_scaling_reduces_distance() {
        let lat = 60.0;
        let graph = make_graph(vec![
            ([0.0, 0.0], [lat, 10.0]),
            ([1000.0, 0.0], [lat, 10.0]),
        ]);
        let euclid_equator = Heuristic::Euclidean.eval(
            &make_graph(vec![([0.0, 0.0], [0.0, 0.0]), ([1000.0, 0.0], [0.0, 0.0])]),
            0,
            1,
            CostMode::ShortestPath,
        );
        let euclid_60 = Heuristic::Euclidean.eval(&graph, 0, 1, CostMode::ShortestPath);
        assert!(
            euclid_60 < euclid_equator,
            "euclid_60={}, euclid_equator={}",
            euclid_60,
            euclid_equator
        );
        let expected_scale = (60.0_f64).to_radians().cos();
        let expected = 1000.0 * expected_scale;
        assert!(
            (euclid_60 - expected).abs() < 1e-6,
            "expected ~{}, got {}",
            expected,
            euclid_60
        );
    }

    #[test]
    fn test_shortest_time_divides_by_max_speed() {
        let graph = equator_graph();
        let dist = Heuristic::Euclidean.eval(&graph, 0, 1, CostMode::ShortestPath);
        let time = Heuristic::Euclidean.eval(&graph, 0, 1, CostMode::ShortestTime);
        let expected_time = dist / MAX_SPEED_MPS;
        assert!(
            (time - expected_time).abs() < 1e-9,
            "expected time {}, got {}",
            expected_time,
            time
        );
    }

    #[test]
    fn test_zero_is_zero_in_time_mode() {
        let graph = equator_graph();
        let val = Heuristic::Zero.eval(&graph, 0, 1, CostMode::ShortestTime);
        assert_eq!(val, 0.0);
    }

    #[test]
    fn test_zero_is_admissible() {
        let graph = make_graph(vec![
            ([123.0, 456.0], [45.0, 9.0]),
            ([789.0, 10.0], [45.0, 9.0]),
        ]);
        let h = Heuristic::Zero.eval(&graph, 0, 1, CostMode::ShortestPath);
        assert!(h <= 0.0 + 1e-9);
    }

    #[test]
    fn test_euclidean_le_manhattan() {
        let graph = make_graph(vec![([0.0, 0.0], [0.0, 0.0]), ([3.0, 4.0], [0.0, 0.0])]);
        let euclid = Heuristic::Euclidean.eval(&graph, 0, 1, CostMode::ShortestPath);
        let manhat = Heuristic::Manhattan.eval(&graph, 0, 1, CostMode::ShortestPath);
        assert!(
            euclid <= manhat + 1e-9,
            "euclid={}, manhattan={}",
            euclid,
            manhat
        );
    }

    #[test]
    fn test_euclidean_le_octile() {
        let graph = make_graph(vec![([0.0, 0.0], [0.0, 0.0]), ([3.0, 4.0], [0.0, 0.0])]);
        let euclid = Heuristic::Euclidean.eval(&graph, 0, 1, CostMode::ShortestPath);
        let octile = Heuristic::Octile.eval(&graph, 0, 1, CostMode::ShortestPath);
        assert!(
            euclid <= octile + 1e-9,
            "euclid={}, octile={}",
            euclid,
            octile
        );
    }

    #[test]
    fn test_heuristics_are_symmetric() {
        let graph = make_graph(vec![
            ([100.0, 200.0], [30.0, 5.0]),
            ([500.0, 600.0], [30.0, 5.0]),
        ]);
        for h in Heuristic::all() {
            for mode in &[CostMode::ShortestPath, CostMode::ShortestTime] {
                let fwd = h.eval(&graph, 0, 1, *mode);
                let rev = h.eval(&graph, 1, 0, *mode);
                assert!(
                    (fwd - rev).abs() < 1e-9,
                    "{:?} {:?}: fwd={}, rev={}",
                    h,
                    mode,
                    fwd,
                    rev
                );
            }
        }
    }
}
