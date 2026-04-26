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
