use crate::graph::RoadGraph;

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

    pub fn eval(&self, graph: &RoadGraph, from: usize, to: usize) -> f64 {
        let n = graph.nodes.len();
        if from >= n || to >= n {
            return 0.0;
        }
        let p1 = graph.nodes[from].world_pos;
        let p2 = graph.nodes[to].world_pos;
        let dx = (p1[0] - p2[0]).abs();
        let dy = (p1[1] - p2[1]).abs();

        match self {
            Heuristic::Zero => 0.0,
            Heuristic::Euclidean => (dx * dx + dy * dy).sqrt(),
            Heuristic::Manhattan => dx + dy,
            Heuristic::Octile => {
                let dmin = dx.min(dy);
                let dmax = dx.max(dy);
                (dmax - dmin) + dmin * 2.0_f64.sqrt()
            }
        }
    }
}
