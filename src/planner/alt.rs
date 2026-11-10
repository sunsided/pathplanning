use crate::graph::RoadGraph;
use crate::planner::cache::{Deadline, DijkstraRun};
use crate::planner::state::CostMode;

const LANDMARK_COUNT: usize = 8;

/// ALT (A*, Landmarks, Triangle inequality) lower bounds.
///
/// For a landmark `L`, the triangle inequality gives
/// `d(v, t) >= d(L, t) - d(L, v)` and `d(v, t) >= d(v, L) - d(t, L)`; the
/// heuristic is the maximum over all landmarks. It is consistent, so it is safe
/// for every A* variant including bidirectional ones.
pub struct Landmarks {
    /// from[i][v] = d(L_i, v)
    from: Vec<Vec<f64>>,
    /// to[i][v] = d(v, L_i)
    to: Vec<Vec<f64>>,
}

/// Resumable landmark selection and distance computation.
///
/// Picks landmarks by farthest selection: seed with the round-trip distances
/// from the node nearest the graph's center, then repeatedly take the node
/// farthest (by round-trip network distance) from everything chosen so far.
/// Requiring finite round trips keeps landmarks inside the center's strongly
/// connected component; a landmark on a dead end (common at clipped map
/// borders) would bound almost nothing.
pub(crate) struct LandmarksBuilder {
    mode: CostMode,
    /// Forward and backward Dijkstra from the current seed/landmark.
    runs: Option<[DijkstraRun; 2]>,
    seeded: bool,
    /// closest[v] = min round-trip distance from v to the seed and to every
    /// landmark chosen so far.
    closest: Vec<f64>,
    from: Vec<Vec<f64>>,
    to: Vec<Vec<f64>>,
}

impl LandmarksBuilder {
    pub(crate) fn new(graph: &RoadGraph, mode: CostMode) -> Self {
        let seed = graph.bounding_box().and_then(|(min, max)| {
            let center = [(min[0] + max[0]) * 0.5, (min[1] + max[1]) * 0.5];
            let sq_dist = |v: usize| {
                let p = graph.nodes[v].world_pos;
                (p[0] - center[0]).powi(2) + (p[1] - center[1]).powi(2)
            };
            (0..graph.nodes.len())
                .filter(|&v| !graph.adjacency[v].is_empty())
                .min_by(|&a, &b| sq_dist(a).total_cmp(&sq_dist(b)))
        });
        Self {
            mode,
            runs: seed.map(|s| Self::runs_from(graph, s)),
            seeded: false,
            closest: Vec::new(),
            from: Vec::new(),
            to: Vec::new(),
        }
    }

    fn runs_from(graph: &RoadGraph, source: usize) -> [DijkstraRun; 2] {
        [false, true].map(|backward| DijkstraRun::new(graph, source, backward))
    }

    pub(crate) fn progress(&self) -> f32 {
        (self.from.len() + self.seeded as usize) as f32 / (LANDMARK_COUNT + 1) as f32
    }

    /// Works until `deadline`; returns the landmarks once complete.
    pub(crate) fn advance(&mut self, graph: &RoadGraph, deadline: &Deadline) -> Option<Landmarks> {
        while let Some(runs) = &mut self.runs {
            for run in runs.iter_mut() {
                if !run.advance(graph, self.mode, deadline) {
                    return None;
                }
            }
            let [d_from, d_to] = self.runs.take().unwrap().map(|r| r.dist);
            if self.seeded {
                for (c, (a, b)) in self.closest.iter_mut().zip(d_from.iter().zip(&d_to)) {
                    *c = c.min(a + b);
                }
                self.from.push(d_from);
                self.to.push(d_to);
            } else {
                self.closest = d_from.iter().zip(&d_to).map(|(a, b)| a + b).collect();
                self.seeded = true;
            }
            if self.from.len() < LANDMARK_COUNT {
                let closest = &self.closest;
                self.runs = (0..closest.len())
                    .filter(|&v| closest[v].is_finite() && closest[v] > 0.0)
                    .max_by(|&a, &b| closest[a].total_cmp(&closest[b]))
                    .map(|landmark| Self::runs_from(graph, landmark));
            }
        }
        Some(Landmarks {
            from: std::mem::take(&mut self.from),
            to: std::mem::take(&mut self.to),
        })
    }
}

impl Landmarks {
    /// Builds all landmarks in one go (blocking).
    pub(crate) fn build(graph: &RoadGraph, mode: CostMode) -> Self {
        LandmarksBuilder::new(graph, mode)
            .advance(graph, &Deadline::never())
            .expect("unbounded build completes")
    }

    pub(crate) fn lower_bound(&self, v: usize, t: usize) -> f64 {
        let mut best = 0.0_f64;
        for (from, to) in self.from.iter().zip(&self.to) {
            // Skip landmarks that cannot see both nodes: inf - inf is NaN.
            if from[t].is_finite() && from[v].is_finite() {
                best = best.max(from[t] - from[v]);
            }
            if to[v].is_finite() && to[t].is_finite() {
                best = best.max(to[v] - to[t]);
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use crate::planner::cache::dijkstra_all;
    use crate::planner::heuristic::Heuristic;
    use crate::planner::state::CostMode;
    use crate::planner::test_graphs::one_way_grid;

    #[test]
    fn alt_is_admissible_and_consistent() {
        let graph = one_way_grid(6);
        let mode = CostMode::ShortestPath;
        for t in 0..graph.nodes.len() {
            let exact = dijkstra_all(&graph, t, true, mode);
            for (v, &d) in exact.iter().enumerate() {
                let h = Heuristic::Alt.eval(&graph, v, t, mode);
                assert!(h <= d + 1e-9, "h({v},{t})={h} > d={d}");
                for (w, c) in graph.neighbors(v, false, mode) {
                    let hw = Heuristic::Alt.eval(&graph, w, t, mode);
                    assert!(h <= c + hw + 1e-9, "inconsistent at {v}->{w}");
                }
            }
        }
    }

    #[test]
    fn alt_beats_euclidean_somewhere() {
        let graph = one_way_grid(6);
        let mode = CostMode::ShortestPath;
        let n = graph.nodes.len();
        let tighter = (0..n)
            .flat_map(|v| (0..n).map(move |t| (v, t)))
            .any(|(v, t)| {
                Heuristic::Alt.eval(&graph, v, t, mode)
                    > Heuristic::Euclidean.eval(&graph, v, t, mode) + 1.0
            });
        assert!(tighter);
    }
}
