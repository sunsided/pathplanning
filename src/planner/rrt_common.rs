use rand::RngExt;
use rand::rngs::SmallRng;
use rstar::{AABB, PointDistance, RTreeObject};

use crate::graph::RoadGraph;

#[derive(Clone, PartialEq)]
pub(crate) struct TreePoint {
    pub(crate) pos: [f64; 2],
    pub(crate) node_id: usize,
}

impl RTreeObject for TreePoint {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_point(self.pos)
    }
}

impl PointDistance for TreePoint {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dx = self.pos[0] - point[0];
        let dy = self.pos[1] - point[1];
        dx * dx + dy * dy
    }
}

pub(crate) fn clamp_bbox(bbox: [f64; 2], graph_min: [f64; 2], graph_max: [f64; 2]) -> [f64; 2] {
    [
        bbox[0].max(graph_min[0]).min(graph_max[0]),
        bbox[1].max(graph_min[1]).min(graph_max[1]),
    ]
}

pub(crate) fn ensure_bbox_margin(
    min: [f64; 2],
    max: [f64; 2],
    margin: f64,
) -> ([f64; 2], [f64; 2]) {
    let cx = (min[0] + max[0]) / 2.0;
    let cy = (min[1] + max[1]) / 2.0;
    let half_w = ((max[0] - min[0]) / 2.0).max(margin);
    let half_h = ((max[1] - min[1]) / 2.0).max(margin);
    ([cx - half_w, cy - half_h], [cx + half_w, cy + half_h])
}

pub(crate) fn sample_goal(
    rng: &mut SmallRng,
    bbox_min: &[f64; 2],
    bbox_max: &[f64; 2],
    goal: usize,
    graph: &RoadGraph,
    goal_bias: f64,
) -> [f64; 2] {
    if rng.random::<f64>() < goal_bias {
        graph.nodes[goal].world_pos
    } else {
        [
            rng.random_range(bbox_min[0]..bbox_max[0]),
            rng.random_range(bbox_min[1]..bbox_max[1]),
        ]
    }
}
