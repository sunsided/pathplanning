use rstar::{RTree, RTreeObject, AABB, PointDistance};
use crate::graph::RoadGraph;

#[derive(Clone)]
struct NodePoint {
    pos: [f64; 2],
    node_id: usize,
}

impl RTreeObject for NodePoint {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_point(self.pos)
    }
}

impl PointDistance for NodePoint {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dx = self.pos[0] - point[0];
        let dy = self.pos[1] - point[1];
        dx * dx + dy * dy
    }
}

pub struct SpatialIndex {
    tree: RTree<NodePoint>,
}

impl SpatialIndex {
    pub fn build(graph: &RoadGraph) -> Self {
        let points: Vec<NodePoint> = graph
            .nodes
            .iter()
            .map(|n| NodePoint {
                pos: n.world_pos,
                node_id: n.id,
            })
            .collect();
        Self {
            tree: RTree::bulk_load(points),
        }
    }

    /// Returns the node id of the nearest graph node to the given world position.
    pub fn nearest_node(&self, world_pos: [f64; 2]) -> Option<usize> {
        self.tree
            .nearest_neighbor(&world_pos)
            .map(|p| p.node_id)
    }
}
