#[allow(dead_code)]
use space_partitioning::quadtree::{QuadRect, QuadTree, QuadTreeElement, AABB};
#[allow(dead_code)]
type ElementId = i32;
use rstar::{RTree, RTreeObject, AABB as RStarAABB};

use crate::graph::RoadGraph;

#[allow(dead_code)]
pub(crate) const MAX_DEPTH: u8 = 8;
#[allow(dead_code)]
pub(crate) const SPLIT_THRESHOLD: u32 = 8;
#[allow(dead_code)]
pub(crate) const MIN_NODE_SIZE: u32 = 4;
#[allow(dead_code)]
pub(crate) const MARGIN_FRACTION: f64 = 0.01;

pub trait ViewportIndex {
    fn edges_in(&self, min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Vec<usize>;
    #[allow(dead_code)]
    fn decorations_in(&self, min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Vec<usize>;
}

#[allow(dead_code)]
pub struct QuadTreeViewIndex {
    edge_tree: QuadTree<ElementId>,
    decor_tree: QuadTree<ElementId>,
    edge_aabbs: Vec<[f64; 4]>,
    decor_aabbs: Vec<[f64; 4]>,
    origin: [f64; 2],
    scale: f64,
}

struct RStarEdge {
    id: usize,
    aabb_idx: usize,
    envelope: RStarAABB<[f64; 2]>,
}

impl RTreeObject for RStarEdge {
    type Envelope = RStarAABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

struct RStarDecor {
    id: usize,
    envelope: RStarAABB<[f64; 2]>,
}

impl RTreeObject for RStarDecor {
    type Envelope = RStarAABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

pub struct RStarViewIndex {
    edge_tree: RTree<RStarEdge>,
    decor_tree: RTree<RStarDecor>,
    edge_aabbs: Vec<[f64; 4]>,
    decor_aabbs: Vec<[f64; 4]>,
}

fn compute_polyline_aabb(polyline: &[[f64; 2]]) -> (f64, f64, f64, f64) {
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for p in polyline {
        min_x = min_x.min(p[0]);
        min_y = min_y.min(p[1]);
        max_x = max_x.max(p[0]);
        max_y = max_y.max(p[1]);
    }
    (min_x, min_y, max_x, max_y)
}

#[allow(dead_code)]
fn world_to_quant(wx: f64, origin: f64, scale: f64) -> i32 {
    ((wx - origin) * scale).round() as i32
}

fn intersects(a: &[f64; 4], min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> bool {
    a[0] <= max_x && a[2] >= min_x && a[1] <= max_y && a[3] >= min_y
}

#[allow(dead_code)]
#[allow(clippy::type_complexity)]
fn build_quant_index(
    graph: &RoadGraph,
) -> (
    QuadTree<ElementId>,
    Vec<[f64; 4]>,
    QuadTree<ElementId>,
    Vec<[f64; 4]>,
    [f64; 2],
    f64,
) {
    let (bmin, bmax) = graph.bounding_box().unwrap_or(([0.0, 0.0], [1.0, 1.0]));
    let span_x = bmax[0] - bmin[0];
    let span_y = bmax[1] - bmin[1];
    let max_span = span_x.max(span_y).max(1.0);
    let scale = (i32::MAX as f64 * 0.4 / max_span).min(100.0);
    let origin = [bmin[0], bmin[1]];

    let q_max_x = world_to_quant(bmax[0], origin[0], scale);
    let q_max_y = world_to_quant(bmax[1], origin[1], scale);

    let margin = ((max_span * MARGIN_FRACTION) * scale).ceil() as i32;
    let root = QuadRect::new(-margin, -margin, q_max_x + margin, q_max_y + margin);

    let mut edge_tree = QuadTree::new(root, MAX_DEPTH, SPLIT_THRESHOLD, MIN_NODE_SIZE);
    let mut edge_aabbs = Vec::with_capacity(graph.edges.len());

    for (i, edge) in graph.edges.iter().enumerate() {
        let (min_x, min_y, max_x, max_y) = compute_polyline_aabb(&edge.polyline_world);
        edge_aabbs.push([min_x, min_y, max_x, max_y]);
        let qlx = world_to_quant(min_x, origin[0], scale);
        let qly = world_to_quant(min_y, origin[1], scale);
        let qrx = world_to_quant(max_x, origin[0], scale);
        let qry = world_to_quant(max_y, origin[1], scale);
        let aabb = AABB::new(qlx, qly, qrx, qry);
        let _ = edge_tree.insert(QuadTreeElement::new(i as i32, aabb));
    }

    let mut decor_tree = QuadTree::new(root, MAX_DEPTH, SPLIT_THRESHOLD, MIN_NODE_SIZE);
    let mut decor_aabbs = Vec::with_capacity(graph.decorations.shapes.len());

    for (i, shape) in graph.decorations.shapes.iter().enumerate() {
        let (min_x, min_y, max_x, max_y) = compute_polyline_aabb(&shape.polyline_world);
        decor_aabbs.push([min_x, min_y, max_x, max_y]);
        let qlx = world_to_quant(min_x, origin[0], scale);
        let qly = world_to_quant(min_y, origin[1], scale);
        let qrx = world_to_quant(max_x, origin[0], scale);
        let qry = world_to_quant(max_y, origin[1], scale);
        let aabb = AABB::new(qlx, qly, qrx, qry);
        let _ = decor_tree.insert(QuadTreeElement::new(i as i32, aabb));
    }

    (
        edge_tree,
        edge_aabbs,
        decor_tree,
        decor_aabbs,
        origin,
        scale,
    )
}

#[allow(dead_code)]
impl QuadTreeViewIndex {
    pub fn build(graph: &RoadGraph) -> Self {
        let (edge_tree, edge_aabbs, decor_tree, decor_aabbs, origin, scale) =
            build_quant_index(graph);
        Self {
            edge_tree,
            decor_tree,
            edge_aabbs,
            decor_aabbs,
            origin,
            scale,
        }
    }
}

impl ViewportIndex for QuadTreeViewIndex {
    fn edges_in(&self, min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Vec<usize> {
        let qlx = world_to_quant(min_x, self.origin[0], self.scale);
        let qly = world_to_quant(min_y, self.origin[1], self.scale);
        let qrx = world_to_quant(max_x, self.origin[0], self.scale);
        let qry = world_to_quant(max_y, self.origin[1], self.scale);
        let query_aabb = AABB::new(qlx, qly, qrx, qry);
        self.edge_tree
            .intersect_aabb(&query_aabb)
            .into_iter()
            .filter_map(|id| {
                if id < 0 {
                    return None;
                }
                let idx = id as usize;
                if idx < self.edge_aabbs.len()
                    && intersects(&self.edge_aabbs[idx], min_x, min_y, max_x, max_y)
                {
                    Some(idx)
                } else {
                    None
                }
            })
            .collect()
    }

    fn decorations_in(&self, min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Vec<usize> {
        let qlx = world_to_quant(min_x, self.origin[0], self.scale);
        let qly = world_to_quant(min_y, self.origin[1], self.scale);
        let qrx = world_to_quant(max_x, self.origin[0], self.scale);
        let qry = world_to_quant(max_y, self.origin[1], self.scale);
        let query_aabb = AABB::new(qlx, qly, qrx, qry);
        self.decor_tree
            .intersect_aabb(&query_aabb)
            .into_iter()
            .filter_map(|id| {
                if id < 0 {
                    return None;
                }
                let idx = id as usize;
                if idx < self.decor_aabbs.len()
                    && intersects(&self.decor_aabbs[idx], min_x, min_y, max_x, max_y)
                {
                    Some(idx)
                } else {
                    None
                }
            })
            .collect()
    }
}

impl RStarViewIndex {
    pub fn build(graph: &RoadGraph) -> Self {
        let mut edge_items = Vec::with_capacity(graph.edges.len());
        let mut edge_aabbs = Vec::with_capacity(graph.edges.len());
        for (i, edge) in graph.edges.iter().enumerate() {
            let (min_x, min_y, max_x, max_y) = compute_polyline_aabb(&edge.polyline_world);
            edge_aabbs.push([min_x, min_y, max_x, max_y]);
            edge_items.push(RStarEdge {
                id: i,
                aabb_idx: i,
                envelope: RStarAABB::from_corners([min_x, min_y], [max_x, max_y]),
            });
        }

        let mut decor_items = Vec::with_capacity(graph.decorations.shapes.len());
        let mut decor_aabbs = Vec::with_capacity(graph.decorations.shapes.len());
        for (i, shape) in graph.decorations.shapes.iter().enumerate() {
            let (min_x, min_y, max_x, max_y) = compute_polyline_aabb(&shape.polyline_world);
            decor_aabbs.push([min_x, min_y, max_x, max_y]);
            decor_items.push(RStarDecor {
                id: i,
                envelope: RStarAABB::from_corners([min_x, min_y], [max_x, max_y]),
            });
        }

        Self {
            edge_tree: RTree::bulk_load(edge_items),
            decor_tree: RTree::bulk_load(decor_items),
            edge_aabbs,
            decor_aabbs,
        }
    }

    pub fn build_from_raw_aabbs(edge_aabbs: Vec<[f64; 4]>, edge_ids: Vec<usize>) -> Self {
        let mut edge_items = Vec::with_capacity(edge_aabbs.len());
        for (i, aabb) in edge_aabbs.iter().enumerate() {
            let id = if i < edge_ids.len() { edge_ids[i] } else { i };
            edge_items.push(RStarEdge {
                id,
                aabb_idx: i,
                envelope: RStarAABB::from_corners([aabb[0], aabb[1]], [aabb[2], aabb[3]]),
            });
        }
        Self {
            edge_tree: RTree::bulk_load(edge_items),
            decor_tree: RTree::bulk_load(Vec::new()),
            edge_aabbs,
            decor_aabbs: Vec::new(),
        }
    }

    pub fn build_decor_only(decor_aabbs: Vec<[f64; 4]>) -> Self {
        let mut decor_items = Vec::with_capacity(decor_aabbs.len());
        for (i, aabb) in decor_aabbs.iter().enumerate() {
            decor_items.push(RStarDecor {
                id: i,
                envelope: RStarAABB::from_corners([aabb[0], aabb[1]], [aabb[2], aabb[3]]),
            });
        }
        Self {
            edge_tree: RTree::bulk_load(Vec::new()),
            decor_tree: RTree::bulk_load(decor_items),
            edge_aabbs: Vec::new(),
            decor_aabbs,
        }
    }

    #[allow(dead_code)]
    pub fn build_from_aabbs(edge_aabbs: Vec<[f64; 4]>, decor_aabbs: Vec<[f64; 4]>) -> Self {
        let mut edge_items = Vec::with_capacity(edge_aabbs.len());
        for (i, aabb) in edge_aabbs.iter().enumerate() {
            edge_items.push(RStarEdge {
                id: i,
                aabb_idx: i,
                envelope: RStarAABB::from_corners([aabb[0], aabb[1]], [aabb[2], aabb[3]]),
            });
        }
        let mut decor_items = Vec::with_capacity(decor_aabbs.len());
        for (i, aabb) in decor_aabbs.iter().enumerate() {
            decor_items.push(RStarDecor {
                id: i,
                envelope: RStarAABB::from_corners([aabb[0], aabb[1]], [aabb[2], aabb[3]]),
            });
        }
        Self {
            edge_tree: RTree::bulk_load(edge_items),
            decor_tree: RTree::bulk_load(decor_items),
            edge_aabbs,
            decor_aabbs,
        }
    }
}

impl ViewportIndex for RStarViewIndex {
    fn edges_in(&self, min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Vec<usize> {
        let mut buf = Vec::new();
        self.edges_in_into(min_x, min_y, max_x, max_y, &mut buf);
        buf
    }

    fn decorations_in(&self, min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Vec<usize> {
        let mut buf = Vec::new();
        self.decorations_in_into(min_x, min_y, max_x, max_y, &mut buf);
        buf
    }
}

impl RStarViewIndex {
    pub fn edges_in_into(
        &self,
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
        out: &mut Vec<usize>,
    ) {
        out.clear();
        let envelope = RStarAABB::from_corners([min_x, min_y], [max_x, max_y]);
        self.edge_tree
            .locate_in_envelope_intersecting(&envelope)
            .for_each(|item| {
                if intersects(&self.edge_aabbs[item.aabb_idx], min_x, min_y, max_x, max_y) {
                    out.push(item.id);
                }
            });
    }

    pub fn decorations_in_into(
        &self,
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
        out: &mut Vec<usize>,
    ) {
        out.clear();
        let envelope = RStarAABB::from_corners([min_x, min_y], [max_x, max_y]);
        self.decor_tree
            .locate_in_envelope_intersecting(&envelope)
            .for_each(|item| {
                if intersects(&self.decor_aabbs[item.id], min_x, min_y, max_x, max_y) {
                    out.push(item.id);
                }
            });
    }
}
