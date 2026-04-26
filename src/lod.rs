use crate::graph::{DecorationKind, RoadClass, RoadGraph};
use crate::view_index::RStarViewIndex;

// LOD tier thresholds (tunable)
const L1_ZOOM_THRESHOLD: f64 = 0.5;
const L2_ZOOM_THRESHOLD: f64 = 0.05;

// Simplification tolerances per tier (world-space meters)
const L0_EPS: f64 = 0.0;
const L1_EDGE_EPS: f64 = 2.0;
const L2_EDGE_EPS: f64 = 10.0;
const L1_DECOR_EPS: f64 = 4.0;

/// Douglas–Peucker polyline simplification in world-space.
/// Always preserves first and last points.
pub fn simplify_dp(points: &[[f64; 2]], eps: f64) -> Vec<[f64; 2]> {
    if points.len() <= 2 || eps <= 0.0 {
        return points.to_vec();
    }
    let mut use_point = vec![false; points.len()];
    use_point[0] = true;
    use_point[points.len() - 1] = true;
    dp_recursive(points, 0, points.len() - 1, eps, &mut use_point);
    points
        .iter()
        .zip(use_point.iter())
        .filter(|(_, &used)| used)
        .map(|(&p, _)| p)
        .collect()
}

fn dp_recursive(points: &[[f64; 2]], start: usize, end: usize, eps: f64, use_point: &mut [bool]) {
    if end <= start + 1 {
        return;
    }
    let mut max_dist = 0.0f64;
    let mut max_idx = start;
    let (sx, sy) = (points[start][0], points[start][1]);
    let (ex, ey) = (points[end][0], points[end][1]);
    let dx = ex - sx;
    let dy = ey - sy;
    let len_sq = dx * dx + dy * dy;

    for (i, pt) in points
        .iter()
        .enumerate()
        .skip(start + 1)
        .take(end - start - 1)
    {
        let dist = if len_sq < 1e-12 {
            hypot(pt[0] - sx, pt[1] - sy)
        } else {
            let t = ((pt[0] - sx) * dx + (pt[1] - sy) * dy) / len_sq;
            let t = t.clamp(0.0, 1.0);
            let px = sx + t * dx;
            let py = sy + t * dy;
            hypot(pt[0] - px, pt[1] - py)
        };
        if dist > max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }

    if max_dist > eps {
        use_point[max_idx] = true;
        dp_recursive(points, start, max_idx, eps, use_point);
        dp_recursive(points, max_idx, end, eps, use_point);
    }
}

fn hypot(a: f64, b: f64) -> f64 {
    (a * a + b * b).sqrt()
}

fn compute_aabb(polyline: &[[f64; 2]]) -> [f64; 4] {
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
    [min_x, min_y, max_x, max_y]
}

#[derive(Clone)]
pub struct EdgeLod {
    pub polyline_world: Vec<[f64; 2]>,
    pub road_class: RoadClass,
    pub aabb: [f64; 4],
    #[allow(dead_code)]
    pub original_edge_idx: usize,
}

#[derive(Clone)]
pub struct DecorLod {
    pub polyline_world: Vec<[f64; 2]>,
    pub kind: DecorationKind,
    pub closed: bool,
    pub aabb: [f64; 4],
}

pub struct LodLevel {
    pub edges: Vec<EdgeLod>,
    pub decorations: Vec<DecorLod>,
    pub edge_index: RStarViewIndex,
    pub decor_index: RStarViewIndex,
}

pub struct LodPyramid {
    pub l0: LodLevel,
    pub l1: LodLevel,
    pub l2: LodLevel,
}

fn should_include_edge(road_class: &RoadClass, tier: u8) -> bool {
    match tier {
        0 => true,
        1 => matches!(
            road_class,
            RoadClass::Motorway
                | RoadClass::Primary
                | RoadClass::Secondary
                | RoadClass::Tertiary
                | RoadClass::Residential
        ),
        2 => matches!(
            road_class,
            RoadClass::Motorway | RoadClass::Primary | RoadClass::Secondary | RoadClass::Tertiary
        ),
        _ => true,
    }
}

fn should_include_decor(kind: &DecorationKind, tier: u8) -> bool {
    match tier {
        0 => true,
        1 => *kind != DecorationKind::Building,
        2 => *kind == DecorationKind::Water,
        _ => true,
    }
}

fn build_tier(graph: &RoadGraph, tier: u8) -> LodLevel {
    let eps = match tier {
        0 => L0_EPS,
        1 => L1_EDGE_EPS,
        2 => L2_EDGE_EPS,
        _ => 0.0,
    };
    let decor_eps = if tier == 0 { 0.0 } else { L1_DECOR_EPS };

    let mut edges = Vec::with_capacity(graph.edges.len());
    let mut edge_aabbs = Vec::with_capacity(graph.edges.len());
    let mut edge_items_rstar = Vec::with_capacity(graph.edges.len());

    for (i, edge) in graph.edges.iter().enumerate() {
        if !should_include_edge(&edge.road_class, tier) {
            continue;
        }
        let simplified = simplify_dp(&edge.polyline_world, eps);
        let aabb = compute_aabb(&simplified);
        edge_aabbs.push(aabb);
        edges.push(EdgeLod {
            original_edge_idx: i,
            polyline_world: simplified,
            road_class: edge.road_class,
            aabb,
        });
        edge_items_rstar.push((edges.len() - 1, aabb));
    }

    let decor_eps_actual = if tier == 0 { 0.0 } else { decor_eps };
    let mut decorations = Vec::with_capacity(graph.decorations.shapes.len());
    let mut decor_aabbs = Vec::with_capacity(graph.decorations.shapes.len());

    for shape in &graph.decorations.shapes {
        if !should_include_decor(&shape.kind, tier) {
            continue;
        }
        let simplified = if shape.closed {
            simplify_dp_preserve_closed(&shape.polyline_world, decor_eps_actual)
        } else {
            simplify_dp(&shape.polyline_world, decor_eps_actual)
        };
        let aabb = compute_aabb(&simplified);
        decor_aabbs.push(aabb);
        decorations.push(DecorLod {
            polyline_world: simplified,
            kind: shape.kind,
            closed: shape.closed,
            aabb,
        });
    }

    let edge_index = RStarViewIndex::build_from_raw_aabbs(
        edge_aabbs,
        edge_items_rstar.iter().map(|(id, _)| *id).collect(),
    );
    let decor_index = RStarViewIndex::build_decor_only(decor_aabbs);

    LodLevel {
        edges,
        decorations,
        edge_index,
        decor_index,
    }
}

fn simplify_dp_preserve_closed(points: &[[f64; 2]], eps: f64) -> Vec<[f64; 2]> {
    if points.len() <= 2 {
        return points.to_vec();
    }
    let mut result = simplify_dp(&points[..points.len() - 1], eps);
    if result.first() != result.last() {
        result.push(result[0]);
    }
    result
}

impl LodPyramid {
    pub fn build(graph: &RoadGraph) -> Self {
        let l0 = build_tier(graph, 0);
        let l1 = build_tier(graph, 1);
        let l2 = build_tier(graph, 2);
        Self { l0, l1, l2 }
    }

    pub fn pick(&self, zoom: f64) -> &LodLevel {
        if zoom >= L1_ZOOM_THRESHOLD {
            &self.l0
        } else if zoom >= L2_ZOOM_THRESHOLD {
            &self.l1
        } else {
            &self.l2
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simplify_empty() {
        let empty: Vec<[f64; 2]> = vec![];
        assert!(simplify_dp(&empty, 1.0).is_empty());
    }

    #[test]
    fn simplify_single_point() {
        let pts = vec![[0.0, 0.0]];
        let result = simplify_dp(&pts, 1.0);
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn simplify_two_points_preserves_endpoints() {
        let pts = vec![[0.0, 0.0], [10.0, 10.0]];
        let result = simplify_dp(&pts, 1.0);
        assert_eq!(result.len(), 2);
        assert_eq!(result[0], [0.0, 0.0]);
        assert_eq!(result[1], [10.0, 10.0]);
    }

    #[test]
    fn simplify_straight_line_removes_middle() {
        let pts = vec![[0.0, 0.0], [5.0, 5.0], [10.0, 10.0]];
        let result = simplify_dp(&pts, 1.0);
        assert_eq!(result.len(), 2);
        assert_eq!(result[0], [0.0, 0.0]);
        assert_eq!(result[1], [10.0, 10.0]);
    }

    #[test]
    fn simplify_zero_eps_returns_original() {
        let pts = vec![[0.0, 0.0], [5.0, 3.0], [10.0, 10.0]];
        let result = simplify_dp(&pts, 0.0);
        assert_eq!(result.len(), pts.len());
    }

    #[test]
    fn simplify_preserves_endpoints_with_curve() {
        let pts = vec![
            [0.0, 0.0],
            [2.0, 10.0],
            [5.0, 5.0],
            [8.0, -10.0],
            [10.0, 0.0],
        ];
        let result = simplify_dp(&pts, 3.0);
        assert_eq!(result.first().unwrap(), &pts[0]);
        assert_eq!(result.last().unwrap(), pts.last().unwrap());
    }
}
