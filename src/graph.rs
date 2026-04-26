/// Classification of routable road edges.
/// `Path` and `Other` variants are kept for exhaustive matching in the renderer,
/// but are no longer produced by the OSM loader (car-only profile).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoadClass {
    Motorway,
    Primary,
    Secondary,
    Tertiary,
    Residential,
    Service,
    /// Unused by the car-only loader; kept for renderer exhaustiveness.
    Path,
    /// Unused by the car-only loader; kept for renderer exhaustiveness.
    Other,
}

impl RoadClass {
    pub fn from_tag(tag: &str) -> Self {
        match tag {
            "motorway" | "motorway_link" | "trunk" | "trunk_link" => RoadClass::Motorway,
            "primary" | "primary_link" => RoadClass::Primary,
            "secondary" | "secondary_link" => RoadClass::Secondary,
            "tertiary" | "tertiary_link" => RoadClass::Tertiary,
            "residential" | "unclassified" | "living_street" | "road" => RoadClass::Residential,
            "service" => RoadClass::Service,
            "path" | "footway" | "cycleway" | "track" => RoadClass::Path,
            _ => RoadClass::Other,
        }
    }

    #[allow(dead_code)]
    pub fn stroke_width(&self) -> f32 {
        match self {
            RoadClass::Motorway => 3.0,
            RoadClass::Primary => 2.5,
            RoadClass::Secondary => 2.0,
            RoadClass::Tertiary => 1.5,
            RoadClass::Residential => 1.2,
            RoadClass::Service => 1.0,
            RoadClass::Path => 0.8,
            RoadClass::Other => 0.8,
        }
    }
}

#[derive(Debug, Clone)]
pub struct GraphNode {
    pub id: usize,
    pub world_pos: [f64; 2],
    #[allow(dead_code)]
    pub lat_lon: [f64; 2],
}

#[derive(Debug, Clone)]
pub struct GraphEdge {
    pub from: usize,
    pub to: usize,
    pub weight_meters: f64,
    pub polyline_world: Vec<[f64; 2]>,
    pub road_class: RoadClass,
    #[allow(dead_code)]
    pub one_way: bool,
}

/// Decoration kinds for visual-context-only geometry (never routable).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecorationKind {
    Building,
    Landuse,
    PedestrianArea,
    ServiceArea,
    Water,
}

/// A single decoration shape — either a closed polygon or an open polyline.
#[derive(Debug, Clone)]
pub struct DecorationShape {
    pub kind: DecorationKind,
    pub polyline_world: Vec<[f64; 2]>,
    pub closed: bool,
}

/// Collection of all non-routable decoration geometry.
#[derive(Debug, Default)]
pub struct DecorationLayer {
    pub shapes: Vec<DecorationShape>,
}

#[derive(Debug, Default)]
pub struct RoadGraph {
    pub nodes: Vec<GraphNode>,
    pub edges: Vec<GraphEdge>,
    /// adjacency[node_id] = list of edge indices where edge.from == node_id
    pub adjacency: Vec<Vec<usize>>,
    /// Non-routable visual context (buildings, landuse, pedestrian plazas).
    pub decorations: DecorationLayer,
}

impl RoadGraph {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Returns (min_world, max_world) bounding box in Web Mercator meters.
    pub fn bounding_box(&self) -> Option<([f64; 2], [f64; 2])> {
        if self.nodes.is_empty() {
            return None;
        }
        let mut min = self.nodes[0].world_pos;
        let mut max = self.nodes[0].world_pos;
        for n in &self.nodes {
            min[0] = min[0].min(n.world_pos[0]);
            min[1] = min[1].min(n.world_pos[1]);
            max[0] = max[0].max(n.world_pos[0]);
            max[1] = max[1].max(n.world_pos[1]);
        }
        Some((min, max))
    }
}
