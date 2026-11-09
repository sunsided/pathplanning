/// Classification of routable road edges.
/// `Path` and `Other` variants are kept for exhaustive matching in the renderer,
/// but are no longer produced by the OSM loader (car-only profile).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
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

    pub fn default_speed_kmh(&self) -> f64 {
        match self {
            RoadClass::Motorway => 90.0,
            RoadClass::Primary => 70.0,
            RoadClass::Secondary => 55.0,
            RoadClass::Tertiary => 45.0,
            RoadClass::Residential => 30.0,
            RoadClass::Service => 15.0,
            RoadClass::Path => 10.0,
            RoadClass::Other => 30.0,
        }
    }

    pub fn intersection_stop_s(&self) -> f64 {
        match self {
            RoadClass::Motorway => 0.0,
            RoadClass::Primary => 3.0,
            RoadClass::Secondary => 5.0,
            RoadClass::Tertiary => 7.0,
            RoadClass::Residential => 10.0,
            RoadClass::Service => 12.0,
            RoadClass::Path => 12.0,
            RoadClass::Other => 10.0,
        }
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct GraphNode {
    #[serde(skip)]
    pub id: usize,
    pub world_pos: [f64; 2],
    #[serde(skip)]
    pub lat_lon: [f64; 2],
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct GraphEdge {
    pub from: usize,
    pub to: usize,
    pub weight_meters: f64,
    pub travel_time_s: f64,
    #[serde(with = "polyline")]
    pub polyline_world: Vec<[f64; 2]>,
    pub road_class: RoadClass,
    #[allow(dead_code)]
    pub one_way: bool,
}

/// Decoration kinds for visual-context-only geometry (never routable).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum DecorationKind {
    Building,
    Landuse,
    PedestrianArea,
    ServiceArea,
    Water,
}

/// A single decoration shape — either a closed polygon or an open polyline.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DecorationShape {
    pub kind: DecorationKind,
    #[serde(with = "polyline")]
    pub polyline_world: Vec<[f64; 2]>,
    pub closed: bool,
}

/// Collection of all non-routable decoration geometry.
#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct DecorationLayer {
    pub shapes: Vec<DecorationShape>,
}

#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct RoadGraph {
    pub nodes: Vec<GraphNode>,
    pub edges: Vec<GraphEdge>,
    /// adjacency[node_id] = list of edge indices where edge.from == node_id
    #[serde(skip)]
    pub adjacency: Vec<Vec<usize>>,
    /// Precomputed distinct outgoing neighbor count per node.
    #[serde(skip)]
    pub node_out_degree: Vec<u8>,
    /// Non-routable visual context (buildings, landuse, pedestrian plazas).
    pub decorations: DecorationLayer,
}

/// Serializes world-space polylines as an absolute first point plus `f32`
/// offsets, which roughly halves the asset size compared to `f64` pairs.
mod polyline {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    #[derive(Serialize, Deserialize)]
    struct EncodedPolyline {
        len: u32,
        first: [f64; 2],
        deltas: Vec<[f32; 2]>,
    }

    pub fn serialize<S: Serializer>(points: &[[f64; 2]], serializer: S) -> Result<S::Ok, S::Error> {
        let (first, rest) = points.split_first().unwrap_or((&[0.0, 0.0], &[]));
        let encoded = EncodedPolyline {
            len: points.len() as u32,
            first: *first,
            deltas: rest
                .iter()
                .map(|p| [(p[0] - first[0]) as f32, (p[1] - first[1]) as f32])
                .collect(),
        };
        encoded.serialize(serializer)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(
        deserializer: D,
    ) -> Result<Vec<[f64; 2]>, D::Error> {
        let encoded = EncodedPolyline::deserialize(deserializer)?;
        if encoded.len == 0 {
            return Ok(Vec::new());
        }
        let mut points = Vec::with_capacity(encoded.len as usize);
        points.push(encoded.first);
        points.extend(encoded.deltas.iter().map(|d| {
            [
                encoded.first[0] + d[0] as f64,
                encoded.first[1] + d[1] as f64,
            ]
        }));
        Ok(points)
    }
}

impl RoadGraph {
    pub fn new() -> Self {
        Self::default()
    }

    /// Keeps only edges and decorations that intersect the given Web Mercator
    /// bounding box, then prunes unreferenced nodes.
    pub fn retain_world_bbox(&mut self, min: [f64; 2], max: [f64; 2]) {
        let contains =
            |p: [f64; 2]| p[0] >= min[0] && p[0] <= max[0] && p[1] >= min[1] && p[1] <= max[1];

        self.edges.retain(|edge| {
            contains(self.nodes[edge.from].world_pos)
                || contains(self.nodes[edge.to].world_pos)
                || edge.polyline_world.iter().any(|&p| contains(p))
        });
        self.decorations
            .shapes
            .retain(|shape| shape.polyline_world.iter().any(|&p| contains(p)));
        self.prune_unused_nodes();
    }

    /// Drops nodes that are not endpoints of any edge and remaps edge indices.
    ///
    /// The OSM loader adds every way vertex (including buildings and other
    /// decoration geometry) to `nodes`, even though only routable edge endpoints
    /// are ever needed at runtime. The web asset is pruned before serialization
    /// to keep the download small.
    pub fn prune_unused_nodes(&mut self) {
        let mut used = vec![false; self.nodes.len()];
        for edge in &self.edges {
            used[edge.from] = true;
            used[edge.to] = true;
        }

        let old_nodes = std::mem::take(&mut self.nodes);
        let mut remap = vec![usize::MAX; old_nodes.len()];
        let mut nodes = Vec::with_capacity(old_nodes.len());
        for (old_id, node) in old_nodes.into_iter().enumerate() {
            if used[old_id] {
                remap[old_id] = nodes.len();
                nodes.push(node);
            }
        }
        for (id, node) in nodes.iter_mut().enumerate() {
            node.id = id;
        }

        for edge in &mut self.edges {
            edge.from = remap[edge.from];
            edge.to = remap[edge.to];
        }

        self.nodes = nodes;
        self.rebuild_derived();
    }

    /// Rebuilds the node ids, geographic coordinates, adjacency list and
    /// out-degrees after deserialization or node pruning.
    pub fn rebuild_derived(&mut self) {
        for (id, node) in self.nodes.iter_mut().enumerate() {
            node.id = id;
            node.lat_lon = crate::projection::world_to_latlon(node.world_pos[0], node.world_pos[1]);
        }

        self.adjacency = vec![Vec::new(); self.nodes.len()];
        for (edge_id, edge) in self.edges.iter().enumerate() {
            self.adjacency[edge.from].push(edge_id);
        }
        self.node_out_degree = (0..self.nodes.len())
            .map(|i| {
                let neighbors: std::collections::HashSet<usize> = self.adjacency[i]
                    .iter()
                    .map(|&e| self.edges[e].to)
                    .collect();
                neighbors.len().min(u8::MAX as usize) as u8
            })
            .collect();
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
