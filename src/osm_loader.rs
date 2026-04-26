use crate::graph::{
    DecorationKind, DecorationLayer, DecorationShape, GraphEdge, GraphNode, RoadClass, RoadGraph,
};
use crate::projection::latlon_to_world;
use fast_osmpbf::prelude::*;
use fast_osmpbf::{ElementBlock, ElementFilter, OsmReader};
use std::collections::HashMap;
use std::collections::HashSet;
use std::time::Instant;

/// Routable highway tags (car/vehicle profile).
const ROUTABLE_HIGHWAYS: &[&str] = &[
    "motorway",
    "motorway_link",
    "trunk",
    "trunk_link",
    "primary",
    "primary_link",
    "secondary",
    "secondary_link",
    "tertiary",
    "tertiary_link",
    "residential",
    "unclassified",
    "living_street",
    "road",
    "service",
];

/// Highway tags to drop entirely (not routable, not decoration).
#[allow(dead_code)]
const DROPPED_HIGHWAYS: &[&str] = &[
    "footway",
    "path",
    "cycleway",
    "bridleway",
    "steps",
    "corridor",
    "track",
    "pedestrian",
    "bus_guideway",
    "raceway",
    "proposed",
    "construction",
    "planned",
];

/// Landuse values to load as decoration polygons.
const LANDUSE_KEEP: &[&str] = &[
    "residential",
    "commercial",
    "industrial",
    "retail",
    "forest",
    "grass",
    "park",
    "meadow",
    "recreation_ground",
    "cemetery",
    "railway",
];

/// Waterway values to load as decoration polylines.
/// `riverbank` here is the (usually closed) polygon form — treated as water area.
const WATERWAY_KEEP: &[&str] = &["river", "canal", "stream", "drain", "ditch", "riverbank"];

/// Service values that disqualify a way from being routable.
const SERVICE_REJECT: &[&str] = &["parking_aisle", "driveway", "emergency_access"];

/// Access values that disqualify a way from being routable.
const ACCESS_REJECT: &[&str] = &["no", "private", "customers", "delivery"];

struct WayRecord {
    #[allow(dead_code)]
    id: i64,
    tags: Vec<(String, String)>,
    node_refs: Vec<i64>,
}

/// A multipolygon relation whose outer rings should be rendered as water.
struct WaterRelation {
    /// Member way IDs with role == "outer" (or empty role, which historically
    /// defaults to outer). Inner rings (islands) are intentionally skipped —
    /// rendering them correctly would require hole punching in the fill.
    outer_way_ids: Vec<i64>,
}

fn is_water_relation_tags(tags: &[(String, String)]) -> bool {
    let is_multipolygon = find_tag(tags, "type") == Some("multipolygon");
    if !is_multipolygon {
        return false;
    }
    if find_tag(tags, "natural").is_some_and(|v| v == "water" || v == "wetland") {
        return true;
    }
    if find_tag(tags, "landuse") == Some("reservoir") {
        return true;
    }
    if find_tag(tags, "waterway") == Some("riverbank") {
        return true;
    }
    false
}

fn find_tag<'a>(tags: &'a [(String, String)], key: &str) -> Option<&'a str> {
    tags.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
}

fn haversine_dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    const R: f64 = 6_371_000.0;
    let dlat = (b[0] - a[0]).to_radians();
    let dlon = (b[1] - a[1]).to_radians();
    let lat1 = a[0].to_radians();
    let lat2 = b[0].to_radians();
    let a = (dlat / 2.0).sin().powi(2) + (dlon / 2.0).sin().powi(2) * lat1.cos() * lat2.cos();
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    R * c
}

fn is_routable_highway(hw: &str) -> bool {
    ROUTABLE_HIGHWAYS.contains(&hw)
}

fn is_rejected_access(tags: &[(String, String)]) -> bool {
    find_tag(tags, "access")
        .map(|v| ACCESS_REJECT.contains(&v))
        .unwrap_or(false)
}

fn is_motor_barred(tags: &[(String, String)]) -> bool {
    find_tag(tags, "motor_vehicle") == Some("no") || find_tag(tags, "vehicle") == Some("no")
}

fn is_rejected_service(tags: &[(String, String)]) -> bool {
    find_tag(tags, "service")
        .map(|v| SERVICE_REJECT.contains(&v))
        .unwrap_or(false)
}

fn way_passes_filter<'a>(tags: impl Iterator<Item = (&'a str, &'a str)>) -> bool {
    let mut highway = None;
    let mut building_value: Option<&str> = None;
    let mut landuse = None;
    let mut natural_val = None;
    let mut waterway = None;
    let mut area = false;
    for (k, v) in tags {
        match k {
            "highway" => highway = Some(v),
            "building" => building_value = Some(v),
            "landuse" => landuse = Some(v),
            "natural" => natural_val = Some(v),
            "waterway" => waterway = Some(v),
            "area" if v == "yes" => area = true,
            _ => {}
        }
    }
    if let Some(hw) = highway {
        if is_routable_highway(hw) {
            return true;
        }
        if hw == "pedestrian" && area {
            return true;
        }
        return false;
    }
    if building_value.is_some_and(|v| !matches!(v, "no" | "false" | "0")) {
        return true;
    }
    if let Some(lu) = landuse {
        if LANDUSE_KEEP.contains(&lu) {
            return true;
        }
        if lu == "reservoir" {
            return true;
        }
    }
    if let Some(n) = natural_val {
        if n == "water" || n == "wetland" {
            return true;
        }
    }
    if let Some(ww) = waterway {
        if WATERWAY_KEEP.contains(&ww) {
            return true;
        }
    }
    false
}

pub fn load_graph(path: &str) -> Result<RoadGraph, Box<dyn std::error::Error>> {
    log::info!("Loading OSM PBF from: {}", path);
    let total_start = Instant::now();

    // Pass 1a: collect water multipolygon relations.
    let t0 = Instant::now();
    let reader = OsmReader::from_path(path)?;
    reader.apply_element_filter(ElementFilter {
        nodes: false,
        relations: true,
        ways: false,
    });
    let water_relations: Vec<WaterRelation> = reader
        .par_blocks()
        .map(|block| {
            let mut out: Vec<WaterRelation> = Vec::new();
            if let ElementBlock::RelationBlock(block) = block {
                for rel in block.iter() {
                    let tags: Vec<(String, String)> = rel
                        .tags()
                        .map(|(k, v)| (k.to_string(), v.to_string()))
                        .collect();
                    if !is_water_relation_tags(&tags) {
                        continue;
                    }
                    let mut outer_way_ids = Vec::new();
                    for m in rel.members() {
                        if m.member_type() != fast_osmpbf::MemberType::WAY {
                            continue;
                        }
                        let role = m.role();
                        // Empty role on multipolygon historically means outer.
                        if role == "outer" || role.is_empty() {
                            outer_way_ids.push(m.id());
                        }
                    }
                    if !outer_way_ids.is_empty() {
                        out.push(WaterRelation { outer_way_ids });
                    }
                }
            }
            out
        })
        .reduce(Vec::new, |mut a, b| {
            a.extend(b);
            a
        });

    let mut relation_member_way_ids: HashSet<i64> = HashSet::new();
    for rel in &water_relations {
        for &wid in &rel.outer_way_ids {
            relation_member_way_ids.insert(wid);
        }
    }

    log::info!(
        "Pass 1a: collected {} water relations ({} member ways) in {:.2?}",
        water_relations.len(),
        relation_member_way_ids.len(),
        t0.elapsed()
    );

    // Pass 1b: collect matching ways (existing filter) PLUS any way whose ID is
    // a member of a water relation (even if the way itself is untagged).
    let t0 = Instant::now();
    let reader = OsmReader::from_path(path)?;
    reader.apply_element_filter(ElementFilter {
        nodes: false,
        relations: false,
        ways: true,
    });

    let relation_member_way_ids_ref = &relation_member_way_ids;
    let (ways, all_node_refs): (Vec<WayRecord>, Vec<i64>) = reader
        .par_blocks()
        .map(|block| {
            let mut ways = Vec::new();
            let mut refs = Vec::new();
            if let ElementBlock::WayBlock(block) = block {
                for way in block.iter() {
                    let id = way.id();
                    let borrowed_tags: Vec<(&str, &str)> = way.tags().collect();
                    let passes_filter = way_passes_filter(borrowed_tags.iter().copied());
                    let needed_for_relation = relation_member_way_ids_ref.contains(&id);
                    if !passes_filter && !needed_for_relation {
                        continue;
                    }
                    let node_refs: Vec<i64> = way.node_ids().collect();
                    let tags: Vec<(String, String)> = borrowed_tags
                        .into_iter()
                        .map(|(k, v)| (k.to_string(), v.to_string()))
                        .collect();
                    refs.extend_from_slice(&node_refs);
                    ways.push(WayRecord {
                        id,
                        tags,
                        node_refs,
                    });
                }
            }
            (ways, refs)
        })
        .reduce(
            || (Vec::new(), Vec::new()),
            |mut a, b| {
                a.0.extend(b.0);
                a.1.extend(b.1);
                a
            },
        );

    log::info!(
        "Pass 1b: collected {} ways in {:.2?}",
        ways.len(),
        t0.elapsed()
    );

    // Dedupe node refs into a HashSet
    let referenced_nodes: HashSet<i64> = all_node_refs.into_iter().collect();

    // Pass 2: collect coords for referenced node IDs
    let t0 = Instant::now();
    let reader = OsmReader::from_path(path)?;
    reader.apply_element_filter(ElementFilter {
        nodes: true,
        relations: false,
        ways: false,
    });
    reader.apply_tag_filter(&[]).ok();

    let nodes_flat: Vec<(i64, f64, f64)> = reader
        .par_blocks()
        .map(|block| {
            let mut out = Vec::new();
            match block {
                ElementBlock::DenseNodeBlock(b) => {
                    for mut n in b.iter() {
                        let id = n.id();
                        if referenced_nodes.contains(&id) {
                            out.push((id, n.lat(), n.lon()));
                        }
                    }
                }
                ElementBlock::NodeBlock(b) => {
                    for mut n in b.iter() {
                        let id = n.id();
                        if referenced_nodes.contains(&id) {
                            out.push((id, n.lat(), n.lon()));
                        }
                    }
                }
                _ => {}
            }
            out
        })
        .reduce(Vec::new, |mut a, b| {
            a.extend(b);
            a
        });

    log::info!(
        "Pass 2: collected {} nodes in {:.2?}",
        nodes_flat.len(),
        t0.elapsed()
    );

    // Build coord map
    let coord_map: HashMap<i64, (f64, f64)> = nodes_flat
        .into_iter()
        .map(|(id, lat, lon)| (id, (lat, lon)))
        .collect();

    // Graph construction
    let t0 = Instant::now();
    let mut osm_node_to_graph: HashMap<i64, usize> = HashMap::new();
    let mut graph = RoadGraph::new();

    for way in &ways {
        for &osm_id in &way.node_refs {
            if osm_node_to_graph.contains_key(&osm_id) {
                continue;
            }
            if let Some(&(lat, lon)) = coord_map.get(&osm_id) {
                let world = latlon_to_world(lat, lon);
                let graph_id = graph.nodes.len();
                osm_node_to_graph.insert(osm_id, graph_id);
                graph.nodes.push(GraphNode {
                    id: graph_id,
                    world_pos: world,
                    lat_lon: [lat, lon],
                });
            }
        }
    }

    log::info!("Graph nodes: {}", graph.nodes.len());

    // Pre-allocate adjacency list.
    graph.adjacency = vec![Vec::new(); graph.nodes.len()];

    // Process ways — routable edges and decoration shapes.
    for way in &ways {
        if let Some(hw_tag) = find_tag(&way.tags, "highway") {
            if is_routable_highway(hw_tag) {
                let is_area = find_tag(&way.tags, "area") == Some("yes");
                if is_area
                    || is_rejected_access(&way.tags)
                    || is_motor_barred(&way.tags)
                    || is_rejected_service(&way.tags)
                {
                    build_decoration_shape(
                        &way.node_refs,
                        &osm_node_to_graph,
                        &graph.nodes,
                        &mut graph.decorations,
                        DecorationKind::ServiceArea,
                    );
                    continue;
                }

                let road_class = RoadClass::from_tag(hw_tag);

                let oneway_tag = find_tag(&way.tags, "oneway");
                let junction_tag = find_tag(&way.tags, "junction");
                let is_one_way = oneway_tag == Some("yes")
                    || oneway_tag == Some("1")
                    || oneway_tag == Some("true")
                    || junction_tag == Some("roundabout");
                let is_reverse_oneway = oneway_tag == Some("-1") || oneway_tag == Some("reverse");

                let node_ids: Vec<usize> = way
                    .node_refs
                    .iter()
                    .filter_map(|nid| osm_node_to_graph.get(nid).copied())
                    .collect();

                if node_ids.len() < 2 {
                    continue;
                }

                let positions: Vec<[f64; 2]> = node_ids
                    .iter()
                    .map(|&gid| graph.nodes[gid].world_pos)
                    .collect();
                let latlons: Vec<[f64; 2]> = node_ids
                    .iter()
                    .map(|&gid| graph.nodes[gid].lat_lon)
                    .collect();

                for i in 0..(node_ids.len() - 1) {
                    let (from, to) = if is_reverse_oneway {
                        (node_ids[i + 1], node_ids[i])
                    } else {
                        (node_ids[i], node_ids[i + 1])
                    };
                    let polyline = if is_reverse_oneway {
                        vec![positions[i + 1], positions[i]]
                    } else {
                        vec![positions[i], positions[i + 1]]
                    };
                    let weight = haversine_dist(latlons[i], latlons[i + 1]);

                    let fwd_idx = graph.edges.len();
                    graph.edges.push(GraphEdge {
                        from,
                        to,
                        weight_meters: weight,
                        polyline_world: polyline.clone(),
                        road_class,
                        one_way: true,
                    });
                    graph.adjacency[from].push(fwd_idx);

                    if !is_one_way && !is_reverse_oneway {
                        let rev_idx = graph.edges.len();
                        graph.edges.push(GraphEdge {
                            from: to,
                            to: from,
                            weight_meters: weight,
                            polyline_world: {
                                let mut rev = polyline.clone();
                                rev.reverse();
                                rev
                            },
                            road_class,
                            one_way: false,
                        });
                        graph.adjacency[to].push(rev_idx);
                    }
                }
                continue;
            }
        }

        if find_tag(&way.tags, "building").is_some() {
            build_decoration_shape(
                &way.node_refs,
                &osm_node_to_graph,
                &graph.nodes,
                &mut graph.decorations,
                DecorationKind::Building,
            );
            continue;
        }
        if let Some(lu) = find_tag(&way.tags, "landuse") {
            if LANDUSE_KEEP.contains(&lu) {
                build_decoration_shape(
                    &way.node_refs,
                    &osm_node_to_graph,
                    &graph.nodes,
                    &mut graph.decorations,
                    DecorationKind::Landuse,
                );
                continue;
            }
        }
        if find_tag(&way.tags, "highway") == Some("pedestrian")
            && find_tag(&way.tags, "area") == Some("yes")
        {
            build_decoration_shape(
                &way.node_refs,
                &osm_node_to_graph,
                &graph.nodes,
                &mut graph.decorations,
                DecorationKind::PedestrianArea,
            );
            continue;
        }

        let mut is_water = false;
        if let Some(n) = find_tag(&way.tags, "natural") {
            if n == "water" || n == "wetland" {
                is_water = true;
            }
        }
        if let Some(lu) = find_tag(&way.tags, "landuse") {
            if lu == "reservoir" {
                is_water = true;
            }
        }
        if let Some(ww) = find_tag(&way.tags, "waterway") {
            if WATERWAY_KEEP.contains(&ww) {
                is_water = true;
            }
        }
        if is_water {
            build_decoration_shape(
                &way.node_refs,
                &osm_node_to_graph,
                &graph.nodes,
                &mut graph.decorations,
                DecorationKind::Water,
            );
        }
    }

    // Assemble water multipolygon relations into outer-ring decoration shapes.
    // Each relation may reference several ways that must be stitched together
    // by matching endpoints to form closed rings. Inner rings (islands) are
    // skipped — see WaterRelation docstring.
    if !water_relations.is_empty() {
        let mut ways_by_id: HashMap<i64, &WayRecord> = HashMap::with_capacity(ways.len());
        for w in &ways {
            ways_by_id.insert(w.id, w);
        }

        let mut ring_count = 0usize;
        for rel in &water_relations {
            let mut segments: Vec<Vec<i64>> = Vec::new();
            for &wid in &rel.outer_way_ids {
                if let Some(w) = ways_by_id.get(&wid) {
                    if w.node_refs.len() >= 2 {
                        segments.push(w.node_refs.clone());
                    }
                }
            }
            let rings = stitch_rings(&mut segments);
            for ring in rings {
                build_decoration_shape(
                    &ring,
                    &osm_node_to_graph,
                    &graph.nodes,
                    &mut graph.decorations,
                    DecorationKind::Water,
                );
                ring_count += 1;
            }
        }
        log::info!(
            "Water relations: assembled {} outer ring(s) from {} relation(s)",
            ring_count,
            water_relations.len()
        );
    }

    debug_assert!(
        graph.edges.iter().all(|e| matches!(
            e.road_class,
            RoadClass::Motorway
                | RoadClass::Primary
                | RoadClass::Secondary
                | RoadClass::Tertiary
                | RoadClass::Residential
                | RoadClass::Service
        )),
        "All edges must be routable road classes"
    );

    log::info!(
        "Graph built: {} nodes, {} edges, {} decoration shapes in {:.2?}",
        graph.node_count(),
        graph.edge_count(),
        graph.decorations.shapes.len(),
        t0.elapsed()
    );

    log::info!("Total load time: {:.2?}", total_start.elapsed());

    Ok(graph)
}

fn build_decoration_shape(
    node_refs: &[i64],
    osm_node_to_graph: &HashMap<i64, usize>,
    graph_nodes: &[GraphNode],
    decorations: &mut DecorationLayer,
    kind: DecorationKind,
) {
    let mut polyline_world: Vec<[f64; 2]> = Vec::with_capacity(node_refs.len());
    for &nid in node_refs {
        if let Some(&gid) = osm_node_to_graph.get(&nid) {
            if gid < graph_nodes.len() {
                polyline_world.push(graph_nodes[gid].world_pos);
            }
        }
    }
    if polyline_world.len() < 2 {
        return;
    }

    let closed = node_refs.first() == node_refs.last();

    decorations.shapes.push(DecorationShape {
        kind,
        polyline_world,
        closed,
    });
}

/// Stitch a set of linestrings (each a sequence of OSM node IDs) into closed
/// rings by matching endpoints. Consumes `segments` (leaves unused segments
/// in place). Returns a `Vec` of closed rings (first == last node id).
///
/// Greedy endpoint-matching: pick an unused segment, extend by finding any
/// unused segment that shares its current end node, reverse if necessary,
/// append (deduping the shared node). Stop when the ring closes or no match
/// is found. A ring with at least 4 node-ids and first==last is considered
/// valid; partial/open chains are discarded.
fn stitch_rings(segments: &mut [Vec<i64>]) -> Vec<Vec<i64>> {
    let mut rings: Vec<Vec<i64>> = Vec::new();
    let mut used = vec![false; segments.len()];

    for i in 0..segments.len() {
        if used[i] {
            continue;
        }
        used[i] = true;
        let mut ring = segments[i].clone();

        loop {
            if ring.len() >= 2 && ring.first() == ring.last() {
                break;
            }
            let tail = *ring.last().unwrap();
            let mut matched = None;
            for (j, seg) in segments.iter().enumerate() {
                if used[j] || seg.len() < 2 {
                    continue;
                }
                if *seg.first().unwrap() == tail {
                    matched = Some((j, false));
                    break;
                } else if *seg.last().unwrap() == tail {
                    matched = Some((j, true));
                    break;
                }
            }
            match matched {
                Some((j, reverse)) => {
                    used[j] = true;
                    let seg = &segments[j];
                    if reverse {
                        // append seg reversed, skipping the shared first node
                        for &nid in seg.iter().rev().skip(1) {
                            ring.push(nid);
                        }
                    } else {
                        for &nid in seg.iter().skip(1) {
                            ring.push(nid);
                        }
                    }
                }
                None => break,
            }
        }

        if ring.len() >= 4 && ring.first() == ring.last() {
            rings.push(ring);
        }
    }

    rings
}
