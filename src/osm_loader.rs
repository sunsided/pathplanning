use crate::graph::{
    DecorationKind, DecorationLayer, DecorationShape, GraphEdge, GraphNode, RoadClass, RoadGraph,
};
use crate::projection::latlon_to_world;
use osmpbfreader::{OsmObj, OsmPbfReader};
use std::collections::HashMap;

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
/// Kept for documentation; the PBF filter implicitly drops these by only
/// accepting `ROUTABLE_HIGHWAYS` and specific decoration cases.
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

/// Service values that disqualify a way from being routable.
const SERVICE_REJECT: &[&str] = &["parking_aisle", "driveway", "emergency_access"];

/// Access values that disqualify a way from being routable.
const ACCESS_REJECT: &[&str] = &["no", "private", "customers", "delivery"];

fn euclidean_dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    (dx * dx + dy * dy).sqrt()
}

fn is_routable_highway(hw: &str) -> bool {
    ROUTABLE_HIGHWAYS.contains(&hw)
}

fn is_rejected_access(tags: &osmpbfreader::Tags) -> bool {
    tags.get("access")
        .map(|v| ACCESS_REJECT.contains(&v.as_str()))
        .unwrap_or(false)
}

fn is_motor_barred(tags: &osmpbfreader::Tags) -> bool {
    tags.get("motor_vehicle")
        .map(|v| v == "no")
        .unwrap_or(false)
        || tags.get("vehicle").map(|v| v == "no").unwrap_or(false)
}

fn is_rejected_service(tags: &osmpbfreader::Tags) -> bool {
    tags.get("service")
        .map(|v| SERVICE_REJECT.contains(&v.as_str()))
        .unwrap_or(false)
}

pub fn load_graph(path: &str) -> Result<RoadGraph, Box<dyn std::error::Error>> {
    log::info!("Loading OSM PBF from: {}", path);

    let file = std::fs::File::open(path)?;
    // SAFETY: map file is read-only and not modified while mapped.
    let mmap = unsafe { memmap2::Mmap::map(&file)? };
    let cursor = std::io::Cursor::new(&mmap[..]);
    let mut pbf = OsmPbfReader::new(cursor);

    // Collect ways we care about: routable highways, buildings, landuse, pedestrian areas.
    let objs = pbf.get_objs_and_deps(|obj| match obj {
        OsmObj::Way(w) => {
            if let Some(hw) = w.tags.get("highway") {
                if is_routable_highway(hw.as_str()) {
                    return true;
                }
                if hw == "pedestrian" && w.tags.get("area").map(|s| s.as_str()) == Some("yes") {
                    return true;
                }
                return false;
            }
            if w.tags.contains_key("building") {
                return true;
            }
            if let Some(lu) = w.tags.get("landuse") {
                return LANDUSE_KEEP.contains(&lu.as_str());
            }
            false
        }
        _ => false,
    })?;

    log::info!("Loaded {} OSM objects", objs.len());

    // Pass 1: index ALL nodes from the dependency set.
    // Decoration-only nodes are included; their adjacency slots stay empty.
    // Simpler first cut — revisit with separate coord table if memory is a concern.
    let mut osm_node_to_graph: HashMap<i64, usize> = HashMap::new();
    let mut graph = RoadGraph::new();

    for obj in objs.values() {
        if let OsmObj::Node(node) = obj {
            let osm_id = node.id.0;
            if osm_node_to_graph.contains_key(&osm_id) {
                continue;
            }
            let lat = node.lat();
            let lon = node.lon();
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

    log::info!("Graph nodes: {}", graph.nodes.len());

    // Pre-allocate adjacency list.
    graph.adjacency = vec![Vec::new(); graph.nodes.len()];

    // Pass 2: process ways — routable edges and decoration shapes.
    for obj in objs.values() {
        if let OsmObj::Way(way) = obj {
            // --- Routable highway ---
            if let Some(hw_tag) = way.tags.get("highway") {
                if is_routable_highway(hw_tag.as_str()) {
                    // Apply rejection rules
                    let is_area = way.tags.get("area").map(|s| s.as_str()) == Some("yes");
                    if is_area
                        || is_rejected_access(&way.tags)
                        || is_motor_barred(&way.tags)
                        || is_rejected_service(&way.tags)
                    {
                        // Demote rejected routable candidates to decoration.
                        build_decoration_shape(
                            way,
                            &osm_node_to_graph,
                            &graph.nodes,
                            &mut graph.decorations,
                            DecorationKind::ServiceArea,
                        );
                        continue;
                    }

                    let road_class = RoadClass::from_tag(hw_tag.as_str());

                    // Determine one-way status.
                    let oneway_tag = way.tags.get("oneway").map(|s| s.as_str());
                    let junction_tag = way.tags.get("junction").map(|s| s.as_str());
                    let is_one_way = oneway_tag == Some("yes")
                        || oneway_tag == Some("1")
                        || oneway_tag == Some("true")
                        || junction_tag == Some("roundabout");

                    // Collect valid node ids for this way.
                    let node_ids: Vec<usize> = way
                        .nodes
                        .iter()
                        .filter_map(|nid| osm_node_to_graph.get(&nid.0).copied())
                        .collect();

                    if node_ids.len() < 2 {
                        continue;
                    }

                    // Collect world positions.
                    let positions: Vec<[f64; 2]> = node_ids
                        .iter()
                        .map(|&gid| graph.nodes[gid].world_pos)
                        .collect();

                    // Add one edge per consecutive pair.
                    for i in 0..(node_ids.len() - 1) {
                        let from = node_ids[i];
                        let to = node_ids[i + 1];
                        let polyline = vec![positions[i], positions[i + 1]];
                        let weight = euclidean_dist(positions[i], positions[i + 1]);

                        // Forward edge.
                        let fwd_idx = graph.edges.len();
                        graph.edges.push(GraphEdge {
                            from,
                            to,
                            weight_meters: weight,
                            polyline_world: polyline.clone(),
                            road_class,
                            one_way: is_one_way,
                        });
                        graph.adjacency[from].push(fwd_idx);

                        // Reverse edge (if bidirectional).
                        if !is_one_way {
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

            // --- Decoration geometry ---
            if way.tags.contains_key("building") {
                build_decoration_shape(
                    way,
                    &osm_node_to_graph,
                    &graph.nodes,
                    &mut graph.decorations,
                    DecorationKind::Building,
                );
                continue;
            }
            if let Some(lu) = way.tags.get("landuse") {
                if LANDUSE_KEEP.contains(&lu.as_str()) {
                    build_decoration_shape(
                        way,
                        &osm_node_to_graph,
                        &graph.nodes,
                        &mut graph.decorations,
                        DecorationKind::Landuse,
                    );
                    continue;
                }
            }
            if way.tags.get("highway").map(|s| s.as_str()) == Some("pedestrian")
                && way.tags.get("area").map(|s| s.as_str()) == Some("yes")
            {
                build_decoration_shape(
                    way,
                    &osm_node_to_graph,
                    &graph.nodes,
                    &mut graph.decorations,
                    DecorationKind::PedestrianArea,
                );
            }
        }
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
        "Graph built: {} nodes, {} edges, {} decoration shapes",
        graph.node_count(),
        graph.edge_count(),
        graph.decorations.shapes.len()
    );

    Ok(graph)
}

fn build_decoration_shape(
    way: &osmpbfreader::Way,
    osm_node_to_graph: &HashMap<i64, usize>,
    graph_nodes: &[GraphNode],
    decorations: &mut DecorationLayer,
    kind: DecorationKind,
) {
    let mut polyline_world: Vec<[f64; 2]> = Vec::with_capacity(way.nodes.len());
    for nid in &way.nodes {
        if let Some(&gid) = osm_node_to_graph.get(&nid.0) {
            if gid < graph_nodes.len() {
                polyline_world.push(graph_nodes[gid].world_pos);
            }
        }
    }
    if polyline_world.len() < 2 {
        return;
    }

    // Closed if first and last OSM node ids are the same.
    let closed = way.nodes.first() == way.nodes.last();

    decorations.shapes.push(DecorationShape {
        kind,
        polyline_world,
        closed,
    });
}
