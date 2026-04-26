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

/// Service values that disqualify a way from being routable.
const SERVICE_REJECT: &[&str] = &["parking_aisle", "driveway", "emergency_access"];

/// Access values that disqualify a way from being routable.
const ACCESS_REJECT: &[&str] = &["no", "private", "customers", "delivery"];

struct WayRecord {
    tags: Vec<(String, String)>,
    node_refs: Vec<i64>,
}

fn find_tag<'a>(tags: &'a [(String, String)], key: &str) -> Option<&'a str> {
    tags.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
}

fn euclidean_dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    (dx * dx + dy * dy).sqrt()
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
    let mut has_building = false;
    let mut landuse = None;
    let mut area = false;
    for (k, v) in tags {
        match k {
            "highway" => highway = Some(v),
            "building" => has_building = true,
            "landuse" => landuse = Some(v),
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
    if has_building {
        return true;
    }
    if let Some(lu) = landuse {
        return LANDUSE_KEEP.contains(&lu);
    }
    false
}

pub fn load_graph(path: &str) -> Result<RoadGraph, Box<dyn std::error::Error>> {
    log::info!("Loading OSM PBF from: {}", path);
    let total_start = Instant::now();

    // Pass 1: collect matching ways + all referenced node IDs
    let t0 = Instant::now();
    let reader = OsmReader::from_path(path)?;
    reader.apply_element_filter(ElementFilter {
        nodes: false,
        relations: false,
        ways: true,
    });

    let (ways, all_node_refs): (Vec<WayRecord>, Vec<i64>) = reader
        .par_blocks()
        .map(|block| {
            let mut ways = Vec::new();
            let mut refs = Vec::new();
            if let ElementBlock::WayBlock(block) = block {
                for way in block.iter() {
                    let borrowed_tags: Vec<(&str, &str)> = way.tags().collect();
                    if !way_passes_filter(borrowed_tags.iter().copied()) {
                        continue;
                    }
                    let node_refs: Vec<i64> = way.node_ids().collect();
                    let tags: Vec<(String, String)> = borrowed_tags
                        .into_iter()
                        .map(|(k, v)| (k.to_string(), v.to_string()))
                        .collect();
                    refs.extend_from_slice(&node_refs);
                    ways.push(WayRecord { tags, node_refs });
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
        "Pass 1: collected {} ways in {:.2?}",
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

                for i in 0..(node_ids.len() - 1) {
                    let from = node_ids[i];
                    let to = node_ids[i + 1];
                    let polyline = vec![positions[i], positions[i + 1]];
                    let weight = euclidean_dist(positions[i], positions[i + 1]);

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
