use std::collections::HashMap;
use osmpbfreader::{OsmObj, OsmPbfReader};
use crate::graph::{GraphEdge, GraphNode, RoadGraph, RoadClass};
use crate::projection::latlon_to_world;

const HIGHWAY_TAGS: &[&str] = &[
    "motorway", "motorway_link", "trunk", "trunk_link",
    "primary", "primary_link",
    "secondary", "secondary_link",
    "tertiary", "tertiary_link",
    "residential", "unclassified", "living_street", "road",
    "service",
    "path", "footway", "cycleway", "track",
];

fn is_highway(tag: &str) -> bool {
    HIGHWAY_TAGS.contains(&tag)
}

fn euclidean_dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    (dx * dx + dy * dy).sqrt()
}

pub fn load_graph(path: &str) -> Result<RoadGraph, Box<dyn std::error::Error>> {
    log::info!("Loading OSM PBF from: {}", path);

    let file = std::fs::File::open(path)?;
    let mut pbf = OsmPbfReader::new(file);

    // Collect all highway ways and their dependency nodes.
    let objs = pbf.get_objs_and_deps(|obj| {
        if let OsmObj::Way(ref w) = *obj {
            if let Some(hw) = w.tags.get("highway") {
                return is_highway(hw);
            }
        }
        false
    })?;

    log::info!("Loaded {} OSM objects", objs.len());

    // Build node-id → graph-id mapping and collect node coordinates.
    let mut osm_node_to_graph: HashMap<i64, usize> = HashMap::new();
    let mut graph = RoadGraph::new();

    // First, index all nodes from the dependency set.
    for (_, obj) in &objs {
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

    // Process ways and create edges.
    for (_, obj) in &objs {
        if let OsmObj::Way(way) = obj {
            let hw_tag = match way.tags.get("highway") {
                Some(t) => t,
                None => continue,
            };
            if !is_highway(hw_tag) {
                continue;
            }

            let road_class = RoadClass::from_tag(hw_tag);

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

            // Add one edge per consecutive pair (forward, and reverse if bidirectional).
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
        }
    }

    log::info!(
        "Graph built: {} nodes, {} edges",
        graph.node_count(),
        graph.edge_count()
    );

    Ok(graph)
}
