use std::io::{Read, Write};

use flate2::Compression;
use flate2::read::GzDecoder;
use flate2::write::GzEncoder;

use crate::graph::RoadGraph;

const MAGIC: &[u8; 4] = b"PPG1";

/// Serializes a road graph into the compact binary asset served to the web build.
pub fn encode(graph: &RoadGraph) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let raw = bincode::serialize(graph)?;
    let mut encoder = GzEncoder::new(Vec::new(), Compression::default());
    encoder.write_all(MAGIC)?;
    encoder.write_all(&raw)?;
    Ok(encoder.finish()?)
}

/// Deserializes a road graph from the compact binary asset produced by [`encode`].
pub fn decode(bytes: &[u8]) -> Result<RoadGraph, Box<dyn std::error::Error>> {
    let mut decoder = GzDecoder::new(bytes);
    let mut magic = [0u8; 4];
    decoder.read_exact(&mut magic)?;
    if &magic != MAGIC {
        return Err("invalid graph asset magic".into());
    }
    let mut raw = Vec::new();
    decoder.read_to_end(&mut raw)?;
    let mut graph: RoadGraph = bincode::deserialize(&raw)?;
    graph.rebuild_derived();
    Ok(graph)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{DecorationLayer, GraphEdge, GraphNode, RoadClass, RoadGraph};

    #[test]
    fn polyline_roundtrip() {
        let mut graph = RoadGraph::new();
        graph.nodes = vec![
            GraphNode {
                id: 0,
                world_pos: [1_493_394.0, 6_894_652.0],
                lat_lon: [0.0, 0.0],
            },
            GraphNode {
                id: 1,
                world_pos: [1_493_694.5, 6_894_952.25],
                lat_lon: [0.0, 0.0],
            },
        ];
        graph.edges = vec![GraphEdge {
            from: 0,
            to: 1,
            weight_meters: 424.0,
            travel_time_s: 30.0,
            polyline_world: vec![[1_493_394.0, 6_894_652.0], [1_493_694.5, 6_894_952.25]],
            road_class: RoadClass::Residential,
            one_way: false,
        }];
        graph.decorations = DecorationLayer::default();
        graph.rebuild_derived();

        let bytes = encode(&graph).unwrap();
        let decoded = decode(&bytes).unwrap();
        assert_eq!(decoded.nodes.len(), 2);
        assert_eq!(decoded.edges.len(), 1);
        assert_eq!(
            decoded.edges[0].polyline_world[1],
            [1_493_694.5, 6_894_952.25]
        );
    }

    #[test]
    fn generated_asset_edges_are_near_their_nodes() {
        let Ok(bytes) = std::fs::read("web/graph.bin.gz") else {
            return;
        };
        let graph = decode(&bytes).unwrap();
        let (mut min_x, mut min_y) = (f64::INFINITY, f64::INFINITY);
        let (mut max_x, mut max_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
        for edge in &graph.edges {
            for point in &edge.polyline_world {
                min_x = min_x.min(point[0]);
                min_y = min_y.min(point[1]);
                max_x = max_x.max(point[0]);
                max_y = max_y.max(point[1]);
            }
        }
        eprintln!("edge bbox: ({min_x}, {min_y}) .. ({max_x}, {max_y})");
        let bbox = graph.bounding_box().unwrap();
        eprintln!("node bbox: {bbox:?}");
        assert!(min_x >= bbox.0[0] - 100.0 && max_x <= bbox.1[0] + 100.0);
        assert!(min_y >= bbox.0[1] - 100.0 && max_y <= bbox.1[1] + 100.0);
    }
}
