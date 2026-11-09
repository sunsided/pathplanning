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
