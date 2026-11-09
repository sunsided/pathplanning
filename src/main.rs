#![cfg(not(target_arch = "wasm32"))]

use std::sync::Arc;

use pathplanning::app;
use pathplanning::{asset, osm_loader};

fn main() {
    env_logger::init();

    let args: Vec<String> = std::env::args().collect();

    if args.get(1).is_some_and(|arg| arg == "--export") {
        let input = args
            .get(2)
            .map(String::as_str)
            .unwrap_or("maps/berlin.osm.pbf");
        let output = args
            .get(3)
            .map(String::as_str)
            .unwrap_or("web/graph.bin.gz");
        let bbox = args.get(4).map(String::as_str);
        export(input, output, bbox);
        return;
    }

    let pbf_path = args
        .get(1)
        .cloned()
        .unwrap_or_else(|| "maps/berlin.osm.pbf".to_string());

    let graph = Arc::new(osm_loader::load_graph(&pbf_path).unwrap_or_else(|e| {
        eprintln!("Error: could not load '{}': {}", pbf_path, e);
        std::process::exit(1);
    }));

    if let Err(e) = app::run(graph) {
        eprintln!("Error: event loop failed: {e}");
        std::process::exit(1);
    }
}

fn export(input: &str, output: &str, bbox: Option<&str>) {
    eprintln!("Loading {input} ...");
    let mut graph = osm_loader::load_graph(input).unwrap_or_else(|e| {
        eprintln!("Error: could not load '{input}': {e}");
        std::process::exit(1);
    });

    if let Some(bbox) = bbox {
        let coordinates: Vec<f64> = bbox
            .split(',')
            .map(|value| {
                value.trim().parse::<f64>().unwrap_or_else(|e| {
                    eprintln!("Error: invalid bbox component '{value}': {e}");
                    std::process::exit(1);
                })
            })
            .collect();
        if coordinates.len() != 4 {
            eprintln!("Error: bbox must be min_lat,min_lon,max_lat,max_lon");
            std::process::exit(1);
        }
        let a = pathplanning::projection::latlon_to_world(coordinates[0], coordinates[1]);
        let b = pathplanning::projection::latlon_to_world(coordinates[2], coordinates[3]);
        let min = [a[0].min(b[0]), a[1].min(b[1])];
        let max = [a[0].max(b[0]), a[1].max(b[1])];
        eprintln!("Clipping to bbox {bbox} ...");
        graph.retain_world_bbox(min, max);
    }

    graph.prune_unused_nodes();

    eprintln!(
        "Exporting {} nodes, {} edges to {output} ...",
        graph.node_count(),
        graph.edge_count()
    );
    let bytes = asset::encode(&graph).unwrap_or_else(|e| {
        eprintln!("Error: could not encode graph: {e}");
        std::process::exit(1);
    });

    if let Some(parent) = std::path::Path::new(output).parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent).unwrap_or_else(|e| {
            eprintln!("Error: could not create '{}': {e}", parent.display());
            std::process::exit(1);
        });
    }

    std::fs::write(output, &bytes).unwrap_or_else(|e| {
        eprintln!("Error: could not write '{output}': {e}");
        std::process::exit(1);
    });
    eprintln!("Wrote {} bytes.", bytes.len());
}
