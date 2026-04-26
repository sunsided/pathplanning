use crate::astar::{PlannerState, PlannerStatus};
use crate::camera::Camera;
use crate::graph::{RoadClass, RoadGraph};
use crate::input::InputState;
use tiny_skia::{Color, FillRule, Paint, PathBuilder, Pixmap, Stroke, Transform};

// ---------------------------------------------------------------------------
// Minimal 8×8 bitmap font (row-major, MSB = leftmost pixel).
// Only covers the characters needed for the debug overlay.
// ---------------------------------------------------------------------------

fn get_glyph(c: u8) -> [u8; 8] {
    match c {
        b' ' => [0x00; 8],
        b'-' => [0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00],
        b'.' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18],
        b':' => [0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x00],
        b'/' => [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x00],
        b'0' => [0x3C, 0x66, 0x6E, 0x76, 0x66, 0x66, 0x3C, 0x00],
        b'1' => [0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00],
        b'2' => [0x3C, 0x66, 0x06, 0x0C, 0x18, 0x30, 0x7E, 0x00],
        b'3' => [0x3C, 0x66, 0x06, 0x1C, 0x06, 0x66, 0x3C, 0x00],
        b'4' => [0x0E, 0x1E, 0x36, 0x66, 0x7F, 0x06, 0x06, 0x00],
        b'5' => [0x7E, 0x60, 0x60, 0x7C, 0x06, 0x06, 0x7C, 0x00],
        b'6' => [0x1C, 0x30, 0x60, 0x7C, 0x66, 0x66, 0x3C, 0x00],
        b'7' => [0x7E, 0x06, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x00],
        b'8' => [0x3C, 0x66, 0x66, 0x3C, 0x66, 0x66, 0x3C, 0x00],
        b'9' => [0x3C, 0x66, 0x66, 0x3E, 0x06, 0x0C, 0x38, 0x00],
        b'A' => [0x18, 0x3C, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x00],
        b'B' => [0x7C, 0x66, 0x66, 0x7C, 0x66, 0x66, 0x7C, 0x00],
        b'C' => [0x3C, 0x66, 0x60, 0x60, 0x60, 0x66, 0x3C, 0x00],
        b'D' => [0x78, 0x6C, 0x66, 0x66, 0x66, 0x6C, 0x78, 0x00],
        b'E' => [0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x7E, 0x00],
        b'F' => [0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x00],
        b'G' => [0x3C, 0x66, 0x60, 0x6E, 0x66, 0x66, 0x3C, 0x00],
        b'H' => [0x66, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x00],
        b'I' => [0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00],
        b'J' => [0x0E, 0x06, 0x06, 0x06, 0x06, 0x66, 0x3C, 0x00],
        b'K' => [0x66, 0x6C, 0x78, 0x70, 0x78, 0x6C, 0x66, 0x00],
        b'L' => [0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E, 0x00],
        b'M' => [0x63, 0x77, 0x7F, 0x6B, 0x63, 0x63, 0x63, 0x00],
        b'N' => [0x66, 0x76, 0x7E, 0x7E, 0x6E, 0x66, 0x66, 0x00],
        b'O' => [0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00],
        b'P' => [0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x00],
        b'Q' => [0x3C, 0x66, 0x66, 0x66, 0x6E, 0x3C, 0x06, 0x00],
        b'R' => [0x7C, 0x66, 0x66, 0x7C, 0x6C, 0x66, 0x66, 0x00],
        b'S' => [0x3C, 0x66, 0x60, 0x3C, 0x06, 0x66, 0x3C, 0x00],
        b'T' => [0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00],
        b'U' => [0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00],
        b'V' => [0x66, 0x66, 0x66, 0x66, 0x3C, 0x3C, 0x18, 0x00],
        b'W' => [0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x63, 0x00],
        b'X' => [0x66, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x66, 0x00],
        b'Y' => [0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x18, 0x00],
        b'Z' => [0x7E, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x7E, 0x00],
        _ => [0x00; 8],
    }
}

/// Draw a single 8×8 glyph at pixel coordinates (x, y) using direct pixel writes.
fn draw_glyph(pixmap: &mut Pixmap, x: i32, y: i32, glyph: [u8; 8], r: u8, g: u8, b: u8) {
    let pw = pixmap.width() as i32;
    let ph = pixmap.height() as i32;
    let data = pixmap.data_mut();

    for row in 0..8i32 {
        let byte = glyph[row as usize];
        for col in 0..8i32 {
            if byte & (0x80u8 >> col as u8) != 0 {
                let px = x + col;
                let py = y + row;
                if px >= 0 && px < pw && py >= 0 && py < ph {
                    // tiny-skia stores pixels as premultiplied RGBA in memory order
                    // with the layout [r, g, b, a] (confirmed by PremultipliedColorU8 accessors).
                    let idx = ((py * pw + px) * 4) as usize;
                    data[idx] = r;
                    data[idx + 1] = g;
                    data[idx + 2] = b;
                    data[idx + 3] = 255;
                }
            }
        }
    }
}

/// Draw an ASCII string starting at (x, y) with each character 9 pixels wide.
fn draw_text(pixmap: &mut Pixmap, x: i32, y: i32, text: &str, r: u8, g: u8, b: u8) {
    let mut cx = x;
    for c in text.bytes() {
        draw_glyph(pixmap, cx, y, get_glyph(c), r, g, b);
        cx += 9;
    }
}

// ---------------------------------------------------------------------------
// Path helpers
// ---------------------------------------------------------------------------

fn circle_path(cx: f32, cy: f32, radius: f32) -> Option<tiny_skia::Path> {
    const K: f32 = 0.552_284_8;
    let r = radius;
    let mut pb = PathBuilder::new();
    pb.move_to(cx + r, cy);
    pb.cubic_to(cx + r, cy - K * r, cx + K * r, cy - r, cx, cy - r);
    pb.cubic_to(cx - K * r, cy - r, cx - r, cy - K * r, cx - r, cy);
    pb.cubic_to(cx - r, cy + K * r, cx - K * r, cy + r, cx, cy + r);
    pb.cubic_to(cx + K * r, cy + r, cx + r, cy + K * r, cx + r, cy);
    pb.close();
    pb.finish()
}

fn fill_circle(pixmap: &mut Pixmap, cx: f32, cy: f32, radius: f32, r: u8, g: u8, b: u8, a: u8) {
    if let Some(path) = circle_path(cx, cy, radius) {
        let mut paint = Paint::default();
        paint.set_color_rgba8(r, g, b, a);
        pixmap.fill_path(
            &path,
            &paint,
            FillRule::Winding,
            Transform::identity(),
            None,
        );
    }
}

fn stroke_polyline(
    pixmap: &mut Pixmap,
    points: &[[f64; 2]],
    camera: &Camera,
    r: u8,
    g: u8,
    b: u8,
    a: u8,
    width: f32,
) {
    if points.len() < 2 {
        return;
    }
    let mut pb = PathBuilder::new();
    let sp0 = camera.world_to_screen(points[0]);
    pb.move_to(sp0[0], sp0[1]);
    for &wp in &points[1..] {
        let sp = camera.world_to_screen(wp);
        pb.line_to(sp[0], sp[1]);
    }
    if let Some(path) = pb.finish() {
        let mut paint = Paint::default();
        paint.set_color_rgba8(r, g, b, a);
        let stroke = Stroke {
            width,
            ..Default::default()
        };
        pixmap.stroke_path(&path, &paint, &stroke, Transform::identity(), None);
    }
}

// ---------------------------------------------------------------------------
// Public render entry point
// ---------------------------------------------------------------------------

pub fn render(
    graph: &RoadGraph,
    camera: &Camera,
    planner: &PlannerState,
    input_state: &InputState,
    pixmap: &mut Pixmap,
) {
    // Background.
    pixmap.fill(Color::from_rgba8(10, 14, 26, 255));

    // Layer 1: all road edges (static).
    draw_all_edges(graph, camera, pixmap);

    // Layer 2: explored (closed-set) edges.
    draw_explored_edges(graph, camera, planner, pixmap);

    // Layer 3: frontier (open-set) nodes.
    draw_frontier(graph, camera, planner, pixmap);

    // Layer 4: current best path (neon cyan).
    if let Some(ref path) = planner.best_path {
        draw_node_path(graph, camera, path, pixmap, 0, 255, 255, 220, 2.5);
    }

    // Layer 5: locked path (electric blue).
    if let Some(ref path) = planner.locked_path {
        draw_node_path(graph, camera, path, pixmap, 0, 128, 255, 255, 3.0);
    }

    // Layer 6: markers.
    draw_markers(camera, input_state, pixmap);

    // Layer 7: debug overlay.
    draw_debug(graph, camera, planner, pixmap);
}

fn draw_all_edges(graph: &RoadGraph, camera: &Camera, pixmap: &mut Pixmap) {
    for edge in &graph.edges {
        let width = edge.road_class.stroke_width();
        let (er, eg, eb) = match edge.road_class {
            RoadClass::Motorway => (40, 60, 100),
            RoadClass::Primary => (30, 55, 90),
            _ => (26, 58, 92),
        };
        stroke_polyline(pixmap, &edge.polyline_world, camera, er, eg, eb, 200, width);
    }
}

fn draw_explored_edges(
    graph: &RoadGraph,
    camera: &Camera,
    planner: &PlannerState,
    pixmap: &mut Pixmap,
) {
    if planner.explored.is_empty() {
        return;
    }
    for edge in &graph.edges {
        if planner.explored.contains(&edge.from) && planner.explored.contains(&edge.to) {
            stroke_polyline(
                pixmap,
                &edge.polyline_world,
                camera,
                30,
                77,
                122, // #1e4d7a
                180,
                1.2,
            );
        }
    }
}

fn draw_frontier(graph: &RoadGraph, camera: &Camera, planner: &PlannerState, pixmap: &mut Pixmap) {
    for &node_id in &planner.frontier {
        if node_id >= graph.nodes.len() {
            continue;
        }
        let sp = camera.world_to_screen(graph.nodes[node_id].world_pos);
        fill_circle(pixmap, sp[0], sp[1], 2.5, 0, 212, 255, 200);
    }
}

fn draw_node_path(
    graph: &RoadGraph,
    camera: &Camera,
    path: &[usize],
    pixmap: &mut Pixmap,
    r: u8,
    g: u8,
    b: u8,
    a: u8,
    width: f32,
) {
    if path.len() < 2 {
        return;
    }
    let n = graph.nodes.len();
    let mut pb = PathBuilder::new();
    let mut started = false;
    for &nid in path {
        if nid >= n {
            continue;
        }
        let sp = camera.world_to_screen(graph.nodes[nid].world_pos);
        if !started {
            pb.move_to(sp[0], sp[1]);
            started = true;
        } else {
            pb.line_to(sp[0], sp[1]);
        }
    }
    if let Some(path_geom) = pb.finish() {
        let mut paint = Paint::default();
        paint.set_color_rgba8(r, g, b, a);
        let stroke = Stroke {
            width,
            ..Default::default()
        };
        pixmap.stroke_path(&path_geom, &paint, &stroke, Transform::identity(), None);
    }
}

fn draw_markers(camera: &Camera, input_state: &InputState, pixmap: &mut Pixmap) {
    if let Some(ref m) = input_state.start_marker {
        let sp = camera.world_to_screen(m.world_pos);
        fill_circle(pixmap, sp[0], sp[1], 7.0, 0, 255, 128, 230);
        fill_circle(pixmap, sp[0], sp[1], 3.0, 255, 255, 255, 255);
    }
    if let Some(ref m) = input_state.end_marker {
        let sp = camera.world_to_screen(m.world_pos);
        fill_circle(pixmap, sp[0], sp[1], 7.0, 255, 0, 170, 230);
        fill_circle(pixmap, sp[0], sp[1], 3.0, 255, 255, 255, 255);
    }
}

fn draw_debug(graph: &RoadGraph, camera: &Camera, planner: &PlannerState, pixmap: &mut Pixmap) {
    let mut y = 8i32;
    let x = 8i32;

    let status_str = match planner.status {
        PlannerStatus::Idle => "IDLE",
        PlannerStatus::Searching => "SEARCHING",
        PlannerStatus::FirstMatchFound => "FOUND",
        PlannerStatus::Done => "DONE",
    };

    let lines: [String; 6] = [
        format!("NODES: {}", graph.node_count()),
        format!("EDGES: {}", graph.edge_count()),
        format!("ZOOM: {:.4}", camera.zoom),
        format!("STATUS: {}", status_str),
        format!("EXPANDED: {}", planner.expanded_count),
        format!("PATH: {:.0}M", planner.locked_path_dist),
    ];

    for line in &lines {
        // Shadow.
        draw_text(pixmap, x + 1, y + 1, line, 0, 0, 0);
        // Text in light cyan.
        draw_text(pixmap, x, y, line, 180, 220, 255);
        y += 12;
    }
}
