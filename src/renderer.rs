use crate::astar::{PlannerState, PlannerStatus};
use crate::camera::Camera;
use crate::graph::{DecorationKind, RoadClass, RoadGraph};
use crate::input::InputState;
use crate::lod::LodPyramid;
use crate::view_index::ViewportIndex;
use std::fmt::Write;

use vello::kurbo::{Affine, BezPath, Circle, Stroke};
use vello::peniko::{Color, Fill};
use vello::Scene;

pub struct DebugOverlayState {
    pub line_buf: String,
}

impl Default for DebugOverlayState {
    fn default() -> Self {
        Self {
            line_buf: String::with_capacity(64),
        }
    }
}

struct ColorRGBA {
    r: u8,
    g: u8,
    b: u8,
    a: u8,
}

impl From<ColorRGBA> for Color {
    fn from(c: ColorRGBA) -> Self {
        Color::from_rgba8(c.r, c.g, c.b, c.a)
    }
}

fn rgba(r: u8, g: u8, b: u8, a: u8) -> Color {
    Color::from_rgba8(r, g, b, a)
}

// ---------------------------------------------------------------------------
// Minimal 8×8 bitmap font rendered as rects.
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
        b'a' => [0x00, 0x00, 0x3C, 0x06, 0x3E, 0x66, 0x3E, 0x00],
        b'b' => [0x60, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x7C, 0x00],
        b'c' => [0x00, 0x00, 0x3C, 0x60, 0x60, 0x60, 0x3C, 0x00],
        b'd' => [0x06, 0x06, 0x3E, 0x66, 0x66, 0x66, 0x3E, 0x00],
        b'e' => [0x00, 0x00, 0x3C, 0x66, 0x7E, 0x60, 0x3C, 0x00],
        b'f' => [0x1C, 0x30, 0x30, 0x7C, 0x30, 0x30, 0x30, 0x00],
        b'g' => [0x00, 0x00, 0x3E, 0x66, 0x66, 0x3E, 0x06, 0x3C],
        b'h' => [0x60, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x66, 0x00],
        b'i' => [0x18, 0x00, 0x38, 0x18, 0x18, 0x18, 0x3C, 0x00],
        b'j' => [0x06, 0x00, 0x06, 0x06, 0x06, 0x06, 0x66, 0x3C],
        b'k' => [0x60, 0x60, 0x6C, 0x78, 0x78, 0x6C, 0x66, 0x00],
        b'l' => [0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00],
        b'm' => [0x00, 0x00, 0x66, 0x7F, 0x7F, 0x6B, 0x63, 0x00],
        b'n' => [0x00, 0x00, 0x7C, 0x66, 0x66, 0x66, 0x66, 0x00],
        b'o' => [0x00, 0x00, 0x3C, 0x66, 0x66, 0x66, 0x3C, 0x00],
        b'p' => [0x00, 0x00, 0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60],
        b'q' => [0x00, 0x00, 0x3E, 0x66, 0x66, 0x3E, 0x06, 0x06],
        b'r' => [0x00, 0x00, 0x6C, 0x76, 0x60, 0x60, 0x60, 0x00],
        b's' => [0x00, 0x00, 0x3E, 0x60, 0x3C, 0x06, 0x7C, 0x00],
        b't' => [0x30, 0x30, 0x7C, 0x30, 0x30, 0x30, 0x1C, 0x00],
        b'u' => [0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x3E, 0x00],
        b'v' => [0x00, 0x00, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00],
        b'w' => [0x00, 0x00, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00],
        b'x' => [0x00, 0x00, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x00],
        b'y' => [0x00, 0x00, 0x66, 0x66, 0x66, 0x3E, 0x06, 0x3C],
        b'z' => [0x00, 0x00, 0x7E, 0x0C, 0x18, 0x30, 0x7E, 0x00],
        _ => [0x00; 8],
    }
}

fn draw_glyph(scene: &mut Scene, x: f32, y: f32, glyph: [u8; 8], color: Color) {
    use vello::kurbo::PathEl;
    let mut path = BezPath::new();
    for row in 0..8i32 {
        let byte = glyph[row as usize];
        for col in 0..8i32 {
            if byte & (0x80u8 >> col as u8) != 0 {
                let px = x as f64 + col as f64;
                let py = y as f64 + row as f64;
                path.push(PathEl::MoveTo(vello::kurbo::Point::new(px, py)));
                path.push(PathEl::LineTo(vello::kurbo::Point::new(px + 1.0, py)));
                path.push(PathEl::LineTo(vello::kurbo::Point::new(px + 1.0, py + 1.0)));
                path.push(PathEl::LineTo(vello::kurbo::Point::new(px, py + 1.0)));
                path.push(PathEl::ClosePath);
            }
        }
    }
    scene.fill(Fill::NonZero, Affine::IDENTITY, color, None, &path);
}

fn draw_text(scene: &mut Scene, x: f32, y: f32, text: &str, color: Color) {
    let mut cx = x;
    for c in text.bytes() {
        draw_glyph(scene, cx, y, get_glyph(c), color);
        cx += 9.0;
    }
}

// ---------------------------------------------------------------------------
// Scratch buffers reused across frames to avoid per-frame allocations
// ---------------------------------------------------------------------------

type FillStyle = (u8, u8, u8, u8);
type StrokeStyle = (u8, u8, u8, u8, f32);
type EdgeStyle = (u8, u8, u8, f32);

pub struct RenderScratch {
    pub edge_buf: Vec<usize>,
    pub decor_buf: Vec<usize>,
    pub screen_pts: Vec<[f32; 2]>,
}

impl Default for RenderScratch {
    fn default() -> Self {
        Self {
            edge_buf: Vec::with_capacity(4096),
            decor_buf: Vec::with_capacity(4096),
            screen_pts: Vec::with_capacity(256),
        }
    }
}

// ---------------------------------------------------------------------------
// Path helpers
// ---------------------------------------------------------------------------

fn screen_path(points: &[[f32; 2]], closed: bool) -> BezPath {
    let mut path = BezPath::new();
    if points.is_empty() {
        return path;
    }
    path.move_to(vello::kurbo::Point::new(
        points[0][0] as f64,
        points[0][1] as f64,
    ));
    for &p in &points[1..] {
        path.line_to(vello::kurbo::Point::new(p[0] as f64, p[1] as f64));
    }
    if closed {
        path.close_path();
    }
    path
}

// ---------------------------------------------------------------------------
// Public render entry point
// ---------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
pub fn render(
    graph: &RoadGraph,
    camera: &Camera,
    planner: &PlannerState,
    input_state: &InputState,
    view_index: &impl ViewportIndex,
    pyramid: &LodPyramid,
    scene: &mut Scene,
    debug_overlay: &mut DebugOverlayState,
    scratch: &mut RenderScratch,
    width: u32,
    _height: u32,
) {
    let (vmin_x, vmin_y, vmax_x, vmax_y) = camera.visible_world_aabb(32.0);

    let lod = pyramid.pick(camera.zoom);

    // Layer 0: decoration fills (buildings, landuse, plazas) — behind everything.
    draw_decorations_lod(lod, camera, vmin_x, vmin_y, vmax_x, vmax_y, scene, scratch);

    // Layer 1: all road edges (static).
    draw_all_edges_lod(lod, camera, vmin_x, vmin_y, vmax_x, vmax_y, scene, scratch);

    // Layer 2: explored (closed-set) edges.
    draw_explored_edges(
        graph,
        camera,
        planner,
        view_index,
        vmin_x,
        vmin_y,
        vmax_x,
        vmax_y,
        scene,
        &mut scratch.screen_pts,
    );

    // Layer 3: frontier (open-set) nodes.
    draw_frontier(graph, camera, planner, scene, width);

    // Layer 4: current best path (neon cyan).
    if let Some(ref path) = planner.best_path {
        draw_node_path(
            graph,
            camera,
            path,
            scene,
            ColorRGBA {
                r: 0,
                g: 255,
                b: 255,
                a: 220,
            },
            2.5,
        );
    }

    // Layer 5: locked path (electric blue).
    if let Some(ref path) = planner.locked_path {
        draw_node_path(
            graph,
            camera,
            path,
            scene,
            ColorRGBA {
                r: 0,
                g: 128,
                b: 255,
                a: 255,
            },
            3.0,
        );
    }

    // Layer 6: markers.
    draw_markers(camera, input_state, scene);

    // Layer 7: debug overlay.
    draw_debug(graph, camera, planner, pyramid, scene, debug_overlay);
}

fn decor_fill_style(kind: DecorationKind) -> (u8, u8, u8, u8) {
    match kind {
        DecorationKind::Building => (26, 31, 46, 180),
        DecorationKind::Landuse => (20, 24, 34, 160),
        DecorationKind::PedestrianArea => (28, 30, 40, 150),
        DecorationKind::ServiceArea => (20, 24, 34, 140),
        DecorationKind::Water => (22, 58, 110, 220),
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_decorations_lod(
    lod: &crate::lod::LodLevel,
    camera: &Camera,
    vmin_x: f64,
    vmin_y: f64,
    vmax_x: f64,
    vmax_y: f64,
    scene: &mut Scene,
    scratch: &mut RenderScratch,
) {
    if lod.decorations.is_empty() {
        return;
    }

    lod.decor_index
        .decorations_in_into(vmin_x, vmin_y, vmax_x, vmax_y, &mut scratch.decor_buf);

    let zoom = camera.zoom;
    let min_screen_dim = 0.3;

    let mut fill_paths: Vec<(FillStyle, BezPath)> = Vec::new();
    let mut stroke_paths: Vec<(StrokeStyle, BezPath)> = Vec::new();

    for &idx in scratch.decor_buf.iter() {
        let decor = &lod.decorations[idx];
        let a = &decor.aabb;
        let aw = (a[2] - a[0]) * zoom;
        let ah = (a[3] - a[1]) * zoom;
        if aw.max(ah) < min_screen_dim {
            continue;
        }

        let pts = &decor.polyline_world;
        if pts.len() < 2 {
            continue;
        }

        scratch.screen_pts.clear();
        let mut smin_x = f32::INFINITY;
        let mut smin_y = f32::INFINITY;
        let mut smax_x = f32::NEG_INFINITY;
        let mut smax_y = f32::NEG_INFINITY;

        for &wp in pts {
            let sp = camera.world_to_screen(wp);
            scratch.screen_pts.push(sp);
            smin_x = smin_x.min(sp[0]);
            smin_y = smin_y.min(sp[1]);
            smax_x = smax_x.max(sp[0]);
            smax_y = smax_y.max(sp[1]);
        }

        let is_too_small = (smax_x - smin_x) < 0.5 && (smax_y - smin_y) < 0.5;
        if decor.closed && is_too_small {
            continue;
        }

        let fill_style = decor_fill_style(decor.kind);

        if decor.closed {
            let path = screen_path(&scratch.screen_pts, true);
            let fill_entry = fill_paths.iter_mut().find(|(s, _)| *s == fill_style);
            if let Some((_, existing)) = fill_entry {
                existing.extend(path.iter());
            } else {
                fill_paths.push((fill_style, path));
            }

            if decor.kind == DecorationKind::Building {
                let stroke_style = (36u8, 42u8, 58u8, 200u8, 0.8f32);
                let stroke_entry = stroke_paths.iter_mut().find(|(s, _)| *s == stroke_style);
                let stroke_path = screen_path(&scratch.screen_pts, true);
                if let Some((_, existing)) = stroke_entry {
                    existing.extend(stroke_path.iter());
                } else {
                    stroke_paths.push((stroke_style, stroke_path));
                }
            }
        } else if !is_too_small {
            let stroke_width = if decor.kind == DecorationKind::Water {
                1.6f32
            } else {
                0.8f32
            };
            let stroke_style = (
                fill_style.0,
                fill_style.1,
                fill_style.2,
                fill_style.3,
                stroke_width,
            );
            let stroke_entry = stroke_paths.iter_mut().find(|(s, _)| *s == stroke_style);
            let stroke_path = screen_path(&scratch.screen_pts, false);
            if let Some((_, existing)) = stroke_entry {
                existing.extend(stroke_path.iter());
            } else {
                stroke_paths.push((stroke_style, stroke_path));
            }
        }
    }

    for ((r, g, b, a), path) in fill_paths {
        scene.fill(
            Fill::EvenOdd,
            Affine::IDENTITY,
            rgba(r, g, b, a),
            None,
            &path,
        );
    }

    for ((r, g, b, a, width), path) in stroke_paths {
        scene.stroke(
            &Stroke::new(width as f64),
            Affine::IDENTITY,
            rgba(r, g, b, a),
            None,
            &path,
        );
    }
}

fn edge_style(rc: RoadClass) -> EdgeStyle {
    match rc {
        RoadClass::Motorway => (40, 60, 100, 3.0),
        RoadClass::Primary => (30, 55, 90, 2.5),
        RoadClass::Secondary => (26, 58, 92, 2.0),
        RoadClass::Tertiary => (26, 58, 92, 1.5),
        RoadClass::Residential => (26, 58, 92, 1.2),
        RoadClass::Service => (26, 58, 92, 1.0),
        RoadClass::Path => (26, 58, 92, 0.8),
        RoadClass::Other => (26, 58, 92, 0.8),
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_all_edges_lod(
    lod: &crate::lod::LodLevel,
    camera: &Camera,
    vmin_x: f64,
    vmin_y: f64,
    vmax_x: f64,
    vmax_y: f64,
    scene: &mut Scene,
    scratch: &mut RenderScratch,
) {
    if lod.edges.is_empty() {
        return;
    }

    lod.edge_index
        .edges_in_into(vmin_x, vmin_y, vmax_x, vmax_y, &mut scratch.edge_buf);

    let zoom = camera.zoom;
    let min_screen_dim = 0.3;

    let mut paths_by_style: Vec<(EdgeStyle, BezPath)> = Vec::new();

    for &idx in scratch.edge_buf.iter() {
        let edge = &lod.edges[idx];
        let a = &edge.aabb;
        let aw = (a[2] - a[0]) * zoom;
        let ah = (a[3] - a[1]) * zoom;
        if aw.max(ah) < min_screen_dim {
            continue;
        }

        let style = edge_style(edge.road_class);

        scratch.screen_pts.clear();
        let mut min_x = f32::INFINITY;
        let mut min_y = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut max_y = f32::NEG_INFINITY;

        let pts = &edge.polyline_world;
        if pts.len() < 2 {
            continue;
        }

        for &wp in pts {
            let sp = camera.world_to_screen(wp);
            scratch.screen_pts.push(sp);
            min_x = min_x.min(sp[0]);
            min_y = min_y.min(sp[1]);
            max_x = max_x.max(sp[0]);
            max_y = max_y.max(sp[1]);
        }

        if (max_x - min_x) < 0.5 && (max_y - min_y) < 0.5 {
            continue;
        }

        let path = screen_path(&scratch.screen_pts, false);
        let entry = paths_by_style.iter_mut().find(|(s, _)| *s == style);

        if let Some((_, existing)) = entry {
            existing.extend(path.iter());
        } else {
            paths_by_style.push((style, path));
        }
    }

    for ((r, g, b, width), path) in paths_by_style {
        scene.stroke(
            &Stroke::new(width as f64),
            Affine::IDENTITY,
            rgba(r, g, b, 200),
            None,
            &path,
        );
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_explored_edges(
    graph: &RoadGraph,
    camera: &Camera,
    planner: &PlannerState,
    view_index: &impl ViewportIndex,
    vmin_x: f64,
    vmin_y: f64,
    vmax_x: f64,
    vmax_y: f64,
    scene: &mut Scene,
    screen_pts: &mut Vec<[f32; 2]>,
) {
    if planner.explored.is_empty() {
        return;
    }
    let visible_ids = view_index.edges_in(vmin_x, vmin_y, vmax_x, vmax_y);

    let mut path = BezPath::new();
    let mut started = false;

    for &idx in &visible_ids {
        let edge = &graph.edges[idx];
        if !planner.explored.contains(&edge.from) || !planner.explored.contains(&edge.to) {
            continue;
        }

        let pts = &edge.polyline_world;
        if pts.len() < 2 {
            continue;
        }

        screen_pts.clear();
        let mut smin_x = f32::INFINITY;
        let mut smin_y = f32::INFINITY;
        let mut smax_x = f32::NEG_INFINITY;
        let mut smax_y = f32::NEG_INFINITY;

        for &wp in pts {
            let sp = camera.world_to_screen(wp);
            screen_pts.push(sp);
            smin_x = smin_x.min(sp[0]);
            smin_y = smin_y.min(sp[1]);
            smax_x = smax_x.max(sp[0]);
            smax_y = smax_y.max(sp[1]);
        }

        if (smax_x - smin_x) < 0.5 && (smax_y - smin_y) < 0.5 {
            continue;
        }

        path.move_to(vello::kurbo::Point::new(
            screen_pts[0][0] as f64,
            screen_pts[0][1] as f64,
        ));
        for &sp in &screen_pts[1..] {
            path.line_to(vello::kurbo::Point::new(sp[0] as f64, sp[1] as f64));
        }
        started = true;
    }

    if started {
        scene.stroke(
            &Stroke::new(1.2),
            Affine::IDENTITY,
            rgba(30, 77, 122, 180),
            None,
            &path,
        );
    }
}

fn draw_frontier(
    graph: &RoadGraph,
    camera: &Camera,
    planner: &PlannerState,
    scene: &mut Scene,
    width: u32,
) {
    let pw = width as f32;
    let radius = 3.0;
    let color = Color::from_rgba8(0, 212, 255, 200);
    for &node_id in &planner.frontier {
        if node_id >= graph.nodes.len() {
            continue;
        }
        let sp = camera.world_to_screen(graph.nodes[node_id].world_pos);
        if sp[0] < -radius || sp[0] > pw + radius {
            continue;
        }
        scene.fill(
            Fill::NonZero,
            Affine::IDENTITY,
            color,
            None,
            &Circle::new((sp[0] as f64, sp[1] as f64), 2.5),
        );
    }
}

fn draw_node_path(
    graph: &RoadGraph,
    camera: &Camera,
    path: &[usize],
    scene: &mut Scene,
    color: ColorRGBA,
    width: f32,
) {
    if path.len() < 2 {
        return;
    }
    let n = graph.nodes.len();
    let mut bez = BezPath::new();
    let mut started = false;
    for &nid in path {
        if nid >= n {
            continue;
        }
        let sp = camera.world_to_screen(graph.nodes[nid].world_pos);
        if !started {
            bez.move_to(vello::kurbo::Point::new(sp[0] as f64, sp[1] as f64));
            started = true;
        } else {
            bez.line_to(vello::kurbo::Point::new(sp[0] as f64, sp[1] as f64));
        }
    }
    scene.stroke(
        &Stroke::new(width as f64),
        Affine::IDENTITY,
        Color::from(color),
        None,
        &bez,
    );
}

fn draw_markers(camera: &Camera, input_state: &InputState, scene: &mut Scene) {
    if let Some(ref m) = input_state.start_marker {
        let sp = camera.world_to_screen(m.world_pos);
        scene.fill(
            Fill::NonZero,
            Affine::IDENTITY,
            rgba(0, 255, 128, 230),
            None,
            &Circle::new((sp[0] as f64, sp[1] as f64), 7.0),
        );
        scene.fill(
            Fill::NonZero,
            Affine::IDENTITY,
            rgba(255, 255, 255, 255),
            None,
            &Circle::new((sp[0] as f64, sp[1] as f64), 3.0),
        );
    }
    if let Some(ref m) = input_state.end_marker {
        let sp = camera.world_to_screen(m.world_pos);
        scene.fill(
            Fill::NonZero,
            Affine::IDENTITY,
            rgba(255, 0, 170, 230),
            None,
            &Circle::new((sp[0] as f64, sp[1] as f64), 7.0),
        );
        scene.fill(
            Fill::NonZero,
            Affine::IDENTITY,
            rgba(255, 255, 255, 255),
            None,
            &Circle::new((sp[0] as f64, sp[1] as f64), 3.0),
        );
    }
}

fn draw_debug(
    graph: &RoadGraph,
    camera: &Camera,
    planner: &PlannerState,
    pyramid: &LodPyramid,
    scene: &mut Scene,
    state: &mut DebugOverlayState,
) {
    let mut y = 8f32;
    let x = 8f32;

    let status_str = match planner.status {
        PlannerStatus::Idle => "IDLE",
        PlannerStatus::Searching => "SEARCHING",
        PlannerStatus::FirstMatchFound => "FOUND",
        PlannerStatus::Done => "DONE",
    };

    let lod = pyramid.pick(camera.zoom);
    let lod_label = if camera.zoom >= 0.5 {
        "L0"
    } else if camera.zoom >= 0.05 {
        "L1"
    } else {
        "L2"
    };

    let buf = &mut state.line_buf;

    let text_color = rgba(180, 220, 255, 255);
    let shadow_color = rgba(0, 0, 0, 255);

    macro_rules! emit_line {
        ($($arg:tt)*) => {{
            buf.clear();
            let _ = write!(buf, $($arg)*);
            draw_text(scene, x + 1.0, y + 1.0, buf.as_str(), shadow_color);
            draw_text(scene, x, y, buf.as_str(), text_color);
            y += 12.0;
            let _ = y;
        }};
    }

    emit_line!("NODES: {}", graph.node_count());
    emit_line!("EDGES: {}", graph.edge_count());
    emit_line!("DECOR: {}", graph.decorations.shapes.len());
    emit_line!("ZOOM: {:.4}", camera.zoom);
    emit_line!("LOD: {}", lod_label);
    emit_line!("EDGES_LOD: {}", lod.edges.len());
    emit_line!("DECOR_LOD: {}", lod.decorations.len());
    emit_line!("STATUS: {}", status_str);
    emit_line!("EXPANDED: {}", planner.expanded_count);
    emit_line!("PATH: {:.0}M", planner.locked_path_dist);
}
