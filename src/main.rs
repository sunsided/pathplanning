use std::num::NonZeroU32;
use std::rc::Rc;

use winit::event::{ElementState, Event, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ControlFlow, EventLoop};
use winit::window::WindowBuilder;

mod astar;
mod camera;
mod graph;
mod input;
mod lod;
mod osm_loader;
mod projection;
mod renderer;
mod spatial_index;
mod view_index;

use astar::{PlannerState, PlannerStatus};
use camera::Camera;
use input::{InputState, Marker, MarkerKind};
use lod::LodPyramid;
use spatial_index::SpatialIndex;
use view_index::RStarViewIndex;

fn main() {
    env_logger::init();

    // Parse CLI arguments.
    let pbf_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "maps/berlin.osm.pbf".to_string());

    // Load graph.
    let graph = match osm_loader::load_graph(&pbf_path) {
        Ok(g) => {
            log::info!(
                "Graph loaded: {} nodes, {} edges",
                g.node_count(),
                g.edge_count()
            );
            g
        }
        Err(e) => {
            log::error!("Failed to load '{}': {}", pbf_path, e);
            eprintln!("Error: could not load '{}': {}", pbf_path, e);
            std::process::exit(1);
        }
    };

    // Build spatial index.
    let spatial_index = SpatialIndex::build(&graph);
    let view_index = RStarViewIndex::build(&graph);
    let lod_pyramid = LodPyramid::build(&graph);
    log::info!("LOD pyramid built");

    // Initialise input and planner state.
    let mut input_state = InputState::new();
    let mut planner = PlannerState::new();

    // ---------------------------------------------------------------------------
    // Window / event loop
    // ---------------------------------------------------------------------------
    let event_loop = EventLoop::new().unwrap();
    let window = Rc::new(
        WindowBuilder::new()
            .with_title("Path Planning – OSM A*")
            .with_inner_size(winit::dpi::LogicalSize::new(1280u32, 720u32))
            .build(&event_loop)
            .expect("Failed to create window"),
    );

    // Initialise camera with real window size.
    let bbox = graph.bounding_box();
    let size = window.inner_size();
    let mut camera = Camera::new_from_bbox(bbox, size.width as f32, size.height as f32);

    let context = softbuffer::Context::new(window.clone()).expect("softbuffer context");
    let mut surface =
        softbuffer::Surface::new(&context, window.clone()).expect("softbuffer surface");

    event_loop
        .run(move |event, elwt| {
            elwt.set_control_flow(ControlFlow::Poll);

            match event {
                Event::WindowEvent { window_id, event } if window_id == window.id() => {
                    match event {
                        // -------------------------------------------------------
                        // Close / resize
                        // -------------------------------------------------------
                        WindowEvent::CloseRequested => elwt.exit(),

                        WindowEvent::Resized(size) => {
                            camera.screen_width = size.width as f32;
                            camera.screen_height = size.height as f32;
                        }

                        // -------------------------------------------------------
                        // Rendering
                        // -------------------------------------------------------
                        WindowEvent::RedrawRequested => {
                            let width = camera.screen_width as u32;
                            let height = camera.screen_height as u32;

                            if width == 0 || height == 0 {
                                return;
                            }

                            // Run incremental A* steps when searching.
                            if planner.status == PlannerStatus::Searching {
                                planner.step(&graph, 500);
                            }

                            // Build pixmap and render.
                            let mut pixmap = match tiny_skia::Pixmap::new(width, height) {
                                Some(p) => p,
                                None => return,
                            };

                            renderer::render(
                                &graph,
                                &camera,
                                &planner,
                                &input_state,
                                &view_index,
                                &lod_pyramid,
                                &mut pixmap,
                            );

                            // Present to screen via softbuffer.
                            if let (Some(w), Some(h)) =
                                (NonZeroU32::new(width), NonZeroU32::new(height))
                            {
                                if surface.resize(w, h).is_err() {
                                    return;
                                }
                                if let Ok(mut buf) = surface.buffer_mut() {
                                    // tiny-skia pixel layout: each PremultipliedColorU8
                                    // stores [r, g, b, a] at indices [0..3].
                                    // softbuffer expects 0x00RRGGBB as u32.
                                    for (dst, src) in buf.iter_mut().zip(pixmap.pixels()) {
                                        *dst = (src.red() as u32) << 16
                                            | (src.green() as u32) << 8
                                            | src.blue() as u32;
                                    }
                                    let _ = buf.present();
                                }
                            }
                        }

                        // -------------------------------------------------------
                        // Mouse movement
                        // -------------------------------------------------------
                        WindowEvent::CursorMoved { position, .. } => {
                            let new_pos = [position.x as f32, position.y as f32];
                            let old_pos = input_state.drag_screen_pos;
                            input_state.drag_screen_pos = new_pos;

                            if input_state.left_button_down {
                                // Detect drag threshold (5 px).
                                if !input_state.is_drag && input_state.drag_distance() > 5.0 {
                                    input_state.is_drag = true;
                                }

                                if input_state.is_drag {
                                    match input_state.dragging {
                                        Some(MarkerKind::Start) => {
                                            let wp = camera.screen_to_world(new_pos);
                                            if let Some(ref mut m) = input_state.start_marker {
                                                m.world_pos = wp;
                                                m.snapped_node = spatial_index.nearest_node(wp);
                                            }
                                        }
                                        Some(MarkerKind::End) => {
                                            let wp = camera.screen_to_world(new_pos);
                                            if let Some(ref mut m) = input_state.end_marker {
                                                m.world_pos = wp;
                                                m.snapped_node = spatial_index.nearest_node(wp);
                                            }
                                        }
                                        None => {
                                            // Pan the map.
                                            if let (Some(pan_start), Some(pan_center)) = (
                                                input_state.pan_start,
                                                input_state.pan_center_start,
                                            ) {
                                                let dx = (new_pos[0] - pan_start[0]) as f64
                                                    / camera.zoom;
                                                let dy = (new_pos[1] - pan_start[1]) as f64
                                                    / camera.zoom;
                                                camera.center =
                                                    [pan_center[0] - dx, pan_center[1] + dy];
                                            }
                                        }
                                    }
                                }
                            }

                            // Update dragged marker world pos continuously.
                            let _ = old_pos; // suppress unused warning
                        }

                        // -------------------------------------------------------
                        // Mouse buttons
                        // -------------------------------------------------------
                        WindowEvent::MouseInput { state, button, .. } => {
                            let screen_pos = input_state.drag_screen_pos;

                            match button {
                                MouseButton::Left => match state {
                                    ElementState::Pressed => {
                                        input_state.left_button_down = true;
                                        input_state.press_pos = screen_pos;
                                        input_state.is_drag = false;

                                        // Check if clicking near a marker.
                                        let near =
                                            input_state.marker_near(screen_pos, 12.0, &camera);
                                        if let Some(kind) = near {
                                            input_state.dragging = Some(kind);
                                        } else {
                                            // Start pan.
                                            input_state.pan_start = Some(screen_pos);
                                            input_state.pan_center_start = Some(camera.center);
                                        }
                                    }
                                    ElementState::Released => {
                                        input_state.left_button_down = false;

                                        if input_state.dragging.is_some() {
                                            // Finalize marker drag – re-snap.
                                            snap_and_trigger(
                                                &mut input_state,
                                                &mut planner,
                                                &spatial_index,
                                            );
                                            input_state.dragging = None;
                                        } else if !input_state.is_drag {
                                            // It was a plain click → place marker.
                                            let click_pos = input_state.press_pos;
                                            place_marker(
                                                &mut input_state,
                                                &mut planner,
                                                click_pos,
                                                &camera,
                                                &spatial_index,
                                            );
                                        }

                                        input_state.pan_start = None;
                                        input_state.pan_center_start = None;
                                        input_state.is_drag = false;
                                    }
                                },

                                MouseButton::Right => match state {
                                    ElementState::Pressed => {
                                        input_state.right_button_down = true;
                                        // Remove nearest marker.
                                        remove_nearest_marker(
                                            &mut input_state,
                                            &mut planner,
                                            screen_pos,
                                            &camera,
                                        );
                                    }
                                    ElementState::Released => {
                                        input_state.right_button_down = false;
                                    }
                                },

                                _ => {}
                            }
                        }

                        // -------------------------------------------------------
                        // Scroll wheel zoom
                        // -------------------------------------------------------
                        WindowEvent::MouseWheel { delta, .. } => {
                            let scroll_y: f64 = match delta {
                                MouseScrollDelta::LineDelta(_, y) => y as f64,
                                MouseScrollDelta::PixelDelta(pos) => pos.y * 0.01,
                            };
                            let factor = if scroll_y > 0.0 { 1.15 } else { 1.0 / 1.15 };
                            camera.zoom_around(input_state.drag_screen_pos, factor);
                        }

                        _ => {}
                    }
                }

                // Request a redraw every frame.
                Event::AboutToWait => {
                    window.request_redraw();
                }

                _ => {}
            }
        })
        .unwrap();
}

// ---------------------------------------------------------------------------
// Marker placement helpers
// ---------------------------------------------------------------------------

fn place_marker(
    input_state: &mut InputState,
    planner: &mut PlannerState,
    screen_pos: [f32; 2],
    camera: &Camera,
    spatial_index: &SpatialIndex,
) {
    let world_pos = camera.screen_to_world(screen_pos);
    let snapped = spatial_index.nearest_node(world_pos);
    let marker = Marker {
        world_pos,
        snapped_node: snapped,
    };

    match (&input_state.start_marker, &input_state.end_marker) {
        (None, _) => {
            input_state.start_marker = Some(marker);
        }
        (Some(_), None) => {
            input_state.end_marker = Some(marker);
        }
        (Some(s), Some(e)) => {
            // Replace nearest marker.
            let ss = camera.world_to_screen(s.world_pos);
            let es = camera.world_to_screen(e.world_pos);
            let ds = dist2(ss, screen_pos);
            let de = dist2(es, screen_pos);
            if ds <= de {
                input_state.start_marker = Some(marker);
            } else {
                input_state.end_marker = Some(marker);
            }
        }
    }

    try_start_search(input_state, planner);
}

fn remove_nearest_marker(
    input_state: &mut InputState,
    planner: &mut PlannerState,
    screen_pos: [f32; 2],
    camera: &Camera,
) {
    let ds = input_state.start_marker.as_ref().map(|m| {
        let sp = camera.world_to_screen(m.world_pos);
        dist2(sp, screen_pos)
    });
    let de = input_state.end_marker.as_ref().map(|m| {
        let sp = camera.world_to_screen(m.world_pos);
        dist2(sp, screen_pos)
    });

    match (ds, de) {
        (Some(ds), Some(de)) => {
            if ds <= de {
                input_state.start_marker = None;
            } else {
                input_state.end_marker = None;
            }
        }
        (Some(_), None) => input_state.start_marker = None,
        (None, Some(_)) => input_state.end_marker = None,
        (None, None) => {}
    }

    planner.reset();
}

fn snap_and_trigger(
    input_state: &mut InputState,
    planner: &mut PlannerState,
    spatial_index: &SpatialIndex,
) {
    if let Some(ref mut m) = input_state.start_marker {
        m.snapped_node = spatial_index.nearest_node(m.world_pos);
    }
    if let Some(ref mut m) = input_state.end_marker {
        m.snapped_node = spatial_index.nearest_node(m.world_pos);
    }
    try_start_search(input_state, planner);
}

fn try_start_search(input_state: &mut InputState, planner: &mut PlannerState) {
    let start_node = input_state
        .start_marker
        .as_ref()
        .and_then(|m| m.snapped_node);
    let end_node = input_state.end_marker.as_ref().and_then(|m| m.snapped_node);

    if let (Some(s), Some(e)) = (start_node, end_node) {
        if s != e {
            planner.start_search(s, e);
        }
    }
}

fn dist2(a: [f32; 2], b: [f32; 2]) -> f32 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    dx * dx + dy * dy
}
