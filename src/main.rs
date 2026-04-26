use std::num::NonZeroUsize;
use std::sync::Arc;

use winit::application::ApplicationHandler;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::EventLoop;
use winit::keyboard::Key;
use winit::window::Window;

mod camera;
mod graph;
mod input;
mod lod;
mod osm_loader;
mod planner;
mod projection;
mod renderer;
mod spatial_index;
mod view_index;

use camera::Camera;
use input::{InputState, Marker, MarkerKind};
use lod::LodPyramid;
use planner::{Algorithm, Heuristic, PlannerState, PlannerStatus};
use renderer::{DebugOverlayState, MenuItemKind, RenderScratch};
use spatial_index::SpatialIndex;
use view_index::RStarViewIndex;

use vello::util::{RenderContext, RenderSurface};
use vello::wgpu::{self, PresentMode};
use vello::{AaConfig, AaSupport, Renderer, RendererOptions, Scene};

// ---------------------------------------------------------------------------
// App state
// ---------------------------------------------------------------------------

struct RenderState {
    surface: RenderSurface<'static>,
    renderer: Renderer,
    scene: Scene,
}

struct App {
    // Loaded once, shared via Arc.
    graph: Arc<graph::RoadGraph>,
    spatial_index: SpatialIndex,
    view_index: RStarViewIndex,
    lod_pyramid: LodPyramid,

    // Mutable per-frame state.
    camera: Camera,
    input_state: InputState,
    planner: PlannerState,
    debug_overlay: DebugOverlayState,
    render_scratch: RenderScratch,
    needs_redraw: bool,

    // Vello / wgpu.
    render_context: RenderContext,
    render_state: Option<RenderState>,
    window: Option<Arc<Window>>,
}

impl App {
    fn new(pbf_path: &str) -> Self {
        let graph = Arc::new(osm_loader::load_graph(pbf_path).unwrap_or_else(|e| {
            log::error!("Failed to load '{}': {}", pbf_path, e);
            eprintln!("Error: could not load '{}': {}", pbf_path, e);
            std::process::exit(1);
        }));
        log::info!(
            "Graph loaded: {} nodes, {} edges",
            graph.node_count(),
            graph.edge_count()
        );

        let spatial_index = SpatialIndex::build(&graph);
        let view_index = RStarViewIndex::build(&graph);
        let lod_pyramid = LodPyramid::build(&graph);
        log::info!("LOD pyramid built");

        let input_state = InputState::new();
        let planner = PlannerState::new();
        let debug_overlay = DebugOverlayState::default();
        let render_scratch = RenderScratch::default();

        let camera = Camera::new([0.0, 0.0], 0.05, 1280.0, 720.0);

        let render_context = RenderContext::new();

        Self {
            graph,
            spatial_index,
            view_index,
            lod_pyramid,
            camera,
            input_state,
            planner,
            debug_overlay,
            render_scratch,
            needs_redraw: true,
            render_context,
            render_state: None,
            window: None,
        }
    }

    fn init_surface(&mut self, window: Arc<Window>, width: u32, height: u32) {
        camera_init(&mut self.camera, &self.graph, width, height);

        let present_mode = PresentMode::AutoVsync;
        let surface = pollster::block_on(self.render_context.create_surface(
            window.clone(),
            width,
            height,
            present_mode,
        ))
        .expect("Failed to create surface");

        let device_handle = &self.render_context.devices[surface.dev_id];
        let renderer = Renderer::new(
            &device_handle.device,
            RendererOptions {
                use_cpu: false,
                antialiasing_support: AaSupport::all(),
                num_init_threads: NonZeroUsize::new(1),
                pipeline_cache: None,
            },
        )
        .expect("Failed to create renderer");

        self.render_state = Some(RenderState {
            surface,
            renderer,
            scene: Scene::new(),
        });
        self.window = Some(window);
        self.needs_redraw = true;
    }

    fn render_frame(&mut self) {
        let Some(render_state) = self.render_state.as_mut() else {
            return;
        };
        let Some(window) = self.window.as_ref() else {
            return;
        };

        let width = render_state.surface.config.width;
        let height = render_state.surface.config.height;
        if width == 0 || height == 0 {
            return;
        }

        let device_handle = &self.render_context.devices[render_state.surface.dev_id];
        let device = &device_handle.device;
        let queue = &device_handle.queue;

        render_state.scene.reset();

        renderer::render(
            &self.graph,
            &self.camera,
            &self.planner,
            &self.input_state,
            &self.view_index,
            &self.lod_pyramid,
            &mut render_state.scene,
            &mut self.debug_overlay,
            &mut self.render_scratch,
            width,
            height,
        );

        let params = vello::RenderParams {
            base_color: vello::peniko::Color::from_rgba8(10, 14, 26, 255),
            width,
            height,
            antialiasing_method: AaConfig::Msaa16,
        };

        render_state
            .renderer
            .render_to_texture(
                device,
                queue,
                &render_state.scene,
                &render_state.surface.target_view,
                &params,
            )
            .expect("Failed to render to texture");

        let surface_texture = match render_state.surface.surface.get_current_texture() {
            Ok(tex) => tex,
            Err(vello::wgpu::SurfaceError::Timeout | vello::wgpu::SurfaceError::Outdated) => {
                window.request_redraw();
                return;
            }
            Err(vello::wgpu::SurfaceError::Lost) => {
                panic!("Surface lost");
            }
            Err(vello::wgpu::SurfaceError::OutOfMemory) => {
                panic!("Out of video memory");
            }
            Err(vello::wgpu::SurfaceError::Other) => {
                window.request_redraw();
                return;
            }
        };

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("blit"),
        });
        render_state.surface.blitter.copy(
            device,
            &mut encoder,
            &render_state.surface.target_view,
            &surface_texture
                .texture
                .create_view(&wgpu::TextureViewDescriptor::default()),
        );
        queue.submit([encoder.finish()]);
        surface_texture.present();

        device.poll(wgpu::PollType::Poll).unwrap();

        self.needs_redraw = false;
    }

    fn resize(&mut self, width: u32, height: u32) {
        if let Some(render_state) = self.render_state.as_mut() {
            self.render_context
                .resize_surface(&mut render_state.surface, width, height);
        }
        self.camera.screen_width = width as f32;
        self.camera.screen_height = height as f32;
        self.needs_redraw = true;
    }
}

fn camera_init(camera: &mut Camera, graph: &graph::RoadGraph, width: u32, height: u32) {
    let bbox = graph.bounding_box();
    *camera = Camera::new_from_bbox(bbox, width as f32, height as f32);
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &winit::event_loop::ActiveEventLoop) {
        if self.render_state.is_some() {
            return;
        }
        let window = Arc::new(
            event_loop
                .create_window(
                    Window::default_attributes()
                        .with_title("Path Planning – OSM A*")
                        .with_inner_size(winit::dpi::LogicalSize::new(1280u32, 720u32)),
                )
                .expect("Failed to create window"),
        );
        let size = window.inner_size();
        self.init_surface(window, size.width, size.height);
    }

    fn window_event(
        &mut self,
        _event_loop: &winit::event_loop::ActiveEventLoop,
        _window_id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => {
                _event_loop.exit();
            }

            WindowEvent::Resized(size) => {
                let w = size.width.max(1);
                let h = size.height.max(1);
                self.resize(w, h);
            }

            WindowEvent::RedrawRequested => {
                if matches!(
                    self.planner.status,
                    PlannerStatus::Searching | PlannerStatus::FirstMatchFound
                ) {
                    self.planner.step(&self.graph, 500);
                }
                self.render_frame();
            }

            WindowEvent::CursorMoved { position, .. } => {
                let new_pos = [position.x as f32, position.y as f32];
                self.input_state.drag_screen_pos = new_pos;

                if !self.input_state.left_button_down {
                    self.input_state.hover_menu_item =
                        self.debug_overlay.menu_layout.hit_test(new_pos);
                    self.needs_redraw = true;
                }

                if self.input_state.left_button_down {
                    if !self.input_state.is_drag && self.input_state.drag_distance() > 5.0 {
                        self.input_state.is_drag = true;
                    }

                    if self.input_state.is_drag {
                        match self.input_state.dragging {
                            Some(MarkerKind::Start) => {
                                let wp = self.camera.screen_to_world(new_pos);
                                if let Some(ref mut m) = self.input_state.start_marker {
                                    m.world_pos = wp;
                                    m.snapped_node = self.spatial_index.nearest_node(wp);
                                    if let Some(id) = m.snapped_node {
                                        m.world_pos = self.graph.nodes[id].world_pos;
                                    }
                                }
                                self.needs_redraw = true;
                            }
                            Some(MarkerKind::End) => {
                                let wp = self.camera.screen_to_world(new_pos);
                                if let Some(ref mut m) = self.input_state.end_marker {
                                    m.world_pos = wp;
                                    m.snapped_node = self.spatial_index.nearest_node(wp);
                                    if let Some(id) = m.snapped_node {
                                        m.world_pos = self.graph.nodes[id].world_pos;
                                    }
                                }
                                self.needs_redraw = true;
                            }
                            None => {
                                if let (Some(pan_start), Some(pan_center)) = (
                                    self.input_state.pan_start,
                                    self.input_state.pan_center_start,
                                ) {
                                    let dx = (new_pos[0] - pan_start[0]) as f64 / self.camera.zoom;
                                    let dy = (new_pos[1] - pan_start[1]) as f64 / self.camera.zoom;
                                    self.camera.center = [pan_center[0] - dx, pan_center[1] + dy];
                                    self.needs_redraw = true;
                                }
                            }
                        }
                    }
                }
            }

            WindowEvent::MouseInput { state, button, .. } => {
                let screen_pos = self.input_state.drag_screen_pos;

                match button {
                    MouseButton::Left => match state {
                        ElementState::Pressed => {
                            self.input_state.left_button_down = true;
                            self.input_state.press_pos = screen_pos;
                            self.input_state.is_drag = false;

                            let hud_hit = self.debug_overlay.menu_layout.hit_test(screen_pos);
                            let hud_panel =
                                self.debug_overlay.menu_layout.hit_test_panel(screen_pos);

                            if hud_hit.is_some() {
                                self.input_state.pressed_menu_item = hud_hit;
                            } else if hud_panel {
                                // Inside panel but not on an item — consume to prevent pan/marker
                            } else {
                                let near =
                                    self.input_state.marker_near(screen_pos, 12.0, &self.camera);
                                if let Some(kind) = near {
                                    self.input_state.dragging = Some(kind);
                                } else {
                                    self.input_state.pan_start = Some(screen_pos);
                                    self.input_state.pan_center_start = Some(self.camera.center);
                                }
                            }
                        }
                        ElementState::Released => {
                            self.input_state.left_button_down = false;

                            if let Some(pressed) = self.input_state.pressed_menu_item.take() {
                                let released = self.debug_overlay.menu_layout.hit_test(screen_pos);
                                if released == Some(pressed) {
                                    apply_menu_choice(
                                        pressed,
                                        &mut self.planner,
                                        &mut self.input_state,
                                        &self.graph,
                                        &self.camera,
                                    );
                                }
                                self.needs_redraw = true;
                            } else if self.input_state.dragging.is_some() {
                                if snap_and_trigger(
                                    &mut self.input_state,
                                    &mut self.planner,
                                    &self.spatial_index,
                                    &self.graph,
                                ) {
                                    self.needs_redraw = true;
                                }
                                self.input_state.dragging = None;
                            } else if !self.input_state.is_drag {
                                let click_pos = self.input_state.press_pos;
                                let _started_search = place_marker(
                                    &mut self.input_state,
                                    &mut self.planner,
                                    click_pos,
                                    &self.camera,
                                    &self.spatial_index,
                                    &self.graph,
                                );
                                self.needs_redraw = true;
                            }

                            self.input_state.pan_start = None;
                            self.input_state.pan_center_start = None;
                            self.input_state.is_drag = false;
                        }
                    },

                    MouseButton::Right => match state {
                        ElementState::Pressed => {
                            self.input_state.right_button_down = true;
                            if remove_nearest_marker(
                                &mut self.input_state,
                                &mut self.planner,
                                screen_pos,
                                &self.camera,
                            ) {
                                self.needs_redraw = true;
                            }
                        }
                        ElementState::Released => {
                            self.input_state.right_button_down = false;
                        }
                    },

                    _ => {}
                }
            }

            WindowEvent::MouseWheel { delta, .. } => {
                let scroll_y: f64 = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y as f64,
                    MouseScrollDelta::PixelDelta(pos) => pos.y * 0.01,
                };
                let factor = if scroll_y > 0.0 {
                    1.15
                } else if scroll_y < 0.0 {
                    1.0 / 1.15
                } else {
                    1.0
                };
                self.camera
                    .zoom_around(self.input_state.drag_screen_pos, factor);
                self.needs_redraw = true;
            }

            WindowEvent::KeyboardInput {
                event,
                is_synthetic: false,
                ..
            } if event.state == ElementState::Pressed => {
                let handled = handle_keyboard_input(
                    &event.logical_key,
                    &mut self.planner,
                    &mut self.input_state,
                    &self.graph,
                    &self.camera,
                );
                if handled {
                    self.needs_redraw = true;
                }
            }

            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &winit::event_loop::ActiveEventLoop) {
        let planner_animating = matches!(
            self.planner.status,
            PlannerStatus::Searching | PlannerStatus::FirstMatchFound
        );
        if self.needs_redraw || planner_animating {
            if let Some(window) = self.window.as_ref() {
                window.request_redraw();
            }
        }
    }
}

fn main() {
    env_logger::init();

    let pbf_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "maps/berlin.osm.pbf".to_string());

    let mut app = App::new(&pbf_path);

    let event_loop = EventLoop::new().unwrap();
    event_loop.run_app(&mut app).unwrap();
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
    graph: &graph::RoadGraph,
) -> bool {
    let world_pos = camera.screen_to_world(screen_pos);
    let snapped = spatial_index.nearest_node(world_pos);
    let world_pos = match snapped {
        Some(id) => graph.nodes[id].world_pos,
        None => world_pos,
    };
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

    try_start_search(input_state, planner)
}

fn remove_nearest_marker(
    input_state: &mut InputState,
    planner: &mut PlannerState,
    screen_pos: [f32; 2],
    camera: &Camera,
) -> bool {
    let ds = input_state.start_marker.as_ref().map(|m| {
        let sp = camera.world_to_screen(m.world_pos);
        dist2(sp, screen_pos)
    });
    let de = input_state.end_marker.as_ref().map(|m| {
        let sp = camera.world_to_screen(m.world_pos);
        dist2(sp, screen_pos)
    });

    let had_marker = input_state.start_marker.is_some() || input_state.end_marker.is_some();

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
    had_marker
}

fn snap_and_trigger(
    input_state: &mut InputState,
    planner: &mut PlannerState,
    spatial_index: &SpatialIndex,
    graph: &graph::RoadGraph,
) -> bool {
    if let Some(ref mut m) = input_state.start_marker {
        m.snapped_node = spatial_index.nearest_node(m.world_pos);
        if let Some(id) = m.snapped_node {
            m.world_pos = graph.nodes[id].world_pos;
        }
    }
    if let Some(ref mut m) = input_state.end_marker {
        m.snapped_node = spatial_index.nearest_node(m.world_pos);
        if let Some(id) = m.snapped_node {
            m.world_pos = graph.nodes[id].world_pos;
        }
    }
    try_start_search(input_state, planner)
}

fn try_start_search(input_state: &mut InputState, planner: &mut PlannerState) -> bool {
    let start_node = input_state
        .start_marker
        .as_ref()
        .and_then(|m| m.snapped_node);
    let end_node = input_state.end_marker.as_ref().and_then(|m| m.snapped_node);

    if let (Some(s), Some(e)) = (start_node, end_node) {
        if s != e {
            planner.start_search(s, e);
            return true;
        }
    }
    false
}

fn dist2(a: [f32; 2], b: [f32; 2]) -> f32 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    dx * dx + dy * dy
}

fn apply_menu_choice(
    item: MenuItemKind,
    planner: &mut PlannerState,
    input_state: &mut InputState,
    graph: &graph::RoadGraph,
    camera: &Camera,
) {
    let changed = match item {
        MenuItemKind::Algorithm(a) => {
            if planner.config.algorithm != a {
                planner.config.algorithm = a;
                true
            } else {
                false
            }
        }
        MenuItemKind::Heuristic(h) => {
            if planner.config.heuristic != h {
                planner.config.heuristic = h;
                true
            } else {
                false
            }
        }
        MenuItemKind::Random => randomize_route(input_state, graph, camera),
    };

    if !changed {
        return;
    }

    let start_node = input_state
        .start_marker
        .as_ref()
        .and_then(|m| m.snapped_node);
    let end_node = input_state.end_marker.as_ref().and_then(|m| m.snapped_node);

    if let (Some(s), Some(e)) = (start_node, end_node) {
        if s != e {
            planner.start_search(s, e);
        }
    } else {
        planner.reset();
    }
}

fn randomize_route(
    input_state: &mut InputState,
    graph: &graph::RoadGraph,
    camera: &Camera,
) -> bool {
    let (vmin_x, vmin_y, vmax_x, vmax_y) = camera.visible_world_aabb(0.0);

    let routable: Vec<usize> = graph
        .adjacency
        .iter()
        .enumerate()
        .filter(|(id, edges)| {
            if edges.is_empty() {
                return false;
            }
            let pos = graph.nodes[*id].world_pos;
            pos[0] >= vmin_x && pos[0] <= vmax_x && pos[1] >= vmin_y && pos[1] <= vmax_y
        })
        .map(|(id, _)| id)
        .collect();

    if routable.len() < 2 {
        return false;
    }

    let mut seed = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64;
    let mut rng = |max: usize| -> usize {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed as usize) % max
    };

    let start_idx = rng(routable.len());
    let mut end_idx = rng(routable.len());
    while end_idx == start_idx {
        end_idx = rng(routable.len());
    }

    let start_node = routable[start_idx];
    let end_node = routable[end_idx];

    input_state.start_marker = Some(Marker {
        world_pos: graph.nodes[start_node].world_pos,
        snapped_node: Some(start_node),
    });
    input_state.end_marker = Some(Marker {
        world_pos: graph.nodes[end_node].world_pos,
        snapped_node: Some(end_node),
    });

    true
}

fn handle_keyboard_input(
    key: &Key,
    planner: &mut PlannerState,
    input_state: &mut InputState,
    graph: &graph::RoadGraph,
    camera: &Camera,
) -> bool {
    let mut changed = false;

    match key {
        Key::Character(c) if c.as_str() == "p" || c.as_str() == "P" => {
            let algs = Algorithm::all();
            let idx = algs
                .iter()
                .position(|&a| a == planner.config.algorithm)
                .unwrap_or(0);
            let new_idx = if c.as_str() == "P" {
                (idx + algs.len() - 1) % algs.len()
            } else {
                (idx + 1) % algs.len()
            };
            planner.config.algorithm = algs[new_idx];
            changed = true;
        }
        Key::Character(c)
            if (c.as_str() == "h" || c.as_str() == "H")
                && planner.config.algorithm.uses_heuristic() =>
        {
            let heurs = Heuristic::all();
            let idx = heurs
                .iter()
                .position(|&h| h == planner.config.heuristic)
                .unwrap_or(0);
            let new_idx = if c.as_str() == "H" {
                (idx + heurs.len() - 1) % heurs.len()
            } else {
                (idx + 1) % heurs.len()
            };
            planner.config.heuristic = heurs[new_idx];
            changed = true;
        }
        Key::Character(c) if c.as_str() == "1" => {
            planner.config.algorithm = Algorithm::AStar;
            changed = true;
        }
        Key::Character(c) if c.as_str() == "2" => {
            planner.config.algorithm = Algorithm::Dijkstra;
            changed = true;
        }
        Key::Character(c) if c.as_str() == "3" => {
            planner.config.algorithm = Algorithm::GreedyBestFirst;
            changed = true;
        }
        Key::Character(c) if c.as_str() == "r" => {
            changed = randomize_route(input_state, graph, camera);
        }
        Key::Character(c) if c.as_str() == "q" => {
            input_state.start_marker = None;
            input_state.end_marker = None;
            planner.reset();
            changed = true;
        }
        Key::Named(winit::keyboard::NamedKey::Escape) => {
            input_state.start_marker = None;
            input_state.end_marker = None;
            planner.reset();
            changed = true;
        }
        _ => {}
    }

    if changed {
        let start_node = input_state
            .start_marker
            .as_ref()
            .and_then(|m| m.snapped_node);
        let end_node = input_state.end_marker.as_ref().and_then(|m| m.snapped_node);

        if let (Some(s), Some(e)) = (start_node, end_node) {
            if s != e {
                planner.start_search(s, e);
            }
        } else {
            planner.reset();
        }
    }

    changed
}
