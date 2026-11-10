use std::num::NonZeroUsize;
use std::sync::Arc;

use winit::application::ApplicationHandler;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::keyboard::Key;
use winit::window::Window;

use crate::camera::Camera;
use crate::graph;
use crate::graph::RoadGraph;
use crate::input::{InputState, Marker, MarkerKind};
use crate::lod::LodPyramid;
use crate::planner::{Algorithm, CostMode, Heuristic, PlannerState, PlannerStatus};
use crate::renderer;
use crate::renderer::{DebugOverlayState, MenuItemKind, RenderScratch};
use crate::spatial_index::SpatialIndex;
use crate::view_index::RStarViewIndex;

use vello::util::{RenderContext, RenderSurface};
use vello::wgpu::{self, PresentMode};
use vello::{AaConfig, AaSupport, Renderer, RendererOptions, Scene};

const WINDOW_TITLE: &str = "Path Planning - OSM*";
#[cfg(not(target_arch = "wasm32"))]
const DEFAULT_WIDTH: u32 = 1280;
#[cfg(not(target_arch = "wasm32"))]
const DEFAULT_HEIGHT: u32 = 720;

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
    manual_lod_tier: Option<u8>,
    camera_initialized: bool,
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    first_frame_presented: bool,

    // Vello / wgpu.
    render_context: Option<RenderContext>,
    render_state: Option<RenderState>,
    window: Option<Arc<Window>>,
}

impl App {
    fn new(graph: Arc<RoadGraph>) -> Self {
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

        let render_context = Some(RenderContext::new());

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
            manual_lod_tier: None,
            camera_initialized: false,
            first_frame_presented: false,
            render_context,
            render_state: None,
            window: None,
        }
    }

    async fn create_render_state(
        mut render_context: RenderContext,
        window: Arc<Window>,
        width: u32,
        height: u32,
    ) -> Result<(RenderContext, RenderState), String> {
        let surface = render_context
            .create_surface(window, width, height, PresentMode::AutoVsync)
            .await
            .map_err(|e| format!("Failed to create the WebGPU surface: {e}"))?;

        let device_handle = &render_context.devices[surface.dev_id];
        let renderer = Renderer::new(
            &device_handle.device,
            RendererOptions {
                use_cpu: false,
                antialiasing_support: AaSupport::all(),
                num_init_threads: NonZeroUsize::new(1),
                pipeline_cache: None,
            },
        )
        .map_err(|e| format!("Failed to create the Vello renderer: {e}"))?;

        Ok((
            render_context,
            RenderState {
                surface,
                renderer,
                scene: Scene::new(),
            },
        ))
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn init_surface(&mut self, window: Arc<Window>, width: u32, height: u32) {
        let render_context = self
            .render_context
            .take()
            .expect("render context already consumed");
        let (render_context, render_state) = pollster::block_on(Self::create_render_state(
            render_context,
            window.clone(),
            width,
            height,
        ))
        .expect("Failed to create the WGPU surface");

        self.render_context = Some(render_context);
        self.render_state = Some(render_state);
        self.window = Some(window);
        self.needs_redraw = true;
        camera_init(&mut self.camera, &self.graph, width, height);
        self.camera_initialized = true;
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

        let device_handle = &self
            .render_context
            .as_ref()
            .expect("render context")
            .devices[render_state.surface.dev_id];
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
            self.manual_lod_tier,
            &mut render_state.scene,
            &mut self.debug_overlay,
            &mut self.render_scratch,
            width,
            height,
        );

        #[cfg(not(target_arch = "wasm32"))]
        let params = vello::RenderParams {
            base_color: vello::peniko::Color::from_rgba8(10, 14, 26, 255),
            width,
            height,
            antialiasing_method: AaConfig::Msaa16,
        };
        #[cfg(target_arch = "wasm32")]
        let params = vello::RenderParams {
            base_color: vello::peniko::Color::from_rgba8(10, 14, 26, 255),
            width,
            height,
            // Prefer the cheaper area AA on mobile GPUs, which may not be able
            // to allocate Vello's large MSAA targets.
            antialiasing_method: AaConfig::Area,
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

        #[cfg(target_arch = "wasm32")]
        if !self.first_frame_presented {
            self.first_frame_presented = true;
            log::info!("First frame presented at {width}x{height}");
            crate::web::ready(width, height);
        }

        self.needs_redraw = false;
    }

    fn resize(&mut self, width: u32, height: u32) {
        if let Some(render_state) = self.render_state.as_mut()
            && let Some(render_context) = self.render_context.as_ref()
        {
            render_context.resize_surface(&mut render_state.surface, width, height);
        }

        // The web backend reports a zero-sized window until its resize observer
        // fires, so the initial camera must be fitted once real dimensions are
        // known; otherwise the zoom is computed for a 1x1 viewport.
        if !self.camera_initialized {
            if width >= 2 && height >= 2 {
                let graph = self.graph.clone();
                camera_init(&mut self.camera, &graph, width, height);
                self.camera_initialized = true;
            }
        } else {
            self.camera.screen_width = width as f32;
            self.camera.screen_height = height as f32;
        }
        self.needs_redraw = true;
    }
}

fn camera_init(camera: &mut Camera, graph: &graph::RoadGraph, width: u32, height: u32) {
    let bbox = graph.bounding_box();
    *camera = Camera::new_from_bbox(bbox, width as f32, height as f32);
}

#[cfg(target_arch = "wasm32")]
fn canvas_backing_size() -> Option<(u32, u32)> {
    use wasm_bindgen::JsCast;

    let window = web_sys::window()?;
    let dpr = window.device_pixel_ratio().max(1.0);
    let canvas = window
        .document()?
        .get_element_by_id("canvas")?
        .dyn_into::<web_sys::HtmlCanvasElement>()
        .ok()?;

    let (css_w, css_h) = (canvas.client_width(), canvas.client_height());
    let (css_w, css_h): (f64, f64) = if css_w > 0 && css_h > 0 {
        (css_w as f64, css_h as f64)
    } else {
        (
            window.inner_width().ok()?.as_f64()?,
            window.inner_height().ok()?.as_f64()?,
        )
    };
    if css_w <= 0.0 || css_h <= 0.0 {
        return None;
    }

    Some(((css_w * dpr).round() as u32, (css_h * dpr).round() as u32))
}

impl App {
    fn create_window(&self, event_loop: &ActiveEventLoop) -> Arc<Window> {
        #[allow(unused_mut)]
        let mut attributes = Window::default_attributes().with_title(WINDOW_TITLE);

        #[cfg(not(target_arch = "wasm32"))]
        {
            attributes = attributes
                .with_inner_size(winit::dpi::LogicalSize::new(DEFAULT_WIDTH, DEFAULT_HEIGHT));
        }

        #[cfg(target_arch = "wasm32")]
        {
            use wasm_bindgen::JsCast;
            use winit::platform::web::WindowAttributesExtWebSys;
            let canvas = web_sys::window()
                .and_then(|w| w.document())
                .and_then(|d| d.get_element_by_id("canvas"))
                .and_then(|e| e.dyn_into::<web_sys::HtmlCanvasElement>().ok());
            attributes = attributes.with_canvas(canvas).with_prevent_default(true);
        }

        Arc::new(
            event_loop
                .create_window(attributes)
                .expect("Failed to create window"),
        )
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn on_resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.render_state.is_some() {
            return;
        }
        let window = self.create_window(event_loop);
        let size = window.inner_size();
        self.init_surface(window, size.width, size.height);
    }

    fn on_window_event(
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
                } else {
                    self.planner.precompute_background(&self.graph);
                }
                self.render_frame();
            }

            WindowEvent::CursorMoved { position, .. } => {
                self.pointer_moved([position.x as f32, position.y as f32]);
            }

            WindowEvent::Touch(touch) => {
                use winit::event::TouchPhase;
                let pos = [touch.location.x as f32, touch.location.y as f32];
                match touch.phase {
                    TouchPhase::Started => {
                        if self.input_state.active_touch.is_none() {
                            self.input_state.active_touch = Some(touch.id);
                            self.pointer_pressed(pos);
                        }
                    }
                    TouchPhase::Moved => {
                        if self.input_state.active_touch == Some(touch.id) {
                            self.pointer_moved(pos);
                        }
                    }
                    TouchPhase::Ended | TouchPhase::Cancelled => {
                        if self.input_state.active_touch == Some(touch.id) {
                            self.pointer_moved(pos);
                            self.pointer_released();
                            self.input_state.active_touch = None;
                        }
                    }
                }
            }

            WindowEvent::MouseInput { state, button, .. } => {
                let screen_pos = self.input_state.drag_screen_pos;

                match button {
                    MouseButton::Left => match state {
                        ElementState::Pressed => {
                            self.pointer_pressed(screen_pos);
                        }
                        ElementState::Released => {
                            self.pointer_released();
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
                    &mut self.manual_lod_tier,
                    &mut self.debug_overlay,
                );
                if handled {
                    self.needs_redraw = true;
                }
            }

            _ => {}
        }
    }

    fn pointer_pressed(&mut self, screen_pos: [f32; 2]) {
        self.input_state.drag_screen_pos = screen_pos;
        self.input_state.left_button_down = true;
        self.input_state.press_pos = screen_pos;
        self.input_state.is_drag = false;

        let hud_hit = self.debug_overlay.menu_layout.hit_test(screen_pos);
        let hud_panel = self.debug_overlay.menu_layout.hit_test_panel(screen_pos);

        if hud_hit.is_some() {
            self.input_state.pressed_menu_item = hud_hit;
        } else if hud_panel {
            // Inside panel but not on an item — consume to prevent pan/marker
        } else {
            let near = self.input_state.marker_near(screen_pos, 12.0, &self.camera);
            if let Some(kind) = near {
                self.input_state.dragging = Some(kind);
            } else {
                self.input_state.pan_start = Some(screen_pos);
                self.input_state.pan_center_start = Some(self.camera.center);
            }
        }
    }

    fn pointer_moved(&mut self, new_pos: [f32; 2]) {
        self.input_state.drag_screen_pos = new_pos;

        if !self.input_state.left_button_down {
            self.input_state.hover_menu_item = self.debug_overlay.menu_layout.hit_test(new_pos);
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

    fn pointer_released(&mut self) {
        self.input_state.left_button_down = false;

        if let Some(pressed) = self.input_state.pressed_menu_item.take() {
            let released = self
                .debug_overlay
                .menu_layout
                .hit_test(self.input_state.drag_screen_pos);
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

    fn on_about_to_wait(&mut self, _event_loop: &winit::event_loop::ActiveEventLoop) {
        let planner_animating = matches!(
            self.planner.status,
            PlannerStatus::Searching | PlannerStatus::FirstMatchFound
        ) || self
            .planner
            .precompute_pending(&self.graph)
            .next()
            .is_some();
        if (self.needs_redraw || planner_animating)
            && let Some(window) = self.window.as_ref()
        {
            window.request_redraw();
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
struct NativeRunner {
    app: App,
}

#[cfg(not(target_arch = "wasm32"))]
impl ApplicationHandler for NativeRunner {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.app.on_resumed(event_loop);
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        window_id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        self.app.on_window_event(event_loop, window_id, event);
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        self.app.on_about_to_wait(event_loop);
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub fn run(graph: Arc<RoadGraph>) -> Result<(), winit::error::EventLoopError> {
    let mut runner = NativeRunner {
        app: App::new(graph),
    };
    EventLoop::new()?.run_app(&mut runner)
}

#[cfg(target_arch = "wasm32")]
pub fn run(graph: Arc<RoadGraph>) {
    use std::cell::Cell;
    use std::cell::RefCell;
    use std::rc::Rc;

    use winit::platform::web::EventLoopExtWebSys;

    struct WasmRunner {
        inner: Rc<RefCell<App>>,
        init_started: Rc<Cell<bool>>,
    }

    impl ApplicationHandler for WasmRunner {
        fn resumed(&mut self, event_loop: &ActiveEventLoop) {
            if self.init_started.replace(true) {
                return;
            }

            let (window, width, height, render_context) = {
                let mut app = self.inner.borrow_mut();
                let window = app.create_window(event_loop);
                // winit's web backend starts with a zero-sized canvas until its
                // resize observer fires; fall back to the laid-out canvas size.
                let mut size = window.inner_size();
                if (size.width == 0 || size.height == 0)
                    && let Some((w, h)) = canvas_backing_size()
                {
                    size = winit::dpi::PhysicalSize::new(w, h);
                }
                app.window = Some(window.clone());
                (
                    window,
                    size.width.max(1),
                    size.height.max(1),
                    app.render_context.take().expect("render context"),
                )
            };

            let inner = self.inner.clone();
            wasm_bindgen_futures::spawn_local(async move {
                let (render_context, render_state) =
                    match App::create_render_state(render_context, window, width, height).await {
                        Ok(state) => state,
                        Err(error) => {
                            crate::web::show_error(&error);
                            return;
                        }
                    };

                let mut app = inner.borrow_mut();
                app.render_context = Some(render_context);
                app.render_state = Some(render_state);
                app.needs_redraw = true;
                if width > 1 && height > 1 {
                    let graph = app.graph.clone();
                    camera_init(&mut app.camera, &graph, width, height);
                    app.camera_initialized = true;
                }
            });
        }

        fn window_event(
            &mut self,
            event_loop: &ActiveEventLoop,
            window_id: winit::window::WindowId,
            event: WindowEvent,
        ) {
            self.inner
                .borrow_mut()
                .on_window_event(event_loop, window_id, event);
        }

        fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
            self.inner.borrow_mut().on_about_to_wait(event_loop);
        }
    }

    let event_loop = EventLoop::new().expect("Failed to create event loop");
    event_loop.spawn_app(WasmRunner {
        inner: Rc::new(RefCell::new(App::new(graph))),
        init_started: Rc::new(Cell::new(false)),
    });
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

    if let (Some(s), Some(e)) = (start_node, end_node)
        && s != e
    {
        planner.start_search(s, e);
        return true;
    }
    false
}

fn dist2(a: [f32; 2], b: [f32; 2]) -> f32 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    dx * dx + dy * dy
}

#[cfg(not(target_arch = "wasm32"))]
fn now_seed() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

#[cfg(target_arch = "wasm32")]
fn now_seed() -> u64 {
    (js_sys::Date::now() * 1_000_000.0) as u64
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
        MenuItemKind::CostMode(m) => {
            if planner.config.cost_mode != m {
                planner.config.cost_mode = m;
                true
            } else {
                false
            }
        }
        MenuItemKind::Swap => swap_markers(input_state, graph),
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

fn swap_markers(input_state: &mut InputState, _graph: &graph::RoadGraph) -> bool {
    if input_state.start_marker.is_some() && input_state.end_marker.is_some() {
        std::mem::swap(&mut input_state.start_marker, &mut input_state.end_marker);
        return true;
    }
    false
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

    let mut seed = now_seed();
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
    manual_lod_tier: &mut Option<u8>,
    debug_overlay: &mut DebugOverlayState,
) -> bool {
    let mut changed = false;

    match key {
        Key::Named(winit::keyboard::NamedKey::F1) => {
            debug_overlay.hud_visible = !debug_overlay.hud_visible;
            return true;
        }
        Key::Named(winit::keyboard::NamedKey::Backspace) => {
            let start_node = input_state
                .start_marker
                .as_ref()
                .and_then(|m| m.snapped_node);
            let end_node = input_state.end_marker.as_ref().and_then(|m| m.snapped_node);
            if let (Some(s), Some(e)) = (start_node, end_node)
                && s != e
            {
                planner.start_search(s, e);
                return true;
            }
            return false;
        }
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
        Key::Character(c) if c.as_str() == "c" || c.as_str() == "C" => {
            let modes = CostMode::all();
            let idx = modes
                .iter()
                .position(|&m| m == planner.config.cost_mode)
                .unwrap_or(0);
            let new_idx = if c.as_str() == "C" {
                (idx + modes.len() - 1) % modes.len()
            } else {
                (idx + 1) % modes.len()
            };
            planner.config.cost_mode = modes[new_idx];
            changed = true;
        }
        Key::Character(c) if c.as_str() == "a" || c.as_str() == "A" => {
            let algs = Algorithm::all();
            let idx = algs
                .iter()
                .position(|&a| a == planner.config.algorithm)
                .unwrap_or(0);
            let new_idx = (idx + 1) % algs.len();
            planner.config.algorithm = algs[new_idx];
            changed = true;
        }
        Key::Character(c) if c.as_str() == "1" => {
            *manual_lod_tier = Some(0);
            changed = true;
        }
        Key::Character(c) if c.as_str() == "2" => {
            *manual_lod_tier = Some(1);
            changed = true;
        }
        Key::Character(c) if c.as_str() == "3" => {
            *manual_lod_tier = Some(2);
            changed = true;
        }
        Key::Character(c) if c.as_str() == "0" => {
            *manual_lod_tier = None;
            changed = true;
        }
        Key::Character(c) if c.as_str() == "r" => {
            changed = randomize_route(input_state, graph, camera);
        }
        Key::Character(c) if c.as_str() == "s" => {
            changed = swap_markers(input_state, graph);
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
