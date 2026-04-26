#[derive(Debug, Clone)]
pub struct Camera {
    pub center: [f64; 2],
    pub zoom: f64,
    pub screen_width: f32,
    pub screen_height: f32,
}

impl Camera {
    pub fn new(center: [f64; 2], zoom: f64, screen_width: f32, screen_height: f32) -> Self {
        Self { center, zoom, screen_width, screen_height }
    }

    /// Create a camera centered on a bounding box with zoom fitting the view.
    pub fn new_from_bbox(
        bbox: Option<([f64; 2], [f64; 2])>,
        screen_width: f32,
        screen_height: f32,
    ) -> Self {
        match bbox {
            Some((min, max)) => {
                let center = [
                    (min[0] + max[0]) * 0.5,
                    (min[1] + max[1]) * 0.5,
                ];
                let span_x = (max[0] - min[0]).max(1.0);
                let span_y = (max[1] - min[1]).max(1.0);
                let zoom_x = screen_width as f64 / span_x * 0.85;
                let zoom_y = screen_height as f64 / span_y * 0.85;
                let zoom = zoom_x.min(zoom_y).max(1e-6);
                Self::new(center, zoom, screen_width, screen_height)
            }
            None => Self::new([0.0, 0.0], 0.05, screen_width, screen_height),
        }
    }

    /// Convert world-space coordinates (meters) to screen-space pixels.
    pub fn world_to_screen(&self, world: [f64; 2]) -> [f32; 2] {
        let dx = (world[0] - self.center[0]) * self.zoom;
        let dy = (world[1] - self.center[1]) * self.zoom;
        // Y-axis is flipped: world y increases north, screen y increases down.
        let sx = self.screen_width as f64 * 0.5 + dx;
        let sy = self.screen_height as f64 * 0.5 - dy;
        [sx as f32, sy as f32]
    }

    /// Convert screen-space pixels to world-space coordinates (meters).
    pub fn screen_to_world(&self, screen: [f32; 2]) -> [f64; 2] {
        let dx = (screen[0] as f64 - self.screen_width as f64 * 0.5) / self.zoom;
        let dy = -(screen[1] as f64 - self.screen_height as f64 * 0.5) / self.zoom;
        [self.center[0] + dx, self.center[1] + dy]
    }

    /// Zoom around a screen-space point, keeping that point fixed in world space.
    pub fn zoom_around(&mut self, screen_pos: [f32; 2], factor: f64) {
        let world_before = self.screen_to_world(screen_pos);
        self.zoom = (self.zoom * factor).clamp(1e-7, 1e3);
        let world_after = self.screen_to_world(screen_pos);
        self.center[0] += world_before[0] - world_after[0];
        self.center[1] += world_before[1] - world_after[1];
    }
}
