use crate::renderer::MenuItemKind;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MarkerKind {
    Start,
    End,
}

#[derive(Debug, Clone)]
pub struct Marker {
    pub world_pos: [f64; 2],
    pub snapped_node: Option<usize>,
}

#[derive(Debug, Clone)]
pub struct InputState {
    pub start_marker: Option<Marker>,
    pub end_marker: Option<Marker>,
    pub dragging: Option<MarkerKind>,
    /// Current cursor position in screen space (also used as drag position).
    pub drag_screen_pos: [f32; 2],
    pub pan_start: Option<[f32; 2]>,
    pub pan_center_start: Option<[f64; 2]>,
    pub left_button_down: bool,
    pub right_button_down: bool,
    /// Screen position where the left button was pressed (for click-vs-drag detection).
    pub press_pos: [f32; 2],
    /// True if the mouse has moved enough from press_pos to be a drag.
    pub is_drag: bool,
    pub hover_menu_item: Option<MenuItemKind>,
    pub pressed_menu_item: Option<MenuItemKind>,
}

impl InputState {
    pub fn new() -> Self {
        Self {
            start_marker: None,
            end_marker: None,
            dragging: None,
            drag_screen_pos: [0.0, 0.0],
            pan_start: None,
            pan_center_start: None,
            left_button_down: false,
            right_button_down: false,
            press_pos: [0.0, 0.0],
            is_drag: false,
            hover_menu_item: None,
            pressed_menu_item: None,
        }
    }

    /// Distance in screen pixels from the press position to the current cursor.
    pub fn drag_distance(&self) -> f32 {
        let dx = self.drag_screen_pos[0] - self.press_pos[0];
        let dy = self.drag_screen_pos[1] - self.press_pos[1];
        (dx * dx + dy * dy).sqrt()
    }

    /// Returns which marker (if any) is within `threshold_px` of a screen position.
    pub fn marker_near(
        &self,
        screen_pos: [f32; 2],
        threshold_px: f32,
        camera: &crate::camera::Camera,
    ) -> Option<MarkerKind> {
        let check = |marker: &Marker, _kind: MarkerKind| -> bool {
            let sp = camera.world_to_screen(marker.world_pos);
            let dx = sp[0] - screen_pos[0];
            let dy = sp[1] - screen_pos[1];
            (dx * dx + dy * dy).sqrt() < threshold_px
        };
        if let Some(ref m) = self.start_marker {
            if check(m, MarkerKind::Start) {
                return Some(MarkerKind::Start);
            }
        }
        if let Some(ref m) = self.end_marker {
            if check(m, MarkerKind::End) {
                return Some(MarkerKind::End);
            }
        }
        None
    }
}
