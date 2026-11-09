use std::sync::Arc;

use wasm_bindgen::JsCast;
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::JsFuture;

const GRAPH_URL: &str = "graph.bin.gz";

#[wasm_bindgen(start)]
pub fn start() {
    std::panic::set_hook(Box::new(|info| {
        console_error_panic_hook::hook(info);
        show_error(&format!("{info}"));
    }));
    let _ = console_log::init_with_level(log::Level::Info);

    wasm_bindgen_futures::spawn_local(async {
        if let Err(err) = boot().await {
            show_error(&format!("{err:?}"));
        }
    });
}

pub(crate) fn ready(width: u32, height: u32) {
    log::info!("First frame presented at {width}x{height}");
    if let Some(element) = element("loading") {
        let _ = element.set_attribute("style", "display: none");
    }
    if debug_enabled()
        && let Some(element) = element("render-info")
    {
        element.set_text_content(Some(&format!("{width}x{height} px")));
        let _ = element.set_attribute("style", "display: block");
    }
}

fn debug_enabled() -> bool {
    web_sys::window()
        .and_then(|window| window.location().search().ok())
        .is_some_and(|search| search.contains("debug"))
}

async fn boot() -> Result<(), JsValue> {
    set_status("Downloading map data...");
    next_tick().await;

    let bytes = fetch_bytes(GRAPH_URL).await?;

    set_status("Decoding map data...");
    next_tick().await;

    let graph = crate::asset::decode(&bytes).map_err(|e| JsValue::from_str(&e.to_string()))?;

    set_status("Starting renderer...");
    next_tick().await;

    crate::app::run(Arc::new(graph));
    Ok(())
}

async fn fetch_bytes(url: &str) -> Result<Vec<u8>, JsValue> {
    let window = web_sys::window().ok_or_else(|| JsValue::from_str("no browser window"))?;
    let response = JsFuture::from(window.fetch_with_str(url)).await?;
    let response: web_sys::Response = response.dyn_into()?;

    if !response.ok() {
        return Err(JsValue::from_str(&format!(
            "failed to fetch {url}: HTTP {}",
            response.status()
        )));
    }

    let buffer = JsFuture::from(response.array_buffer()?).await?;
    Ok(js_sys::Uint8Array::new(&buffer).to_vec())
}

async fn next_tick() {
    let promise = js_sys::Promise::new(&mut |resolve, _reject| {
        if let Some(window) = web_sys::window() {
            let _ = window.set_timeout_with_callback_and_timeout_and_arguments_0(&resolve, 0);
        }
    });
    let _ = JsFuture::from(promise).await;
}

fn set_status(text: &str) {
    if let Some(element) = element("loading-status") {
        element.set_text_content(Some(text));
    }
}

pub(crate) fn show_error(message: &str) {
    set_status("Failed to start");
    if let Some(element) = element("loading-error") {
        element.set_text_content(Some(message));
        let _ = element.set_attribute("style", "display: block");
    }
    if let Some(element) = element("loading") {
        let _ = element.set_attribute("style", "display: flex");
    }
}

fn element(id: &str) -> Option<web_sys::Element> {
    web_sys::window()?.document()?.get_element_by_id(id)
}
