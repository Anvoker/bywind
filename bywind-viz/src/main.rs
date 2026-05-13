//! Binary entry point for `bywind-viz`, the egui-based GUI editor and
//! search visualiser for the [`bywind`](https://crates.io/crates/bywind)
//! sailing-route optimiser. See the crate's README for screenshots and
//! a feature tour.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

use crate::app::BywindApp;
mod app;
mod bundled_sample;
mod coastlines;
mod config;
mod draw;
#[cfg(not(target_arch = "wasm32"))]
mod fetch;
mod io;
mod search;
mod tools;
mod ui;
mod view;

// When compiling natively:

#[cfg(not(target_arch = "wasm32"))]
fn load_app_icon() -> egui::IconData {
    // The icon bytes are baked in at compile time; a decode failure
    // here is a build-time bug (non-PNG asset), not a runtime concern.
    eframe::icon_data::from_png_bytes(&include_bytes!("../assets/icon-256.png")[..])
        .expect("embedded icon PNG must decode")
}

#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result {
    env_logger::init();

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([400.0, 300.0])
            .with_min_inner_size([300.0, 220.0])
            .with_icon(load_app_icon()),
        ..Default::default()
    };
    eframe::run_native(
        "Bywind",
        native_options,
        Box::new(|cc| Ok(Box::new(BywindApp::new(cc)))),
    )
}

// When compiling to web using trunk:
#[cfg(target_arch = "wasm32")]
fn main() {
    use eframe::wasm_bindgen::JsCast as _;

    // Redirect `log` message to `console.log` and friends:
    eframe::WebLogger::init(log::LevelFilter::Debug).ok();

    let web_options = eframe::WebOptions::default();

    wasm_bindgen_futures::spawn_local(async {
        let document = web_sys::window()
            .expect("No window")
            .document()
            .expect("No document");

        let canvas = document
            .get_element_by_id("the_canvas_id")
            .expect("Failed to find the_canvas_id")
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .expect("the_canvas_id was not a HtmlCanvasElement");

        let start_result = eframe::WebRunner::new()
            .start(
                canvas,
                web_options,
                Box::new(|cc| Ok(Box::new(BywindApp::new(cc)))),
            )
            .await;

        // Remove the loading text and spinner:
        if let Some(loading_text) = document.get_element_by_id("loading_text") {
            match start_result {
                Ok(_) => {
                    loading_text.remove();
                }
                Err(e) => {
                    loading_text.set_inner_html(
                        "<p> The app has crashed. See the developer console for details. </p>",
                    );
                    panic!("Failed to start eframe: {e:?}");
                }
            }
        }
    });
}
