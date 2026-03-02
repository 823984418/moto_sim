use std::fs::read;
use std::sync::Arc;

use eframe::epaint::FontFamily;
use eframe::epaint::text::{FontData, FontDefinitions, FontTweak};
use egui::Context;
use font_kit::family_name::FamilyName;
use font_kit::handle::Handle;
use font_kit::properties::Properties;
use font_kit::source::SystemSource;

fn load_font(
    defintions: &mut FontDefinitions,
    source: &SystemSource,
    name: &str,
    font_family: &[FontFamily],
    family_names: &[FamilyName],
    properties: &Properties,
) {
    let font_handle = source.select_best_match(family_names, properties).unwrap();
    let (font_bytes, index): (Vec<u8>, u32) = match font_handle {
        Handle::Path { path, font_index } => (read(path).unwrap(), font_index),
        Handle::Memory { bytes, font_index } => (
            Arc::try_unwrap(bytes).unwrap_or_else(|b| Vec::clone(&b)),
            font_index,
        ),
    };
    let font_data = Arc::new(FontData {
        font: font_bytes.into(),
        index,
        tweak: FontTweak::default(),
    });
    defintions.font_data.insert(name.to_string(), font_data);
    for i in font_family {
        defintions
            .families
            .entry(i.clone())
            .or_default()
            .push(name.to_string());
    }
}

pub fn load_chinese_font(ctx: &Context) {
    let mut fonts = FontDefinitions::default();

    load_font(
        &mut fonts,
        &SystemSource::new(),
        "中文",
        &[
            FontFamily::Proportional,
            FontFamily::Monospace,
            FontFamily::Name("中文".into()),
        ],
        &[
            FamilyName::Title("等线".to_string()),
            FamilyName::Title("微软雅黑".to_string()),
            FamilyName::Title("宋体".to_string()),
            FamilyName::Title("楷体".to_string()),
        ],
        &Properties::new(),
    );

    ctx.set_fonts(fonts);
}
