//! Brain Topography visualization: wireframe head map with electrode dots.
//!
//! Two-layer rendering:
//! 1. OBJ wireframe head mesh parsed at build time, projected orthographically
//!    with depth-based alpha for a sci-fi wireframe effect.
//! 2. Procedural head outline circle with nose and ears for electrode placement.
//!
//! Electrodes glow using GPU-interpolated radial gradient meshes via fan triangulation,
//! which avoids the tessellation cost of multiple circle_filled calls.

use std::cell::RefCell;

use eframe::egui::{self, Color32, Mesh, Pos2, Rect, Shape, Stroke, Vec2};
use muse_rs::compute::ContactQuality;

use crate::signal_pipeline::{AnalysisFrame, DimensionCard, DimensionReading, ElectrodeState};

// ── Build-time generated mesh data ───────────────────────────────────────────

mod head_mesh_data {
    include!(concat!(env!("OUT_DIR"), "/head_mesh_data.rs"));
}

// ── Electrode layout: 10-20 system coordinates ──────────────────────────────

/// (name, angle_deg_from_north_clockwise, radius_fraction)
const ELECTRODE_LAYOUT: [(&str, f32, f32); 4] = [
    ("TP9", -108.0, 0.85),  // left temporal
    ("AF7", -36.0, 0.65),   // left frontal
    ("AF8", 36.0, 0.65),    // right frontal
    ("TP10", 108.0, 0.85),  // right temporal
];

const ELECTRODE_GLOW_COLOR: Color32 = Color32::from_rgb(0, 200, 255);
const NO_CONTACT_COLOR: Color32 = Color32::from_rgb(80, 80, 80);

// ── Semantic functions (pure geometry) ───────────────────────────────────────

fn electrode_screen_pos(
    head_center: Pos2,
    head_radius: f32,
    angle_deg: f32,
    radius_frac: f32,
) -> Pos2 {
    // 0 deg = north (top), clockwise. Convert to math angle.
    let angle_rad = (angle_deg - 90.0).to_radians();
    let r = head_radius * radius_frac;
    Pos2::new(
        head_center.x + r * angle_rad.cos(),
        head_center.y + r * angle_rad.sin(),
    )
}

fn alpha_to_brightness(power: f32, range: (f32, f32)) -> f32 {
    let (lo, hi) = range;
    if hi <= lo {
        return 0.5;
    }
    ((power - lo) / (hi - lo)).clamp(0.0, 1.0)
}

fn asymmetry_bar_fill(asymmetry: f32, max_asymmetry: f32) -> (f32, bool) {
    let frac = (asymmetry.abs() / max_asymmetry).clamp(0.0, 1.0);
    let is_approach = asymmetry >= 0.0;
    (frac, is_approach)
}

/// Build a radial gradient glow mesh using triangle fan.
/// Center vertex: bright color. Outer ring: transparent.
/// GPU interpolates the gradient, no tessellation overhead.
fn glow_mesh(center: Pos2, radius: f32, color: Color32, segments: usize) -> Mesh {
    let mut mesh = Mesh::default();
    mesh.reserve_vertices(segments + 2);
    mesh.reserve_triangles(segments);

    mesh.colored_vertex(center, color);

    for i in 0..=segments {
        let angle = (i as f32 / segments as f32) * std::f32::consts::TAU;
        let pos = Pos2::new(
            center.x + radius * angle.cos(),
            center.y + radius * angle.sin(),
        );
        mesh.colored_vertex(pos, Color32::TRANSPARENT);
    }

    for i in 0..segments as u32 {
        mesh.add_triangle(0, i + 1, i + 2);
    }

    mesh
}

// ── Wireframe cache ──────────────────────────────────────────────────────────

struct HeadWireframeCache {
    last_center: Pos2,
    last_scale: f32,
    shapes: Vec<Shape>,
}

impl HeadWireframeCache {
    fn new() -> Self {
        Self {
            last_center: Pos2::ZERO,
            last_scale: 0.0,
            shapes: Vec::new(),
        }
    }

    fn get_or_update(&mut self, center: Pos2, scale: f32) -> &[Shape] {
        let center_changed = (self.last_center.x - center.x).abs() > 0.5
            || (self.last_center.y - center.y).abs() > 0.5;
        let scale_changed = (self.last_scale - scale).abs() > 0.5;

        if center_changed || scale_changed || self.shapes.is_empty() {
            self.last_center = center;
            self.last_scale = scale;
            self.shapes = project_obj_wireframe(center, scale);
        }
        &self.shapes
    }
}

thread_local! {
    static WIREFRAME_CACHE: RefCell<HeadWireframeCache> = RefCell::new(HeadWireframeCache::new());
}

/// Project the OBJ head mesh to 2D with depth-based alpha.
fn project_obj_wireframe(center: Pos2, scale: f32) -> Vec<Shape> {
    let verts = head_mesh_data::HEAD_VERTICES;
    let edges = head_mesh_data::HEAD_EDGES;

    let project = |v: &[f32; 3]| -> Pos2 {
        Pos2::new(center.x + v[0] * scale, center.y - v[1] * scale)
    };

    let z_min = verts.iter().map(|v| v[2]).fold(f32::INFINITY, f32::min);
    let z_max = verts
        .iter()
        .map(|v| v[2])
        .fold(f32::NEG_INFINITY, f32::max);
    let z_range = (z_max - z_min).max(0.01);

    edges
        .iter()
        .filter(|[a, b]| {
            let za = verts[*a as usize][2];
            let zb = verts[*b as usize][2];
            // Cull fully back-facing edges
            let z_thresh = z_min + z_range * 0.2;
            za > z_thresh || zb > z_thresh
        })
        .map(|[a, b]| {
            let va = &verts[*a as usize];
            let vb = &verts[*b as usize];
            let avg_z = (va[2] + vb[2]) / 2.0;
            let depth_frac = ((avg_z - z_min) / z_range).clamp(0.0, 1.0);
            let alpha = (depth_frac * 50.0 + 5.0).clamp(5.0, 55.0) as u8;
            let color = Color32::from_rgba_unmultiplied(70, 100, 130, alpha);
            Shape::line_segment([project(va), project(vb)], Stroke::new(0.5, color))
        })
        .collect()
}

// ── Pragmatic rendering ──────────────────────────────────────────────────────

pub fn render_brain_topography(ui: &mut egui::Ui, analysis: &AnalysisFrame) {
    ui.horizontal(|ui| {
        // Left: dimension status cards
        ui.vertical(|ui| {
            ui.set_width(140.0);
            ui.spacing_mut().item_spacing.y = 4.0;
            for card in &analysis.dimensions {
                paint_compact_dimension(ui, card);
            }
        });

        // Right: head map fills remaining space
        paint_head_map(ui, &analysis.electrodes);
    });
}

/// Fixed inner height for compact dimension cards so the panel never resizes.
const COMPACT_CARD_HEIGHT: f32 = 48.0;

fn paint_compact_dimension(ui: &mut egui::Ui, card: &DimensionCard) {
    let color = card.meta.color;

    egui::Frame::none()
        .fill(Color32::from_rgba_unmultiplied(10, 10, 14, 200))
        .stroke(Stroke::new(
            1.0,
            Color32::from_rgba_unmultiplied(40, 40, 56, 80),
        ))
        .rounding(4.0)
        .inner_margin(6.0)
        .show(ui, |ui| {
            ui.set_width(128.0);
            ui.set_min_height(COMPACT_CARD_HEIGHT);
            ui.set_max_height(COMPACT_CARD_HEIGHT);

            // Row 1: Name + status
            ui.horizontal(|ui| {
                ui.label(
                    egui::RichText::new(card.meta.name)
                        .color(color)
                        .size(10.0)
                        .strong(),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let status = match &card.reading {
                        DimensionReading::Idle => "\u{2014}",
                        DimensionReading::Settling { .. } => "Settling",
                        DimensionReading::Measuring { .. } => "Measuring",
                        DimensionReading::Complete(_) => "\u{2713}",
                    };
                    let status_color = match &card.reading {
                        DimensionReading::Complete(_) => color,
                        DimensionReading::Idle => Color32::from_gray(80),
                        _ => Color32::from_gray(140),
                    };
                    ui.label(egui::RichText::new(status).color(status_color).size(9.0));
                });
            });

            // Row 2: Value text (always allocated, blank when idle)
            if let Some(value) = card.reading.display_value() {
                ui.label(
                    egui::RichText::new(format!("{} \u{2022} {}", value.display_text, value.label))
                        .color(color)
                        .size(11.0),
                );
            } else {
                ui.label(egui::RichText::new(" ").size(11.0));
            }

            // Row 3: Progress bar (always allocated)
            let progress = if card.reading.is_active() {
                card.reading.progress()
            } else {
                0.0
            };
            let (bar_rect, _) =
                ui.allocate_exact_size(Vec2::new(ui.available_width(), 3.0), egui::Sense::hover());
            ui.painter()
                .rect_filled(bar_rect, 1.0, Color32::from_gray(30));
            if progress > 0.0 {
                let fill_rect = Rect::from_min_size(
                    bar_rect.left_top(),
                    Vec2::new(bar_rect.width() * progress, 3.0),
                );
                ui.painter().rect_filled(fill_rect, 1.0, color);
            }
        });
}

fn paint_head_map(ui: &mut egui::Ui, electrodes: &ElectrodeState) {
    let available = ui.available_size();
    let radius = available.x.min(available.y) * 0.48;
    let (response, painter) = ui.allocate_painter(available, egui::Sense::hover());
    let center = response.rect.center();

    // Layer 1: detailed OBJ wireframe, cached projection
    WIREFRAME_CACHE.with(|cell| {
        let mut cache = cell.borrow_mut();
        let shapes = cache.get_or_update(center, radius);
        painter.extend(shapes.iter().cloned());
    });

    // Layer 2: procedural electrode circle outline
    paint_head_outline(&painter, center, radius);

    // Normalize alpha power range for brightness
    let min_alpha = electrodes
        .alpha_power
        .iter()
        .copied()
        .fold(f32::INFINITY, f32::min);
    let max_alpha = electrodes
        .alpha_power
        .iter()
        .copied()
        .fold(f32::NEG_INFINITY, f32::max);
    let alpha_range = (min_alpha, max_alpha);

    // Draw each electrode with glow and label
    for (ch, &(name, angle, rfrac)) in ELECTRODE_LAYOUT.iter().enumerate() {
        let pos = electrode_screen_pos(center, radius, angle, rfrac);
        let brightness = alpha_to_brightness(electrodes.alpha_power[ch], alpha_range);
        let color = if electrodes.contact[ch] == ContactQuality::NoContact {
            NO_CONTACT_COLOR
        } else {
            ELECTRODE_GLOW_COLOR
        };
        let alpha_text = format!("{:.1}", electrodes.alpha_power[ch]);
        paint_electrode(&painter, pos, brightness, color, name, &alpha_text);
    }

    // Frontal asymmetry bar between AF7 and AF8
    let af7_pos =
        electrode_screen_pos(center, radius, ELECTRODE_LAYOUT[1].1, ELECTRODE_LAYOUT[1].2);
    let af8_pos =
        electrode_screen_pos(center, radius, ELECTRODE_LAYOUT[2].1, ELECTRODE_LAYOUT[2].2);
    paint_asymmetry_bar(&painter, af7_pos, af8_pos, electrodes.frontal_asymmetry);

    // Asymmetry label
    let bar_center = Pos2::new((af7_pos.x + af8_pos.x) / 2.0, af7_pos.y - 24.0);
    painter.text(
        bar_center,
        egui::Align2::CENTER_CENTER,
        format!("FAA: {:+.3}", electrodes.frontal_asymmetry),
        egui::FontId::proportional(10.0),
        Color32::from_gray(160),
    );
}

fn paint_head_outline(painter: &egui::Painter, center: Pos2, radius: f32) {
    painter.circle_stroke(center, radius, Stroke::new(1.5, Color32::from_gray(60)));

    // Nose indicator
    let nose_tip = Pos2::new(center.x, center.y - radius - 8.0);
    let nose_left = Pos2::new(center.x - 6.0, center.y - radius + 2.0);
    let nose_right = Pos2::new(center.x + 6.0, center.y - radius + 2.0);
    painter.add(Shape::line(
        vec![nose_left, nose_tip, nose_right],
        Stroke::new(1.5, Color32::from_gray(60)),
    ));

    // Ear indicators
    let ear_size = 8.0;
    painter.add(Shape::line(
        vec![
            Pos2::new(center.x - radius - 2.0, center.y - ear_size),
            Pos2::new(center.x - radius - ear_size, center.y),
            Pos2::new(center.x - radius - 2.0, center.y + ear_size),
        ],
        Stroke::new(1.5, Color32::from_gray(60)),
    ));
    painter.add(Shape::line(
        vec![
            Pos2::new(center.x + radius + 2.0, center.y - ear_size),
            Pos2::new(center.x + radius + ear_size, center.y),
            Pos2::new(center.x + radius + 2.0, center.y + ear_size),
        ],
        Stroke::new(1.5, Color32::from_gray(60)),
    ));
}

fn paint_electrode(
    painter: &egui::Painter,
    pos: Pos2,
    brightness: f32,
    color: Color32,
    label: &str,
    alpha_text: &str,
) {
    let dot_radius = 6.0 + brightness * 6.0;

    // Glow: single mesh with fan triangulation, GPU-interpolated radial gradient
    let glow_radius = dot_radius * 3.0;
    let glow_alpha = (brightness * 0.4 * 255.0).clamp(0.0, 255.0) as u8;
    let glow_center_color =
        Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), glow_alpha);
    let mesh = glow_mesh(pos, glow_radius, glow_center_color, 32);
    painter.add(Shape::mesh(mesh));

    // Core dot
    let core_alpha = ((brightness * 0.9 + 0.1) * 255.0).clamp(0.0, 255.0) as u8;
    let core_color = Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), core_alpha);
    painter.circle_filled(pos, dot_radius, core_color);

    // Label above
    painter.text(
        Pos2::new(pos.x, pos.y - dot_radius - 10.0),
        egui::Align2::CENTER_BOTTOM,
        label,
        egui::FontId::proportional(10.0),
        Color32::from_gray(180),
    );

    // Alpha power below
    painter.text(
        Pos2::new(pos.x, pos.y + dot_radius + 4.0),
        egui::Align2::CENTER_TOP,
        alpha_text,
        egui::FontId::proportional(9.0),
        Color32::from_gray(120),
    );
}

fn paint_asymmetry_bar(
    painter: &egui::Painter,
    left_pos: Pos2,
    right_pos: Pos2,
    asymmetry: f32,
) {
    let bar_y = left_pos.y - 12.0;
    let bar_left = left_pos.x + 14.0;
    let bar_right = right_pos.x - 14.0;
    let bar_width = bar_right - bar_left;
    let bar_center_x = (bar_left + bar_right) / 2.0;
    let bar_height = 4.0;

    // Background bar
    let bg_rect = Rect::from_min_size(
        Pos2::new(bar_left, bar_y - bar_height / 2.0),
        Vec2::new(bar_width, bar_height),
    );
    painter.rect_filled(bg_rect, 2.0, Color32::from_gray(30));

    // Center line
    painter.line_segment(
        [
            Pos2::new(bar_center_x, bar_y - 4.0),
            Pos2::new(bar_center_x, bar_y + 4.0),
        ],
        Stroke::new(1.0, Color32::from_gray(60)),
    );

    // Fill from center based on asymmetry
    let (frac, is_approach) = asymmetry_bar_fill(asymmetry, 0.5);
    let fill_width = (bar_width / 2.0) * frac;
    let fill_color = if is_approach {
        Color32::from_rgb(0, 200, 100)
    } else {
        Color32::from_rgb(200, 100, 0)
    };

    let fill_rect = if is_approach {
        Rect::from_min_size(
            Pos2::new(bar_center_x, bar_y - bar_height / 2.0),
            Vec2::new(fill_width, bar_height),
        )
    } else {
        Rect::from_min_size(
            Pos2::new(bar_center_x - fill_width, bar_y - bar_height / 2.0),
            Vec2::new(fill_width, bar_height),
        )
    };
    painter.rect_filled(fill_rect, 2.0, fill_color);
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn topo_electrode_screen_pos_north() {
        // 0 degrees = north (top of head), should be directly above center
        let pos = electrode_screen_pos(Pos2::new(100.0, 100.0), 50.0, 0.0, 1.0);
        assert!((pos.x - 100.0).abs() < 0.01);
        assert!(pos.y < 100.0, "north should be above center");
    }

    #[test]
    fn topo_electrode_screen_pos_south() {
        let pos = electrode_screen_pos(Pos2::new(100.0, 100.0), 50.0, 180.0, 1.0);
        assert!((pos.x - 100.0).abs() < 0.01);
        assert!(pos.y > 100.0, "south should be below center");
    }

    #[test]
    fn topo_electrode_screen_pos_radius_fraction() {
        let pos_full = electrode_screen_pos(Pos2::new(100.0, 100.0), 50.0, 90.0, 1.0);
        let pos_half = electrode_screen_pos(Pos2::new(100.0, 100.0), 50.0, 90.0, 0.5);
        // Half radius should be closer to center
        let dist_full = ((pos_full.x - 100.0).powi(2) + (pos_full.y - 100.0).powi(2)).sqrt();
        let dist_half = ((pos_half.x - 100.0).powi(2) + (pos_half.y - 100.0).powi(2)).sqrt();
        assert!((dist_full - 50.0).abs() < 0.01);
        assert!((dist_half - 25.0).abs() < 0.01);
    }

    #[test]
    fn topo_alpha_to_brightness_normalized() {
        assert!((alpha_to_brightness(5.0, (0.0, 10.0)) - 0.5).abs() < 0.01);
        assert_eq!(alpha_to_brightness(0.0, (0.0, 10.0)), 0.0);
        assert_eq!(alpha_to_brightness(10.0, (0.0, 10.0)), 1.0);
    }

    #[test]
    fn topo_alpha_to_brightness_equal_range() {
        assert_eq!(alpha_to_brightness(5.0, (5.0, 5.0)), 0.5);
    }

    #[test]
    fn topo_asymmetry_bar_fill_positive() {
        let (frac, is_approach) = asymmetry_bar_fill(0.25, 0.5);
        assert!((frac - 0.5).abs() < 0.01);
        assert!(is_approach);
    }

    #[test]
    fn topo_asymmetry_bar_fill_negative() {
        let (frac, is_approach) = asymmetry_bar_fill(-0.25, 0.5);
        assert!((frac - 0.5).abs() < 0.01);
        assert!(!is_approach);
    }

    #[test]
    fn topo_glow_mesh_vertex_count() {
        let mesh = glow_mesh(Pos2::new(50.0, 50.0), 20.0, Color32::WHITE, 16);
        // 1 center + 17 outer (0..=16) = 18 vertices
        assert_eq!(mesh.vertices.len(), 18);
        // 16 triangles
        assert_eq!(mesh.indices.len(), 16 * 3);
    }

    #[test]
    fn topo_glow_mesh_center_is_colored() {
        let color = Color32::from_rgb(100, 200, 50);
        let mesh = glow_mesh(Pos2::new(0.0, 0.0), 10.0, color, 8);
        assert_eq!(mesh.vertices[0].color, color);
    }

    #[test]
    fn topo_glow_mesh_outer_is_transparent() {
        let mesh = glow_mesh(Pos2::new(0.0, 0.0), 10.0, Color32::WHITE, 8);
        // All outer vertices should be transparent
        for v in &mesh.vertices[1..] {
            assert_eq!(v.color, Color32::TRANSPARENT);
        }
    }

    #[test]
    fn topo_head_mesh_data_loaded() {
        // Verify build.rs generated valid data
        assert!(
            head_mesh_data::HEAD_VERTICES.len() > 100,
            "should have >100 vertices"
        );
        assert!(
            head_mesh_data::HEAD_EDGES.len() > 100,
            "should have >100 edges"
        );
    }

    #[test]
    fn topo_head_vertices_normalized() {
        // All vertices should be in [-1, 1] range
        for v in head_mesh_data::HEAD_VERTICES {
            assert!(v[0] >= -1.01 && v[0] <= 1.01, "x out of range: {}", v[0]);
            assert!(v[1] >= -1.01 && v[1] <= 1.01, "y out of range: {}", v[1]);
            assert!(v[2] >= -1.01 && v[2] <= 1.01, "z out of range: {}", v[2]);
        }
    }

    #[test]
    fn topo_wireframe_projection_produces_shapes() {
        let shapes = project_obj_wireframe(Pos2::new(200.0, 200.0), 80.0);
        assert!(!shapes.is_empty(), "should produce line shapes");
    }
}
