//! Neural Aura visualization: radial arc gauge cards for dimension readings.
//!
//! Each dimension gets a glass-morphism card with a neon-glow arc gauge,
//! center value text, progress bar, and sparkline history.

use eframe::egui::{self, Color32, Pos2, Rect, Shape, Stroke, Vec2};

use crate::signal_pipeline::{DimensionCard, DimensionReading};

// ── Arc gauge constants ──────────────────────────────────────────────────────

const ARC_START: f32 = std::f32::consts::PI * 0.75;   // 135 degrees, bottom-left
const ARC_SWEEP: f32 = std::f32::consts::PI * 1.5;    // 270 degrees total sweep
const ARC_POINTS: usize = 64;
const ARC_RADIUS: f32 = 40.0;

/// Glow layers: outermost to innermost, each with stroke width and alpha fraction
const GLOW_LAYERS: [(f32, f32); 4] = [
    (8.0, 0.08),
    (5.0, 0.15),
    (3.0, 0.30),
    (1.5, 0.90),
];

// ── Semantic functions (pure geometry) ───────────────────────────────────────

fn arc_points(center: Pos2, radius: f32, start_angle: f32, sweep: f32, n: usize) -> Vec<Pos2> {
    let n = n.max(2);
    (0..n)
        .map(|i| {
            let t = i as f32 / (n - 1) as f32;
            let angle = start_angle + sweep * t;
            Pos2::new(
                center.x + radius * angle.cos(),
                center.y + radius * angle.sin(),
            )
        })
        .collect()
}

fn glow_strokes(
    points: &[Pos2],
    base_color: Color32,
    layers: &[(f32, f32)],
) -> Vec<Shape> {
    let [r, g, b, _] = base_color.to_array();
    layers
        .iter()
        .map(|&(width, alpha_frac)| {
            let a = (alpha_frac * 255.0).clamp(0.0, 255.0) as u8;
            let color = Color32::from_rgba_unmultiplied(r, g, b, a);
            Shape::line(points.to_vec(), Stroke::new(width, color))
        })
        .collect()
}

fn sparkline_points(values: &[f32], rect: Rect) -> Vec<Pos2> {
    if values.is_empty() {
        return vec![];
    }
    let min = values.iter().copied().fold(f32::INFINITY, f32::min);
    let max = values.iter().copied().fold(f32::NEG_INFINITY, f32::max);
    let range = (max - min).max(1e-6);
    values
        .iter()
        .enumerate()
        .map(|(i, &v)| {
            let x = rect.left() + (i as f32 / (values.len() - 1).max(1) as f32) * rect.width();
            let y = rect.bottom() - ((v - min) / range) * rect.height();
            Pos2::new(x, y)
        })
        .collect()
}

// ── Pragmatic rendering ──────────────────────────────────────────────────────

pub fn render_neural_aura(ui: &mut egui::Ui, cards: &[DimensionCard; 4]) {
    ui.vertical(|ui| {
        ui.spacing_mut().item_spacing.y = 8.0;
        for card in cards {
            paint_dimension_card(ui, card);
        }
    });
}

fn paint_glass_frame(ui: &mut egui::Ui, add_contents: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::none()
        .fill(Color32::from_rgba_unmultiplied(10, 10, 14, 200))
        .stroke(Stroke::new(
            1.0,
            Color32::from_rgba_unmultiplied(40, 40, 56, 80),
        ))
        .rounding(8.0)
        .inner_margin(8.0)
        .show(ui, |ui| {
            add_contents(ui);
        });
}

fn paint_dimension_card(ui: &mut egui::Ui, card: &DimensionCard) {
    paint_glass_frame(ui, |ui| {
        let color = card.meta.color;

        // Dimension name
        ui.label(
            egui::RichText::new(card.meta.name)
                .color(color)
                .size(11.0)
                .strong(),
        );

        // Arc gauge
        let (response, painter) = ui.allocate_painter(
            Vec2::new(ui.available_width(), ARC_RADIUS * 2.0 + 16.0),
            egui::Sense::hover(),
        );
        let center = response.rect.center();

        // Background arc, dim
        let bg_points = arc_points(center, ARC_RADIUS, ARC_START, ARC_SWEEP, ARC_POINTS);
        painter.add(Shape::line(
            bg_points,
            Stroke::new(2.0, Color32::from_rgba_unmultiplied(30, 30, 40, 100)),
        ));

        // Filled arc from reading value
        if let Some(value) = card.reading.display_value() {
            let fill_sweep = ARC_SWEEP * value.gauge_fraction;
            let fill_points = arc_points(center, ARC_RADIUS, ARC_START, fill_sweep, ARC_POINTS);
            if fill_points.len() >= 2 {
                let shapes = glow_strokes(&fill_points, color, &GLOW_LAYERS);
                painter.extend(shapes);
            }

            // Center text: measurement value
            painter.text(
                center,
                egui::Align2::CENTER_CENTER,
                &value.display_text,
                egui::FontId::proportional(14.0),
                color,
            );

            // Label below center
            painter.text(
                Pos2::new(center.x, center.y + 16.0),
                egui::Align2::CENTER_CENTER,
                value.label,
                egui::FontId::proportional(10.0),
                Color32::from_gray(160),
            );
        } else {
            // Show phase text when no value available
            let phase_text = match &card.reading {
                DimensionReading::Idle => "\u{2014}",
                DimensionReading::Settling { .. } => "Settling...",
                _ => "",
            };
            painter.text(
                center,
                egui::Align2::CENTER_CENTER,
                phase_text,
                egui::FontId::proportional(12.0),
                Color32::from_gray(100),
            );
        }

        // Progress bar for active detectors
        if card.reading.is_active() {
            let bar_rect = Rect::from_min_size(
                Pos2::new(response.rect.left(), response.rect.bottom() - 3.0),
                Vec2::new(response.rect.width(), 3.0),
            );
            painter.rect_filled(bar_rect, 1.0, Color32::from_gray(30));
            let fill_rect = Rect::from_min_size(
                bar_rect.left_top(),
                Vec2::new(bar_rect.width() * card.reading.progress(), 3.0),
            );
            painter.rect_filled(fill_rect, 1.0, color);
        }

        // Sparkline history
        if card.history.len() > 1 {
            let spark_rect = ui.allocate_space(Vec2::new(ui.available_width(), 20.0)).1;
            let points = sparkline_points(&card.history, spark_rect);
            if points.len() >= 2 {
                let [r, g, b, _] = color.to_array();
                let spark_color = Color32::from_rgba_unmultiplied(r, g, b, 100);
                ui.painter()
                    .add(Shape::line(points, Stroke::new(1.0, spark_color)));
            }
        }
    });
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aura_arc_points_count() {
        let pts = arc_points(Pos2::new(100.0, 100.0), 50.0, 0.0, std::f32::consts::PI, 32);
        assert_eq!(pts.len(), 32);
    }

    #[test]
    fn aura_arc_points_minimum() {
        let pts = arc_points(Pos2::new(0.0, 0.0), 10.0, 0.0, 1.0, 1);
        assert_eq!(pts.len(), 2); // clamped to min 2
    }

    #[test]
    fn aura_glow_strokes_layer_count() {
        let pts = arc_points(Pos2::new(0.0, 0.0), 10.0, 0.0, 1.0, 8);
        let layers = [(2.0, 0.5), (1.0, 0.9)];
        let shapes = glow_strokes(&pts, Color32::WHITE, &layers);
        assert_eq!(shapes.len(), 2);
    }

    #[test]
    fn aura_sparkline_empty() {
        let pts = sparkline_points(&[], Rect::from_min_size(Pos2::ZERO, Vec2::new(100.0, 20.0)));
        assert!(pts.is_empty());
    }

    #[test]
    fn aura_sparkline_single_value() {
        let pts =
            sparkline_points(&[0.5], Rect::from_min_size(Pos2::ZERO, Vec2::new(100.0, 20.0)));
        assert_eq!(pts.len(), 1);
    }

    #[test]
    fn aura_sparkline_multiple_values_span_rect() {
        let rect = Rect::from_min_size(Pos2::new(10.0, 10.0), Vec2::new(100.0, 50.0));
        let pts = sparkline_points(&[0.0, 0.5, 1.0], rect);
        assert_eq!(pts.len(), 3);
        // First point at left edge
        assert!((pts[0].x - 10.0).abs() < 0.01);
        // Last point at right edge
        assert!((pts[2].x - 110.0).abs() < 0.01);
    }
}
