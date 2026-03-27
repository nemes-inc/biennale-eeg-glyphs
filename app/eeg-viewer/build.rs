//! Build script: parse head.obj and emit head_mesh_data.rs with const arrays.

use std::collections::HashSet;
use std::io::{BufRead, BufReader, Write};
use std::path::Path;

fn main() {
    let obj_path = Path::new("assets/head.obj");
    println!("cargo:rerun-if-changed={}", obj_path.display());

    let file = std::fs::File::open(obj_path).expect("assets/head.obj not found");

    let mut vertices: Vec<[f32; 3]> = Vec::new();
    let mut edges: HashSet<(u32, u32)> = HashSet::new();

    for line in BufReader::new(file).lines() {
        let line = line.unwrap();
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }

        match parts[0] {
            "v" if parts.len() >= 4 => {
                let x: f32 = parts[1].parse().unwrap();
                let y: f32 = parts[2].parse().unwrap();
                let z: f32 = parts[3].parse().unwrap();
                vertices.push([x, y, z]);
            }
            "f" if parts.len() >= 4 => {
                // Parse face indices, 1-based, may have v/vt/vn format
                let indices: Vec<u32> = parts[1..]
                    .iter()
                    .filter_map(|s| s.split('/').next()?.parse::<u32>().ok())
                    .map(|i| i - 1)
                    .collect();
                // Extract edges from face, deduplicate by canonical order
                for i in 0..indices.len() {
                    let a = indices[i];
                    let b = indices[(i + 1) % indices.len()];
                    let edge = if a < b { (a, b) } else { (b, a) };
                    edges.insert(edge);
                }
            }
            _ => {}
        }
    }

    // Normalize vertices to [-1, 1] range centered at origin
    let (mut min_x, mut min_y, mut min_z) = (f32::MAX, f32::MAX, f32::MAX);
    let (mut max_x, mut max_y, mut max_z) = (f32::MIN, f32::MIN, f32::MIN);
    for v in &vertices {
        min_x = min_x.min(v[0]);
        max_x = max_x.max(v[0]);
        min_y = min_y.min(v[1]);
        max_y = max_y.max(v[1]);
        min_z = min_z.min(v[2]);
        max_z = max_z.max(v[2]);
    }
    let cx = (min_x + max_x) / 2.0;
    let cy = (min_y + max_y) / 2.0;
    let cz = (min_z + max_z) / 2.0;
    let scale = ((max_x - min_x).max(max_y - min_y).max(max_z - min_z)) / 2.0;

    let normalized: Vec<[f32; 3]> = vertices
        .iter()
        .map(|v| [
            (v[0] - cx) / scale,
            (v[1] - cy) / scale,
            (v[2] - cz) / scale,
        ])
        .collect();

    let out_dir = std::env::var("OUT_DIR").unwrap();
    let out_path = Path::new(&out_dir).join("head_mesh_data.rs");
    let mut out = std::fs::File::create(out_path).unwrap();

    writeln!(out, "/// Auto-generated from head.obj by build.rs").unwrap();
    writeln!(
        out,
        "pub const HEAD_VERTICES: &[[f32; 3]; {}] = &[",
        normalized.len()
    )
    .unwrap();
    for v in &normalized {
        writeln!(out, "    [{:.6}, {:.6}, {:.6}],", v[0], v[1], v[2]).unwrap();
    }
    writeln!(out, "];").unwrap();

    let edges_vec: Vec<(u32, u32)> = edges.into_iter().collect();
    writeln!(
        out,
        "pub const HEAD_EDGES: &[[u32; 2]; {}] = &[",
        edges_vec.len()
    )
    .unwrap();
    for (a, b) in &edges_vec {
        writeln!(out, "    [{}, {}],", a, b).unwrap();
    }
    writeln!(out, "];").unwrap();

    eprintln!(
        "head_mesh: {} vertices, {} edges",
        normalized.len(),
        edges_vec.len()
    );
}
