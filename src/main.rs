mod cli;
mod export;
mod geometry;
mod model;
mod parsers;

use std::time::Duration;

use anyhow::Result;
use clap::Parser;
use glam::DVec3;
use indicatif::{ProgressBar, ProgressStyle};

use cli::Cli;
use export::obj::ObjWriter;
use geometry::manifold::{build_manifold_building, build_manifold_global_terrain};
use model::building::Building;
use parsers::{
    citygml::{collect_gml_files, stream_buildings},
    dgm1::read_global_terrain,
};

fn main() -> Result<()> {
    let cli = Cli::parse();

    std::fs::create_dir_all(&cli.output)?;

    // A single global XY origin shared by both terrain and buildings.
    // All output coordinates are (utm_x - origin.x, utm_y - origin.y, utm_z).
    // This keeps values small for 3D viewer compatibility while preserving
    // relative positions between all objects. Set from first data encountered.
    let mut global_origin: Option<DVec3> = None;

    // ------------------------------------------------------------------
    // Terrain: merge all DGM1 tiles into one combined elevation grid,
    // build a single watertight mesh, write it out.
    // ------------------------------------------------------------------
    if let Some(dgm1_dir) = &cli.dgm1 {
        match read_global_terrain(dgm1_dir)? {
            None => {
                eprintln!("WARN: no DGM1 tiles loaded from {}", dgm1_dir.display());
            }
            Some(grid) => {
                let origin = *global_origin
                    .get_or_insert_with(|| DVec3::new(grid.origin_x, grid.origin_y, 0.0));

                let terrain_path = cli.output.join("terrain.obj");
                let mut writer = ObjWriter::open(&terrain_path)?;
                writer.write_comment("city-obj-creator — terrain")?;
                writer.write_comment("CRS: ETRS89/UTM32N EPSG:25832")?;
                writer.write_comment(&format!(
                    "global_origin_utm: x={:.3} y={:.3}",
                    origin.x, origin.y
                ))?;
                writer.write_newline()?;

                let build_pb = ProgressBar::new_spinner();
                build_pb.set_style(
                    ProgressStyle::with_template("[{elapsed_precise}] {spinner} {msg}")
                        .unwrap_or_else(|_| ProgressStyle::default_spinner()),
                );
                build_pb.enable_steady_tick(Duration::from_millis(100));
                build_pb.set_message(format!(
                    "building terrain mesh ({}×{} cells)",
                    grid.cols, grid.rows
                ));
                let (verts, faces) = build_manifold_global_terrain(&grid);
                build_pb.finish_with_message(format!(
                    "terrain mesh built: {} vertices, {} triangles",
                    verts.len(),
                    faces.len()
                ));

                if !verts.is_empty() {
                    writer.write_object("Terrain")?;
                    let base = write_vertices_chunked(&mut writer, &verts, origin, "writing terrain vertices")?;
                    write_faces_chunked(&mut writer, &faces, base, "writing terrain faces")?;
                }

                let flush_pb = ProgressBar::new_spinner();
                flush_pb.set_style(
                    ProgressStyle::with_template("[{elapsed_precise}] {spinner} {msg}")
                        .unwrap_or_else(|_| ProgressStyle::default_spinner()),
                );
                flush_pb.enable_steady_tick(Duration::from_millis(100));
                flush_pb.set_message("flushing terrain.obj");
                writer.flush()?;
                flush_pb.finish_with_message("terrain.obj flushed");
                println!(
                    "Terrain written to {} ({} vertices, {} triangles)",
                    terrain_path.display(),
                    verts.len(),
                    faces.len()
                );
            }
        }
    }

    // ------------------------------------------------------------------
    // Buildings: stream one GML file at a time, write incrementally.
    // All buildings share the same global origin established above.
    // ------------------------------------------------------------------
    let gml_files = collect_gml_files(&cli.citygml)?;
    if gml_files.is_empty() {
        eprintln!("WARN: no .gml/.xml files found in {}", cli.citygml.display());
        return Ok(());
    }

    let buildings_path = cli.output.join("buildings.obj");
    let mut writer = ObjWriter::open(&buildings_path)?;
    writer.write_comment("city-obj-creator — buildings")?;
    writer.write_comment("CRS: ETRS89/UTM32N EPSG:25832")?;
    writer.write_newline()?;

    let pb = progress_bar(gml_files.len() as u64, "CityGML files");

    for path in &gml_files {
        pb.set_message(
            path.file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("?")
                .to_string(),
        );

        let result = stream_buildings(path, |building: Building| {
            let (verts, faces) = build_manifold_building(&building);
            if verts.is_empty() || faces.is_empty() {
                return Ok(());
            }

            // Lock global origin from the first building if terrain wasn't processed.
            let origin = *global_origin.get_or_insert_with(|| {
                building
                    .bounds()
                    .map(|(bmin, _)| DVec3::new(bmin.x, bmin.y, 0.0))
                    .unwrap_or(DVec3::ZERO)
            });

            let name = sanitize_id(&building.id);
            writer.write_object(&format!("Building_{name}"))?;
            let base = writer.write_vertices(&verts, origin)?;
            writer.write_faces(&faces, base)?;
            writer.write_newline()?;
            Ok(())
        });

        if let Err(e) = result {
            eprintln!("WARN: error in {}: {e}", path.display());
        }
        pb.inc(1);
    }

    if let Some(origin) = global_origin {
        writer.write_comment(&format!(
            "global_origin_utm: x={:.3} y={:.3}",
            origin.x, origin.y
        ))?;
    }

    pb.finish_with_message("done");
    writer.flush()?;
    println!("Buildings written to {}", buildings_path.display());

    Ok(())
}

fn progress_bar(len: u64, label: &str) -> ProgressBar {
    let pb = ProgressBar::new(len);
    pb.set_style(
        ProgressStyle::with_template(
            &format!("[{{elapsed_precise}}] {{bar:40}} {{pos}}/{{len}} {label} {{msg}}"),
        )
        .unwrap_or_else(|_| ProgressStyle::default_bar()),
    );
    pb
}

/// Write vertices to the OBJ writer in chunks so a progress bar can advance.
/// Returns the base vertex index of the first chunk (suitable for face refs).
fn write_vertices_chunked(
    writer: &mut ObjWriter,
    verts: &[DVec3],
    origin: DVec3,
    label: &str,
) -> Result<u64> {
    const CHUNK: usize = 100_000;
    let pb = progress_bar(verts.len() as u64, label);
    let mut first_base: Option<u64> = None;
    for chunk in verts.chunks(CHUNK) {
        let base = writer.write_vertices(chunk, origin)?;
        first_base.get_or_insert(base);
        pb.inc(chunk.len() as u64);
    }
    pb.finish_with_message("done");
    Ok(first_base.unwrap_or(writer.vertex_count))
}

fn write_faces_chunked(
    writer: &mut ObjWriter,
    faces: &[[u32; 3]],
    base: u64,
    label: &str,
) -> Result<()> {
    const CHUNK: usize = 100_000;
    let pb = progress_bar(faces.len() as u64, label);
    for chunk in faces.chunks(CHUNK) {
        writer.write_faces(chunk, base)?;
        pb.inc(chunk.len() as u64);
    }
    pb.finish_with_message("done");
    Ok(())
}

fn sanitize_id(id: &str) -> String {
    id.chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '_' || c == '-' { c } else { '_' })
        .collect()
}
