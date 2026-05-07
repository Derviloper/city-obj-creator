mod cli;
mod export;
mod geometry;
mod model;
mod parsers;

use std::collections::HashMap;
use std::time::Duration;

use anyhow::{Context, Result};
use clap::Parser;
use glam::DVec3;
use indicatif::{ProgressBar, ProgressStyle};

use cli::Cli;
use export::obj::ObjWriter;
use geometry::clip::clip_mesh_to_aabb_xy;
use geometry::manifold::{build_manifold_building, build_manifold_global_terrain, check_manifold};
use model::building::Building;
use model::tile::{BBox2, Tile};
use parsers::{
    citygml::{collect_gml_files, stream_buildings},
    dgm1::{list_tiles, read_tile_terrain},
};

/// Skip clipped building pieces whose XY footprint is below this many m².
/// Anything smaller is almost certainly a numerical sliver from a building
/// that just barely overlaps a tile boundary.
const MIN_PIECE_AREA_M2: f64 = 1e-3;

fn main() -> Result<()> {
    let cli = Cli::parse();
    std::fs::create_dir_all(&cli.output)?;

    // ------------------------------------------------------------------
    // Discover tiles: every output sub-folder corresponds to one .tif.
    // ------------------------------------------------------------------
    let tiles = list_tiles(&cli.dgm1)?;
    if tiles.is_empty() {
        eprintln!("ERROR: no DGM1 tiles found in {}", cli.dgm1.display());
        std::process::exit(1);
    }
    println!("Found {} DGM1 tile(s)", tiles.len());

    // Pre-create one sub-folder per tile.
    for tile in &tiles {
        let dir = cli.output.join(&tile.name);
        std::fs::create_dir_all(&dir)
            .with_context(|| format!("creating output sub-folder {}", dir.display()))?;
    }

    // ------------------------------------------------------------------
    // Phase 1: terrain. Each tile is its own watertight solid.
    // ------------------------------------------------------------------
    write_per_tile_terrain(&tiles, &cli.output)?;

    // ------------------------------------------------------------------
    // Phase 2: buildings. Stream GMLs, clip each building to overlapping
    // tiles, write per-tile pieces.
    // ------------------------------------------------------------------
    write_per_tile_buildings(&tiles, &cli)?;

    println!("Done.");
    Ok(())
}

fn write_per_tile_terrain(tiles: &[Tile], output: &std::path::Path) -> Result<()> {
    let pb = progress_bar(tiles.len() as u64, "terrain tiles");
    for tile in tiles {
        pb.set_message(tile.name.clone());

        let grid = match read_tile_terrain(tile) {
            Ok(g) => g,
            Err(e) => {
                eprintln!("WARN: terrain read failed for {}: {e}", tile.name);
                pb.inc(1);
                continue;
            }
        };
        let (verts, faces) = build_manifold_global_terrain(&grid);
        if verts.is_empty() || faces.is_empty() {
            pb.inc(1);
            continue;
        }

        let path = output.join(&tile.name).join("terrain.obj");
        let mut writer = ObjWriter::open(&path)
            .with_context(|| format!("opening {}", path.display()))?;
        writer.write_comment("city-obj-creator — terrain")?;
        writer.write_comment("CRS: ETRS89/UTM32N EPSG:25832")?;
        writer.write_comment(&format!(
            "tile_origin_utm: x={:.3} y={:.3}",
            tile.origin_x, tile.origin_y
        ))?;
        writer.write_newline()?;
        writer.write_object("Terrain")?;
        let base = writer.write_vertices(&verts, tile.obj_origin())?;
        writer.write_faces(&faces, base)?;
        writer.flush()?;

        check_manifold(&format!("terrain[{}]", tile.name), &faces);
        pb.inc(1);
    }
    pb.finish_with_message("terrain done");
    Ok(())
}

fn write_per_tile_buildings(tiles: &[Tile], cli: &Cli) -> Result<()> {
    let gml_files = collect_gml_files(&cli.citygml)?;
    if gml_files.is_empty() {
        eprintln!("WARN: no .gml/.xml files found in {}", cli.citygml.display());
        return Ok(());
    }

    // Open one writer per tile up front; close them all at the end.
    let mut writers: HashMap<String, ObjWriter> = HashMap::with_capacity(tiles.len());
    for tile in tiles {
        let path = cli.output.join(&tile.name).join("buildings.obj");
        let mut w = ObjWriter::open(&path)
            .with_context(|| format!("opening {}", path.display()))?;
        w.write_comment("city-obj-creator — buildings")?;
        w.write_comment("CRS: ETRS89/UTM32N EPSG:25832")?;
        w.write_comment(&format!(
            "tile_origin_utm: x={:.3} y={:.3}",
            tile.origin_x, tile.origin_y
        ))?;
        w.write_newline()?;
        writers.insert(tile.name.clone(), w);
    }

    let pb = progress_bar(gml_files.len() as u64, "CityGML files");
    let mut tile_building_counts: HashMap<String, u64> = HashMap::new();

    for path in &gml_files {
        let file_name = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("?")
            .to_string();
        pb.set_message(file_name.clone());

        let result = stream_buildings(path, |building: Building| -> Result<()> {
            let (verts, tris) = build_manifold_building(&building);
            if verts.is_empty() || tris.is_empty() {
                return Ok(());
            }
            let bldg_bbox = match xy_bbox(&verts) {
                Some(b) => b,
                None => return Ok(()),
            };

            for tile in tiles {
                let tile_bbox = tile.bbox();
                if !tile_bbox.intersects(bldg_bbox) {
                    continue;
                }

                // Fast path: building fully inside this tile, no clip needed.
                let (piece_v, piece_t) = if tile_bbox.contains_bbox(bldg_bbox) {
                    (verts.clone(), tris.clone())
                } else {
                    clip_mesh_to_aabb_xy(&verts, &tris, tile_bbox)
                };

                if piece_v.is_empty() || piece_t.is_empty() {
                    continue;
                }
                if let Some(area) = xy_bbox(&piece_v).map(bbox_area)
                    && area < MIN_PIECE_AREA_M2
                {
                    continue;
                }

                check_manifold(&format!("{}@{}", building.id, tile.name), &piece_t);

                let writer = writers
                    .get_mut(&tile.name)
                    .expect("writer must exist for known tile");
                let name = sanitize_id(&building.id);
                writer.write_object(&format!("Building_{name}"))?;
                let base = writer.write_vertices(&piece_v, tile.obj_origin())?;
                writer.write_faces(&piece_t, base)?;
                writer.write_newline()?;
                *tile_building_counts.entry(tile.name.clone()).or_default() += 1;
            }
            Ok(())
        });

        if let Err(e) = result {
            eprintln!("WARN: error in {}: {e}", path.display());
        }
        pb.inc(1);
    }
    pb.finish_with_message("buildings done");

    // Final flush + spinner.
    let flush_pb = ProgressBar::new_spinner();
    flush_pb.set_style(
        ProgressStyle::with_template("[{elapsed_precise}] {spinner} {msg}")
            .unwrap_or_else(|_| ProgressStyle::default_spinner()),
    );
    flush_pb.enable_steady_tick(Duration::from_millis(100));
    flush_pb.set_message("flushing buildings.obj files");
    for (_, mut writer) in writers.drain() {
        writer.flush()?;
    }
    flush_pb.finish_with_message("buildings flushed");

    let total_pieces: u64 = tile_building_counts.values().sum();
    println!(
        "Wrote {} building piece(s) across {} tile folder(s)",
        total_pieces,
        tile_building_counts.len()
    );
    Ok(())
}

fn xy_bbox(verts: &[DVec3]) -> Option<BBox2> {
    if verts.is_empty() {
        return None;
    }
    let mut xmin = f64::INFINITY;
    let mut ymin = f64::INFINITY;
    let mut xmax = f64::NEG_INFINITY;
    let mut ymax = f64::NEG_INFINITY;
    for v in verts {
        if v.x < xmin {
            xmin = v.x;
        }
        if v.y < ymin {
            ymin = v.y;
        }
        if v.x > xmax {
            xmax = v.x;
        }
        if v.y > ymax {
            ymax = v.y;
        }
    }
    Some(BBox2 {
        xmin,
        ymin,
        xmax,
        ymax,
    })
}

fn bbox_area(b: BBox2) -> f64 {
    (b.xmax - b.xmin).max(0.0) * (b.ymax - b.ymin).max(0.0)
}

fn progress_bar(len: u64, label: &str) -> ProgressBar {
    let pb = ProgressBar::new(len);
    pb.set_style(
        ProgressStyle::with_template(&format!(
            "[{{elapsed_precise}}] {{bar:40}} {{pos}}/{{len}} {label} {{msg}}"
        ))
        .unwrap_or_else(|_| ProgressStyle::default_bar()),
    );
    pb
}

fn sanitize_id(id: &str) -> String {
    id.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' || c == '-' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

