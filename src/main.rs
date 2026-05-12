mod cli;
mod export;
mod geometry;
mod model;
mod parsers;

use std::collections::HashMap;
use std::time::Duration;

use anyhow::{Context, Result};
use clap::Parser;
use glam::{DVec2, DVec3};
use indicatif::{ProgressBar, ProgressStyle};

use cli::Cli;
use export::obj::ObjWriter;
use geometry::clip::clip_mesh_to_aabb_xy;
use geometry::manifold::{
    build_manifold_building, build_manifold_global_terrain, build_manifold_road,
    build_manifold_water, check_manifold,
};
use model::alkis::{AlkisKind, SurfaceFeature};
use model::building::Building;
use model::tile::{shared_obj_origin, BBox2, Tile};
use parsers::{
    alkis::{collect_alkis_files, stream_alkis_features},
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
    // Discover tiles: every output filename is prefixed with one tile name.
    // ------------------------------------------------------------------
    let tiles = list_tiles(&cli.dgm1)?;
    if tiles.is_empty() {
        eprintln!("ERROR: no DGM1 tiles found in {}", cli.dgm1.display());
        std::process::exit(1);
    }
    println!("Found {} DGM1 tile(s)", tiles.len());

    // Shared OBJ anchor (SW-most corner across all tiles). Every per-tile
    // OBJ in this run subtracts the same origin, so loading any combination
    // of them into one scene preserves real-world relative positions.
    let shared_origin =
        shared_obj_origin(&tiles).expect("tiles non-empty (checked above)");
    println!(
        "Shared OBJ origin (UTM32N): x={:.3} y={:.3}",
        shared_origin.x, shared_origin.y
    );

    // ------------------------------------------------------------------
    // Phase 1: terrain. Each tile is its own watertight solid.
    // ------------------------------------------------------------------
    write_per_tile_terrain(&tiles, &cli.output, shared_origin)?;

    // ------------------------------------------------------------------
    // Phase 2: buildings. Stream GMLs, clip each building to overlapping
    // tiles, write per-tile pieces.
    // ------------------------------------------------------------------
    write_per_tile_buildings(&tiles, &cli, shared_origin)?;

    // ------------------------------------------------------------------
    // Phase 3: ALKIS surface features (optional). Stream ALKIS, extrude
    // over terrain, clip to tile bbox, write per-tile road.obj / rail.obj.
    // ------------------------------------------------------------------
    if cli.alkis.is_some() {
        write_per_tile_alkis(&tiles, &cli, shared_origin)?;
    }

    println!("Done.");
    Ok(())
}

fn write_per_tile_terrain(
    tiles: &[Tile],
    output: &std::path::Path,
    shared_origin: DVec3,
) -> Result<()> {
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

        let path = output.join(format!("{}_terrain.obj", tile.name));
        let mut writer = ObjWriter::open(&path)
            .with_context(|| format!("opening {}", path.display()))?;
        writer.write_comment("city-obj-creator — terrain")?;
        writer.write_comment("CRS: ETRS89/UTM32N EPSG:25832")?;
        writer.write_comment(&format!(
            "obj_origin_utm: x={:.3} y={:.3}  # subtracted from every vertex; identical across all tiles in this run",
            shared_origin.x, shared_origin.y
        ))?;
        writer.write_comment(&format!(
            "tile_nw_corner_utm: x={:.3} y={:.3}  # this tile's NW pixel center (informational)",
            tile.origin_x, tile.origin_y
        ))?;
        writer.write_newline()?;
        writer.write_object("Terrain")?;
        let base = writer.write_vertices(&verts, shared_origin)?;
        writer.write_faces(&faces, base)?;
        writer.flush()?;

        check_manifold(&format!("terrain[{}]", tile.name), &faces);
        pb.inc(1);
    }
    pb.finish_with_message("terrain done");
    Ok(())
}

fn write_per_tile_buildings(tiles: &[Tile], cli: &Cli, shared_origin: DVec3) -> Result<()> {
    let gml_files = collect_gml_files(&cli.citygml)?;
    if gml_files.is_empty() {
        eprintln!("WARN: no .gml/.xml files found in {}", cli.citygml.display());
        return Ok(());
    }

    // Open one writer per tile up front; close them all at the end.
    let mut writers: HashMap<String, ObjWriter> = HashMap::with_capacity(tiles.len());
    for tile in tiles {
        let path = cli.output.join(format!("{}_buildings.obj", tile.name));
        let mut w = ObjWriter::open(&path)
            .with_context(|| format!("opening {}", path.display()))?;
        w.write_comment("city-obj-creator — buildings")?;
        w.write_comment("CRS: ETRS89/UTM32N EPSG:25832")?;
        w.write_comment(&format!(
            "obj_origin_utm: x={:.3} y={:.3}  # subtracted from every vertex; identical across all tiles in this run",
            shared_origin.x, shared_origin.y
        ))?;
        w.write_comment(&format!(
            "tile_nw_corner_utm: x={:.3} y={:.3}  # this tile's NW pixel center (informational)",
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
                let base = writer.write_vertices(&piece_v, shared_origin)?;
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
        "Wrote {} building piece(s) across {} tile(s)",
        total_pieces,
        tile_building_counts.len()
    );
    Ok(())
}

/// Per-kind output configuration: filename, header label, and OBJ object-name
/// prefix. Adding a new ALKIS surface kind is one entry here plus a parser
/// match arm and an `AlkisKind` variant.
fn alkis_kind_output(kind: AlkisKind) -> (&'static str, &'static str, &'static str) {
    match kind {
        AlkisKind::Strassenverkehr => ("road.obj", "roads", "Road"),
        AlkisKind::Bahnverkehr => ("rail.obj", "rail", "Rail"),
        AlkisKind::Weg => ("path.obj", "paths", "Path"),
        AlkisKind::Fliessgewaesser => ("water.obj", "water", "WaterFlow"),
        AlkisKind::StehendesGewaesser => ("water.obj", "water", "WaterStill"),
    }
}

const ALKIS_KINDS: &[AlkisKind] = &[
    AlkisKind::Strassenverkehr,
    AlkisKind::Bahnverkehr,
    AlkisKind::Weg,
    AlkisKind::Fliessgewaesser,
    AlkisKind::StehendesGewaesser,
];

fn write_per_tile_alkis(tiles: &[Tile], cli: &Cli, shared_origin: DVec3) -> Result<()> {
    let alkis_dir = match cli.alkis.as_ref() {
        Some(d) => d,
        None => return Ok(()),
    };
    let alkis_files = collect_alkis_files(alkis_dir)?;
    if alkis_files.is_empty() {
        eprintln!("WARN: no .xml files found in {}", alkis_dir.display());
        return Ok(());
    }

    // Stage 1: stream once, fan out features into per-tile buckets.
    // Storing 2D rings is cheap; one full terrain grid in memory at a time
    // (Stage 2) is the real memory budget.
    let mut per_tile: HashMap<String, Vec<SurfaceFeature>> = HashMap::new();
    let pb = progress_bar(alkis_files.len() as u64, "ALKIS files");
    for path in &alkis_files {
        let file_name = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("?")
            .to_string();
        pb.set_message(file_name);

        let result = stream_alkis_features(path, |feat: SurfaceFeature| -> Result<()> {
            let bb = match ring_xy_bbox(&feat.exterior) {
                Some(b) => b,
                None => return Ok(()),
            };
            for tile in tiles {
                if tile.bbox().intersects(bb) {
                    per_tile
                        .entry(tile.name.clone())
                        .or_default()
                        .push(feat.clone());
                }
            }
            Ok(())
        });
        if let Err(e) = result {
            eprintln!("WARN: error in {}: {e}", path.display());
        }
        pb.inc(1);
    }
    pb.finish_with_message("ALKIS streamed");

    // Stage 2: per tile, load terrain once, build/clip/write each feature
    // routed to the per-kind writer.
    let pb = progress_bar(tiles.len() as u64, "ALKIS tiles");
    let mut total_pieces: HashMap<AlkisKind, u64> = HashMap::new();
    for tile in tiles {
        pb.set_message(tile.name.clone());
        let features = match per_tile.get(&tile.name) {
            Some(f) if !f.is_empty() => f,
            _ => {
                pb.inc(1);
                continue;
            }
        };

        let terrain = match read_tile_terrain(tile) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("WARN: terrain read failed for {}: {e}", tile.name);
                pb.inc(1);
                continue;
            }
        };

        // One writer per output filename in this tile. Keying by filename
        // (not AlkisKind) lets several kinds share a file — e.g. both water
        // kinds write into `water.obj` while keeping distinct object names.
        let mut writers: HashMap<&'static str, ObjWriter> = HashMap::new();
        let tile_bbox = tile.bbox();
        for feat in features {
            let (piece_v, piece_t) = match feat.kind {
                AlkisKind::Strassenverkehr | AlkisKind::Bahnverkehr | AlkisKind::Weg => {
                    build_manifold_road(feat, &terrain, tile_bbox)
                }
                AlkisKind::Fliessgewaesser | AlkisKind::StehendesGewaesser => {
                    build_manifold_water(feat, &terrain, tile_bbox)
                }
            };
            if piece_v.is_empty() || piece_t.is_empty() {
                continue;
            }
            if let Some(area) = xy_bbox(&piece_v).map(bbox_area)
                && area < MIN_PIECE_AREA_M2
            {
                continue;
            }

            let (filename, label, prefix) = alkis_kind_output(feat.kind);
            check_manifold(&format!("{prefix}_{}@{}", feat.id, tile.name), &piece_t);

            let writer = match writers.entry(filename) {
                std::collections::hash_map::Entry::Occupied(e) => e.into_mut(),
                std::collections::hash_map::Entry::Vacant(e) => {
                    let path = cli.output.join(format!("{}_{}", tile.name, filename));
                    let mut w = ObjWriter::open(&path)
                        .with_context(|| format!("opening {}", path.display()))?;
                    w.write_comment(&format!("city-obj-creator — {label}"))?;
                    w.write_comment("CRS: ETRS89/UTM32N EPSG:25832")?;
                    w.write_comment(&format!(
                        "obj_origin_utm: x={:.3} y={:.3}  # subtracted from every vertex; identical across all tiles in this run",
                        shared_origin.x, shared_origin.y
                    ))?;
                    w.write_comment(&format!(
                        "tile_nw_corner_utm: x={:.3} y={:.3}  # this tile's NW pixel center (informational)",
                        tile.origin_x, tile.origin_y
                    ))?;
                    w.write_newline()?;
                    e.insert(w)
                }
            };

            let name = sanitize_id(&feat.id);
            writer.write_object(&format!("{prefix}_{name}"))?;
            let base = writer.write_vertices(&piece_v, shared_origin)?;
            writer.write_faces(&piece_t, base)?;
            writer.write_newline()?;
            *total_pieces.entry(feat.kind).or_default() += 1;
        }

        for (_, mut writer) in writers.drain() {
            writer.flush()?;
        }
        pb.inc(1);
    }
    pb.finish_with_message("ALKIS done");

    for kind in ALKIS_KINDS {
        let count = total_pieces.get(kind).copied().unwrap_or(0);
        let (_, label, _) = alkis_kind_output(*kind);
        println!("Wrote {count} {label} piece(s)");
    }
    Ok(())
}

fn ring_xy_bbox(ring: &[DVec2]) -> Option<BBox2> {
    if ring.is_empty() {
        return None;
    }
    let mut xmin = f64::INFINITY;
    let mut ymin = f64::INFINITY;
    let mut xmax = f64::NEG_INFINITY;
    let mut ymax = f64::NEG_INFINITY;
    for p in ring {
        if p.x < xmin {
            xmin = p.x;
        }
        if p.y < ymin {
            ymin = p.y;
        }
        if p.x > xmax {
            xmax = p.x;
        }
        if p.y > ymax {
            ymax = p.y;
        }
    }
    Some(BBox2 {
        xmin,
        ymin,
        xmax,
        ymax,
    })
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

