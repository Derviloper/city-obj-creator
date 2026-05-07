use std::path::Path;

use anyhow::{Context, Result};
use tiff::{
    decoder::{Decoder, DecodingResult},
    tags::Tag,
};

use crate::model::terrain::GlobalTerrain;

// GDAL nodata tag
const TAG_GDAL_NODATA: u16 = 42113;

/// Elevation grid for one DGM1 GeoTIFF tile, used internally to merge tiles
/// into a `GlobalTerrain`. Geo-transform is read separately via
/// `read_tile_meta`; this struct only carries pixels + dimensions.
struct TerrainTile {
    cols: usize,
    rows: usize,
    z: Vec<f64>,
}

/// Read pixel data from a single DGM1 GeoTIFF tile.
/// The tile must be a single-band float raster in ETRS89/UTM32N (EPSG:25832).
fn read_tile(path: &Path) -> Result<TerrainTile> {
    let file = std::fs::File::open(path)
        .with_context(|| format!("opening DGM1 tile: {}", path.display()))?;
    let mut dec = Decoder::new(file)
        .context("creating TIFF decoder")?;

    let (cols, rows) = dec.dimensions()
        .context("reading TIFF dimensions")?;
    let cols = cols as usize;
    let rows = rows as usize;

    // Read the GDAL nodata value if present (stored as ASCII string e.g. "-10000000000")
    let nodata: Option<f64> = dec
        .find_tag(Tag::Unknown(TAG_GDAL_NODATA))
        .ok()
        .flatten()
        .and_then(|v| match v {
            tiff::decoder::ifd::Value::Ascii(s) => s.trim_end_matches('\0').parse().ok(),
            _ => None,
        });

    let raw = dec.read_image().context("reading TIFF image data")?;
    let z = raster_to_f64(raw, cols * rows, nodata);

    Ok(TerrainTile { cols, rows, z })
}

fn extract_geo_transform(
    dec: &mut Decoder<std::fs::File>,
    path: &Path,
) -> Result<(f64, f64, f64)> {
    // Prefer ModelTransformationTag (4×4 affine matrix, 16 f64 values).
    if let Ok(Some(floats)) = dec
        .find_tag(Tag::ModelTransformationTag)
        .map(|r| r.map(tag_to_f64_vec))
        && floats.len() >= 12
    {
        let tx = floats[3];
        let ty = floats[7];
        let sx = floats[0];
        let sy = floats[5].abs();
        return Ok((tx, ty, sx.max(sy)));
    }

    // Fallback: ModelPixelScaleTag + ModelTiepointTag
    let scale_vals = dec
        .find_tag(Tag::ModelPixelScaleTag)
        .with_context(|| format!("reading ModelPixelScaleTag in {}", path.display()))?
        .map(tag_to_f64_vec)
        .unwrap_or_default();

    let tie_vals = dec
        .find_tag(Tag::ModelTiepointTag)
        .with_context(|| format!("reading ModelTiepointTag in {}", path.display()))?
        .map(tag_to_f64_vec)
        .unwrap_or_default();

    anyhow::ensure!(
        scale_vals.len() >= 2,
        "ModelPixelScaleTag missing or has fewer than 2 values in {}",
        path.display()
    );
    anyhow::ensure!(
        tie_vals.len() >= 6,
        "ModelTiepointTag missing or has fewer than 6 values in {}",
        path.display()
    );

    // Tiepoint format: [pixel_i, pixel_j, pixel_k, world_x, world_y, world_z, ...]
    let pixel_i = tie_vals[0];
    let pixel_j = tie_vals[1];
    let world_x = tie_vals[3];
    let world_y = tie_vals[4];
    let res_x = scale_vals[0];
    let res_y = scale_vals[1]; // always positive; Y decreases with row

    // The tiepoint gives the world coordinate of the pixel center at (pixel_i, pixel_j).
    // origin = top-left pixel center = world coord adjusted by any tiepoint offset.
    let origin_x = world_x - pixel_i * res_x;
    let origin_y = world_y + pixel_j * res_y; // world_y is north (top), Y goes south

    Ok((origin_x, origin_y, res_x))
}

fn tag_to_f64_vec(v: tiff::decoder::ifd::Value) -> Vec<f64> {
    use tiff::decoder::ifd::Value;

    fn scalar_to_f64(v: &Value) -> Option<f64> {
        match v {
            Value::Double(x) => Some(*x),
            Value::Float(x) => Some(*x as f64),
            Value::Unsigned(x) => Some(*x as f64),
            Value::UnsignedBig(x) => Some(*x as f64),
            Value::Short(x) => Some(*x as f64),
            Value::Byte(x) => Some(*x as f64),
            Value::Signed(x) => Some(*x as f64),
            Value::SignedBig(x) => Some(*x as f64),
            Value::Rational(n, d) => if *d != 0 { Some(*n as f64 / *d as f64) } else { None },
            Value::RationalBig(n, d) => if *d != 0 { Some(*n as f64 / *d as f64) } else { None },
            _ => None,
        }
    }

    match v {
        Value::List(vals) => vals.iter().filter_map(scalar_to_f64).collect(),
        other => scalar_to_f64(&other).into_iter().collect(),
    }
}

fn raster_to_f64(result: DecodingResult, expected: usize, nodata: Option<f64>) -> Vec<f64> {
    // Treat a value as nodata if it equals the declared nodata value (within float tolerance),
    // or falls below a hard floor of -1_000_000 to catch common sentinel values.
    let is_nodata = move |f: f64| -> bool {
        if let Some(nd) = nodata {
            // Use relative comparison for large values
            if (f - nd).abs() < 1.0 {
                return true;
            }
        }
        f < -1_000_000.0
    };

    match result {
        DecodingResult::F32(v) => v.into_iter().map(|x| {
            let f = x as f64;
            if is_nodata(f) { f64::NAN } else { f }
        }).collect(),
        DecodingResult::F64(v) => v.into_iter().map(|x| {
            if is_nodata(x) { f64::NAN } else { x }
        }).collect(),
        DecodingResult::I16(v) => v.into_iter().map(|x| {
            let f = x as f64;
            if is_nodata(f) { f64::NAN } else { f }
        }).collect(),
        DecodingResult::U16(v) => v.into_iter().map(|x| x as f64).collect(),
        DecodingResult::I32(v) => v.into_iter().map(|x| {
            let f = x as f64;
            if is_nodata(f) { f64::NAN } else { f }
        }).collect(),
        DecodingResult::U32(v) => v.into_iter().map(|x| x as f64).collect(),
        DecodingResult::U8(v) => v.into_iter().map(|x| x as f64).collect(),
        _ => vec![f64::NAN; expected],
    }
}

/// Collect all .tif files in a directory, sorted by name.
pub fn collect_tif_files(dir: &Path) -> Result<Vec<std::path::PathBuf>> {
    let mut files: Vec<_> = std::fs::read_dir(dir)
        .with_context(|| format!("reading DGM1 directory: {}", dir.display()))?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| {
            p.is_file()
                && p.extension()
                    .map(|ext| {
                        ext.eq_ignore_ascii_case("tif") || ext.eq_ignore_ascii_case("tiff")
                    })
                    .unwrap_or(false)
        })
        .collect();
    files.sort();
    Ok(files)
}

/// Lightweight tile metadata: extent + dimensions + path.
struct TileMeta {
    path: std::path::PathBuf,
    origin_x: f64,
    origin_y: f64,
    res: f64,
    cols: usize,
    rows: usize,
}

/// Read just the geo-transform of a TIFF without decoding pixel data.
fn read_tile_meta(path: &Path) -> Result<TileMeta> {
    let file = std::fs::File::open(path)
        .with_context(|| format!("opening DGM1 tile: {}", path.display()))?;
    let mut dec = Decoder::new(file)
        .context("creating TIFF decoder")?;
    let (cols, rows) = dec.dimensions().context("reading TIFF dimensions")?;
    let (origin_x, origin_y, res) = extract_geo_transform(&mut dec, path)?;
    Ok(TileMeta {
        path: path.to_path_buf(),
        origin_x,
        origin_y,
        res,
        cols: cols as usize,
        rows: rows as usize,
    })
}

/// Read all DGM1 tiles in a directory and merge them into one combined elevation grid.
/// Adjacent tiles are stitched at their shared boundary pixels (no duplication, no seams).
///
/// Memory cost: peak ≈ size of the combined grid (8 bytes per pixel) + one tile.
pub fn read_global_terrain(dir: &Path) -> Result<Option<GlobalTerrain>> {
    let tif_files = collect_tif_files(dir)?;
    if tif_files.is_empty() {
        return Ok(None);
    }

    // Pass 1: read metadata of every tile.
    let mut metas: Vec<TileMeta> = Vec::new();
    for path in &tif_files {
        let m = match read_tile_meta(path) {
            Ok(m) => m,
            Err(e) => {
                eprintln!("WARN: skipping {}: {e}", path.display());
                continue;
            }
        };
        metas.push(m);
    }

    if metas.is_empty() {
        return Ok(None);
    }

    // All tiles must share the same resolution (true for DGM1 = 1m).
    let res = metas[0].res;
    anyhow::ensure!(
        metas.iter().all(|m| (m.res - res).abs() < 1e-6),
        "DGM1 tiles have inconsistent resolutions"
    );

    // Compute combined extent from all tile pixel centers.
    let global_xmin = metas.iter().map(|m| m.origin_x).fold(f64::INFINITY, f64::min);
    let global_ymax = metas.iter().map(|m| m.origin_y).fold(f64::NEG_INFINITY, f64::max);
    let global_xmax = metas
        .iter()
        .map(|m| m.origin_x + (m.cols as f64 - 1.0) * m.res)
        .fold(f64::NEG_INFINITY, f64::max);
    let global_ymin = metas
        .iter()
        .map(|m| m.origin_y - (m.rows as f64 - 1.0) * m.res)
        .fold(f64::INFINITY, f64::min);

    let cols = ((global_xmax - global_xmin) / res).round() as usize + 1;
    let rows = ((global_ymax - global_ymin) / res).round() as usize + 1;

    let mut z = vec![f64::NAN; cols * rows];

    // Pass 2: read each tile's pixels and copy into the right slot of the global grid.
    let pb = indicatif::ProgressBar::new(metas.len() as u64);
    pb.set_style(
        indicatif::ProgressStyle::with_template(
            "[{elapsed_precise}] {bar:40} {pos}/{len} merging tiles {msg}",
        )
        .unwrap_or_else(|_| indicatif::ProgressStyle::default_bar()),
    );

    for m in &metas {
        pb.set_message(
            m.path
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("?")
                .to_string(),
        );

        let tile = match read_tile(&m.path) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("WARN: skipping {}: {e}", m.path.display());
                pb.inc(1);
                continue;
            }
        };

        let col_offset = ((m.origin_x - global_xmin) / res).round() as usize;
        let row_offset = ((global_ymax - m.origin_y) / res).round() as usize;

        for r in 0..tile.rows {
            let dst_row = row_offset + r;
            if dst_row >= rows {
                break;
            }
            let dst_base = dst_row * cols + col_offset;
            let src_base = r * tile.cols;
            let copy_cols = tile.cols.min(cols.saturating_sub(col_offset));
            // Overwrite (later tile wins on the 1-pixel boundary overlap, values match).
            z[dst_base..dst_base + copy_cols]
                .copy_from_slice(&tile.z[src_base..src_base + copy_cols]);
        }
        pb.inc(1);
        // tile dropped here -> memory freed
    }
    pb.finish_with_message("done");

    Ok(Some(GlobalTerrain {
        origin_x: global_xmin,
        origin_y: global_ymax,
        res,
        cols,
        rows,
        z,
    }))
}
