use std::path::Path;

use anyhow::{Context, Result};
use tiff::{
    decoder::{Decoder, DecodingResult},
    tags::Tag,
};

use crate::model::terrain::GlobalTerrain;
use crate::model::tile::Tile;

// GDAL nodata tag
const TAG_GDAL_NODATA: u16 = 42113;

/// Pixel data of a single DGM1 GeoTIFF tile.
struct TilePixels {
    cols: usize,
    rows: usize,
    z: Vec<f64>,
}

/// Read pixel data from a single DGM1 GeoTIFF tile.
/// The tile must be a single-band float raster in ETRS89/UTM32N (EPSG:25832).
fn read_tile_pixels(path: &Path) -> Result<TilePixels> {
    let file = std::fs::File::open(path)
        .with_context(|| format!("opening DGM1 tile: {}", path.display()))?;
    let mut dec = Decoder::new(file).context("creating TIFF decoder")?;

    let (cols, rows) = dec.dimensions().context("reading TIFF dimensions")?;
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

    Ok(TilePixels { cols, rows, z })
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
        if let Some(nd) = nodata
            && (f - nd).abs() < 1.0
        {
            return true;
        }
        f < -1_000_000.0
    };

    match result {
        DecodingResult::F32(v) => v
            .into_iter()
            .map(|x| {
                let f = x as f64;
                if is_nodata(f) { f64::NAN } else { f }
            })
            .collect(),
        DecodingResult::F64(v) => v
            .into_iter()
            .map(|x| if is_nodata(x) { f64::NAN } else { x })
            .collect(),
        DecodingResult::I16(v) => v
            .into_iter()
            .map(|x| {
                let f = x as f64;
                if is_nodata(f) { f64::NAN } else { f }
            })
            .collect(),
        DecodingResult::U16(v) => v.into_iter().map(|x| x as f64).collect(),
        DecodingResult::I32(v) => v
            .into_iter()
            .map(|x| {
                let f = x as f64;
                if is_nodata(f) { f64::NAN } else { f }
            })
            .collect(),
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
                    .map(|ext| ext.eq_ignore_ascii_case("tif") || ext.eq_ignore_ascii_case("tiff"))
                    .unwrap_or(false)
        })
        .collect();
    files.sort();
    Ok(files)
}

/// Read just the geo-transform of a TIFF without decoding pixel data.
/// Returns a fully-populated `Tile` (path + dimensions + origin).
fn read_tile_meta(path: &Path) -> Result<Tile> {
    let file = std::fs::File::open(path)
        .with_context(|| format!("opening DGM1 tile: {}", path.display()))?;
    let mut dec = Decoder::new(file).context("creating TIFF decoder")?;
    let (cols, rows) = dec.dimensions().context("reading TIFF dimensions")?;
    let (origin_x, origin_y, res) = extract_geo_transform(&mut dec, path)?;
    let name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("tile")
        .to_string();
    Ok(Tile {
        name,
        path: path.to_path_buf(),
        origin_x,
        origin_y,
        res,
        cols: cols as usize,
        rows: rows as usize,
    })
}

/// List every DGM1 tile in `dir`, returning their geo-metadata (no pixels).
pub fn list_tiles(dir: &Path) -> Result<Vec<Tile>> {
    let tif_files = collect_tif_files(dir)?;
    let mut tiles = Vec::with_capacity(tif_files.len());
    for path in &tif_files {
        match read_tile_meta(path) {
            Ok(t) => tiles.push(t),
            Err(e) => {
                eprintln!("WARN: skipping {}: {e}", path.display());
            }
        }
    }
    Ok(tiles)
}

/// Read a tile's pixels and wrap them in a `GlobalTerrain` (single-tile grid).
/// Memory cost: one tile's worth of f64s (≈ cols*rows * 8 bytes).
pub fn read_tile_terrain(tile: &Tile) -> Result<GlobalTerrain> {
    let pixels = read_tile_pixels(&tile.path)?;
    anyhow::ensure!(
        pixels.cols == tile.cols && pixels.rows == tile.rows,
        "pixel dimensions {}x{} of {} don't match metadata {}x{}",
        pixels.cols,
        pixels.rows,
        tile.path.display(),
        tile.cols,
        tile.rows,
    );
    Ok(GlobalTerrain {
        origin_x: tile.origin_x,
        origin_y: tile.origin_y,
        res: tile.res,
        cols: pixels.cols,
        rows: pixels.rows,
        z: pixels.z,
    })
}
