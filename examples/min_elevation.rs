use std::path::PathBuf;

use anyhow::{Context, Result};
use tiff::{
    decoder::{Decoder, DecodingResult},
    tags::Tag,
};

const TAG_GDAL_NODATA: u16 = 42113;

fn main() -> Result<()> {
    let dir = PathBuf::from(
        std::env::args()
            .nth(1)
            .unwrap_or_else(|| "data/DGM1".to_string()),
    );

    let mut files: Vec<_> = std::fs::read_dir(&dir)
        .with_context(|| format!("reading {}", dir.display()))?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| {
            p.is_file()
                && p.extension()
                    .map(|x| x.eq_ignore_ascii_case("tif") || x.eq_ignore_ascii_case("tiff"))
                    .unwrap_or(false)
        })
        .collect();
    files.sort();

    let mut global_min = f64::INFINITY;
    let mut global_min_file: Option<PathBuf> = None;
    let mut global_min_idx: usize = 0;

    for path in &files {
        let (min_val, min_idx, cols) = match scan_tile(path) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("WARN {}: {e}", path.display());
                continue;
            }
        };
        if min_val.is_finite() {
            let row = min_idx / cols;
            let col = min_idx % cols;
            println!(
                "{}: min = {:.3} m at pixel (col={}, row={})",
                path.file_name().unwrap().to_string_lossy(),
                min_val,
                col,
                row
            );
            if min_val < global_min {
                global_min = min_val;
                global_min_file = Some(path.clone());
                global_min_idx = min_idx;
            }
        } else {
            println!(
                "{}: no valid pixels",
                path.file_name().unwrap().to_string_lossy()
            );
        }
    }

    println!();
    match global_min_file {
        Some(p) => println!(
            "LOWEST POINT: {:.3} m in {} (flat index {})",
            global_min,
            p.file_name().unwrap().to_string_lossy(),
            global_min_idx
        ),
        None => println!("No valid pixels found in any tile."),
    }
    Ok(())
}

fn scan_tile(path: &std::path::Path) -> Result<(f64, usize, usize)> {
    let file = std::fs::File::open(path)?;
    let mut dec = Decoder::new(file)?;
    let (cols, _rows) = dec.dimensions()?;
    let cols = cols as usize;

    let nodata: Option<f64> = dec
        .find_tag(Tag::Unknown(TAG_GDAL_NODATA))
        .ok()
        .flatten()
        .and_then(|v| match v {
            tiff::decoder::ifd::Value::Ascii(s) => s.trim_end_matches('\0').parse().ok(),
            _ => None,
        });

    let raw = dec.read_image()?;
    let (min_val, min_idx) = scan_raw(raw, nodata);
    Ok((min_val, min_idx, cols))
}

fn scan_raw(result: DecodingResult, nodata: Option<f64>) -> (f64, usize) {
    let is_nodata = |f: f64| -> bool {
        if let Some(nd) = nodata
            && (f - nd).abs() < 1.0
        {
            return true;
        }
        f < -1_000_000.0
    };
    let mut min_val = f64::INFINITY;
    let mut min_idx: usize = 0;
    let mut update = |f: f64, i: usize| {
        if f.is_finite() && !is_nodata(f) && f < min_val {
            min_val = f;
            min_idx = i;
        }
    };
    match result {
        DecodingResult::F32(v) => {
            for (i, x) in v.into_iter().enumerate() {
                update(x as f64, i);
            }
        }
        DecodingResult::F64(v) => {
            for (i, x) in v.into_iter().enumerate() {
                update(x, i);
            }
        }
        DecodingResult::I16(v) => {
            for (i, x) in v.into_iter().enumerate() {
                update(x as f64, i);
            }
        }
        DecodingResult::I32(v) => {
            for (i, x) in v.into_iter().enumerate() {
                update(x as f64, i);
            }
        }
        DecodingResult::U16(v) => {
            for (i, x) in v.into_iter().enumerate() {
                update(x as f64, i);
            }
        }
        DecodingResult::U32(v) => {
            for (i, x) in v.into_iter().enumerate() {
                update(x as f64, i);
            }
        }
        DecodingResult::U8(v) => {
            for (i, x) in v.into_iter().enumerate() {
                update(x as f64, i);
            }
        }
        _ => {}
    }
    (min_val, min_idx)
}
