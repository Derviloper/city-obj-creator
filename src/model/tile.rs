use std::path::PathBuf;

use glam::DVec3;

/// Axis-aligned 2D bounding box in world XY coordinates.
#[derive(Debug, Clone, Copy)]
pub struct BBox2 {
    pub xmin: f64,
    pub ymin: f64,
    pub xmax: f64,
    pub ymax: f64,
}

impl BBox2 {
    pub fn intersects(&self, other: BBox2) -> bool {
        !(other.xmax < self.xmin
            || other.xmin > self.xmax
            || other.ymax < self.ymin
            || other.ymin > self.ymax)
    }

    pub fn contains_bbox(&self, other: BBox2) -> bool {
        other.xmin >= self.xmin
            && other.xmax <= self.xmax
            && other.ymin >= self.ymin
            && other.ymax <= self.ymax
    }
}

/// One DGM1 tile: pixel-grid metadata + path. Constructed up-front from
/// the GeoTIFF header so all tiles' geo-transforms are known before we
/// start streaming buildings.
#[derive(Debug, Clone)]
pub struct Tile {
    /// File stem (no extension), used as the output sub-folder name.
    pub name: String,
    pub path: PathBuf,
    /// World coordinate of the NW pixel center.
    pub origin_x: f64,
    pub origin_y: f64,
    pub res: f64,
    pub cols: usize,
    pub rows: usize,
}

impl Tile {
    /// XY extent covering all pixel centers — i.e. NW pixel center to SE
    /// pixel center, inclusive. Matches the tile's terrain mesh footprint
    /// exactly so adjacent tiles share their boundary line.
    pub fn bbox(&self) -> BBox2 {
        let xmin = self.origin_x;
        let xmax = self.origin_x + (self.cols.saturating_sub(1)) as f64 * self.res;
        let ymax = self.origin_y;
        let ymin = self.origin_y - (self.rows.saturating_sub(1)) as f64 * self.res;
        BBox2 {
            xmin,
            ymin,
            xmax,
            ymax,
        }
    }

    /// Origin used when writing OBJ vertices (NW pixel center, z=0).
    pub fn obj_origin(&self) -> DVec3 {
        DVec3::new(self.origin_x, self.origin_y, 0.0)
    }
}
