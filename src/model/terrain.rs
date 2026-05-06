
/// A single combined elevation grid stitched together from many DGM1 tiles.
/// Pixel centers at (col, row) are at world coordinates
/// `(origin_x + col*res, origin_y - row*res)`. NaN = no data.
pub struct GlobalTerrain {
    pub origin_x: f64,
    pub origin_y: f64,
    pub res: f64,
    pub cols: usize,
    pub rows: usize,
    pub z: Vec<f64>,
}

impl GlobalTerrain {
    pub fn x_at(&self, col: usize) -> f64 {
        self.origin_x + col as f64 * self.res
    }
    pub fn y_at(&self, row: usize) -> f64 {
        self.origin_y - row as f64 * self.res
    }
    pub fn z_at(&self, col: usize, row: usize) -> f64 {
        self.z[row * self.cols + col]
    }
    pub fn min_z(&self) -> f64 {
        self.z.iter().copied().filter(|v| v.is_finite()).fold(f64::INFINITY, f64::min)
    }
}

/// Elevation grid for one DGM1 GeoTIFF tile.
/// Coordinates are in ETRS89/UTM32N (EPSG:25832).
pub struct TerrainTile {
    /// UTM easting of the center of the top-left pixel.
    pub origin_x: f64,
    /// UTM northing of the center of the top-left pixel.
    pub origin_y: f64,
    /// Pixel spacing in meters (positive; DGM1 = 1.0).
    pub res: f64,
    pub cols: usize,
    pub rows: usize,
    /// Row-major elevation values. f64::NAN = no-data.
    pub z: Vec<f64>,
}

#[allow(dead_code)]
impl TerrainTile {
    pub fn x_at(&self, col: usize) -> f64 {
        self.origin_x + col as f64 * self.res
    }

    pub fn y_at(&self, row: usize) -> f64 {
        // GeoTIFF is north-up, so Y decreases with row index.
        self.origin_y - row as f64 * self.res
    }

    pub fn z_at(&self, col: usize, row: usize) -> f64 {
        self.z[row * self.cols + col]
    }

    pub fn min_z(&self) -> f64 {
        self.z.iter().copied().filter(|v| v.is_finite()).fold(f64::INFINITY, f64::min)
    }
}
