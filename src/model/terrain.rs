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
}
