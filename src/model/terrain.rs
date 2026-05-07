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

    /// Bilinear elevation lookup at world coordinate (x, y). Returns `None`
    /// when the point falls outside the grid, or when any of the four
    /// surrounding samples is NaN (no-data). Callers should treat `None` as
    /// "do not emit geometry here" — falling back to nearest-valid neighbors
    /// risks placing meshes well above true ground at nodata patch edges.
    pub fn sample_z(&self, x: f64, y: f64) -> Option<f64> {
        if self.cols < 2 || self.rows < 2 || self.res <= 0.0 {
            return None;
        }
        let fx = (x - self.origin_x) / self.res;
        let fy = (self.origin_y - y) / self.res;
        let cmax = (self.cols - 1) as f64;
        let rmax = (self.rows - 1) as f64;
        if fx < 0.0 || fy < 0.0 || fx > cmax || fy > rmax {
            return None;
        }
        let c0 = fx.floor() as usize;
        let r0 = fy.floor() as usize;
        let c1 = (c0 + 1).min(self.cols - 1);
        let r1 = (r0 + 1).min(self.rows - 1);
        let z00 = self.z_at(c0, r0);
        let z10 = self.z_at(c1, r0);
        let z01 = self.z_at(c0, r1);
        let z11 = self.z_at(c1, r1);
        if z00.is_nan() || z10.is_nan() || z01.is_nan() || z11.is_nan() {
            return None;
        }
        let tx = fx - c0 as f64;
        let ty = fy - r0 as f64;
        let z0 = z00 * (1.0 - tx) + z10 * tx;
        let z1 = z01 * (1.0 - tx) + z11 * tx;
        Some(z0 * (1.0 - ty) + z1 * ty)
    }
}
