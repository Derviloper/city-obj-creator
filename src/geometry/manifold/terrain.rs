use glam::DVec3;

use crate::model::terrain::GlobalTerrain;

/// Build a single watertight terrain mesh from a combined elevation grid.
///
/// Output anatomy:
///   * Top surface: 2 triangles per grid cell where all four corners are valid.
///     Cells with NaN corners are simply skipped (small holes appear).
///   * Side walls: each of the four outer rectangle faces is fan-triangulated
///     from one shared bottom corner, walking the top boundary vertices.
///     This keeps every edge shared by exactly 2 triangles (manifold).
///   * Bottom cap: just two triangles using the same 4 corner vertices.
///
/// The whole mesh is one closed solid with shared vertices throughout, so all
/// tiles connect seamlessly to their neighbours.
pub fn build_manifold_global_terrain(grid: &GlobalTerrain) -> (Vec<DVec3>, Vec<[u32; 3]>) {
    let mut tris: Vec<[u32; 3]> = Vec::new();

    if grid.cols < 2 || grid.rows < 2 {
        return (Vec::new(), tris);
    }

    let cols = grid.cols;
    let rows = grid.rows;

    let z_top_min = grid.min_z();
    let z_base = if z_top_min.is_finite() { z_top_min - 1.0 } else { -1.0 };

    // --- 1. Top vertices: one per grid pixel center, in row-major order. ---
    let mut verts: Vec<DVec3> = Vec::with_capacity(cols * rows + 4);
    for r in 0..rows {
        let y = grid.y_at(r);
        for c in 0..cols {
            let z = grid.z_at(c, r);
            verts.push(DVec3::new(grid.x_at(c), y, z));
        }
    }
    let idx_top = |r: usize, c: usize| (r * cols + c) as u32;

    // --- 2. Top surface triangles. Skip cells where any corner is NaN. ---
    for r in 0..rows - 1 {
        for c in 0..cols - 1 {
            let z00 = grid.z_at(c, r);
            let z10 = grid.z_at(c + 1, r);
            let z01 = grid.z_at(c, r + 1);
            let z11 = grid.z_at(c + 1, r + 1);
            if !(z00.is_finite() && z10.is_finite() && z01.is_finite() && z11.is_finite()) {
                continue;
            }
            let i00 = idx_top(r, c);
            let i10 = idx_top(r, c + 1);
            let i01 = idx_top(r + 1, c);
            let i11 = idx_top(r + 1, c + 1);
            // CCW from above (outward normal +Z).
            tris.push([i00, i01, i11]);
            tris.push([i00, i11, i10]);
        }
    }

    // --- 3. Four bottom corner vertices (only). ---
    let xmin = grid.x_at(0);
    let xmax = grid.x_at(cols - 1);
    let ymax = grid.y_at(0);
    let ymin = grid.y_at(rows - 1);
    let bot_nw = verts.len() as u32; verts.push(DVec3::new(xmin, ymax, z_base));
    let bot_ne = verts.len() as u32; verts.push(DVec3::new(xmax, ymax, z_base));
    let bot_se = verts.len() as u32; verts.push(DVec3::new(xmax, ymin, z_base));
    let bot_sw = verts.len() as u32; verts.push(DVec3::new(xmin, ymin, z_base));

    // --- 4. Skirt walls. Each wall is fan-triangulated from one bottom corner.
    //
    // North wall (row=0, outward +Y): fan apex = bot_nw, going west→east.
    for c in 0..cols - 1 {
        tris.push([bot_nw, idx_top(0, c), idx_top(0, c + 1)]);
    }
    tris.push([bot_nw, idx_top(0, cols - 1), bot_ne]);

    // East wall (col=cols-1, outward +X): fan apex = bot_ne, going north→south.
    for r in 0..rows - 1 {
        tris.push([bot_ne, idx_top(r, cols - 1), idx_top(r + 1, cols - 1)]);
    }
    tris.push([bot_ne, idx_top(rows - 1, cols - 1), bot_se]);

    // South wall (row=rows-1, outward -Y): fan apex = bot_se, going east→west.
    for c in (1..cols).rev() {
        tris.push([bot_se, idx_top(rows - 1, c), idx_top(rows - 1, c - 1)]);
    }
    tris.push([bot_se, idx_top(rows - 1, 0), bot_sw]);

    // West wall (col=0, outward -X): fan apex = bot_sw, going south→north.
    for r in (1..rows).rev() {
        tris.push([bot_sw, idx_top(r, 0), idx_top(r - 1, 0)]);
    }
    tris.push([bot_sw, idx_top(0, 0), bot_nw]);

    // --- 5. Bottom cap: 2 triangles, outward normal -Z (CW viewed from above). ---
    tris.push([bot_nw, bot_ne, bot_se]);
    tris.push([bot_nw, bot_se, bot_sw]);

    (verts, tris)
}
