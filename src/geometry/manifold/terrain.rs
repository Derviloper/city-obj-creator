use glam::DVec3;

use crate::model::terrain::GlobalTerrain;

/// Fixed elevation of the underside of the terrain mesh.
const Z_BASE: f64 = 250.0;

/// Build a single watertight terrain mesh from a combined elevation grid.
///
/// Output anatomy:
///   * Top surface: 2 triangles per grid cell where all four corners are valid.
///     Cells with NaN corners are simply skipped (small holes appear).
///   * Side walls: a vertical skirt. Every top boundary vertex has a matching
///     bottom vertex directly below at `Z_BASE`, and each boundary segment
///     becomes a quad (2 triangles) connecting top edge to bottom edge.
///   * Bottom cap: fan-triangulated over the bottom boundary loop. Some fan
///     triangles are degenerate (collinear points along a rectangle edge);
///     that is harmless — manifoldness only requires edge-incidence counts.
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

    // --- 1. Top vertices: one per grid pixel center, in row-major order. ---
    let n_boundary = 2 * (cols + rows) - 4;
    let mut verts: Vec<DVec3> = Vec::with_capacity(cols * rows + n_boundary);
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

    // --- 3. Boundary loop in CW-from-above order (NW → NE → SE → SW),
    // with one bottom vertex emitted directly below each top boundary vertex. ---
    let mut boundary: Vec<(usize, usize)> = Vec::with_capacity(n_boundary);
    for c in 0..cols { boundary.push((0, c)); }                  // North: NW → NE
    for r in 1..rows { boundary.push((r, cols - 1)); }           // East:  → SE
    for c in (0..cols - 1).rev() { boundary.push((rows - 1, c)); } // South: → SW
    for r in (1..rows - 1).rev() { boundary.push((r, 0)); }      // West:  → (back to NW)

    let n_b = boundary.len();
    let bot_start = verts.len() as u32;
    for &(r, c) in &boundary {
        verts.push(DVec3::new(grid.x_at(c), grid.y_at(r), Z_BASE));
    }
    let idx_bot = |i: usize| bot_start + i as u32;

    // --- 4. Skirt walls. Each boundary segment becomes a vertical quad
    // (top_curr, top_next, bot_next, bot_curr) split into 2 triangles.
    // Walking the boundary CW-from-above keeps outward normals pointing away. ---
    for i in 0..n_b {
        let i_next = (i + 1) % n_b;
        let (r0, c0) = boundary[i];
        let (r1, c1) = boundary[i_next];
        let top_curr = idx_top(r0, c0);
        let top_next = idx_top(r1, c1);
        let bot_curr = idx_bot(i);
        let bot_next = idx_bot(i_next);
        tris.push([bot_curr, top_curr, top_next]);
        tris.push([bot_curr, top_next, bot_next]);
    }

    // --- 5. Bottom cap: fan-triangulate the boundary loop from bot[0].
    // Outward normal -Z (CCW when viewed from below). ---
    for i in 1..n_b - 1 {
        tris.push([idx_bot(0), idx_bot(i), idx_bot(i + 1)]);
    }

    (verts, tris)
}
