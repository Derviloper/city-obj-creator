use std::collections::HashMap;

use glam::DVec3;

use crate::model::{
    building::{Building, Polygon, Surface, SurfaceType},
    terrain::{GlobalTerrain, TerrainTile},
};
use super::{
    normals::{fan_triangulate, polygon_normal},
    vertex_merge::VertexMerger,
};

// ---------------------------------------------------------------------------
// Combined-terrain manifold mesh (preferred over per-tile)
// ---------------------------------------------------------------------------

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
pub fn build_manifold_global_terrain(
    grid: &GlobalTerrain,
) -> (Vec<DVec3>, Vec<[u32; 3]>) {
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

// ---------------------------------------------------------------------------
// Per-tile terrain manifold mesh (legacy; not used in the main pipeline)
// ---------------------------------------------------------------------------

#[allow(dead_code)]
pub fn build_manifold_terrain(tile: &TerrainTile) -> (Vec<DVec3>, Vec<[u32; 3]>) {
    let mut merger = VertexMerger::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();

    // Height of the skirt bottom (1m below the tile minimum elevation).
    let z_base = tile.min_z() - 1.0;

    // --- 1. Top surface and skirt ---
    // Process each grid cell [row, col].
    // valid[row][col] = true if the cell has finite elevation.
    let valid: Vec<bool> = tile.z.iter().map(|z| z.is_finite()).collect();
    let idx = |r: usize, c: usize| r * tile.cols + c;

    // Track which top-surface edges are boundary edges (one valid cell, the other missing/OOB).
    // We'll build skirt quads for these.
    struct SkirtEdge {
        a: DVec3, // world-space top vertex 1
        b: DVec3, // world-space top vertex 2
    }
    let mut skirt_edges: Vec<SkirtEdge> = Vec::new();

    for row in 0..tile.rows.saturating_sub(1) {
        for col in 0..tile.cols.saturating_sub(1) {
            // Four corners of the cell.
            let tl = (row, col);
            let tr = (row, col + 1);
            let bl = (row + 1, col);
            let br = (row + 1, col + 1);

            let all_valid = valid[idx(tl.0, tl.1)]
                && valid[idx(tr.0, tr.1)]
                && valid[idx(bl.0, bl.1)]
                && valid[idx(br.0, br.1)];

            if !all_valid {
                continue;
            }

            let vtl = DVec3::new(tile.x_at(tl.1), tile.y_at(tl.0), tile.z_at(tl.1, tl.0));
            let vtr = DVec3::new(tile.x_at(tr.1), tile.y_at(tr.0), tile.z_at(tr.1, tr.0));
            let vbl = DVec3::new(tile.x_at(bl.1), tile.y_at(bl.0), tile.z_at(bl.1, bl.0));
            let vbr = DVec3::new(tile.x_at(br.1), tile.y_at(br.0), tile.z_at(br.1, br.0));

            // Two triangles for the top cell — CCW viewed from above.
            let itl = merger.insert(vtl);
            let itr = merger.insert(vtr);
            let ibl = merger.insert(vbl);
            let ibr = merger.insert(vbr);

            tris.push([itl, ibl, ibr]);
            tris.push([itl, ibr, itr]);

            // Collect boundary edges for the skirt.
            // Top edge (row, col) → (row, col+1): boundary if row == 0
            if row == 0 {
                skirt_edges.push(SkirtEdge { a: vtl, b: vtr });
            }
            // Bottom edge: boundary if next row is OOB or invalid
            if row + 2 == tile.rows
                || !(valid[idx(row + 2, col)] && valid[idx(row + 2, col + 1)])
            {
                skirt_edges.push(SkirtEdge { a: vbr, b: vbl }); // reversed for outward normal
            }
            // Left edge: boundary if col == 0
            if col == 0 {
                skirt_edges.push(SkirtEdge { a: vbl, b: vtl });
            }
            // Right edge: boundary if next col is OOB or invalid
            if col + 2 == tile.cols
                || !(valid[idx(row, col + 2)] && valid[idx(row + 1, col + 2)])
            {
                skirt_edges.push(SkirtEdge { a: vtr, b: vbr });
            }
        }
    }

    // --- 2. Skirt walls ---
    // For each boundary edge, emit a quad (two triangles) dropping to z_base.
    let mut bottom_perimeter: Vec<DVec3> = Vec::new();
    for edge in &skirt_edges {
        let a_top = edge.a;
        let b_top = edge.b;
        let a_bot = DVec3::new(a_top.x, a_top.y, z_base);
        let b_bot = DVec3::new(b_top.x, b_top.y, z_base);

        let ia_top = merger.insert(a_top);
        let ib_top = merger.insert(b_top);
        let ia_bot = merger.insert(a_bot);
        let ib_bot = merger.insert(b_bot);

        // Quad: a_top, b_top, b_bot, a_bot — CCW from outside (outward normal points outward)
        tris.push([ia_top, ia_bot, ib_bot]);
        tris.push([ia_top, ib_bot, ib_top]);

        bottom_perimeter.push(a_bot);
    }

    // --- 3. Bottom cap ---
    // Collect unique bottom perimeter vertices and fan-triangulate.
    if !bottom_perimeter.is_empty() {
        // Compute centroid of bottom perimeter.
        let centroid = bottom_perimeter.iter().copied().sum::<DVec3>()
            / bottom_perimeter.len() as f64;
        let centroid = DVec3::new(centroid.x, centroid.y, z_base);
        let ic = merger.insert(centroid);

        // Fan from centroid to each consecutive pair of perimeter vertices.
        // Reverse winding (CW from above = CCW from below = outward normal down).
        for i in 0..bottom_perimeter.len() {
            let a = bottom_perimeter[i];
            let b = bottom_perimeter[(i + 1) % bottom_perimeter.len()];
            let ia = merger.insert(a);
            let ib = merger.insert(b);
            tris.push([ic, ib, ia]); // reversed for downward-facing normal
        }
    }

    (merger.verts, tris)
}

// ---------------------------------------------------------------------------
// Building manifold mesh
// ---------------------------------------------------------------------------

/// Vertex-snap tolerance used by the building merger and T-junction fixer (5 mm).
const BUILDING_TOL: f64 = 0.005;

/// Convert a building's surfaces into a manifold mesh.
/// Pipeline:
///  1. Snap vertices in every polygon to a shared 5 mm grid (collapses near-duplicates).
///  2. Insert intermediate vertices to remove T-junctions (the dominant source
///     of non-manifold edges in CityGML LoD2 data).
///  3. Synthesize a ground surface if the source has none.
///  4. Re-orient face windings so all normals point outward.
///  5. Warn if any boundary edges remain after all of the above.
pub fn build_manifold_building(building: &Building) -> (Vec<DVec3>, Vec<[u32; 3]>) {
    // ---- (1) Snap vertices on a per-polygon basis to a global 5mm grid. ----
    let mut polygons: Vec<(SurfaceType, Vec<DVec3>)> = Vec::new();
    for surface in &building.surfaces {
        for polygon in &surface.polygons {
            if polygon.0.len() < 3 {
                continue;
            }
            let snapped: Vec<DVec3> = polygon.0.iter().map(|v| snap_vec(*v, BUILDING_TOL)).collect();
            polygons.push((surface.surface_type, snapped));
        }
    }

    // ---- (2) T-junction fixing: ensure every vertex on a polygon edge is
    //          part of every other polygon that touches that edge. ----
    fix_t_junctions(&mut polygons, BUILDING_TOL);

    // ---- (3) Drop spurious internal horizontal "floors".
    //
    // Some LVermGeo CityGML files mark a BuildingPart's upper-level floor as
    // a `GroundSurface`. Including it produces an internal slab (faces shared
    // by 3+ triangles, non-manifold). We only want exterior-skin polygons:
    //   * Keep ground polygons at min_z (the real bottom).
    //   * Drop any `GroundSurface` polygon whose vertices lie >1 m above min_z.
    //   * Likewise, drop any flat horizontal polygon (any type) whose vertices
    //     sit at the same Z, that Z isn't min_z, and there are walls extending
    //     both above and below it.
    let min_z = polygons
        .iter()
        .flat_map(|(_, p)| p.iter())
        .map(|v| v.z)
        .fold(f64::INFINITY, f64::min);

    polygons.retain(|(t, verts)| {
        if *t == SurfaceType::Ground {
            // Keep only if at the actual ground.
            verts.iter().all(|v| (v.z - min_z).abs() < 0.5)
        } else {
            true
        }
    });

    // ---- (4) Synthesize a ground polygon at min_z if nothing closes the bottom. ----
    let has_floor_at_min_z = polygons.iter().any(|(_, verts)| {
        verts.iter().all(|v| (v.z - min_z).abs() < 0.05)
    });

    if !has_floor_at_min_z && min_z.is_finite() {
        if let Some(ground_verts) = synthesize_ground_from_polygons(&polygons, min_z) {
            polygons.push((SurfaceType::Ground, ground_verts));
            // Re-run T-junction fixer with the new ground polygon included.
            fix_t_junctions(&mut polygons, BUILDING_TOL);
        }
    }

    // ---- (4) Build the actual mesh: vertex-merge + fan-triangulate + winding. ----
    let mut merger = VertexMerger::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();

    let centroid = building.centroid_xy().unwrap_or(DVec3::ZERO);

    for (surface_type, verts) in &polygons {
        if verts.len() < 3 {
            continue;
        }
        let indices: Vec<u32> = verts.iter().map(|v| merger.insert(*v)).collect();

        let normal = polygon_normal(verts);
        let expected_outward = expected_normal_direction(*surface_type, centroid, verts);

        let raw_tris = fan_triangulate(&indices);
        let flip = normal.dot(expected_outward) < 0.0;

        for tri in raw_tris {
            // Drop degenerate triangles created by collinear/duplicate vertices.
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
                continue;
            }
            if flip {
                tris.push([tri[0], tri[2], tri[1]]);
            } else {
                tris.push(tri);
            }
        }
    }

    // ---- (5) Manifold check. ----
    check_manifold(building.id.as_str(), &tris, merger.len());

    (merger.verts, tris)
}

/// Snap a vertex to a fixed grid for stable equality testing.
fn snap_vec(v: DVec3, tol: f64) -> DVec3 {
    let inv = 1.0 / tol;
    DVec3::new(
        (v.x * inv).round() / inv,
        (v.y * inv).round() / inv,
        (v.z * inv).round() / inv,
    )
}

/// Insert intermediate vertices into polygon edges so that T-junctions become
/// proper shared vertices. For each edge (a, b) in every polygon, if any other
/// vertex `v` from any polygon lies on the open segment (a, b) within `tol`,
/// `v` is inserted between `a` and `b` in the polygon's vertex list.
///
/// Two passes: first collect every vertex that ever appears anywhere; then
/// rewrite every polygon's vertex list with extra splits.
fn fix_t_junctions(polygons: &mut [(SurfaceType, Vec<DVec3>)], tol: f64) {
    // Build the global set of unique snapped vertices.
    use std::collections::HashSet;
    let mut all_verts: Vec<DVec3> = Vec::new();
    let mut seen: HashSet<(i64, i64, i64)> = HashSet::new();
    let key_of = |v: DVec3, tol: f64| -> (i64, i64, i64) {
        let inv = 1.0 / tol;
        (
            (v.x * inv).round() as i64,
            (v.y * inv).round() as i64,
            (v.z * inv).round() as i64,
        )
    };
    for (_, verts) in polygons.iter() {
        for v in verts {
            if seen.insert(key_of(*v, tol)) {
                all_verts.push(*v);
            }
        }
    }

    // Helper: is vertex `v` on the open segment (a, b)?
    // Returns Some(t) where 0 < t < 1 if yes, None otherwise.
    fn on_segment(a: DVec3, b: DVec3, v: DVec3, tol: f64) -> Option<f64> {
        let d = b - a;
        let d_len_sq = d.length_squared();
        if d_len_sq < tol * tol {
            return None;
        }
        let t = (v - a).dot(d) / d_len_sq;
        // Strictly between endpoints, with a small tolerance margin.
        let margin = (tol / d_len_sq.sqrt()).max(1e-6);
        if t <= margin || t >= 1.0 - margin {
            return None;
        }
        let p = a + t * d;
        if (v - p).length() < tol {
            Some(t)
        } else {
            None
        }
    }

    let tol_sq = tol * tol;
    for (_, verts) in polygons.iter_mut() {
        let mut new_verts: Vec<DVec3> = Vec::with_capacity(verts.len() * 2);
        let n = verts.len();
        for i in 0..n {
            let a = verts[i];
            let b = verts[(i + 1) % n];
            new_verts.push(a);

            // Find every other unique vertex sitting on segment (a, b).
            let mut splits: Vec<(f64, DVec3)> = Vec::new();
            for v in &all_verts {
                // Skip vertices that ARE a or b.
                if (*v - a).length_squared() < tol_sq {
                    continue;
                }
                if (*v - b).length_squared() < tol_sq {
                    continue;
                }
                if let Some(t) = on_segment(a, b, *v, tol) {
                    splits.push((t, *v));
                }
            }
            splits.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
            for (_, v) in splits {
                new_verts.push(v);
            }
        }
        *verts = new_verts;
    }
}

/// Determine the expected outward normal direction for a surface.
fn expected_normal_direction(
    surface_type: SurfaceType,
    centroid: DVec3,
    polygon_verts: &[DVec3],
) -> DVec3 {
    match surface_type {
        SurfaceType::Roof => DVec3::new(0.0, 0.0, 1.0), // upward
        SurfaceType::Ground => DVec3::new(0.0, 0.0, -1.0), // downward
        SurfaceType::Wall | SurfaceType::Closure | SurfaceType::Unknown => {
            // Outward from building centroid.
            let face_center = polygon_verts.iter().copied().sum::<DVec3>()
                / polygon_verts.len() as f64;
            let outward = face_center - DVec3::new(centroid.x, centroid.y, face_center.z);
            if outward.length_squared() > 1e-10 {
                outward.normalize()
            } else {
                DVec3::X
            }
        }
    }
}

/// Stitch wall-bottom edges from the polygon list into a closed ground polygon.
///
/// Algorithm: collect every horizontal edge whose two endpoints both sit at
/// `min_z`, then walk the directed-edge graph until we return to the starting
/// vertex. The result follows the actual building footprint (handles L-shapes,
/// concave outlines, etc.) far better than a convex hull would.
fn synthesize_ground_from_polygons(
    polygons: &[(SurfaceType, Vec<DVec3>)],
    min_z: f64,
) -> Option<Vec<DVec3>> {
    let z_tol = 0.05;

    // Collect every polygon edge whose two endpoints sit at min_z (within tol).
    // Each edge is stored as (from, to) world coords.
    let mut edges: Vec<(DVec3, DVec3)> = Vec::new();
    for (_, verts) in polygons {
        let n = verts.len();
        for i in 0..n {
            let a = verts[i];
            let b = verts[(i + 1) % n];
            if (a.z - min_z).abs() < z_tol && (b.z - min_z).abs() < z_tol {
                // Skip degenerate edges
                if (a - b).length_squared() < 1e-8 {
                    continue;
                }
                edges.push((a, b));
            }
        }
    }
    if edges.len() < 3 {
        return None;
    }

    // Snap all coordinates so we can use them as map keys.
    let snap_xy = |v: DVec3| -> (i64, i64) {
        ((v.x * 1000.0).round() as i64, (v.y * 1000.0).round() as i64)
    };

    // Build adjacency map: vertex → list of next vertices.
    use std::collections::HashMap;
    let mut next: HashMap<(i64, i64), Vec<DVec3>> = HashMap::new();
    let mut all_points: HashMap<(i64, i64), DVec3> = HashMap::new();
    for (a, b) in &edges {
        let ka = snap_xy(*a);
        all_points.entry(ka).or_insert(*a);
        all_points.entry(snap_xy(*b)).or_insert(*b);
        next.entry(ka).or_default().push(*b);
    }

    // Each edge that's used by exactly one polygon (i.e. an outer perimeter
    // edge) appears in our list once; an internal edge between two adjacent
    // polygons appears twice in opposite directions. We want only the
    // perimeter edges (those whose reverse is NOT in the list).
    let mut edge_keys: HashMap<((i64, i64), (i64, i64)), i32> = HashMap::new();
    for (a, b) in &edges {
        let ka = snap_xy(*a);
        let kb = snap_xy(*b);
        *edge_keys.entry((ka, kb)).or_insert(0) += 1;
    }
    let perimeter_edges: Vec<(DVec3, DVec3)> = edges
        .iter()
        .copied()
        .filter(|(a, b)| {
            let ka = snap_xy(*a);
            let kb = snap_xy(*b);
            // Perimeter edge: forward count = 1 and reverse count = 0
            edge_keys.get(&(ka, kb)).copied().unwrap_or(0) == 1
                && edge_keys.get(&(kb, ka)).copied().unwrap_or(0) == 0
        })
        .collect();

    if perimeter_edges.len() < 3 {
        return None;
    }

    // Walk the perimeter graph: start from any edge and follow next-edges
    // until we close the loop.
    let mut next_perim: HashMap<(i64, i64), DVec3> = HashMap::new();
    for (a, b) in &perimeter_edges {
        next_perim.insert(snap_xy(*a), *b);
    }

    let start = perimeter_edges[0].0;
    let start_key = snap_xy(start);
    let mut loop_verts: Vec<DVec3> = vec![start];
    let mut current = start;
    for _ in 0..perimeter_edges.len() + 1 {
        let key = snap_xy(current);
        let Some(nxt) = next_perim.get(&key).copied() else {
            return None;
        };
        if snap_xy(nxt) == start_key {
            break;
        }
        loop_verts.push(nxt);
        current = nxt;
    }

    if loop_verts.len() < 3 {
        return None;
    }

    // Project all vertices to the exact min_z so the ground is perfectly flat.
    let flat: Vec<DVec3> = loop_verts
        .into_iter()
        .map(|v| DVec3::new(v.x, v.y, min_z))
        .collect();

    Some(flat)
}

/// Legacy convex-hull ground synthesizer (kept for reference, currently unused).
#[allow(dead_code)]
fn synthesize_ground(building: &Building, min_z: f64) -> Surface {
    // Collect all vertices at or near the minimum Z.
    let threshold = min_z + 0.1; // 10cm tolerance
    let mut base_verts: Vec<DVec3> = building
        .surfaces
        .iter()
        .flat_map(|s| s.polygons.iter())
        .flat_map(|p| p.0.iter())
        .filter(|v| v.z <= threshold)
        .map(|v| DVec3::new(v.x, v.y, min_z))
        .collect();

    if base_verts.is_empty() {
        return Surface {
            surface_type: SurfaceType::Ground,
            polygons: Vec::new(),
        };
    }

    // Deduplicate
    base_verts.dedup_by(|a, b| {
        (a.x - b.x).abs() < 0.01 && (a.y - b.y).abs() < 0.01
    });

    // Compute convex hull (simple 2D Graham scan).
    let hull = convex_hull_2d(&base_verts);

    Surface {
        surface_type: SurfaceType::Ground,
        polygons: if hull.len() >= 3 {
            vec![Polygon(hull)]
        } else {
            Vec::new()
        },
    }
}

/// 2D convex hull (Graham scan). Returns CCW-ordered vertices.
fn convex_hull_2d(points: &[DVec3]) -> Vec<DVec3> {
    if points.len() < 3 {
        return points.to_vec();
    }

    let mut pts: Vec<DVec3> = points.to_vec();
    // Find the point with the lowest Y (then lowest X) as pivot.
    let pivot_idx = pts
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            a.y.partial_cmp(&b.y)
                .unwrap()
                .then(a.x.partial_cmp(&b.x).unwrap())
        })
        .map(|(i, _)| i)
        .unwrap();
    pts.swap(0, pivot_idx);
    let pivot = pts[0];

    // Sort by polar angle from pivot.
    pts[1..].sort_by(|a, b| {
        let angle_a = (a.y - pivot.y).atan2(a.x - pivot.x);
        let angle_b = (b.y - pivot.y).atan2(b.x - pivot.x);
        angle_a.partial_cmp(&angle_b).unwrap()
    });

    // Graham scan.
    let mut hull: Vec<DVec3> = Vec::new();
    for p in pts {
        while hull.len() >= 2 {
            let n = hull.len();
            let cross = cross_z(hull[n - 2], hull[n - 1], p);
            if cross <= 0.0 {
                hull.pop();
            } else {
                break;
            }
        }
        hull.push(p);
    }

    hull
}

fn cross_z(o: DVec3, a: DVec3, b: DVec3) -> f64 {
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
}

/// Warn if the mesh has boundary edges (non-manifold indicator).
/// If the env var `DEBUG_MANIFOLD` is set, the boundary edge endpoints are dumped.
fn check_manifold(building_id: &str, tris: &[[u32; 3]], _vertex_count: usize) {
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in tris {
        for e in 0..3 {
            let a = tri[e];
            let b = tri[(e + 1) % 3];
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_count.entry(key).or_insert(0) += 1;
        }
    }
    let boundary: usize = edge_count.values().filter(|&&c| c == 1).count();
    let nonmanifold: usize = edge_count.values().filter(|&&c| c > 2).count();
    if boundary > 0 || nonmanifold > 0 {
        eprintln!(
            "WARN: building '{}' has {} boundary edge(s), {} edges with >2 faces",
            building_id, boundary, nonmanifold
        );
    }
}

/// Dump the world-space endpoints of every boundary edge for a building.
/// Helpful for debugging non-manifold meshes.
pub fn dump_boundary_edges(
    building_id: &str,
    verts: &[DVec3],
    tris: &[[u32; 3]],
) {
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in tris {
        for e in 0..3 {
            let a = tri[e];
            let b = tri[(e + 1) % 3];
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_count.entry(key).or_insert(0) += 1;
        }
    }
    let mut boundary_edges: Vec<(u32, u32)> = edge_count
        .iter()
        .filter(|&(_, &c)| c == 1)
        .map(|(k, _)| *k)
        .collect();
    boundary_edges.sort();
    eprintln!("=== boundary edges for {} ({}) ===", building_id, boundary_edges.len());
    for (a, b) in &boundary_edges {
        let va = verts[*a as usize];
        let vb = verts[*b as usize];
        eprintln!(
            "  edge {a}->{b}  ({:.4},{:.4},{:.4}) → ({:.4},{:.4},{:.4})  len={:.4}",
            va.x, va.y, va.z, vb.x, vb.y, vb.z,
            (vb - va).length()
        );
    }
    let nonmanifold: Vec<(u32, u32, usize)> = edge_count
        .iter()
        .filter(|&(_, &c)| c > 2)
        .map(|(k, c)| (k.0, k.1, *c))
        .collect();
    if !nonmanifold.is_empty() {
        eprintln!("=== over-shared edges for {} ===", building_id);
        for (a, b, c) in &nonmanifold {
            let va = verts[*a as usize];
            let vb = verts[*b as usize];
            eprintln!(
                "  edge {a}->{b} (count {}): ({:.4},{:.4},{:.4}) → ({:.4},{:.4},{:.4})",
                c, va.x, va.y, va.z, vb.x, vb.y, vb.z
            );
        }
    }
}
