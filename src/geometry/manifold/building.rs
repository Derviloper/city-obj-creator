use glam::DVec3;

use super::{
    check::check_manifold,
    cleanup::{
        close_open_boundary_loops, cluster_merge_close_vertices, dedup_polygons,
        drop_internal_ground_floors, drop_redundant_polygons, fix_t_junctions,
        split_self_touching_polygons,
    },
    helpers::snap_vec,
    repair::{orient_consistently, repair_overshared_edges},
};
use crate::{
    geometry::{normals::ear_clip_triangulate, vertex_merge::VertexMerger},
    model::building::{Building, SurfaceType},
};

/// Vertex-snap tolerance used by the building merger and T-junction fixer (5 mm).
const BUILDING_TOL: f64 = 0.005;

/// Secondary cluster-merge tolerance used after the initial snap to collapse
/// near-duplicate vertices that LVermGeo encodes as separate corners a few
/// centimetres apart. Applied only when both xy and z deltas are small, so
/// genuine wall corners (same xy, different z) are not collapsed.
const BUILDING_CLUSTER_XY: f64 = 0.05;
const BUILDING_CLUSTER_Z: f64 = 0.02;

/// Convert a building's surfaces into a manifold mesh.
///
/// Pipeline:
///  1. Snap vertices to a shared 5 mm grid.
///  2. Cluster-merge near-duplicates within `BUILDING_CLUSTER_XY/Z`.
///  3. Split self-touching rings.
///  4. Drop duplicate polygons.
///  5. Insert T-junction split vertices.
///  6. Drop redundant polygons that create over-shared edges.
///  7. Drop spurious internal "floor" ground polygons.
///  8. Close open boundary loops with cap polygons.
///  9. Triangulate, vertex-merge, repair over-shared edges, and orient
///     consistently outward.
pub fn build_manifold_building(building: &Building) -> (Vec<DVec3>, Vec<[u32; 3]>) {
    // ---- (1) Snap every polygon vertex to the 5 mm grid. ----
    let mut polygons: Vec<(SurfaceType, Vec<DVec3>)> = Vec::new();
    for surface in &building.surfaces {
        for polygon in &surface.polygons {
            if polygon.0.len() < 3 {
                continue;
            }
            let snapped: Vec<DVec3> =
                polygon.0.iter().map(|v| snap_vec(*v, BUILDING_TOL)).collect();
            polygons.push((surface.surface_type, snapped));
        }
    }

    // ---- (2) Cluster-merge near-duplicate vertices.
    //
    // After the 5 mm snap, LVermGeo data sometimes still leaves the "same"
    // corner as two vertices a few centimetres apart in xy at essentially the
    // same z (often 1–4 cm). Triangulating across the gap produces sliver
    // triangles whose internal diagonals collide with neighbouring polygons'
    // diagonals — over-shared edges. Collapse such clusters to a single point
    // (the cluster centroid). The tight z window keeps real wall corners
    // (same xy, very different z) untouched.
    cluster_merge_close_vertices(
        &mut polygons,
        BUILDING_TOL,
        BUILDING_CLUSTER_XY,
        BUILDING_CLUSTER_Z,
    );

    // ---- (3) Split self-touching polygons.
    //
    // LVermGeo encodes some roof surfaces as a single ring that revisits the
    // same vertex twice — used to lasso around a tiny pillar/turret. Fan-
    // triangulating such a ring produces overlapping triangles and over-shared
    // edges. Split each self-touching ring into an outer ring (with the
    // detour removed) and the detour itself as its own closed loop.
    split_self_touching_polygons(&mut polygons, BUILDING_TOL);

    // ---- (4) Drop duplicate polygons.
    //
    // LVermGeo LoD2 data sometimes encodes the same quadrilateral as both a
    // `WallSurface` and a `ClosureSurface`, or as two `ClosureSurface`s with
    // opposite winding. Each duplicate emits its own triangulation, so every
    // interior edge ends up shared by 4 triangles. Two polygons are
    // duplicates if their cyclic vertex sequences match — up to rotation and
    // direction reversal — once snapped to the building grid.
    dedup_polygons(&mut polygons, BUILDING_TOL);

    // ---- (5) T-junction fixing: ensure every vertex on a polygon edge is
    //          part of every other polygon that touches that edge. ----
    fix_t_junctions(&mut polygons, BUILDING_TOL);

    // ---- (6) Drop redundant polygons that create non-manifold edges.
    //
    // CityGML LoD2 from LVermGeo sometimes encodes the same wall as both a
    // semantic `WallSurface` and a `ClosureSurface` (or as two closures), or
    // packs three coplanar walls onto the same vertical edge at a tight
    // corner. Each redundant polygon adds an extra triangulation to edges
    // that are already covered, pushing edge count past 2.
    drop_redundant_polygons(&mut polygons, BUILDING_TOL);

    // ---- (7) Drop spurious internal horizontal "floors".
    //
    // Some LVermGeo CityGML files mark a BuildingPart's upper-level floor as
    // a `GroundSurface`. Including it produces an internal slab. But sloped/
    // stepped buildings legitimately have ground polygons spanning a meter
    // or more vertically, so a flat z-threshold incorrectly drops real ground
    // there. Heuristic: a ground polygon A is internal iff some other ground
    // polygon B sits below it in xy AND at least 1 m below it in Z.
    drop_internal_ground_floors(&mut polygons);

    // ---- (8) Close any open boundary loops with cap polygons.
    //
    // Some LVermGeo buildings (notably bridges) ship without a polygon for
    // the underside, leaving a hole bordered by a closed loop of unmatched
    // polygon edges. Detect every such loop in the directed-edge graph,
    // ear-clip-triangulate it in its best-fit plane, and emit cap polygons.
    //
    // No second T-junction pass: the cap is built from the same vertices
    // that already exist on the wall side, so cap edges and wall edges
    // already share endpoints. Re-running fix_t_junctions here would
    // *re-insert* a collinear cap-perimeter vertex into another cap
    // triangle's internal diagonal, creating duplicate degenerate triangles.
    close_open_boundary_loops(&mut polygons, BUILDING_TOL);

    // ---- (9) Triangulate, merge vertices, repair, orient. ----
    //
    // We don't try to flip individual faces toward an "expected" outward
    // normal here. The downstream `orient_consistently` pass propagates a
    // consistent winding via BFS over face adjacency and then flips each
    // closed component to outward via signed-volume — that is sufficient and
    // strictly more reliable than a per-face heuristic, which is wrong for
    // walls at concave corners. Empirically, omitting the per-face flip
    // produces byte-identical output across the LVermGeo test set.
    let mut merger = VertexMerger::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();

    for (_surface_type, verts) in &polygons {
        if verts.len() < 3 {
            continue;
        }
        let indices: Vec<u32> = verts.iter().map(|v| merger.insert(*v)).collect();

        // Ear-clipping (rather than naive fan-triangulation) picks each
        // diagonal based on local convexity, so two unrelated polygons that
        // happen to share two non-adjacent vertices won't both produce the
        // same internal diagonal. If ear-clipping stalls on a degenerate
        // polygon, fall back to a fan from vertex 0.
        let raw_tri_idx = ear_clip_triangulate(verts);
        let raw_tris: Vec<[u32; 3]> = if raw_tri_idx.is_empty() && verts.len() >= 3 {
            (1..verts.len() - 1)
                .map(|i| [indices[0], indices[i], indices[i + 1]])
                .collect()
        } else {
            raw_tri_idx
                .into_iter()
                .map(|t| [indices[t[0]], indices[t[1]], indices[t[2]]])
                .collect()
        };

        for tri in raw_tris {
            // Drop degenerate triangles created by collinear/duplicate vertices.
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
                continue;
            }
            tris.push(tri);
        }
    }

    // Greedy manifold repair: drop triangles touching over-shared edges
    // when doing so strictly improves the manifold metric.
    repair_overshared_edges(&mut tris);

    // BFS-based winding consistency + per-component outward orientation.
    orient_consistently(&mut tris, &merger.verts);

    check_manifold(building.id.as_str(), &tris);

    (merger.verts, tris)
}
