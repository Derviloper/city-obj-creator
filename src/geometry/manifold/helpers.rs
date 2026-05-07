use glam::DVec3;

/// Snapped grid coordinate used as a hash key for tolerance-aware vertex equality.
pub(super) type GridKey = (i64, i64, i64);

/// Snap a vertex to a fixed grid and return the integer key.
pub(super) fn snap_key(v: DVec3, tol: f64) -> GridKey {
    let inv = 1.0 / tol;
    (
        (v.x * inv).round() as i64,
        (v.y * inv).round() as i64,
        (v.z * inv).round() as i64,
    )
}

/// Snap a vertex to a fixed grid for stable equality testing, returning a
/// vertex aligned to the grid (not the integer key).
pub(super) fn snap_vec(v: DVec3, tol: f64) -> DVec3 {
    let inv = 1.0 / tol;
    DVec3::new(
        (v.x * inv).round() / inv,
        (v.y * inv).round() / inv,
        (v.z * inv).round() / inv,
    )
}
