use glam::DVec2;

/// Type tag for an ALKIS surface feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AlkisKind {
    Strassenverkehr,
    Bahnverkehr,
    Weg,
    Platz,
    Fliessgewaesser,
    StehendesGewaesser,
}

/// A single 2D ALKIS surface feature as parsed from an AX_Bestandsdatenauszug.
/// Coordinates are in the source CRS (ETRS89/UTM32N, EPSG:25832). The exterior
/// ring is normalized to CCW (when viewed from +z) and does not repeat its
/// first vertex at the end.
#[derive(Debug, Clone)]
pub struct SurfaceFeature {
    pub id: String,
    pub kind: AlkisKind,
    pub exterior: Vec<DVec2>,
    /// Interior rings (holes). Honored by the road and water meshers: each
    /// interior is clipped to the tile, added as a CDT constraint loop, and
    /// excluded from the top/bottom caps with inward-facing side walls.
    pub interiors: Vec<Vec<DVec2>>,
}
