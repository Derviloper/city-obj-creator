use glam::DVec2;

/// Type tag for an ALKIS surface feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AlkisKind {
    Strassenverkehr,
    Bahnverkehr,
    Weg,
    Fliessgewaesser,
    StehendesGewaesser,
}

/// A single 2D ALKIS surface feature as parsed from an AX_Bestandsdatenauszug.
/// Coordinates are in the source CRS (ETRS89/UTM32N, EPSG:25832). The exterior
/// ring is normalized to CCW (when viewed from +z) and does not repeat its
/// first vertex at the end.
#[derive(Debug, Clone)]
#[allow(dead_code)] // `kind` and `interiors` are populated for future ALKIS types and hole support.
pub struct SurfaceFeature {
    pub id: String,
    pub kind: AlkisKind,
    pub exterior: Vec<DVec2>,
    /// Interior rings (holes). Currently parsed but ignored by the mesher;
    /// kept on the struct so adding hole support later is non-breaking.
    pub interiors: Vec<Vec<DVec2>>,
}
