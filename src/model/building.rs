use glam::DVec3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceType {
    Roof,
    Wall,
    Ground,
    Closure,
    Unknown,
}

/// A single polygon (exterior ring). The closing repeat vertex from CityGML is NOT stored.
#[derive(Debug, Clone)]
pub struct Polygon(pub Vec<DVec3>);


#[derive(Debug, Clone)]
pub struct Surface {
    pub surface_type: SurfaceType,
    pub polygons: Vec<Polygon>,
}

#[derive(Debug, Clone)]
pub struct Building {
    pub id: String,
    pub surfaces: Vec<Surface>,
}

