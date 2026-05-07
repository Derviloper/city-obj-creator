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

impl Building {
    /// Compute the bounding box of all surface vertices.
    pub fn bounds(&self) -> Option<(DVec3, DVec3)> {
        let mut min = DVec3::splat(f64::INFINITY);
        let mut max = DVec3::splat(f64::NEG_INFINITY);
        let mut any = false;
        for s in &self.surfaces {
            for p in &s.polygons {
                for v in &p.0 {
                    min = min.min(*v);
                    max = max.max(*v);
                    any = true;
                }
            }
        }
        if any { Some((min, max)) } else { None }
    }

    /// Average of xy across every vertex; z is forced to 0. Used as a
    /// reference point for outward-normal heuristics where z is irrelevant.
    pub fn centroid_xy(&self) -> Option<DVec3> {
        let mut sx = 0.0;
        let mut sy = 0.0;
        let mut count = 0usize;
        for s in &self.surfaces {
            for p in &s.polygons {
                for v in &p.0 {
                    sx += v.x;
                    sy += v.y;
                    count += 1;
                }
            }
        }
        if count > 0 {
            let n = count as f64;
            Some(DVec3::new(sx / n, sy / n, 0.0))
        } else {
            None
        }
    }
}
