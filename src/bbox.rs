/// Axis-aligned bounding box in UTM coordinates (EPSG:25832).
#[derive(Debug, Clone, Copy)]
pub struct BBox {
    pub xmin: f64,
    pub ymin: f64,
    pub xmax: f64,
    pub ymax: f64,
}

impl BBox {
    pub fn intersects(&self, other: &BBox) -> bool {
        self.xmin <= other.xmax
            && self.xmax >= other.xmin
            && self.ymin <= other.ymax
            && self.ymax >= other.ymin
    }
}

pub fn parse_bbox(s: &str) -> anyhow::Result<BBox> {
    let parts: Vec<f64> = s
        .split(',')
        .map(|p| p.trim().parse::<f64>())
        .collect::<Result<_, _>>()
        .map_err(|e| anyhow::anyhow!("invalid bbox value: {e}"))?;
    anyhow::ensure!(
        parts.len() == 4,
        "bbox must be 'xmin,ymin,xmax,ymax', got {} parts",
        parts.len()
    );
    Ok(BBox {
        xmin: parts[0],
        ymin: parts[1],
        xmax: parts[2],
        ymax: parts[3],
    })
}
