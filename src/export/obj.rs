use std::{
    fs::File,
    io::{BufWriter, Write},
    path::Path,
};

use anyhow::Result;
use glam::DVec3;

/// Streaming Wavefront OBJ writer.
///
/// Vertices are written immediately as they are pushed; the caller must track
/// the returned vertex offset to compute correct 1-indexed face references.
pub struct ObjWriter {
    w: BufWriter<File>,
    pub vertex_count: u64,
}

impl ObjWriter {
    pub fn open(path: &Path) -> Result<Self> {
        let file = File::create(path)?;
        Ok(Self {
            w: BufWriter::with_capacity(256 * 1024, file),
            vertex_count: 0,
        })
    }

    pub fn write_comment(&mut self, s: &str) -> Result<()> {
        writeln!(self.w, "# {s}")?;
        Ok(())
    }

    pub fn write_newline(&mut self) -> Result<()> {
        writeln!(self.w)?;
        Ok(())
    }

    /// Write an object group header.
    pub fn write_object(&mut self, name: &str) -> Result<()> {
        writeln!(self.w, "o {name}")?;
        Ok(())
    }

    /// Write a batch of vertices, subtracting `origin` from each.
    /// Returns the base vertex index (value of `vertex_count` BEFORE writing).
    pub fn write_vertices(&mut self, verts: &[DVec3], origin: DVec3) -> Result<u64> {
        let base = self.vertex_count;
        for v in verts {
            let x = v.x - origin.x;
            let y = v.y - origin.y;
            let z = v.z - origin.z;
            writeln!(self.w, "v {x:.6} {y:.6} {z:.6}")?;
            self.vertex_count += 1;
        }
        Ok(base)
    }

    /// Write triangular faces using 1-based indices.
    /// `base` is the vertex_count value returned by the matching `write_vertices` call.
    pub fn write_faces(&mut self, faces: &[[u32; 3]], base: u64) -> Result<()> {
        for tri in faces {
            let a = tri[0] as u64 + base + 1;
            let b = tri[1] as u64 + base + 1;
            let c = tri[2] as u64 + base + 1;
            writeln!(self.w, "f {a} {b} {c}")?;
        }
        Ok(())
    }

    pub fn flush(&mut self) -> Result<()> {
        self.w.flush()?;
        Ok(())
    }
}
