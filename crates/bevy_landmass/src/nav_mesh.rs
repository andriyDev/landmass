use bevy_mesh::{Indices, Mesh, PrimitiveTopology, VertexAttributeValues};

use crate::{coords::CoordinateSystem, NavigationMesh};

/// A conversion error for Bevy meshes to `landmass` nav meshes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConvertMeshError {
  /// The Bevy mesh had an unsupported topology. Only
  /// [`PrimitiveTopology::TriangleList`] is supported.
  InvalidTopology,
  /// The Bevy mesh does not have a [`Mesh::ATTRIBUTE_POSITION`].
  MissingVertexPositions,
  /// The mesh has an unsupported type for its indices. This may either be
  /// because the mesh has **no** indices, or an unknown index format is used
  /// (currently u16 and u32 are both supported). Indices are necessary since
  /// they inform connectivity.
  WrongTypeForIndices,
}

/// Converts a Bevy Mesh to a landmass NavigationMesh. This is done naively -
/// each triangle forms a single polygon in the navigation mesh, which can cause
/// strange paths to form (agents may take turns inside if wide open regions).
/// This function is provided as a convenience, and a better method for
/// generating navigation meshes should be used.
pub fn bevy_mesh_to_landmass_nav_mesh<CS: CoordinateSystem>(
  mesh: &Mesh,
) -> Result<NavigationMesh<CS>, ConvertMeshError> {
  let PrimitiveTopology::TriangleList = mesh.primitive_topology() else {
    return Err(ConvertMeshError::InvalidTopology);
  };

  let Some(values) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) else {
    return Err(ConvertMeshError::MissingVertexPositions);
  };

  let vertices = match values {
    VertexAttributeValues::Float32x3(vertices) => {
      vertices.iter().map(CS::from_mesh_vertex).collect()
    }
    _ => panic!("Mesh POSITION must be Float32x3"),
  };

  let polygons = match mesh.indices() {
    Some(Indices::U16(indices)) => {
      assert!(indices.len() % 3 == 0);
      let mut polygons = Vec::with_capacity(indices.len() / 3);
      for i in (0..indices.len()).step_by(3) {
        polygons.push(vec![
          indices[i] as usize,
          indices[i + 1] as usize,
          indices[i + 2] as usize,
        ]);
      }
      polygons
    }
    Some(Indices::U32(indices)) => {
      assert!(indices.len() % 3 == 0);
      let mut polygons = Vec::with_capacity(indices.len() / 3);
      for i in (0..indices.len()).step_by(3) {
        polygons.push(vec![
          indices[i] as usize,
          indices[i + 1] as usize,
          indices[i + 2] as usize,
        ]);
      }
      polygons
    }
    _ => return Err(ConvertMeshError::WrongTypeForIndices),
  };

  Ok(NavigationMesh {
    vertices,
    polygon_type_indices: (0..polygons.len()).map(|_| 0).collect(),
    polygons,
  })
}

#[cfg(test)]
#[path = "nav_mesh_test.rs"]
mod test;
