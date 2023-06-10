use bevy::{
  prelude::Mesh,
  render::{
    mesh::{
      Indices,
      VertexAttributeValues::{Float32x3, Float32x4},
    },
    render_resource::PrimitiveTopology,
  },
};
use landmass::NavigationMesh;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConvertMeshError {
  InvalidTopology,
  MissingVertexPositions,
  WrongTypeForVertexPositions,
}

// Converts a Bevy Mesh to a landmass NavigationMesh. This is done naively -
// each triangle forms a single polygon in the navigation mesh, which can cause
// strange paths to form (agents may take turns inside if wide open regions).
// This function is provided as a convenience, and a better method for
// generating navigation meshes should be used.
pub fn bevy_mesh_to_landmass_nav_mesh(
  mesh: &Mesh,
) -> Result<NavigationMesh, ConvertMeshError> {
  let PrimitiveTopology::TriangleList = mesh.primitive_topology() else {
    return Err(ConvertMeshError::InvalidTopology);
  };

  let Some(values) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) else {
    return Err(ConvertMeshError::MissingVertexPositions);
  };

  let vertices: Vec<glam::Vec3> = match values {
    Float32x4(vertices) => vertices
      .iter()
      .map(|vert| glam::Vec3::new(vert[0], vert[1], vert[2]))
      .collect(),
    Float32x3(vertices) => vertices
      .iter()
      .map(|vert| glam::Vec3::new(vert[0], vert[1], vert[2]))
      .collect(),
    _ => return Err(ConvertMeshError::WrongTypeForVertexPositions),
  };

  let polygons = match mesh.indices() {
    None => {
      assert!(vertices.len() % 3 == 0);
      let mut polygons = Vec::with_capacity(vertices.len() / 3);
      for i in (0..vertices.len()).step_by(3) {
        polygons.push(vec![i, i + 2, i + 1]);
      }
      polygons
    }
    Some(Indices::U16(indices)) => {
      assert!(indices.len() % 3 == 0);
      let mut polygons = Vec::with_capacity(indices.len() / 3);
      for i in (0..indices.len()).step_by(3) {
        polygons.push(vec![
          indices[i] as usize,
          indices[i + 2] as usize,
          indices[i + 1] as usize,
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
          indices[i + 2] as usize,
          indices[i + 1] as usize,
        ]);
      }
      polygons
    }
  };

  Ok(NavigationMesh { mesh_bounds: None, vertices, polygons })
}
