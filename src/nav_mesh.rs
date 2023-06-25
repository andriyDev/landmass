use bevy::{
  prelude::Mesh,
  render::{
    mesh::{Indices, VertexAttributeValues::Float32x3},
    render_resource::PrimitiveTopology,
  },
};
use landmass::NavigationMesh;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConvertMeshError {
  InvalidTopology,
  MissingVertexPositions,
  WrongTypeForIndices,
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
    Float32x3(vertices) => vertices
      .iter()
      .map(|vert| glam::Vec3::new(vert[0], vert[1], vert[2]))
      .collect(),
    _ => panic!("Mesh POSITION must be Float32x3"),
  };

  let polygons = match mesh.indices() {
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
    _ => return Err(ConvertMeshError::WrongTypeForIndices),
  };

  Ok(NavigationMesh { mesh_bounds: None, vertices, polygons })
}

#[cfg(test)]
mod tests {
  use bevy::{prelude::Mesh, render::render_resource::PrimitiveTopology};

  use crate::nav_mesh::ConvertMeshError;

  use super::bevy_mesh_to_landmass_nav_mesh;

  #[test]
  fn error_on_wrong_topology() {
    let mesh = Mesh::new(PrimitiveTopology::LineStrip);
    match bevy_mesh_to_landmass_nav_mesh(&mesh) {
      Ok(_) => panic!("Conversion succeeded."),
      Err(error) => assert_eq!(error, ConvertMeshError::InvalidTopology),
    }
  }

  #[test]
  fn converts_u16_indices() {
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    mesh.insert_attribute(
      Mesh::ATTRIBUTE_POSITION,
      vec![
        [1.0, 1.0, 1.0],
        [2.0, 1.0, 1.0],
        [2.0, 1.0, 2.0],
        [1.0, 1.0, 2.0],
        [2.0, 1.0, 3.0],
        [1.0, 1.0, 3.0],
        [3.0, 1.0, 2.0],
        [3.0, 1.0, 3.0],
      ],
    );
    mesh.set_indices(Some(bevy::render::mesh::Indices::U16(vec![
      0, 1, 2, 2, 3, 0, 3, 2, 4, 3, 4, 5, 4, 2, 6, 4, 6, 7,
    ])));
    let nav_mesh =
      bevy_mesh_to_landmass_nav_mesh(&mesh).expect("conversion succeeds");

    assert_eq!(
      nav_mesh.vertices,
      [
        glam::Vec3::new(1.0, 1.0, 1.0),
        glam::Vec3::new(2.0, 1.0, 1.0),
        glam::Vec3::new(2.0, 1.0, 2.0),
        glam::Vec3::new(1.0, 1.0, 2.0),
        glam::Vec3::new(2.0, 1.0, 3.0),
        glam::Vec3::new(1.0, 1.0, 3.0),
        glam::Vec3::new(3.0, 1.0, 2.0),
        glam::Vec3::new(3.0, 1.0, 3.0),
      ]
    );

    assert_eq!(
      nav_mesh.polygons,
      vec![
        vec![0, 2, 1],
        vec![2, 0, 3],
        vec![3, 4, 2],
        vec![3, 5, 4],
        vec![4, 6, 2],
        vec![4, 7, 6],
      ]
    );
  }

  #[test]
  fn converts_u32_indices() {
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    mesh.insert_attribute(
      Mesh::ATTRIBUTE_POSITION,
      vec![
        [1.0, 1.0, 1.0],
        [2.0, 1.0, 1.0],
        [2.0, 1.0, 2.0],
        [1.0, 1.0, 2.0],
        [2.0, 1.0, 3.0],
        [1.0, 1.0, 3.0],
        [3.0, 1.0, 2.0],
        [3.0, 1.0, 3.0],
      ],
    );
    mesh.set_indices(Some(bevy::render::mesh::Indices::U32(vec![
      0, 1, 2, 2, 3, 0, 3, 2, 4, 3, 4, 5, 4, 2, 6, 4, 6, 7,
    ])));
    let nav_mesh =
      bevy_mesh_to_landmass_nav_mesh(&mesh).expect("conversion succeeds");

    assert_eq!(
      nav_mesh.vertices,
      [
        glam::Vec3::new(1.0, 1.0, 1.0),
        glam::Vec3::new(2.0, 1.0, 1.0),
        glam::Vec3::new(2.0, 1.0, 2.0),
        glam::Vec3::new(1.0, 1.0, 2.0),
        glam::Vec3::new(2.0, 1.0, 3.0),
        glam::Vec3::new(1.0, 1.0, 3.0),
        glam::Vec3::new(3.0, 1.0, 2.0),
        glam::Vec3::new(3.0, 1.0, 3.0),
      ]
    );

    assert_eq!(
      nav_mesh.polygons,
      vec![
        vec![0, 2, 1],
        vec![2, 0, 3],
        vec![3, 4, 2],
        vec![3, 5, 4],
        vec![4, 6, 2],
        vec![4, 7, 6],
      ]
    );
  }
}
