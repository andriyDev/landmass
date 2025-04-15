use bevy_asset::RenderAssetUsages;
use bevy_mesh::{Mesh, PrimitiveTopology};

use crate::{
  coords::{ThreeD, TwoD},
  nav_mesh::ConvertMeshError,
};

use super::bevy_mesh_to_landmass_nav_mesh;

#[test]
fn error_on_wrong_topology() {
  let mesh =
    Mesh::new(PrimitiveTopology::LineStrip, RenderAssetUsages::MAIN_WORLD);
  match bevy_mesh_to_landmass_nav_mesh::<ThreeD>(&mesh) {
    Ok(_) => panic!("Conversion succeeded."),
    Err(error) => assert_eq!(error, ConvertMeshError::InvalidTopology),
  }
}

#[test]
fn converts_u16_indices() {
  let mut mesh =
    Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::MAIN_WORLD);
  mesh.insert_attribute(
    Mesh::ATTRIBUTE_POSITION,
    vec![
      [1.0, 1.0, 1.0],
      [2.0, 1.0, 1.0],
      [2.0, 2.0, 1.0],
      [1.0, 2.0, 1.0],
      [2.0, 3.0, 1.0],
      [1.0, 3.0, 1.0],
      [3.0, 2.0, 1.0],
      [3.0, 3.0, 1.0],
    ],
  );
  mesh.insert_indices(bevy::render::mesh::Indices::U16(vec![
    0, 1, 2, 2, 3, 0, 3, 2, 4, 3, 4, 5, 4, 2, 6, 4, 6, 7,
  ]));
  let nav_mesh =
    bevy_mesh_to_landmass_nav_mesh::<TwoD>(&mesh).expect("conversion succeeds");

  assert_eq!(
    nav_mesh.vertices,
    [
      bevy::math::Vec2::new(1.0, 1.0),
      bevy::math::Vec2::new(2.0, 1.0),
      bevy::math::Vec2::new(2.0, 2.0),
      bevy::math::Vec2::new(1.0, 2.0),
      bevy::math::Vec2::new(2.0, 3.0),
      bevy::math::Vec2::new(1.0, 3.0),
      bevy::math::Vec2::new(3.0, 2.0),
      bevy::math::Vec2::new(3.0, 3.0),
    ]
  );

  assert_eq!(
    nav_mesh.polygons,
    vec![
      vec![0, 1, 2],
      vec![2, 3, 0],
      vec![3, 2, 4],
      vec![3, 4, 5],
      vec![4, 2, 6],
      vec![4, 6, 7],
    ]
  );
}

#[test]
fn converts_u32_indices() {
  let mut mesh =
    Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::MAIN_WORLD);
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
  mesh.insert_indices(bevy::render::mesh::Indices::U32(vec![
    0, 1, 2, 2, 3, 0, 3, 2, 4, 3, 4, 5, 4, 2, 6, 4, 6, 7,
  ]));
  let nav_mesh = bevy_mesh_to_landmass_nav_mesh::<ThreeD>(&mesh)
    .expect("conversion succeeds");

  assert_eq!(
    nav_mesh.vertices,
    [
      bevy::math::Vec3::new(1.0, 1.0, 1.0),
      bevy::math::Vec3::new(2.0, 1.0, 1.0),
      bevy::math::Vec3::new(2.0, 1.0, 2.0),
      bevy::math::Vec3::new(1.0, 1.0, 2.0),
      bevy::math::Vec3::new(2.0, 1.0, 3.0),
      bevy::math::Vec3::new(1.0, 1.0, 3.0),
      bevy::math::Vec3::new(3.0, 1.0, 2.0),
      bevy::math::Vec3::new(3.0, 1.0, 3.0),
    ]
  );

  assert_eq!(
    nav_mesh.polygons,
    vec![
      vec![0, 1, 2],
      vec![2, 3, 0],
      vec![3, 2, 4],
      vec![3, 4, 5],
      vec![4, 2, 6],
      vec![4, 6, 7],
    ]
  );
}
