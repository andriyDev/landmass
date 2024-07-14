use std::sync::Arc;

use glam::Vec2;

use crate::{
  coords::XY, Archipelago, NavigationMesh, SamplePointError, Transform,
};

use super::sample_point;

#[test]
fn error_on_dirty_nav_mesh() {
  let mut archipelago = Archipelago::<XY>::new();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  archipelago.add_island().set_nav_mesh(Transform::default(), nav_mesh);
  assert_eq!(
    sample_point(
      &archipelago,
      /* point= */ Vec2::new(0.5, 0.5),
      /* distance_to_node= */ 1.0
    ),
    Err(SamplePointError::NavDataDirty)
  );
}

#[test]
fn error_on_out_of_range() {
  let mut archipelago = Archipelago::<XY>::new();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  archipelago.add_island().set_nav_mesh(Transform::default(), nav_mesh);
  archipelago.update(1.0);

  assert_eq!(
    sample_point(
      &archipelago,
      /* point= */ Vec2::new(-0.5, 0.5),
      /* distance_to_node= */ 0.1
    ),
    Err(SamplePointError::OutOfRange)
  );
}

#[test]
fn samples_point_on_nav_mesh_or_near_nav_mesh() {
  let mut archipelago = Archipelago::<XY>::new();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let offset = Vec2::new(10.0, 10.0);
  archipelago
    .add_island()
    .set_nav_mesh(Transform { translation: offset, rotation: 0.0 }, nav_mesh);
  archipelago.update(1.0);

  assert_eq!(
    sample_point(
      &archipelago,
      /* point= */ offset + Vec2::new(-0.5, 0.5),
      /* distance_to_node= */ 0.6
    ),
    Ok(offset + Vec2::new(0.0, 0.5))
  );
  assert_eq!(
    sample_point(
      &archipelago,
      /* point= */ offset + Vec2::new(0.5, 0.5),
      /* distance_to_node= */ 0.6
    ),
    Ok(offset + Vec2::new(0.5, 0.5))
  );
  assert_eq!(
    sample_point(
      &archipelago,
      /* point= */ offset + Vec2::new(1.2, 1.2),
      /* distance_to_node= */ 0.6
    ),
    Ok(offset + Vec2::new(1.0, 1.0))
  );
}
