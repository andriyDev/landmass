use std::{collections::HashMap, sync::Arc};

use glam::Vec2;

use crate::{
  coords::XY, Archipelago, FindPathError, NavigationMesh, SamplePointError,
  Transform,
};

use super::{find_path, sample_point};

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
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  archipelago.add_island().set_nav_mesh(
    Transform::default(),
    nav_mesh,
    HashMap::new(),
  );
  assert_eq!(
    sample_point(
      &archipelago,
      /* point= */ Vec2::new(0.5, 0.5),
      /* distance_to_node= */ 1.0
    )
    .map(|p| p.point()),
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
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  archipelago.add_island().set_nav_mesh(
    Transform::default(),
    nav_mesh,
    HashMap::new(),
  );
  archipelago.update(1.0);

  assert_eq!(
    sample_point(
      &archipelago,
      /* point= */ Vec2::new(-0.5, 0.5),
      /* distance_to_node= */ 0.1
    )
    .map(|p| p.point()),
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
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let offset = Vec2::new(10.0, 10.0);
  archipelago.add_island().set_nav_mesh(
    Transform { translation: offset, rotation: 0.0 },
    nav_mesh,
    HashMap::new(),
  );
  archipelago.update(1.0);

  assert_eq!(
    sample_point(
      &archipelago,
      /* point= */ offset + Vec2::new(-0.5, 0.5),
      /* distance_to_node= */ 0.6
    )
    .map(|p| p.point()),
    Ok(offset + Vec2::new(0.0, 0.5))
  );
  assert_eq!(
    sample_point(
      &archipelago,
      /* point= */ offset + Vec2::new(0.5, 0.5),
      /* distance_to_node= */ 0.6
    )
    .map(|p| p.point()),
    Ok(offset + Vec2::new(0.5, 0.5))
  );
  assert_eq!(
    sample_point(
      &archipelago,
      /* point= */ offset + Vec2::new(1.2, 1.2),
      /* distance_to_node= */ 0.6
    )
    .map(|p| p.point()),
    Ok(offset + Vec2::new(1.0, 1.0))
  );
}

#[test]
fn no_path() {
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
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let offset = Vec2::new(10.0, 10.0);
  archipelago.add_island().set_nav_mesh(
    Transform { translation: offset, rotation: 0.0 },
    nav_mesh.clone(),
    HashMap::new(),
  );
  archipelago.add_island().set_nav_mesh(
    Transform { translation: offset + Vec2::new(2.0, 0.0), rotation: 0.0 },
    nav_mesh,
    HashMap::new(),
  );
  archipelago.update(1.0);

  let start_point = archipelago
    .sample_point(offset + Vec2::new(0.5, 0.5), 1e-5)
    .expect("point is on nav mesh.");
  let end_point = archipelago
    .sample_point(offset + Vec2::new(2.5, 0.5), 1e-5)
    .expect("point is on nav mesh.");
  assert_eq!(
    find_path(&archipelago, &start_point, &end_point, &HashMap::new()),
    Err(FindPathError::NoPathFound)
  );
}

#[test]
fn finds_path() {
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
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let offset = Vec2::new(10.0, 10.0);
  archipelago.add_island().set_nav_mesh(
    Transform { translation: offset, rotation: 0.0 },
    nav_mesh.clone(),
    HashMap::new(),
  );
  archipelago.add_island().set_nav_mesh(
    Transform { translation: offset + Vec2::new(1.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
    HashMap::new(),
  );
  archipelago.add_island().set_nav_mesh(
    Transform { translation: offset + Vec2::new(2.0, 0.5), rotation: 0.0 },
    nav_mesh,
    HashMap::new(),
  );
  archipelago.update(1.0);

  let start_point = archipelago
    .sample_point(offset + Vec2::new(0.5, 0.5), 1e-5)
    .expect("point is on nav mesh.");
  let end_point = archipelago
    .sample_point(offset + Vec2::new(2.5, 1.25), 1e-5)
    .expect("point is on nav mesh.");
  assert_eq!(
    find_path(&archipelago, &start_point, &end_point, &HashMap::new()),
    Ok(vec![
      offset + Vec2::new(0.5, 0.5),
      offset + Vec2::new(2.0, 1.0),
      offset + Vec2::new(2.5, 1.25)
    ])
  );
}

#[test]
fn finds_path_with_override_node_types() {
  let mut archipelago = Archipelago::<XY>::new();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
        //
        Vec2::new(2.0, 0.0),
        Vec2::new(2.0, 1.0),
        //
        Vec2::new(3.0, 0.0),
        Vec2::new(3.0, 1.0),
        //
        Vec2::new(2.0, 11.0),
        Vec2::new(3.0, 11.0),
        //
        Vec2::new(2.0, 12.0),
        Vec2::new(3.0, 12.0),
        //
        Vec2::new(1.0, 12.0),
        Vec2::new(1.0, 11.0),
        //
        Vec2::new(0.0, 12.0),
        Vec2::new(0.0, 11.0),
      ],
      polygons: vec![
        vec![0, 1, 2, 3],
        vec![2, 1, 4, 5],
        vec![5, 4, 6, 7],
        //
        vec![5, 7, 9, 8],
        vec![8, 9, 11, 10],
        //
        vec![8, 10, 12, 13],
        vec![13, 12, 14, 15],
        //
        vec![3, 2, 13, 15],
      ],
      polygon_type_indices: vec![0, 0, 0, 0, 0, 0, 0, 1],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let node_type = archipelago.add_node_type(1.0).unwrap();

  archipelago.add_island().set_nav_mesh(
    Transform::default(),
    nav_mesh,
    HashMap::from([(1, node_type)]),
  );

  archipelago.update(1.0);

  let start_point = sample_point(
    &archipelago,
    Vec2::new(0.5, 0.5),
    /* distance_to_node= */ 0.1,
  )
  .unwrap();
  let end_point = sample_point(
    &archipelago,
    Vec2::new(0.5, 11.5),
    /* distance_to_node= */ 0.1,
  )
  .unwrap();

  let path = find_path(
    &archipelago,
    &start_point,
    &end_point,
    &HashMap::from([(node_type, 10.0)]),
  )
  .expect("Path found");

  assert_eq!(
    path,
    [
      Vec2::new(0.5, 0.5),
      Vec2::new(2.0, 1.0),
      Vec2::new(2.0, 11.0),
      Vec2::new(0.5, 11.5)
    ]
  );
}

#[test]
fn find_path_returns_error_on_invalid_node_cost() {
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
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let node_type = archipelago.add_node_type(1.0).unwrap();

  archipelago.add_island().set_nav_mesh(
    Transform::default(),
    nav_mesh,
    HashMap::from([(0, node_type)]),
  );

  archipelago.update(1.0);

  let start_point = archipelago
    .sample_point(Vec2::new(0.25, 0.25), /* distance_to_node= */ 0.1)
    .unwrap();
  let end_point = archipelago
    .sample_point(Vec2::new(0.25, 0.25), /* distance_to_node= */ 0.1)
    .unwrap();

  assert_eq!(
    find_path(
      &archipelago,
      &start_point,
      &end_point,
      &HashMap::from([(node_type, 0.0)]),
    ),
    Err(FindPathError::NonPositiveNodeTypeCost(node_type, 0.0))
  );
  assert_eq!(
    find_path(
      &archipelago,
      &start_point,
      &end_point,
      &HashMap::from([(node_type, -0.5)]),
    ),
    Err(FindPathError::NonPositiveNodeTypeCost(node_type, -0.5))
  );
}
