use std::{collections::HashMap, f32::consts::PI, sync::Arc};

use glam::Vec3;

use crate::{
  island::Island,
  nav_data::{NavigationData, NodeRef},
  nav_mesh::NavigationMesh,
  Archipelago, BoundingBox, Transform, ValidNavigationMesh,
};

use super::Path;

fn collect_straight_path(
  path: &Path,
  nav_data: &NavigationData,
  start: (usize, Vec3),
  end: (usize, Vec3),
  iteration_limit: u32,
) -> Vec<(usize, Vec3)> {
  let mut straight_path = Vec::with_capacity(iteration_limit as usize);

  let mut current = start;
  let mut iterations = 0;
  while current.0 != end.0 && iterations < iteration_limit {
    iterations += 1;
    current = path.find_next_point_in_straight_path(
      nav_data, current.0, current.1, end.0, end.1,
    );
    straight_path.push(current);
  }

  straight_path
}

#[test]
fn finds_next_point_for_organic_map() {
  let nav_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(4.0, 0.0, 1.0),
      Vec3::new(4.0, 0.0, 2.0),
      Vec3::new(2.0, 0.0, 3.0),
      Vec3::new(1.0, 0.0, 3.0),
      Vec3::new(0.0, 0.0, 2.0),
      Vec3::new(0.0, 0.0, 1.0),
      Vec3::new(1.0, 0.0, 5.0),
      Vec3::new(2.0, 0.0, 5.0),
      Vec3::new(2.0, 0.0, 4.0),
      Vec3::new(3.0, 1.0, 5.0),
      Vec3::new(3.0, 1.0, 4.0),
      Vec3::new(3.0, -2.0, 4.0),
      Vec3::new(3.0, -2.0, 3.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3, 4, 5, 6, 7],
      vec![5, 4, 10, 9, 8],
      vec![9, 10, 12, 11],
      vec![10, 4, 14, 13],
    ],
  }
  .validate()
  .expect("Mesh is valid.");

  let transform =
    Transform { translation: Vec3::new(5.0, 7.0, 9.0), rotation: PI * 0.35 };
  let mut archipelago = Archipelago::new();
  let island_id = archipelago.add_island();
  archipelago
    .get_island_mut(island_id)
    .set_nav_mesh(transform, Arc::new(nav_mesh));

  let path = Path {
    corridor: vec![
      NodeRef { island_id, polygon_index: 0 },
      NodeRef { island_id, polygon_index: 1 },
      NodeRef { island_id, polygon_index: 2 },
    ],
    portal_edge_index: vec![4, 2],
  };

  assert_eq!(
    collect_straight_path(
      &path,
      &archipelago.nav_data,
      /* start= */ (0, transform.apply(Vec3::new(3.0, 0.0, 1.5))),
      /* end= */ (2, transform.apply(Vec3::new(2.5, 0.5, 4.5))),
      /* iteration_limit= */ 3,
    ),
    [
      (0, transform.apply(Vec3::new(2.0, 0.0, 3.0))),
      (1, transform.apply(Vec3::new(2.0, 0.0, 4.0))),
      (2, transform.apply(Vec3::new(2.5, 0.5, 4.5))),
    ]
  );
}

#[test]
fn finds_next_point_in_zig_zag() {
  let nav_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 1.0),
      Vec3::new(0.0, 0.0, 1.0),
      Vec3::new(1.0, 0.0, 2.0),
      Vec3::new(0.0, 0.0, 2.0),
      Vec3::new(1.0, 0.0, 3.0),
      Vec3::new(0.0, 0.0, 3.0),
      Vec3::new(1.0, 0.0, 4.0),
      Vec3::new(0.0, 0.0, 4.0),
      Vec3::new(1.0, 0.0, 5.0), // Turn right
      Vec3::new(2.0, 0.0, 4.0),
      Vec3::new(2.0, 0.0, 5.0),
      Vec3::new(3.0, 0.0, 4.0),
      Vec3::new(3.0, 0.0, 5.0),
      Vec3::new(4.0, 0.0, 4.0),
      Vec3::new(4.0, 0.0, 5.0),
      Vec3::new(5.0, 0.0, 5.0), // Turn left
      Vec3::new(5.0, 0.0, 6.0),
      Vec3::new(4.0, 0.0, 6.0),
      Vec3::new(5.0, 0.0, 7.0),
      Vec3::new(4.0, 0.0, 7.0),
      Vec3::new(4.0, 0.0, 8.0), // Turn left
      Vec3::new(-3.0, 0.0, 8.0),
      Vec3::new(-3.0, 0.0, 7.0),
      Vec3::new(-4.0, 0.0, 8.0), // Turn right
      Vec3::new(-3.0, 0.0, 15.0),
      Vec3::new(-4.0, 0.0, 15.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3],
      vec![3, 2, 4, 5],
      vec![5, 4, 6, 7],
      vec![7, 6, 8, 9],
      vec![9, 8, 10],
      vec![10, 8, 11, 12],
      vec![12, 11, 13, 14],
      vec![14, 13, 15, 16],
      vec![16, 15, 17],
      vec![16, 17, 18, 19],
      vec![19, 18, 20, 21],
      vec![21, 20, 22],
      vec![21, 22, 23, 24],
      vec![24, 23, 25],
      vec![25, 23, 26, 27],
    ],
  }
  .validate()
  .expect("Mesh is valid.");

  let transform =
    Transform { translation: Vec3::new(-1.0, -10.0, -3.0), rotation: PI * 1.8 };
  let mut archipelago = Archipelago::new();
  let island_id = archipelago.add_island();
  archipelago
    .get_island_mut(island_id)
    .set_nav_mesh(transform, Arc::new(nav_mesh));

  let path = Path {
    corridor: vec![
      NodeRef { island_id, polygon_index: 0 },
      NodeRef { island_id, polygon_index: 1 },
      NodeRef { island_id, polygon_index: 2 },
      NodeRef { island_id, polygon_index: 3 },
      NodeRef { island_id, polygon_index: 4 },
      NodeRef { island_id, polygon_index: 5 },
      NodeRef { island_id, polygon_index: 6 },
      NodeRef { island_id, polygon_index: 7 },
      NodeRef { island_id, polygon_index: 8 },
      NodeRef { island_id, polygon_index: 9 },
      NodeRef { island_id, polygon_index: 10 },
      NodeRef { island_id, polygon_index: 11 },
      NodeRef { island_id, polygon_index: 12 },
      NodeRef { island_id, polygon_index: 13 },
      NodeRef { island_id, polygon_index: 14 },
    ],
    portal_edge_index: vec![2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1],
  };

  assert_eq!(
    collect_straight_path(
      &path,
      &archipelago.nav_data,
      /* start= */ (0, transform.apply(Vec3::new(0.5, 0.0, 0.5))),
      /* end= */ (14, transform.apply(Vec3::new(-3.5, 0.0, 14.0))),
      /* iteration_limit= */ 5,
    ),
    [
      (4, transform.apply(Vec3::new(1.0, 0.0, 4.0))),
      (8, transform.apply(Vec3::new(4.0, 0.0, 5.0))),
      (11, transform.apply(Vec3::new(4.0, 0.0, 7.0))),
      (13, transform.apply(Vec3::new(-3.0, 0.0, 8.0))),
      (14, transform.apply(Vec3::new(-3.5, 0.0, 14.0))),
    ]
  );
}

#[test]
fn starts_at_end_index_goes_to_end_point() {
  let nav_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 1.0),
      Vec3::new(0.0, 0.0, 1.0),
      Vec3::new(1.0, 0.0, 2.0),
      Vec3::new(0.0, 0.0, 2.0),
    ],
    polygons: vec![vec![0, 1, 2, 3], vec![3, 2, 4, 5]],
  }
  .validate()
  .expect("Mesh is valid.");

  let mut archipelago = Archipelago::new();
  let island_id = archipelago.add_island();
  archipelago.get_island_mut(island_id).set_nav_mesh(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::new(nav_mesh),
  );

  let path = Path {
    corridor: vec![
      NodeRef { island_id, polygon_index: 0 },
      NodeRef { island_id, polygon_index: 1 },
    ],
    portal_edge_index: vec![2],
  };

  assert_eq!(
    path.find_next_point_in_straight_path(
      &archipelago.nav_data,
      /* start_index= */ 1,
      /* start_point= */ Vec3::new(0.25, 0.0, 1.1),
      /* end_index= */ 1,
      /* end_point= */ Vec3::new(0.75, 0.0, 1.9),
    ),
    (1, Vec3::new(0.75, 0.0, 1.9))
  );
}

#[test]
fn path_not_valid_for_missing_islands_or_dirty_islands() {
  let path = Path {
    corridor: vec![
      NodeRef { island_id: 1, polygon_index: 0 },
      NodeRef { island_id: 2, polygon_index: 0 },
      NodeRef { island_id: 2, polygon_index: 1 },
      NodeRef { island_id: 3, polygon_index: 0 },
    ],
    portal_edge_index: vec![0, 1, 2],
  };

  let nav_mesh = ValidNavigationMesh {
    mesh_bounds: BoundingBox::Empty,
    boundary_edges: vec![],
    connectivity: vec![],
    polygons: vec![],
    vertices: vec![],
  };
  let nav_mesh = Arc::new(nav_mesh);

  let mut island_1 = Island::new();
  island_1.set_nav_mesh(Transform::default(), Arc::clone(&nav_mesh));

  let mut island_3 = Island::new();
  island_3.set_nav_mesh(Transform::default(), Arc::clone(&nav_mesh));

  // Pretend we updated the islands so they aren't dirty.
  island_1.dirty = false;
  island_3.dirty = false;

  let mut islands = HashMap::new();
  islands.insert(1, island_1);
  islands.insert(3, island_3);

  let mut nav_data = NavigationData::new();
  nav_data.islands = islands;
  assert!(!path.is_valid(&nav_data));

  let mut island_2 = Island::new();
  island_2.set_nav_mesh(Transform::default(), nav_mesh);

  nav_data.islands.insert(2, island_2);
  assert!(!path.is_valid(&nav_data));

  // Pretend we updated island_2.
  nav_data.islands.get_mut(&2).unwrap().dirty = false;
  assert!(path.is_valid(&nav_data));

  // Clear one of the islands of its nav mesh which should make the path
  // invalid again.
  nav_data.islands.get_mut(&1).unwrap().clear_nav_mesh();
  assert!(!path.is_valid(&nav_data));
}
