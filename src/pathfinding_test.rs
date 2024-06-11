use std::{f32::consts::PI, sync::Arc};

use glam::Vec3;

use crate::{
  nav_data::NodeRef, nav_mesh::NavigationMesh, path::Path, Archipelago,
  Transform,
};

use super::find_path;

#[test]
fn finds_path_in_archipelago() {
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

  let mut archipelago = Archipelago::new();
  let island_id = archipelago.add_island();
  archipelago.get_island_mut(island_id).set_nav_mesh(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::new(nav_mesh),
  );

  let nav_data = &archipelago.nav_data;

  let path_result = find_path(
    nav_data,
    NodeRef { island_id, polygon_index: 0 },
    NodeRef { island_id, polygon_index: 2 },
  )
  .expect("found path");

  assert_eq!(
    path_result.path,
    Path {
      corridor: vec![
        NodeRef { island_id, polygon_index: 0 },
        NodeRef { island_id, polygon_index: 1 },
        NodeRef { island_id, polygon_index: 2 }
      ],
      portal_edge_index: vec![4, 2],
    }
  );

  let path_result = find_path(
    nav_data,
    NodeRef { island_id, polygon_index: 2 },
    NodeRef { island_id, polygon_index: 0 },
  )
  .expect("found path");

  assert_eq!(
    path_result.path,
    Path {
      corridor: vec![
        NodeRef { island_id, polygon_index: 2 },
        NodeRef { island_id, polygon_index: 1 },
        NodeRef { island_id, polygon_index: 0 }
      ],
      portal_edge_index: vec![0, 0],
    }
  );

  let path_result = find_path(
    nav_data,
    NodeRef { island_id, polygon_index: 3 },
    NodeRef { island_id, polygon_index: 0 },
  )
  .expect("found path");

  assert_eq!(
    path_result.path,
    Path {
      corridor: vec![
        NodeRef { island_id, polygon_index: 3 },
        NodeRef { island_id, polygon_index: 1 },
        NodeRef { island_id, polygon_index: 0 }
      ],
      portal_edge_index: vec![0, 0],
    }
  );
}

#[test]
fn finds_paths_on_two_islands() {
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
  let nav_mesh = Arc::new(nav_mesh);

  let mut archipelago = Archipelago::new();
  let island_id_1 = archipelago.add_island();
  archipelago.get_island_mut(island_id_1).set_nav_mesh(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::clone(&nav_mesh),
  );

  let island_id_2 = archipelago.add_island();
  archipelago.get_island_mut(island_id_2).set_nav_mesh(
    Transform { translation: Vec3::new(6.0, 0.0, 0.0), rotation: PI * 0.5 },
    Arc::clone(&nav_mesh),
  );

  let nav_data = &archipelago.nav_data;

  let path_result = find_path(
    nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_1, polygon_index: 2 },
  )
  .expect("found path");

  assert_eq!(
    path_result.path,
    Path {
      corridor: vec![
        NodeRef { island_id: island_id_1, polygon_index: 0 },
        NodeRef { island_id: island_id_1, polygon_index: 1 },
        NodeRef { island_id: island_id_1, polygon_index: 2 }
      ],
      portal_edge_index: vec![4, 2],
    }
  );

  let path_result = find_path(
    nav_data,
    NodeRef { island_id: island_id_2, polygon_index: 0 },
    NodeRef { island_id: island_id_2, polygon_index: 2 },
  )
  .expect("found path");

  assert_eq!(
    path_result.path,
    Path {
      corridor: vec![
        NodeRef { island_id: island_id_2, polygon_index: 0 },
        NodeRef { island_id: island_id_2, polygon_index: 1 },
        NodeRef { island_id: island_id_2, polygon_index: 2 }
      ],
      portal_edge_index: vec![4, 2],
    }
  );
}

#[test]
fn no_path_between_disconnected_islands() {
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
  let nav_mesh = Arc::new(nav_mesh);

  let mut archipelago = Archipelago::new();
  let island_id_1 = archipelago.add_island();
  archipelago.get_island_mut(island_id_1).set_nav_mesh(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::clone(&nav_mesh),
  );

  let island_id_2 = archipelago.add_island();
  archipelago.get_island_mut(island_id_2).set_nav_mesh(
    Transform { translation: Vec3::new(6.0, 0.0, 0.0), rotation: PI * 0.5 },
    Arc::clone(&nav_mesh),
  );

  let nav_data = &archipelago.nav_data;

  assert!(find_path(
    nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_2, polygon_index: 0 },
  )
  .is_err());

  assert!(find_path(
    nav_data,
    NodeRef { island_id: island_id_2, polygon_index: 0 },
    NodeRef { island_id: island_id_1, polygon_index: 0 },
  )
  .is_err());
}
