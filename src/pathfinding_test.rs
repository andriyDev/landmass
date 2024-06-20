use std::{collections::HashMap, f32::consts::PI, sync::Arc};

use glam::Vec3;

use crate::{
  nav_data::NodeRef,
  nav_mesh::NavigationMesh,
  path::{BoundaryLinkSegment, IslandSegment, Path},
  Archipelago, Transform,
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
      island_segments: vec![IslandSegment {
        island_id,
        corridor: vec![0, 1, 2],
        portal_edge_index: vec![4, 2],
      }],
      boundary_link_segments: vec![],
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
      island_segments: vec![IslandSegment {
        island_id,
        corridor: vec![2, 1, 0],
        portal_edge_index: vec![0, 0],
      }],
      boundary_link_segments: vec![],
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
      island_segments: vec![IslandSegment {
        island_id,
        corridor: vec![3, 1, 0],
        portal_edge_index: vec![0, 0],
      }],
      boundary_link_segments: vec![],
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
      island_segments: vec![IslandSegment {
        island_id: island_id_1,
        corridor: vec![0, 1, 2],
        portal_edge_index: vec![4, 2],
      }],
      boundary_link_segments: vec![],
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
      island_segments: vec![IslandSegment {
        island_id: island_id_2,
        corridor: vec![0, 1, 2],
        portal_edge_index: vec![4, 2],
      }],
      boundary_link_segments: vec![],
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

#[test]
fn find_path_across_connected_islands() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      mesh_bounds: None,
      vertices: vec![
        Vec3::new(-0.5, 0.0, -0.5),
        Vec3::new(0.5, 0.0, -0.5),
        Vec3::new(0.5, 0.0, 0.5),
        Vec3::new(-0.5, 0.0, 0.5),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
    }
    .validate()
    .expect("Mesh is valid."),
  );

  let mut archipelago = Archipelago::new();

  let island_id_1 = archipelago.add_island();
  let island_id_2 = archipelago.add_island();
  let island_id_3 = archipelago.add_island();
  let island_id_4 = archipelago.add_island();
  let island_id_5 = archipelago.add_island();

  archipelago.get_island_mut(island_id_1).set_nav_mesh(
    Transform { rotation: 0.0, translation: Vec3::ZERO },
    Arc::clone(&nav_mesh),
  );
  archipelago.get_island_mut(island_id_2).set_nav_mesh(
    Transform { rotation: 0.0, translation: Vec3::new(1.0, 0.0, 0.0) },
    Arc::clone(&nav_mesh),
  );
  archipelago.get_island_mut(island_id_3).set_nav_mesh(
    Transform { rotation: 0.0, translation: Vec3::new(1.0, 0.0, -1.0) },
    Arc::clone(&nav_mesh),
  );
  archipelago.get_island_mut(island_id_4).set_nav_mesh(
    Transform { rotation: 0.0, translation: Vec3::new(1.0, 0.0, 1.0) },
    Arc::clone(&nav_mesh),
  );
  archipelago.get_island_mut(island_id_5).set_nav_mesh(
    Transform { rotation: 0.0, translation: Vec3::new(1.0, 0.0, 2.0) },
    Arc::clone(&nav_mesh),
  );

  archipelago.update(1.0);

  let boundary_links = archipelago
    .nav_data
    .boundary_links
    .iter()
    .flat_map(|(node_ref, link_id_to_link)| {
      link_id_to_link.iter().map(|(link_id, link)| {
        ((node_ref.island_id, link.destination_node.island_id), *link_id)
      })
    })
    .collect::<HashMap<_, _>>();

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_5, polygon_index: 0 },
  )
  .expect("found path");

  assert_eq!(
    path_result.path,
    Path {
      island_segments: vec![
        IslandSegment {
          island_id: island_id_1,
          corridor: vec![0],
          portal_edge_index: vec![],
        },
        IslandSegment {
          island_id: island_id_2,
          corridor: vec![0],
          portal_edge_index: vec![],
        },
        IslandSegment {
          island_id: island_id_4,
          corridor: vec![0],
          portal_edge_index: vec![],
        },
        IslandSegment {
          island_id: island_id_5,
          corridor: vec![0],
          portal_edge_index: vec![],
        },
      ],
      boundary_link_segments: vec![
        BoundaryLinkSegment {
          starting_node: NodeRef { island_id: island_id_1, polygon_index: 0 },
          boundary_link: boundary_links[&(island_id_1, island_id_2)],
        },
        BoundaryLinkSegment {
          starting_node: NodeRef { island_id: island_id_2, polygon_index: 0 },
          boundary_link: boundary_links[&(island_id_2, island_id_4)],
        },
        BoundaryLinkSegment {
          starting_node: NodeRef { island_id: island_id_4, polygon_index: 0 },
          boundary_link: boundary_links[&(island_id_4, island_id_5)],
        },
      ],
    }
  );
}

#[test]
fn finds_path_across_different_islands() {
  let nav_mesh_1 = Arc::new(
    NavigationMesh {
      mesh_bounds: None,
      vertices: vec![
        Vec3::new(-0.5, 0.0, -0.5),
        Vec3::new(0.5, 0.0, -0.5),
        Vec3::new(0.5, 0.0, 0.5),
        Vec3::new(-0.5, 0.0, 0.5),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
    }
    .validate()
    .expect("Mesh is valid."),
  );
  let nav_mesh_2 = Arc::new(
    NavigationMesh {
      mesh_bounds: None,
      vertices: vec![
        Vec3::new(-0.5, 0.0, -0.5),
        Vec3::new(0.5, 0.0, -0.5),
        Vec3::new(0.5, 0.0, 0.5),
        Vec3::new(-0.5, 0.0, 0.5),
        Vec3::new(1.5, 0.0, -0.5),
        Vec3::new(1.5, 0.0, 0.5),
      ],
      polygons: vec![vec![0, 1, 2, 3], vec![2, 1, 4, 5]],
    }
    .validate()
    .expect("Mesh is valid."),
  );

  let mut archipelago = Archipelago::new();

  let island_id_1 = archipelago.add_island();
  let island_id_2 = archipelago.add_island();

  archipelago.get_island_mut(island_id_1).set_nav_mesh(
    Transform { rotation: 0.0, translation: Vec3::ZERO },
    nav_mesh_1,
  );
  archipelago.get_island_mut(island_id_2).set_nav_mesh(
    Transform { rotation: 0.0, translation: Vec3::new(1.0, 0.0, 0.0) },
    nav_mesh_2,
  );

  archipelago.update(1.0);

  let boundary_links = archipelago
    .nav_data
    .boundary_links
    .iter()
    .flat_map(|(node_ref, link_id_to_link)| {
      link_id_to_link.iter().map(|(link_id, link)| {
        ((node_ref.island_id, link.destination_node.island_id), *link_id)
      })
    })
    .collect::<HashMap<_, _>>();

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_2, polygon_index: 1 },
  )
  .expect("found path");

  assert_eq!(
    path_result.path,
    Path {
      island_segments: vec![
        IslandSegment {
          island_id: island_id_1,
          corridor: vec![0],
          portal_edge_index: vec![],
        },
        IslandSegment {
          island_id: island_id_2,
          corridor: vec![0, 1],
          portal_edge_index: vec![1],
        },
      ],
      boundary_link_segments: vec![BoundaryLinkSegment {
        starting_node: NodeRef { island_id: island_id_1, polygon_index: 0 },
        boundary_link: boundary_links[&(island_id_1, island_id_2)],
      }],
    }
  );
}
