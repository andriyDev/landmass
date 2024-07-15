use std::{collections::HashMap, f32::consts::PI, sync::Arc};

use glam::{Vec2, Vec3};

use crate::{
  coords::{XY, XYZ},
  nav_data::NodeRef,
  nav_mesh::NavigationMesh,
  path::{BoundaryLinkSegment, IslandSegment, Path},
  Archipelago, Transform,
};

use super::find_path;

#[test]
fn finds_path_in_archipelago() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(4.0, 1.0, 0.0),
      Vec3::new(4.0, 2.0, 0.0),
      Vec3::new(2.0, 3.0, 0.0),
      Vec3::new(1.0, 3.0, 0.0),
      Vec3::new(0.0, 2.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
      Vec3::new(1.0, 5.0, 0.0),
      Vec3::new(2.0, 5.0, 0.0),
      Vec3::new(2.0, 4.0, 0.0),
      Vec3::new(3.0, 5.0, 1.0),
      Vec3::new(3.0, 4.0, 1.0),
      Vec3::new(3.0, 4.0, -2.0),
      Vec3::new(3.0, 3.0, -2.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3, 4, 5, 6, 7],
      vec![5, 4, 10, 9, 8],
      vec![9, 10, 12, 11],
      vec![10, 4, 14, 13],
    ],
    polygon_type_indices: vec![0, 0, 0, 0],
  }
  .validate()
  .expect("Mesh is valid.");

  let mut archipelago = Archipelago::<XYZ>::new();
  let island_id = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::new(nav_mesh),
      HashMap::new(),
    )
    .id();

  let nav_data = &archipelago.nav_data;

  let path_result = find_path(
    nav_data,
    NodeRef { island_id, polygon_index: 0 },
    NodeRef { island_id, polygon_index: 2 },
  );

  assert_eq!(
    path_result.path,
    Some(Path {
      island_segments: vec![IslandSegment {
        island_id,
        corridor: vec![0, 1, 2],
        portal_edge_index: vec![4, 2],
      }],
      boundary_link_segments: vec![],
    })
  );

  let path_result = find_path(
    nav_data,
    NodeRef { island_id, polygon_index: 2 },
    NodeRef { island_id, polygon_index: 0 },
  );

  assert_eq!(
    path_result.path,
    Some(Path {
      island_segments: vec![IslandSegment {
        island_id,
        corridor: vec![2, 1, 0],
        portal_edge_index: vec![0, 0],
      }],
      boundary_link_segments: vec![],
    })
  );

  let path_result = find_path(
    nav_data,
    NodeRef { island_id, polygon_index: 3 },
    NodeRef { island_id, polygon_index: 0 },
  );

  assert_eq!(
    path_result.path,
    Some(Path {
      island_segments: vec![IslandSegment {
        island_id,
        corridor: vec![3, 1, 0],
        portal_edge_index: vec![0, 0],
      }],
      boundary_link_segments: vec![],
    })
  );
}

#[test]
fn finds_paths_on_two_islands() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(4.0, 1.0, 0.0),
      Vec3::new(4.0, 2.0, 0.0),
      Vec3::new(2.0, 3.0, 0.0),
      Vec3::new(1.0, 3.0, 0.0),
      Vec3::new(0.0, 2.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
      Vec3::new(1.0, 5.0, 0.0),
      Vec3::new(2.0, 5.0, 0.0),
      Vec3::new(2.0, 4.0, 0.0),
      Vec3::new(3.0, 5.0, 1.0),
      Vec3::new(3.0, 4.0, 1.0),
      Vec3::new(3.0, 4.0, -2.0),
      Vec3::new(3.0, 3.0, -2.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3, 4, 5, 6, 7],
      vec![5, 4, 10, 9, 8],
      vec![9, 10, 12, 11],
      vec![10, 4, 14, 13],
    ],
    polygon_type_indices: vec![0, 0, 0, 0],
  }
  .validate()
  .expect("Mesh is valid.");
  let nav_mesh = Arc::new(nav_mesh);

  let mut archipelago = Archipelago::<XYZ>::new();
  let island_id_1 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();

  let island_id_2 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::new(6.0, 0.0, 0.0), rotation: PI * -0.5 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();

  let nav_data = &archipelago.nav_data;

  let path_result = find_path(
    nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_1, polygon_index: 2 },
  );

  assert_eq!(
    path_result.path,
    Some(Path {
      island_segments: vec![IslandSegment {
        island_id: island_id_1,
        corridor: vec![0, 1, 2],
        portal_edge_index: vec![4, 2],
      }],
      boundary_link_segments: vec![],
    })
  );

  let path_result = find_path(
    nav_data,
    NodeRef { island_id: island_id_2, polygon_index: 0 },
    NodeRef { island_id: island_id_2, polygon_index: 2 },
  );

  assert_eq!(
    path_result.path,
    Some(Path {
      island_segments: vec![IslandSegment {
        island_id: island_id_2,
        corridor: vec![0, 1, 2],
        portal_edge_index: vec![4, 2],
      }],
      boundary_link_segments: vec![],
    })
  );
}

#[test]
fn no_path_between_disconnected_islands() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(4.0, 1.0, 0.0),
      Vec3::new(4.0, 2.0, 0.0),
      Vec3::new(2.0, 3.0, 0.0),
      Vec3::new(1.0, 3.0, 0.0),
      Vec3::new(0.0, 2.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
      Vec3::new(1.0, 5.0, 0.0),
      Vec3::new(2.0, 5.0, 0.0),
      Vec3::new(2.0, 4.0, 0.0),
      Vec3::new(3.0, 5.0, 1.0),
      Vec3::new(3.0, 4.0, 1.0),
      Vec3::new(3.0, 4.0, -2.0),
      Vec3::new(3.0, 3.0, -2.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3, 4, 5, 6, 7],
      vec![5, 4, 10, 9, 8],
      vec![9, 10, 12, 11],
      vec![10, 4, 14, 13],
    ],
    polygon_type_indices: vec![0, 0, 0, 0],
  }
  .validate()
  .expect("Mesh is valid.");
  let nav_mesh = Arc::new(nav_mesh);

  let mut archipelago = Archipelago::<XYZ>::new();
  let island_id_1 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();

  let island_id_2 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::new(6.0, 0.0, 0.0), rotation: PI * -0.5 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();

  let nav_data = &archipelago.nav_data;

  assert!(find_path(
    nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_2, polygon_index: 0 },
  )
  .path
  .is_none());

  assert!(find_path(
    nav_data,
    NodeRef { island_id: island_id_2, polygon_index: 0 },
    NodeRef { island_id: island_id_1, polygon_index: 0 },
  )
  .path
  .is_none());
}

#[test]
fn find_path_across_connected_islands() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(-0.5, -0.5, 0.0),
        Vec3::new(0.5, -0.5, 0.0),
        Vec3::new(0.5, 0.5, 0.0),
        Vec3::new(-0.5, 0.5, 0.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("Mesh is valid."),
  );

  let mut archipelago = Archipelago::<XYZ>::new();

  let island_id_1 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { rotation: 0.0, translation: Vec3::ZERO },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();
  let island_id_2 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { rotation: 0.0, translation: Vec3::new(1.0, 0.0, 0.0) },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();
  // island_id_3 is unused.
  archipelago.add_island().set_nav_mesh(
    Transform { rotation: 0.0, translation: Vec3::new(1.0, -1.0, 0.0) },
    Arc::clone(&nav_mesh),
    HashMap::new(),
  );
  let island_id_4 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { rotation: 0.0, translation: Vec3::new(1.0, 1.0, 0.0) },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();
  let island_id_5 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { rotation: 0.0, translation: Vec3::new(1.0, 2.0, 0.0) },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();

  archipelago.update(1.0);

  let boundary_links = archipelago
    .nav_data
    .node_to_boundary_link_ids
    .iter()
    .flat_map(|(node_ref, link_ids)| {
      link_ids.iter().map(|link_id| {
        let link = archipelago.nav_data.boundary_links.get(*link_id).unwrap();
        ((node_ref.island_id, link.destination_node.island_id), *link_id)
      })
    })
    .collect::<HashMap<_, _>>();

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_5, polygon_index: 0 },
  );

  assert_eq!(
    path_result.path,
    Some(Path {
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
    })
  );
}

#[test]
fn finds_path_across_different_islands() {
  let nav_mesh_1 = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(-0.5, -0.5, 0.0),
        Vec3::new(0.5, -0.5, 0.0),
        Vec3::new(0.5, 0.5, 0.0),
        Vec3::new(-0.5, 0.5, 0.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("Mesh is valid."),
  );
  let nav_mesh_2 = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(-0.5, -0.5, 0.0),
        Vec3::new(0.5, -0.5, 0.0),
        Vec3::new(0.5, 0.5, 0.0),
        Vec3::new(-0.5, 0.5, 0.0),
        Vec3::new(1.5, -0.5, 0.0),
        Vec3::new(1.5, 0.5, 0.0),
      ],
      polygons: vec![vec![0, 1, 2, 3], vec![2, 1, 4, 5]],
      polygon_type_indices: vec![0, 0],
    }
    .validate()
    .expect("Mesh is valid."),
  );

  let mut archipelago = Archipelago::<XYZ>::new();

  let island_id_1 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { rotation: 0.0, translation: Vec3::ZERO },
      nav_mesh_1,
      HashMap::new(),
    )
    .id();
  let island_id_2 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { rotation: 0.0, translation: Vec3::new(1.0, 0.0, 0.0) },
      nav_mesh_2,
      HashMap::new(),
    )
    .id();

  archipelago.update(1.0);

  let boundary_links = archipelago
    .nav_data
    .node_to_boundary_link_ids
    .iter()
    .flat_map(|(node_ref, link_ids)| {
      link_ids.iter().map(|link_id| {
        let link = archipelago.nav_data.boundary_links.get(*link_id).unwrap();
        ((node_ref.island_id, link.destination_node.island_id), *link_id)
      })
    })
    .collect::<HashMap<_, _>>();

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_2, polygon_index: 1 },
  );

  assert_eq!(
    path_result.path,
    Some(Path {
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
    })
  );
}

#[test]
fn aborts_early_for_unconnected_regions() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(-0.5, -1.5, 0.0),
        Vec3::new(0.5, -1.5, 0.0),
        Vec3::new(0.5, 1.5, 0.0),
        Vec3::new(-0.5, 1.5, 0.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("Mesh is valid."),
  );

  let mut archipelago = Archipelago::<XYZ>::new();

  let island_id_1 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      nav_mesh.clone(),
      HashMap::new(),
    )
    .id();
  let island_id_2 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::new(2.0, 0.0, 0.0), rotation: 0.0 },
      nav_mesh.clone(),
      HashMap::new(),
    )
    .id();
  let island_id_3 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::new(1.5, 2.0, 0.0), rotation: PI * 0.5 },
      nav_mesh.clone(),
      HashMap::new(),
    )
    .id();

  archipelago.update(1.0);

  // Verify that with island_id_3, the islands are connected.
  assert!(find_path(
    &archipelago.nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_2, polygon_index: 0 },
  )
  .path
  .is_some());

  // Remove island_id_3 which will disconnect the other two islands.
  archipelago.remove_island(island_id_3);
  archipelago.update(1.0);

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_2, polygon_index: 0 },
  );

  assert!(path_result.path.is_none());
  assert_eq!(path_result.stats.explored_nodes, 0,
    "No nodes should have been explored since the regions are completely disconnected.");
}

#[test]
fn detour_for_high_cost_path() {
  let mut archipelago = Archipelago::<XY>::new();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
        // Extrude right.
        Vec2::new(2.0, 0.0),
        Vec2::new(2.0, 1.0),
        // Extrude right.
        Vec2::new(3.0, 0.0),
        Vec2::new(3.0, 1.0),
        // Extrude up.
        Vec2::new(2.0, 2.0),
        Vec2::new(3.0, 2.0),
        // Extrude up.
        Vec2::new(2.0, 3.0),
        Vec2::new(3.0, 3.0),
        // Extrude left.
        Vec2::new(1.0, 2.0),
        Vec2::new(1.0, 3.0),
        // Extrude left.
        Vec2::new(0.0, 2.0),
        Vec2::new(0.0, 3.0),
      ],
      polygons: vec![
        // The bottom row.
        vec![0, 1, 2, 3],
        vec![2, 1, 4, 5],
        vec![5, 4, 6, 7],
        // The right two cells.
        vec![5, 7, 9, 8],
        vec![8, 9, 11, 10],
        // The top two cells.
        vec![8, 10, 13, 12],
        vec![12, 13, 15, 14],
        // The "slow" bridge.
        vec![3, 2, 12, 14],
      ],
      polygon_type_indices: vec![0, 0, 0, 0, 0, 0, 0, 1],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let slow_node_type = archipelago.create_node_type(10.0);

  let island_id = archipelago
    .add_island()
    .set_nav_mesh(
      Transform::default(),
      nav_mesh,
      HashMap::from([(1, slow_node_type)]),
    )
    .id();

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id, polygon_index: 0 },
    NodeRef { island_id, polygon_index: 6 },
  );

  assert_eq!(
    path_result.path,
    Some(Path {
      island_segments: vec![IslandSegment {
        island_id,
        corridor: vec![0, 1, 2, 3, 4, 5, 6],
        portal_edge_index: vec![1, 2, 3, 2, 3, 2],
      }],
      boundary_link_segments: vec![],
    })
  );
}

#[test]
fn detour_for_high_cost_path_across_boundary_links() {
  let mut archipelago = Archipelago::<XY>::new();

  let nav_mesh_1 = Arc::new(
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
      ],
      polygons: vec![vec![0, 1, 2, 3], vec![2, 1, 4, 5], vec![5, 4, 6, 7]],
      polygon_type_indices: vec![0, 0, 0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let nav_mesh_2 = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 2.0),
        Vec2::new(1.0, 2.0),
        Vec2::new(1.0, 3.0),
        Vec2::new(0.0, 3.0),
        //
        Vec2::new(2.0, 2.0),
        Vec2::new(2.0, 3.0),
        //
        Vec2::new(3.0, 2.0),
        Vec2::new(3.0, 3.0),
        //
        Vec2::new(0.0, 1.0),
        Vec2::new(1.0, 1.0),
        //
        Vec2::new(2.0, 1.0),
        Vec2::new(3.0, 1.0),
      ],
      polygons: vec![
        vec![0, 1, 2, 3],
        vec![2, 1, 4, 5],
        vec![5, 4, 6, 7],
        vec![1, 0, 8, 9],
        vec![6, 4, 10, 11],
      ],
      polygon_type_indices: vec![0, 0, 0, 1, 0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let slow_node_type = archipelago.create_node_type(5.1);

  let island_id_1 = archipelago
    .add_island()
    .set_nav_mesh(Transform::default(), nav_mesh_1, HashMap::new())
    .id();
  let island_id_2 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform::default(),
      nav_mesh_2,
      HashMap::from([(1, slow_node_type)]),
    )
    .id();

  archipelago.update(1.0);

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_2, polygon_index: 0 },
  );

  assert_eq!(
    path_result.path,
    Some(Path {
      island_segments: vec![
        IslandSegment {
          island_id: island_id_1,
          corridor: vec![0, 1, 2],
          portal_edge_index: vec![1, 2],
        },
        IslandSegment {
          island_id: island_id_2,
          corridor: vec![4, 2, 1, 0],
          portal_edge_index: vec![0, 0, 0],
        }
      ],
      boundary_link_segments: vec![BoundaryLinkSegment {
        boundary_link: archipelago.nav_data.node_to_boundary_link_ids
          [&NodeRef { island_id: island_id_1, polygon_index: 2 }]
          .iter()
          .next()
          .unwrap()
          .clone(),
        starting_node: NodeRef { island_id: island_id_1, polygon_index: 2 },
      }]
    })
  )
}
