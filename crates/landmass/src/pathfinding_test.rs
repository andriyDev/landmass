use std::{collections::HashMap, f32::consts::PI, sync::Arc};

use glam::{Vec2, Vec3};

use crate::{
  coords::{XY, XYZ},
  nav_data::NodeRef,
  nav_mesh::NavigationMesh,
  path::{BoundaryLinkSegment, IslandSegment, Path},
  AgentOptions, Archipelago, FromAgentRadius, Island, Transform,
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

  let mut archipelago =
    Archipelago::<XYZ>::new(AgentOptions::from_agent_radius(0.5));
  let island_id = archipelago.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::new(nav_mesh),
    HashMap::new(),
  ));

  let nav_data = &archipelago.nav_data;

  let path_result = find_path(
    nav_data,
    NodeRef { island_id, polygon_index: 0 },
    NodeRef { island_id, polygon_index: 2 },
    &HashMap::new(),
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
    &HashMap::new(),
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
    &HashMap::new(),
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

  let mut archipelago =
    Archipelago::<XYZ>::new(AgentOptions::from_agent_radius(0.5));
  let island_id_1 = archipelago.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::clone(&nav_mesh),
    HashMap::new(),
  ));

  let island_id_2 = archipelago.add_island(Island::new(
    Transform { translation: Vec3::new(6.0, 0.0, 0.0), rotation: PI * -0.5 },
    Arc::clone(&nav_mesh),
    HashMap::new(),
  ));

  let nav_data = &archipelago.nav_data;

  let path_result = find_path(
    nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_1, polygon_index: 2 },
    &HashMap::new(),
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
    &HashMap::new(),
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

  let mut archipelago =
    Archipelago::<XYZ>::new(AgentOptions::from_agent_radius(0.5));
  let island_id_1 = archipelago.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::clone(&nav_mesh),
    HashMap::new(),
  ));

  let island_id_2 = archipelago.add_island(Island::new(
    Transform { translation: Vec3::new(6.0, 0.0, 0.0), rotation: PI * -0.5 },
    Arc::clone(&nav_mesh),
    HashMap::new(),
  ));

  let nav_data = &archipelago.nav_data;

  assert!(find_path(
    nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_2, polygon_index: 0 },
    &HashMap::new(),
  )
  .path
  .is_none());

  assert!(find_path(
    nav_data,
    NodeRef { island_id: island_id_2, polygon_index: 0 },
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    &HashMap::new(),
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

  let mut archipelago =
    Archipelago::<XYZ>::new(AgentOptions::from_agent_radius(0.5));

  let island_id_1 = archipelago.add_island(Island::new(
    Transform { rotation: 0.0, translation: Vec3::ZERO },
    Arc::clone(&nav_mesh),
    HashMap::new(),
  ));
  let island_id_2 = archipelago.add_island(Island::new(
    Transform { rotation: 0.0, translation: Vec3::new(1.0, 0.0, 0.0) },
    Arc::clone(&nav_mesh),
    HashMap::new(),
  ));
  // island_id_3 is unused.
  archipelago.add_island(Island::new(
    Transform { rotation: 0.0, translation: Vec3::new(1.0, -1.0, 0.0) },
    Arc::clone(&nav_mesh),
    HashMap::new(),
  ));
  let island_id_4 = archipelago.add_island(Island::new(
    Transform { rotation: 0.0, translation: Vec3::new(1.0, 1.0, 0.0) },
    Arc::clone(&nav_mesh),
    HashMap::new(),
  ));
  let island_id_5 = archipelago.add_island(Island::new(
    Transform { rotation: 0.0, translation: Vec3::new(1.0, 2.0, 0.0) },
    Arc::clone(&nav_mesh),
    HashMap::new(),
  ));

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
    &HashMap::new(),
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

  let mut archipelago =
    Archipelago::<XYZ>::new(AgentOptions::from_agent_radius(0.5));

  let island_id_1 = archipelago.add_island(Island::new(
    Transform { rotation: 0.0, translation: Vec3::ZERO },
    nav_mesh_1,
    HashMap::new(),
  ));
  let island_id_2 = archipelago.add_island(Island::new(
    Transform { rotation: 0.0, translation: Vec3::new(1.0, 0.0, 0.0) },
    nav_mesh_2,
    HashMap::new(),
  ));

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
    &HashMap::new(),
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

  let mut archipelago =
    Archipelago::<XYZ>::new(AgentOptions::from_agent_radius(0.5));

  let island_id_1 = archipelago.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    nav_mesh.clone(),
    HashMap::new(),
  ));
  let island_id_2 = archipelago.add_island(Island::new(
    Transform { translation: Vec3::new(2.0, 0.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
    HashMap::new(),
  ));
  let island_id_3 = archipelago.add_island(Island::new(
    Transform { translation: Vec3::new(1.5, 2.0, 0.0), rotation: PI * 0.5 },
    nav_mesh.clone(),
    HashMap::new(),
  ));

  archipelago.update(1.0);

  // Verify that with island_id_3, the islands are connected.
  assert!(find_path(
    &archipelago.nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_2, polygon_index: 0 },
    &HashMap::new(),
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
    &HashMap::new(),
  );

  assert!(path_result.path.is_none());
  assert_eq!(path_result.stats.explored_nodes, 0,
    "No nodes should have been explored since the regions are completely disconnected.");
}

#[test]
fn detour_for_high_cost_path() {
  let mut archipelago =
    Archipelago::<XY>::new(AgentOptions::from_agent_radius(0.5));

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

  let slow_node_type = archipelago.add_node_type(10.0).unwrap();

  let island_id = archipelago.add_island(Island::new(
    Transform::default(),
    nav_mesh,
    HashMap::from([(1, slow_node_type)]),
  ));

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id, polygon_index: 0 },
    NodeRef { island_id, polygon_index: 6 },
    &HashMap::new(),
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
  let mut archipelago =
    Archipelago::<XY>::new(AgentOptions::from_agent_radius(0.5));

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

  let slow_node_type = archipelago.add_node_type(5.1).unwrap();

  let island_id_1 = archipelago.add_island(Island::new(
    Transform::default(),
    nav_mesh_1,
    HashMap::new(),
  ));
  let island_id_2 = archipelago.add_island(Island::new(
    Transform::default(),
    nav_mesh_2,
    HashMap::from([(1, slow_node_type)]),
  ));

  archipelago.update(1.0);

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id: island_id_1, polygon_index: 0 },
    NodeRef { island_id: island_id_2, polygon_index: 0 },
    &HashMap::new(),
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

#[test]
fn fast_path_not_ignored_by_heuristic() {
  let mut archipelago =
    Archipelago::<XY>::new(AgentOptions::from_agent_radius(0.5));

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
        Vec2::new(2.0, 11.0),
        Vec2::new(3.0, 11.0),
        // Extrude up.
        Vec2::new(2.0, 12.0),
        Vec2::new(3.0, 12.0),
        // Extrude left.
        Vec2::new(1.0, 11.0),
        Vec2::new(1.0, 12.0),
        // Extrude left.
        Vec2::new(0.0, 11.0),
        Vec2::new(0.0, 12.0),
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
        // The "fast" bridge.
        vec![3, 2, 12, 14],
      ],
      polygon_type_indices: vec![1, 0, 0, 0, 0, 1, 1, 1],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  // This node type is faster than default.
  let fast_type = archipelago.add_node_type(0.5).unwrap();

  let island_id = archipelago.add_island(Island::new(
    Transform::default(),
    nav_mesh,
    HashMap::from([(1, fast_type)]),
  ));

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id, polygon_index: 2 },
    NodeRef { island_id, polygon_index: 4 },
    &HashMap::new(),
  );

  // The most direct route is [2, 3, 4], but there is a faster detour:
  assert_eq!(
    path_result.path,
    Some(Path {
      island_segments: vec![IslandSegment {
        island_id,
        corridor: vec![2, 1, 0, 7, 6, 5, 4],
        portal_edge_index: vec![0, 0, 2, 2, 0, 0],
      }],
      boundary_link_segments: vec![],
    })
  );
}

#[test]
fn infinite_or_nan_cost_cannot_find_path() {
  let mut archipelago =
    Archipelago::<XY>::new(AgentOptions::from_agent_radius(0.5));

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
        Vec2::new(2.0, 0.0),
        Vec2::new(2.0, 1.0),
        Vec2::new(3.0, 0.0),
        Vec2::new(3.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3], vec![2, 1, 4, 5], vec![5, 4, 6, 7]],
      polygon_type_indices: vec![0, 1, 0],
    }
    .validate()
    .expect("mesh is valid"),
  );

  let node_type = archipelago.add_node_type(f32::INFINITY).unwrap();

  let island_id = archipelago.add_island(Island::new(
    Transform::default(),
    nav_mesh,
    HashMap::from([(1, node_type)]),
  ));

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id, polygon_index: 0 },
    NodeRef { island_id, polygon_index: 2 },
    &HashMap::new(),
  );

  assert_eq!(path_result.path, None);
  assert_eq!(path_result.stats.explored_nodes, 1);

  archipelago.set_node_type_cost(node_type, f32::NAN).unwrap();

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id, polygon_index: 0 },
    NodeRef { island_id, polygon_index: 2 },
    &HashMap::new(),
  );

  assert_eq!(path_result.path, None);
  assert_eq!(path_result.stats.explored_nodes, 1);
}

#[test]
fn detour_for_overridden_high_cost_path() {
  let mut archipelago =
    Archipelago::<XY>::new(AgentOptions::from_agent_radius(0.5));

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

  let slow_node_type = archipelago.add_node_type(1.0).unwrap();

  let island_id = archipelago.add_island(Island::new(
    Transform::default(),
    nav_mesh,
    HashMap::from([(1, slow_node_type)]),
  ));

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id, polygon_index: 0 },
    NodeRef { island_id, polygon_index: 6 },
    &HashMap::from([(slow_node_type, 10.0)]),
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
fn big_node_does_not_skew_pathing() {
  let mut archipelago =
    Archipelago::<XY>::new(AgentOptions::from_agent_radius(0.5));

  // The middle-left node is too long, so its center is really far off compared
  // to the edges that the agent actually travels through. We should go
  // through the middle-left node, but if we always travel through edge
  // centers, the right "detour" would be selected.
  //
  //             +-+-+-+
  //             |E|X|X|
  // +-----------+-+-+-+
  // |XXXXXXXXXXXXX| |X|
  // +-----------+-+-+-+
  //             |S|X|X|
  //             +-+-+-+
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(2.0, 0.0),
        Vec2::new(3.0, 0.0),
        Vec2::new(0.0, 1.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(2.0, 1.0),
        Vec2::new(3.0, 1.0),
        Vec2::new(0.0, 2.0),
        Vec2::new(1.0, 2.0),
        Vec2::new(2.0, 2.0),
        Vec2::new(3.0, 2.0),
        Vec2::new(0.0, 3.0),
        Vec2::new(1.0, 3.0),
        Vec2::new(2.0, 3.0),
        Vec2::new(3.0, 3.0),
        // Long part to the left.
        Vec2::new(-100.0, 1.0),
        Vec2::new(-100.0, 2.0),
      ],
      polygons: vec![
        // The bottom row.
        vec![0, 1, 5, 4],
        vec![1, 2, 6, 5],
        vec![2, 3, 7, 6],
        // The top row.
        vec![8, 9, 13, 12],
        vec![9, 10, 14, 13],
        vec![10, 11, 15, 14],
        // The right node.
        vec![6, 7, 11, 10],
        // The long left node.
        vec![4, 5, 9, 8, 17, 16],
      ],
      polygon_type_indices: vec![0, 0, 0, 0, 0, 0, 0, 0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let island_id = archipelago.add_island(Island::new(
    Transform::default(),
    nav_mesh,
    HashMap::default(),
  ));

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id, polygon_index: 0 },
    NodeRef { island_id, polygon_index: 3 },
    &HashMap::default(),
  );

  assert_eq!(
    path_result.path,
    Some(Path {
      island_segments: vec![IslandSegment {
        island_id,
        corridor: vec![0, 7, 3],
        portal_edge_index: vec![2, 2],
      }],
      boundary_link_segments: vec![],
    })
  );
}

#[test]
fn start_and_end_point_influences_path() {
  let mut archipelago =
    Archipelago::<XY>::new(AgentOptions::from_agent_radius(0.5));

  // We want to ensure that the start and end points of the path are taken into
  // consideration when planning. Previously, we would always assume that the
  // start and end points were always at the center of nodes. However, this can
  // result in odd paths when the centers of the nodes happen to produce shorter
  // paths. The optimal path for these start and end points is going along the
  // left side. However using the centers of the nodes for the start and end
  // points would result in taking the "middle" path.
  //
  // +-+---+-+---+
  // |X|EXXXXXXXX|
  // +-+---+-+---+
  // |X|   |X|
  // +-+---+-+---+
  // |X|SXXXXXXXX|
  // +-+---+-+---+
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(6.0, 0.0),
        Vec2::new(7.0, 0.0),
        Vec2::new(12.0, 0.0),
        Vec2::new(0.0, 1.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(6.0, 1.0),
        Vec2::new(7.0, 1.0),
        Vec2::new(12.0, 1.0),
        Vec2::new(0.0, 2.0),
        Vec2::new(1.0, 2.0),
        Vec2::new(6.0, 2.0),
        Vec2::new(7.0, 2.0),
        Vec2::new(12.0, 2.0),
        Vec2::new(0.0, 3.0),
        Vec2::new(1.0, 3.0),
        Vec2::new(6.0, 3.0),
        Vec2::new(7.0, 3.0),
        Vec2::new(12.0, 3.0),
      ],
      polygons: vec![
        // Bottom row.
        vec![0, 1, 6, 5],
        vec![1, 2, 3, 4, 9, 8, 7, 6],
        // Top row.
        vec![10, 11, 16, 15],
        vec![11, 12, 13, 14, 19, 18, 17, 16],
        // Middle row.
        vec![5, 6, 11, 10],
        vec![7, 8, 13, 12],
      ],
      polygon_type_indices: vec![0, 0, 0, 0, 0, 0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  // TODO: Use these in the test.
  let start_point = Vec3::new(1.5, 0.5, 0.0);
  let end_point = Vec3::new(1.5, 2.5, 0.0);

  let island_id = archipelago.add_island(Island::new(
    Transform::default(),
    nav_mesh,
    HashMap::default(),
  ));

  let path_result = find_path(
    &archipelago.nav_data,
    NodeRef { island_id, polygon_index: 1 },
    NodeRef { island_id, polygon_index: 3 },
    &HashMap::default(),
  );

  assert_eq!(
    path_result.path,
    Some(Path {
      island_segments: vec![IslandSegment {
        island_id,
        corridor: vec![1, 0, 4, 2, 3],
        portal_edge_index: vec![7, 2, 2, 1],
      }],
      boundary_link_segments: vec![],
    })
  );
}
