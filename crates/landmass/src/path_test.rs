use std::{
  collections::{HashMap, HashSet},
  f32::consts::PI,
  sync::Arc,
};

use glam::{Vec2, Vec3};
use slotmap::HopSlotMap;

use crate::{
  coords::{XY, XYZ},
  nav_data::{BoundaryLinkId, NavigationData, NodeRef},
  nav_mesh::NavigationMesh,
  path::{BoundaryLinkSegment, IslandSegment},
  Archipelago, CoordinateSystem, Island, IslandId, Transform,
};

use super::{Path, PathIndex};

fn collect_straight_path<CS: CoordinateSystem>(
  path: &Path,
  nav_data: &NavigationData<CS>,
  start: (PathIndex, Vec3),
  end: (PathIndex, Vec3),
  iteration_limit: u32,
) -> Vec<(PathIndex, Vec3)> {
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

  let transform =
    Transform { translation: Vec3::new(5.0, 9.0, 7.0), rotation: PI * -0.35 };
  let mut archipelago = Archipelago::<XYZ>::new();
  let island_id = archipelago.add_island(Island::new(
    transform.clone(),
    Arc::new(nav_mesh),
    HashMap::new(),
  ));

  let path = Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![0, 1, 2],
      portal_edge_index: vec![4, 2],
    }],
    boundary_link_segments: vec![],
  };

  assert_eq!(
    collect_straight_path(
      &path,
      &archipelago.nav_data,
      /* start= */
      (
        PathIndex::from_corridor_index(0, 0),
        transform.apply(Vec3::new(3.0, 1.5, 0.0))
      ),
      /* end= */
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.5, 4.5, 0.5))
      ),
      /* iteration_limit= */ 3,
    ),
    [
      (
        PathIndex::from_corridor_index(0, 0),
        transform.apply(Vec3::new(2.0, 3.0, 0.0))
      ),
      (
        PathIndex::from_corridor_index(0, 1),
        transform.apply(Vec3::new(2.0, 4.0, 0.0))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.5, 4.5, 0.5))
      ),
    ]
  );
}

#[test]
fn finds_next_point_in_zig_zag() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec2::new(0.0, 0.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(1.0, 1.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(1.0, 2.0),
      Vec2::new(0.0, 2.0),
      Vec2::new(1.0, 3.0),
      Vec2::new(0.0, 3.0),
      Vec2::new(1.0, 4.0),
      Vec2::new(0.0, 4.0),
      Vec2::new(1.0, 5.0), // Turn right
      Vec2::new(2.0, 4.0),
      Vec2::new(2.0, 5.0),
      Vec2::new(3.0, 4.0),
      Vec2::new(3.0, 5.0),
      Vec2::new(4.0, 4.0),
      Vec2::new(4.0, 5.0),
      Vec2::new(5.0, 5.0), // Turn left
      Vec2::new(5.0, 6.0),
      Vec2::new(4.0, 6.0),
      Vec2::new(5.0, 7.0),
      Vec2::new(4.0, 7.0),
      Vec2::new(4.0, 8.0), // Turn left
      Vec2::new(-3.0, 8.0),
      Vec2::new(-3.0, 7.0),
      Vec2::new(-4.0, 8.0), // Turn right
      Vec2::new(-3.0, 15.0),
      Vec2::new(-4.0, 15.0),
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
    polygon_type_indices: vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  }
  .validate()
  .expect("Mesh is valid.");

  let transform =
    Transform { translation: Vec2::new(-1.0, -3.0), rotation: PI * -1.8 };
  let mut archipelago = Archipelago::<XY>::new();
  let island_id = archipelago.add_island(Island::new(
    transform.clone(),
    Arc::new(nav_mesh),
    HashMap::new(),
  ));

  let path = Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14],
      portal_edge_index: vec![2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1],
    }],
    boundary_link_segments: vec![],
  };

  assert_eq!(
    collect_straight_path(
      &path,
      &archipelago.nav_data,
      /* start= */
      (
        PathIndex::from_corridor_index(0, 0),
        transform.apply(Vec3::new(0.5, 0.5, 0.0))
      ),
      /* end= */
      (
        PathIndex::from_corridor_index(0, 14),
        transform.apply(Vec3::new(-3.5, 14.0, 0.0))
      ),
      /* iteration_limit= */ 5,
    ),
    [
      (
        PathIndex::from_corridor_index(0, 4),
        transform.apply(Vec3::new(1.0, 4.0, 0.0))
      ),
      (
        PathIndex::from_corridor_index(0, 8),
        transform.apply(Vec3::new(4.0, 5.0, 0.0))
      ),
      (
        PathIndex::from_corridor_index(0, 11),
        transform.apply(Vec3::new(4.0, 7.0, 0.0))
      ),
      (
        PathIndex::from_corridor_index(0, 13),
        transform.apply(Vec3::new(-3.0, 8.0, 0.0))
      ),
      (
        PathIndex::from_corridor_index(0, 14),
        transform.apply(Vec3::new(-3.5, 14.0, 0.0))
      ),
    ]
  );
}

#[test]
fn starts_at_end_index_goes_to_end_point() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
      Vec3::new(1.0, 2.0, 0.0),
      Vec3::new(0.0, 2.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2, 3], vec![3, 2, 4, 5]],
    polygon_type_indices: vec![0, 0],
  }
  .validate()
  .expect("Mesh is valid.");

  let mut archipelago = Archipelago::<XYZ>::new();
  let island_id = archipelago.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::new(nav_mesh),
    HashMap::new(),
  ));

  let path = Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![0, 1],
      portal_edge_index: vec![2],
    }],
    boundary_link_segments: vec![],
  };

  assert_eq!(
    path.find_next_point_in_straight_path(
      &archipelago.nav_data,
      /* start_index= */ PathIndex::from_corridor_index(0, 1),
      /* start_point= */ Vec3::new(0.25, 1.1, 0.0),
      /* end_index= */ PathIndex::from_corridor_index(0, 1),
      /* end_point= */ Vec3::new(0.75, 1.9, 0.0),
    ),
    (PathIndex::from_corridor_index(0, 1), Vec3::new(0.75, 1.9, 0.0))
  );
}

#[test]
fn path_not_valid_for_invalidated_islands_or_boundary_links() {
  // Create unused slotmaps just to get `IslandId`s and `BoundaryLinkId`s.
  let mut slotmap = HopSlotMap::<IslandId, _>::with_key();
  let island_id_1 = slotmap.insert(0);
  let island_id_2 = slotmap.insert(0);
  let island_id_3 = slotmap.insert(0);
  let mut slotmap = HopSlotMap::<BoundaryLinkId, _>::with_key();
  let boundary_link_id_1 = slotmap.insert(0);
  let boundary_link_id_2 = slotmap.insert(0);

  let path = Path {
    island_segments: vec![
      IslandSegment {
        island_id: island_id_1,
        corridor: vec![0],
        portal_edge_index: vec![],
      },
      IslandSegment {
        island_id: island_id_2,
        corridor: vec![0, 1],
        portal_edge_index: vec![],
      },
      IslandSegment {
        island_id: island_id_3,
        corridor: vec![0],
        portal_edge_index: vec![],
      },
    ],
    boundary_link_segments: vec![
      BoundaryLinkSegment {
        starting_node: NodeRef { island_id: island_id_1, polygon_index: 0 },
        boundary_link: boundary_link_id_1,
      },
      BoundaryLinkSegment {
        starting_node: NodeRef { island_id: island_id_2, polygon_index: 1 },
        boundary_link: boundary_link_id_2,
      },
    ],
  };

  assert!(path.is_valid(
    /* invalidated_boundary_links= */ &HashSet::new(),
    /* invalidated_islands= */ &HashSet::new()
  ));

  // Each island is invalidated.
  assert!(!path.is_valid(
    /* invalidated_boundary_links= */ &HashSet::new(),
    /* invalidated_islands= */ &HashSet::from([island_id_1]),
  ));
  assert!(!path.is_valid(
    /* invalidated_boundary_links= */ &HashSet::new(),
    /* invalidated_islands= */ &HashSet::from([island_id_2]),
  ));
  assert!(!path.is_valid(
    /* invalidated_boundary_links= */ &HashSet::new(),
    /* invalidated_islands= */ &HashSet::from([island_id_3]),
  ));

  // Each boundary link is invalidated.
  assert!(!path.is_valid(
    /* invalidated_boundary_links= */
    &HashSet::from([boundary_link_id_1]),
    /* invalidated_islands= */ &HashSet::new(),
  ));
  assert!(!path.is_valid(
    /* invalidated_boundary_links= */
    &HashSet::from([boundary_link_id_2]),
    /* invalidated_islands= */ &HashSet::new(),
  ));
}

#[test]
fn indices_in_path_are_found() {
  // Create unused slotmaps just to get `IslandId`s and `BoundaryLinkId`s.
  let mut slotmap = HopSlotMap::<IslandId, _>::with_key();
  let island_id_1 = slotmap.insert(0);
  let island_id_2 = slotmap.insert(0);
  let island_id_3 = slotmap.insert(0);
  let island_id_4 = slotmap.insert(0);
  let mut slotmap = HopSlotMap::<BoundaryLinkId, _>::with_key();
  let boundary_link_id_1 = slotmap.insert(0);
  let boundary_link_id_2 = slotmap.insert(0);

  let path = Path {
    island_segments: vec![
      IslandSegment {
        island_id: island_id_1,
        corridor: vec![3],
        portal_edge_index: vec![],
      },
      IslandSegment {
        island_id: island_id_2,
        corridor: vec![2, 1],
        portal_edge_index: vec![],
      },
      IslandSegment {
        island_id: island_id_3,
        corridor: vec![0],
        portal_edge_index: vec![],
      },
    ],
    boundary_link_segments: vec![
      BoundaryLinkSegment {
        starting_node: NodeRef { island_id: island_id_1, polygon_index: 0 },
        boundary_link: boundary_link_id_1,
      },
      BoundaryLinkSegment {
        starting_node: NodeRef { island_id: island_id_2, polygon_index: 1 },
        boundary_link: boundary_link_id_2,
      },
    ],
  };

  assert_eq!(
    path
      .find_index_of_node(NodeRef { island_id: island_id_3, polygon_index: 0 }),
    Some(PathIndex { segment_index: 2, portal_index: 0 })
  );
  assert_eq!(
    path
      .find_index_of_node(NodeRef { island_id: island_id_1, polygon_index: 3 }),
    Some(PathIndex { segment_index: 0, portal_index: 0 })
  );
  assert_eq!(
    path
      .find_index_of_node(NodeRef { island_id: island_id_2, polygon_index: 1 }),
    Some(PathIndex { segment_index: 1, portal_index: 1 })
  );

  // Missing NodeRefs.
  assert_eq!(
    path
      .find_index_of_node(NodeRef { island_id: island_id_3, polygon_index: 3 }),
    None
  );
  assert_eq!(
    path
      .find_index_of_node(NodeRef { island_id: island_id_1, polygon_index: 1 }),
    None
  );
  assert_eq!(
    path
      .find_index_of_node(NodeRef { island_id: island_id_4, polygon_index: 4 }),
    None
  );
}

#[test]
fn indices_in_path_are_found_rev() {
  // Create unused slotmaps just to get `IslandId`s and `BoundaryLinkId`s.
  let mut slotmap = HopSlotMap::<IslandId, _>::with_key();
  let island_id_1 = slotmap.insert(0);
  let island_id_2 = slotmap.insert(0);
  let island_id_3 = slotmap.insert(0);
  let island_id_4 = slotmap.insert(0);
  let mut slotmap = HopSlotMap::<BoundaryLinkId, _>::with_key();
  let boundary_link_id_1 = slotmap.insert(0);
  let boundary_link_id_2 = slotmap.insert(0);

  let path = Path {
    island_segments: vec![
      IslandSegment {
        island_id: island_id_1,
        corridor: vec![3],
        portal_edge_index: vec![],
      },
      IslandSegment {
        island_id: island_id_2,
        corridor: vec![2, 1],
        portal_edge_index: vec![],
      },
      IslandSegment {
        island_id: island_id_3,
        corridor: vec![0],
        portal_edge_index: vec![],
      },
    ],
    boundary_link_segments: vec![
      BoundaryLinkSegment {
        starting_node: NodeRef { island_id: island_id_1, polygon_index: 0 },
        boundary_link: boundary_link_id_1,
      },
      BoundaryLinkSegment {
        starting_node: NodeRef { island_id: island_id_2, polygon_index: 1 },
        boundary_link: boundary_link_id_2,
      },
    ],
  };

  assert_eq!(
    path.find_index_of_node_rev(NodeRef {
      island_id: island_id_3,
      polygon_index: 0
    }),
    Some(PathIndex { segment_index: 2, portal_index: 0 })
  );
  assert_eq!(
    path.find_index_of_node_rev(NodeRef {
      island_id: island_id_1,
      polygon_index: 3
    }),
    Some(PathIndex { segment_index: 0, portal_index: 0 })
  );
  assert_eq!(
    path.find_index_of_node_rev(NodeRef {
      island_id: island_id_2,
      polygon_index: 1
    }),
    Some(PathIndex { segment_index: 1, portal_index: 1 })
  );

  // Missing NodeRefs.
  assert_eq!(
    path.find_index_of_node_rev(NodeRef {
      island_id: island_id_3,
      polygon_index: 3
    }),
    None
  );
  assert_eq!(
    path.find_index_of_node_rev(NodeRef {
      island_id: island_id_1,
      polygon_index: 1
    }),
    None
  );
  assert_eq!(
    path.find_index_of_node_rev(NodeRef {
      island_id: island_id_4,
      polygon_index: 4
    }),
    None
  );
}
