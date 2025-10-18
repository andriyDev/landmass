use std::{collections::HashSet, f32::consts::PI};

use glam::{Vec2, Vec3};
use googletest::{expect_that, matchers::*};
use slotmap::HopSlotMap;

use crate::{
  Archipelago, ArchipelagoOptions, CoordinateSystem, FromAgentRadius, IslandId,
  Transform,
  coords::{XY, XYZ},
  link::{AnimationLink, AnimationLinkId},
  nav_data::{KindedOffMeshLink, NavigationData, NodeRef, OffMeshLinkId},
  nav_mesh::NavigationMesh,
  path::{IslandSegment, OffMeshLinkSegment, StraightPathStep},
};

use super::{Path, PathIndex};

fn collect_straight_path(
  path: &Path,
  nav_data: &NavigationData,
  start: (PathIndex, Vec3),
  end: (PathIndex, Vec3),
  iteration_limit: u32,
) -> Vec<(PathIndex, StraightPathStep)> {
  let mut straight_path = Vec::with_capacity(iteration_limit as usize);

  let mut current = (start.0, StraightPathStep::Waypoint(start.1));
  let mut iterations = 0;
  let mut force_iteration = false;
  while (force_iteration || current.0 != end.0) && iterations < iteration_limit
  {
    force_iteration = false;

    iterations += 1;
    let current_point = match current.1 {
      StraightPathStep::Waypoint(point) => point,
      // After using the animation link, we end up at the `end_point`.
      StraightPathStep::AnimationLink { end_point, .. } => end_point,
    };
    current = path.find_next_point_in_straight_path(
      nav_data,
      current.0,
      current_point,
      end.0,
      end.1,
    );
    if let StraightPathStep::AnimationLink { .. } = &current.1 {
      // Make sure to do an extra iteration after an animation link so we see
      // the end point.
      force_iteration = true;
    }
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
    height_mesh: None,
  }
  .validate()
  .expect("Mesh is valid.");

  let transform =
    Transform { translation: Vec3::new(5.0, 9.0, 7.0), rotation: PI * -0.35 };
  let mut archipelago =
    Archipelago::<XYZ>::new(ArchipelagoOptions::from_agent_radius(0.5));
  let island_id = archipelago.add_island(transform.clone(), nav_mesh);
  let transform = transform.to_core();

  let path = Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![0, 1, 2],
      portal_edge_index: vec![4, 2],
    }],
    off_mesh_link_segments: vec![],
    start_point: transform.apply(Vec3::new(3.0, 1.5, 0.0)),
    end_point: transform.apply(Vec3::new(2.5, 4.5, 0.5)),
  };

  assert_eq!(
    collect_straight_path(
      &path,
      &archipelago.nav_data,
      /* start= */
      (PathIndex::from_corridor_index(0, 0), path.start_point),
      /* end= */
      (PathIndex::from_corridor_index(0, 2), path.end_point),
      /* iteration_limit= */ 3,
    ),
    [
      (
        PathIndex::from_corridor_index(0, 0),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(2.0, 3.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 1),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(2.0, 4.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(2.5, 4.5, 0.5)))
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
    height_mesh: None,
  }
  .validate()
  .expect("Mesh is valid.");

  let transform =
    Transform { translation: Vec2::new(-1.0, -3.0), rotation: PI * -1.8 };
  let mut archipelago =
    Archipelago::<XY>::new(ArchipelagoOptions::from_agent_radius(0.5));
  let island_id = archipelago.add_island(transform.clone(), nav_mesh);
  let transform = transform.to_core();

  let path = Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14],
      portal_edge_index: vec![2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1],
    }],
    off_mesh_link_segments: vec![],
    start_point: transform.apply(Vec3::new(0.5, 0.5, 0.0)),
    end_point: transform.apply(Vec3::new(-3.5, 14.0, 0.0)),
  };

  assert_eq!(
    collect_straight_path(
      &path,
      &archipelago.nav_data,
      /* start= */
      (PathIndex::from_corridor_index(0, 0), path.start_point),
      /* end= */
      (PathIndex::from_corridor_index(0, 14), path.end_point),
      /* iteration_limit= */ 5,
    ),
    [
      (
        PathIndex::from_corridor_index(0, 4),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(1.0, 4.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 8),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(4.0, 5.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 11),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(4.0, 7.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 13),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(-3.0, 8.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 14),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(-3.5, 14.0, 0.0)))
      ),
    ]
  );
}

fn off_mesh_link_for_animation_link<CS: CoordinateSystem>(
  archipelago: &Archipelago<CS>,
  animation_link_id: AnimationLinkId,
) -> OffMeshLinkId {
  for (link_id, link) in archipelago.nav_data.off_mesh_links.iter() {
    let KindedOffMeshLink::AnimationLink { animation_link, .. } = &link.kinded
    else {
      continue;
    };
    if *animation_link == animation_link_id {
      return link_id;
    }
  }
  panic!("No corresponding off mesh link found for {animation_link_id:?}")
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
    height_mesh: None,
  }
  .validate()
  .expect("Mesh is valid.");

  let mut archipelago =
    Archipelago::<XYZ>::new(ArchipelagoOptions::from_agent_radius(0.5));
  let island_id = archipelago
    .add_island(Transform { translation: Vec3::ZERO, rotation: 0.0 }, nav_mesh);

  let path = Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![0, 1],
      portal_edge_index: vec![2],
    }],
    off_mesh_link_segments: vec![],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
  };

  assert_eq!(
    path.find_next_point_in_straight_path(
      &archipelago.nav_data,
      /* start_index= */ PathIndex::from_corridor_index(0, 1),
      /* start_point= */ Vec3::new(0.25, 1.1, 0.0),
      /* end_index= */ PathIndex::from_corridor_index(0, 1),
      /* end_point= */ Vec3::new(0.75, 1.9, 0.0),
    ),
    (
      PathIndex::from_corridor_index(0, 1),
      StraightPathStep::Waypoint(Vec3::new(0.75, 1.9, 0.0))
    )
  );
}

#[googletest::test]
fn straight_path_includes_animation_link() {
  // +-+
  // |E|
  // +-+
  // |X|
  // +-+
  //  L
  // +-+
  // |X|
  // +-+
  // |X|
  // +-+
  // |S|
  // +-+
  let nav_mesh = NavigationMesh {
    vertices: vec![
      // First chunk of the mesh
      Vec2::new(0.0, 0.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(1.0, 1.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(1.0, 2.0),
      Vec2::new(0.0, 2.0),
      Vec2::new(1.0, 3.0),
      Vec2::new(0.0, 3.0),
      // Second disconnected chunk of the mesh.
      Vec2::new(0.0, 5.0),
      Vec2::new(1.0, 5.0),
      Vec2::new(1.0, 6.0),
      Vec2::new(0.0, 6.0),
      Vec2::new(1.0, 7.0),
      Vec2::new(0.0, 7.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3],
      vec![3, 2, 4, 5],
      vec![5, 4, 6, 7],
      vec![8, 9, 10, 11],
      vec![11, 10, 12, 13],
    ],
    polygon_type_indices: vec![0; 5],
    height_mesh: None,
  }
  .validate()
  .expect("Mesh is valid.");

  let mut archipelago =
    Archipelago::<XY>::new(ArchipelagoOptions::from_agent_radius(0.5));
  let island_id = archipelago
    .add_island(Transform { translation: Vec2::ZERO, rotation: 0.0 }, nav_mesh);

  let animation_link_id = archipelago.add_animation_link(AnimationLink {
    start_edge: (Vec2::new(0.0, 3.0), Vec2::new(1.0, 3.0)),
    end_edge: (Vec2::new(0.0, 5.0), Vec2::new(1.0, 5.0)),
    cost: 1.0,
    kind: 0,
    bidirectional: false,
  });

  archipelago.update(1.0);

  let off_mesh_link =
    off_mesh_link_for_animation_link(&archipelago, animation_link_id);

  let path = Path {
    island_segments: vec![
      IslandSegment {
        island_id,
        corridor: vec![0, 1, 2],
        portal_edge_index: vec![2, 2],
      },
      IslandSegment {
        island_id,
        corridor: vec![3, 4],
        portal_edge_index: vec![2],
      },
    ],
    off_mesh_link_segments: vec![OffMeshLinkSegment {
      off_mesh_link,
      starting_node: NodeRef { island_id, polygon_index: 2 },
      end_node: NodeRef { island_id, polygon_index: 3 },
    }],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
  };

  expect_that!(
    collect_straight_path(
      &path,
      &archipelago.nav_data,
      (PathIndex::from_corridor_index(0, 0), Vec3::new(0.1, 0.1, 0.0)),
      (PathIndex::from_corridor_index(1, 1), Vec3::new(0.9, 6.9, 0.0)),
      2,
    ),
    elements_are!(
      &(
        PathIndex::from_corridor_index(1, 0),
        StraightPathStep::AnimationLink {
          start_point: Vec3::new(0.1, 3.0, 0.0),
          end_point: Vec3::new(0.1, 5.0, 0.0),
          link_id: animation_link_id,
          start_node: NodeRef { island_id, polygon_index: 2 },
          end_node: NodeRef { island_id, polygon_index: 3 },
        }
      ),
      &(
        PathIndex::from_corridor_index(1, 1),
        StraightPathStep::Waypoint(Vec3::new(0.9, 6.9, 0.0)),
      )
    ),
  );
}

#[googletest::test]
fn multiple_animation_links_in_a_row() {
  // +-+
  // |E|
  // +-+
  // |X|
  // +-+
  //  L
  // +-+
  // |X|
  // +-+
  //  L
  // +-+
  // |X|
  // +-+
  //  L
  // +-+
  // |S|
  // +-+
  let nav_mesh = NavigationMesh {
    vertices: vec![
      // Step 1
      Vec2::new(0.0, 0.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(1.0, 1.0),
      Vec2::new(0.0, 1.0),
      // Step 2
      Vec2::new(0.0, 2.0),
      Vec2::new(1.0, 2.0),
      Vec2::new(1.0, 3.0),
      Vec2::new(0.0, 3.0),
      // Step 3
      Vec2::new(0.0, 4.0),
      Vec2::new(1.0, 4.0),
      Vec2::new(1.0, 5.0),
      Vec2::new(0.0, 5.0),
      // Step 4
      Vec2::new(0.0, 6.0),
      Vec2::new(1.0, 6.0),
      Vec2::new(1.0, 7.0),
      Vec2::new(0.0, 7.0),
      Vec2::new(1.0, 8.0),
      Vec2::new(0.0, 8.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3],
      vec![4, 5, 6, 7],
      vec![8, 9, 10, 11],
      vec![12, 13, 14, 15],
      vec![15, 14, 16, 17],
    ],
    polygon_type_indices: vec![0; 5],
    height_mesh: None,
  }
  .validate()
  .expect("Mesh is valid.");

  let mut archipelago =
    Archipelago::<XY>::new(ArchipelagoOptions::from_agent_radius(0.5));
  let island_id = archipelago
    .add_island(Transform { translation: Vec2::ZERO, rotation: 0.0 }, nav_mesh);

  let link_id_1 = archipelago.add_animation_link(AnimationLink {
    start_edge: (Vec2::new(0.0, 1.0), Vec2::new(1.0, 1.0)),
    end_edge: (Vec2::new(0.0, 2.0), Vec2::new(1.0, 2.0)),
    cost: 1.0,
    kind: 0,
    bidirectional: false,
  });
  let link_id_2 = archipelago.add_animation_link(AnimationLink {
    start_edge: (Vec2::new(0.0, 3.0), Vec2::new(1.0, 3.0)),
    end_edge: (Vec2::new(0.0, 4.0), Vec2::new(1.0, 4.0)),
    cost: 1.0,
    kind: 0,
    bidirectional: false,
  });
  let link_id_3 = archipelago.add_animation_link(AnimationLink {
    start_edge: (Vec2::new(0.0, 5.0), Vec2::new(1.0, 5.0)),
    end_edge: (Vec2::new(0.0, 6.0), Vec2::new(1.0, 6.0)),
    cost: 1.0,
    kind: 0,
    bidirectional: false,
  });

  archipelago.update(1.0);

  let off_mesh_link_1 =
    off_mesh_link_for_animation_link(&archipelago, link_id_1);
  let off_mesh_link_2 =
    off_mesh_link_for_animation_link(&archipelago, link_id_2);
  let off_mesh_link_3 =
    off_mesh_link_for_animation_link(&archipelago, link_id_3);

  let path = Path {
    island_segments: vec![
      IslandSegment { island_id, corridor: vec![0], portal_edge_index: vec![] },
      IslandSegment { island_id, corridor: vec![1], portal_edge_index: vec![] },
      IslandSegment { island_id, corridor: vec![2], portal_edge_index: vec![] },
      IslandSegment {
        island_id,
        corridor: vec![3, 4],
        portal_edge_index: vec![2],
      },
    ],
    off_mesh_link_segments: vec![
      OffMeshLinkSegment {
        off_mesh_link: off_mesh_link_1,
        starting_node: NodeRef { island_id, polygon_index: 0 },
        end_node: NodeRef { island_id, polygon_index: 1 },
      },
      OffMeshLinkSegment {
        off_mesh_link: off_mesh_link_2,
        starting_node: NodeRef { island_id, polygon_index: 1 },
        end_node: NodeRef { island_id, polygon_index: 2 },
      },
      OffMeshLinkSegment {
        off_mesh_link: off_mesh_link_3,
        starting_node: NodeRef { island_id, polygon_index: 2 },
        end_node: NodeRef { island_id, polygon_index: 3 },
      },
    ],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
  };

  expect_that!(
    collect_straight_path(
      &path,
      &archipelago.nav_data,
      (PathIndex::from_corridor_index(0, 0), Vec3::new(0.5, 0.5, 0.0)),
      (PathIndex::from_corridor_index(3, 1), Vec3::new(0.5, 7.5, 0.0)),
      10,
    ),
    elements_are!(
      &(
        PathIndex::from_corridor_index(1, 0),
        StraightPathStep::AnimationLink {
          start_point: Vec3::new(0.5, 1.0, 0.0),
          end_point: Vec3::new(0.5, 2.0, 0.0),
          link_id: link_id_1,
          start_node: NodeRef { island_id, polygon_index: 0 },
          end_node: NodeRef { island_id, polygon_index: 1 },
        }
      ),
      &(
        PathIndex::from_corridor_index(2, 0),
        StraightPathStep::AnimationLink {
          start_point: Vec3::new(0.5, 3.0, 0.0),
          end_point: Vec3::new(0.5, 4.0, 0.0),
          link_id: link_id_2,
          start_node: NodeRef { island_id, polygon_index: 1 },
          end_node: NodeRef { island_id, polygon_index: 2 },
        }
      ),
      &(
        PathIndex::from_corridor_index(3, 0),
        StraightPathStep::AnimationLink {
          start_point: Vec3::new(0.5, 5.0, 0.0),
          end_point: Vec3::new(0.5, 6.0, 0.0),
          link_id: link_id_3,
          start_node: NodeRef { island_id, polygon_index: 2 },
          end_node: NodeRef { island_id, polygon_index: 3 },
        }
      ),
      &(
        PathIndex::from_corridor_index(3, 1),
        StraightPathStep::Waypoint(Vec3::new(0.5, 7.5, 0.0)),
      )
    ),
  );
}

#[googletest::test]
fn obscured_animation_link_is_made_visible_by_straight_path() {
  // The animation link can be obscured by geometry. Projecting directly onto
  // the animation link can result in invalid paths.
  //
  // +--------+
  // |EXXXXXXX|
  // +--------+
  // |XXXXXXXX|
  // +--------+
  //  LLLLLLLL
  // +--------+
  // |XXXXXXX/
  // |XXXXXX/
  // +-----+-+
  // |XXXXXXS|
  // +-------+
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec2::new(0.0, 0.0),
      Vec2::new(7.0, 0.0),
      Vec2::new(7.0, 1.0),
      Vec2::new(6.0, 1.0),
      Vec2::new(8.0, 3.0),
      Vec2::new(0.0, 3.0),
      Vec2::new(0.0, 1.0),
      // Other chunk.
      Vec2::new(0.0, 5.0),
      Vec2::new(8.0, 5.0),
      Vec2::new(8.0, 6.0),
      Vec2::new(0.0, 6.0),
      Vec2::new(8.0, 7.0),
      Vec2::new(0.0, 7.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3, 6],
      vec![3, 4, 5, 6],
      vec![7, 8, 9, 10],
      vec![10, 9, 11, 12],
    ],
    polygon_type_indices: vec![0; 4],
    height_mesh: None,
  }
  .validate()
  .expect("Mesh is valid.");

  let mut archipelago =
    Archipelago::<XY>::new(ArchipelagoOptions::from_agent_radius(0.5));
  let island_id = archipelago
    .add_island(Transform { translation: Vec2::ZERO, rotation: 0.0 }, nav_mesh);

  let link_id = archipelago.add_animation_link(AnimationLink {
    start_edge: (Vec2::new(0.0, 3.0), Vec2::new(8.0, 3.0)),
    end_edge: (Vec2::new(0.0, 5.0), Vec2::new(8.0, 5.0)),
    cost: 1.0,
    kind: 0,
    bidirectional: false,
  });

  archipelago.update(1.0);

  let off_mesh_link = off_mesh_link_for_animation_link(&archipelago, link_id);

  let path = Path {
    island_segments: vec![
      IslandSegment {
        island_id,
        corridor: vec![0, 1],
        portal_edge_index: vec![3],
      },
      IslandSegment {
        island_id,
        corridor: vec![2, 3],
        portal_edge_index: vec![2],
      },
    ],
    off_mesh_link_segments: vec![OffMeshLinkSegment {
      off_mesh_link,
      starting_node: NodeRef { island_id, polygon_index: 1 },
      end_node: NodeRef { island_id, polygon_index: 2 },
    }],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
  };

  expect_that!(
    collect_straight_path(
      &path,
      &archipelago.nav_data,
      (PathIndex::from_corridor_index(0, 0), Vec3::new(6.5, 0.5, 0.0)),
      (PathIndex::from_corridor_index(1, 1), Vec3::new(0.1, 6.9, 0.0)),
      10,
    ),
    elements_are!(
      &(
        PathIndex::from_corridor_index(0, 0),
        StraightPathStep::Waypoint(Vec3::new(6.0, 1.0, 0.0)),
      ),
      &(
        PathIndex::from_corridor_index(1, 0),
        StraightPathStep::AnimationLink {
          start_point: Vec3::new(6.0, 3.0, 0.0),
          end_point: Vec3::new(6.0, 5.0, 0.0),
          link_id,
          start_node: NodeRef { island_id, polygon_index: 1 },
          end_node: NodeRef { island_id, polygon_index: 2 },
        }
      ),
      &(
        PathIndex::from_corridor_index(1, 1),
        StraightPathStep::Waypoint(Vec3::new(0.1, 6.9, 0.0)),
      )
    ),
  );
}

#[googletest::test]
fn end_point_after_animation_link_is_reported() {
  // We want to make sure that if the end point comes right after an animation
  // link, that is reported. We also check for the case where the end point
  // isn't at the end of the path.
  //
  // +-+
  // |X|
  // +-+
  //  L
  // +-+
  // |E|
  // +-+
  //  L
  // +-+
  // |S|
  // +-+
  let nav_mesh = NavigationMesh {
    vertices: vec![
      // Step 1
      Vec2::new(0.0, 0.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(1.0, 1.0),
      Vec2::new(0.0, 1.0),
      // Step 2
      Vec2::new(0.0, 2.0),
      Vec2::new(1.0, 2.0),
      Vec2::new(1.0, 3.0),
      Vec2::new(0.0, 3.0),
      // Step 3
      Vec2::new(0.0, 4.0),
      Vec2::new(1.0, 4.0),
      Vec2::new(1.0, 5.0),
      Vec2::new(0.0, 5.0),
    ],
    polygons: vec![vec![0, 1, 2, 3], vec![4, 5, 6, 7], vec![8, 9, 10, 11]],
    polygon_type_indices: vec![0; 3],
    height_mesh: None,
  }
  .validate()
  .expect("Mesh is valid.");

  let mut archipelago =
    Archipelago::<XY>::new(ArchipelagoOptions::from_agent_radius(0.5));
  let island_id = archipelago
    .add_island(Transform { translation: Vec2::ZERO, rotation: 0.0 }, nav_mesh);

  let animation_link_1 = archipelago.add_animation_link(AnimationLink {
    start_edge: (Vec2::new(0.0, 1.0), Vec2::new(1.0, 1.0)),
    end_edge: (Vec2::new(0.0, 2.0), Vec2::new(1.0, 2.0)),
    cost: 1.0,
    kind: 0,
    bidirectional: false,
  });
  let animation_link_2 = archipelago.add_animation_link(AnimationLink {
    start_edge: (Vec2::new(0.0, 3.0), Vec2::new(1.0, 3.0)),
    end_edge: (Vec2::new(0.0, 4.0), Vec2::new(1.0, 4.0)),
    cost: 1.0,
    kind: 0,
    bidirectional: false,
  });

  archipelago.update(1.0);

  let off_mesh_link_1 =
    off_mesh_link_for_animation_link(&archipelago, animation_link_1);
  let off_mesh_link_2 =
    off_mesh_link_for_animation_link(&archipelago, animation_link_2);

  let path = Path {
    island_segments: vec![
      IslandSegment { island_id, corridor: vec![0], portal_edge_index: vec![] },
      IslandSegment { island_id, corridor: vec![1], portal_edge_index: vec![] },
      IslandSegment { island_id, corridor: vec![2], portal_edge_index: vec![] },
    ],
    off_mesh_link_segments: vec![
      OffMeshLinkSegment {
        off_mesh_link: off_mesh_link_1,
        starting_node: NodeRef { island_id, polygon_index: 0 },
        end_node: NodeRef { island_id, polygon_index: 1 },
      },
      OffMeshLinkSegment {
        off_mesh_link: off_mesh_link_2,
        starting_node: NodeRef { island_id, polygon_index: 1 },
        end_node: NodeRef { island_id, polygon_index: 2 },
      },
    ],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
  };

  expect_that!(
    collect_straight_path(
      &path,
      &archipelago.nav_data,
      (PathIndex::from_corridor_index(0, 0), Vec3::new(0.5, 0.5, 0.0)),
      // We intentionally don't make the end index be the end of the path. We
      // want to test what happens when the end index is on the animation link.
      (PathIndex::from_corridor_index(1, 0), Vec3::new(0.5, 2.5, 0.0)),
      10,
    ),
    elements_are!(
      &(
        PathIndex::from_corridor_index(1, 0),
        StraightPathStep::AnimationLink {
          start_point: Vec3::new(0.5, 1.0, 0.0),
          end_point: Vec3::new(0.5, 2.0, 0.0),
          link_id: animation_link_1,
          start_node: NodeRef { island_id, polygon_index: 0 },
          end_node: NodeRef { island_id, polygon_index: 1 },
        }
      ),
      &(
        PathIndex::from_corridor_index(1, 0),
        StraightPathStep::Waypoint(Vec3::new(0.5, 2.5, 0.0)),
      )
    ),
  );
}

#[test]
fn path_not_valid_for_invalidated_islands_or_off_mesh_links() {
  // Create unused slotmaps just to get `IslandId`s and `OffMeshLinkId`s.
  let mut slotmap = HopSlotMap::<IslandId, _>::with_key();
  let island_id_1 = slotmap.insert(0);
  let island_id_2 = slotmap.insert(0);
  let island_id_3 = slotmap.insert(0);
  let mut slotmap = HopSlotMap::<OffMeshLinkId, _>::with_key();
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
    off_mesh_link_segments: vec![
      OffMeshLinkSegment {
        starting_node: NodeRef { island_id: island_id_1, polygon_index: 0 },
        end_node: NodeRef { island_id: island_id_2, polygon_index: 0 },
        off_mesh_link: boundary_link_id_1,
      },
      OffMeshLinkSegment {
        starting_node: NodeRef { island_id: island_id_2, polygon_index: 1 },
        end_node: NodeRef { island_id: island_id_3, polygon_index: 0 },
        off_mesh_link: boundary_link_id_2,
      },
    ],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
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
  let mut slotmap = HopSlotMap::<OffMeshLinkId, _>::with_key();
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
    off_mesh_link_segments: vec![
      OffMeshLinkSegment {
        starting_node: NodeRef { island_id: island_id_1, polygon_index: 0 },
        end_node: NodeRef { island_id: island_id_2, polygon_index: 2 },
        off_mesh_link: boundary_link_id_1,
      },
      OffMeshLinkSegment {
        starting_node: NodeRef { island_id: island_id_2, polygon_index: 1 },
        end_node: NodeRef { island_id: island_id_3, polygon_index: 0 },
        off_mesh_link: boundary_link_id_2,
      },
    ],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
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
  let mut slotmap = HopSlotMap::<OffMeshLinkId, _>::with_key();
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
    off_mesh_link_segments: vec![
      OffMeshLinkSegment {
        starting_node: NodeRef { island_id: island_id_1, polygon_index: 3 },
        end_node: NodeRef { island_id: island_id_2, polygon_index: 2 },
        off_mesh_link: boundary_link_id_1,
      },
      OffMeshLinkSegment {
        starting_node: NodeRef { island_id: island_id_2, polygon_index: 1 },
        end_node: NodeRef { island_id: island_id_3, polygon_index: 0 },
        off_mesh_link: boundary_link_id_2,
      },
    ],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
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
