use std::{collections::HashSet, f32::consts::PI};

use glam::{Vec2, Vec3};
use googletest::{expect_that, expect_true, matchers::*};
use slotmap::HopSlotMap;

use crate::{
  Agent, Archipelago, ArchipelagoOptions, CoordinateSystem, FromAgentRadius,
  Island, IslandId, NavigationMesh, NotReachedAnimationLinkError,
  ReachedAnimationLink, TargetReachedCondition, Transform,
  agent::{NotUsingAnimationLinkError, RepathResult, does_agent_need_repath},
  coords::{XY, XYZ},
  link::{AnimationLink, AnimationLinkId},
  nav_data::{KindedOffMeshLink, NodeRef, OffMeshLinkId},
  path::{
    IslandSegment, OffMeshLinkSegment, Path, PathIndex, StraightPathStep,
  },
};

#[test]
fn overrides_type_index_costs() {
  let mut agent = Agent::<XY>::create(
    /* position= */ Vec2::ZERO,
    /* velocity= */ Vec2::ZERO,
    /* radius */ 1.0,
    /* desired_speed= */ 1.0,
    /* max_speed= */ 1.0,
  );
  assert!(agent.override_type_index_cost(1, 3.0));
  assert!(agent.override_type_index_cost(2, 0.5));

  assert_eq!(
    {
      let mut vec = agent.get_type_index_cost_overrides().collect::<Vec<_>>();
      vec.sort_by_key(|&(a, _)| a);
      vec
    },
    [(1, 3.0), (2, 0.5)]
  );

  agent.override_type_index_cost(1, 5.0);

  assert_eq!(
    {
      let mut vec = agent.get_type_index_cost_overrides().collect::<Vec<_>>();
      vec.sort_by_key(|&(a, _)| a);
      vec
    },
    [(1, 5.0), (2, 0.5)]
  );
}

#[test]
fn negative_or_zero_type_index_cost_returns_false() {
  let mut agent = Agent::<XY>::create(
    /* position= */ Vec2::ZERO,
    /* velocity= */ Vec2::ZERO,
    /* radius */ 1.0,
    /* desired_speed= */ 1.0,
    /* max_speed= */ 1.0,
  );
  assert!(!agent.override_type_index_cost(0, 0.0));
  assert!(!agent.override_type_index_cost(0, -0.5));
}

fn agent_position<CS: CoordinateSystem>(agent: &Agent<CS>) -> Vec3 {
  CS::to_landmass(&agent.position)
}

#[test]
fn has_reached_target_at_end_node() {
  let nav_mesh = NavigationMesh {
    vertices: vec![],
    polygons: vec![],
    polygon_type_indices: vec![],
    height_mesh: None,
  }
  .validate()
  .expect("nav mesh is valid");
  let transform =
    Transform { translation: Vec3::new(2.0, 3.0, 4.0), rotation: PI * 0.85 };
  let mut archipelago =
    Archipelago::<XYZ>::new(ArchipelagoOptions::from_agent_radius(0.5));
  let island_id =
    archipelago.add_island(Island::new(transform.clone(), nav_mesh));
  let mut agent = Agent::create(
    /* position= */ transform.apply(Vec3::new(1.0, 0.0, 1.0)),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* desired_speed= */ 0.0,
    /* max_speed= */ 0.0,
  );

  let path = Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![0],
      portal_edge_index: vec![],
    }],
    off_mesh_link_segments: vec![],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
  };

  let first_path_index = PathIndex::from_corridor_index(0, 0);
  for condition in [
    TargetReachedCondition::Distance(Some(2.0)),
    TargetReachedCondition::StraightPathDistance(Some(2.0)),
    TargetReachedCondition::VisibleAtDistance(Some(2.0)),
  ] {
    agent.target_reached_condition = condition;

    assert!(agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      agent_position(&agent),
      (
        first_path_index,
        StraightPathStep::Waypoint(transform.apply(Vec3::new(2.5, 0.0, 1.0)))
      ),
      (first_path_index, transform.apply(Vec3::new(2.5, 0.0, 1.0))),
    ));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      agent_position(&agent),
      (
        first_path_index,
        StraightPathStep::Waypoint(transform.apply(Vec3::new(3.5, 0.0, 1.0)))
      ),
      (first_path_index, transform.apply(Vec3::new(3.5, 0.0, 1.0))),
    ));
    assert!(agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      agent_position(&agent),
      (
        first_path_index,
        StraightPathStep::Waypoint(transform.apply(Vec3::new(2.0, 0.0, 2.0)))
      ),
      (first_path_index, transform.apply(Vec3::new(2.0, 0.0, 2.0))),
    ));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      agent_position(&agent),
      (
        first_path_index,
        StraightPathStep::Waypoint(transform.apply(Vec3::new(2.5, 0.0, 2.5)))
      ),
      (first_path_index, transform.apply(Vec3::new(2.5, 0.0, 2.5))),
    ));
  }
}

#[test]
fn long_detour_reaches_target_in_different_ways() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(1.0, 11.0, 0.0),
      Vec3::new(0.0, 12.0, 0.0),
      Vec3::new(2.0, 11.0, 0.0),
      Vec3::new(3.0, 12.0, 0.0),
      Vec3::new(2.0, 1.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2], vec![2, 1, 3, 4], vec![4, 3, 5]],
    polygon_type_indices: vec![0, 0, 0],
    height_mesh: None,
  }
  .validate()
  .expect("nav mesh is valid");

  let transform =
    Transform { translation: Vec3::new(2.0, 4.0, 3.0), rotation: PI * -0.85 };
  let mut archipelago =
    Archipelago::<XYZ>::new(ArchipelagoOptions::from_agent_radius(0.5));
  let island_id =
    archipelago.add_island(Island::new(transform.clone(), nav_mesh));

  let mut agent = Agent::create(
    /* position= */ Vec3::ZERO,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* desired_speed= */ 0.0,
    /* max_speed= */ 0.0,
  );

  let path = Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![0, 1, 2],
      portal_edge_index: vec![1, 2],
    }],
    off_mesh_link_segments: vec![],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
  };

  {
    agent.position = transform.apply(Vec3::new(1.0, 1.0, 0.0));
    agent.target_reached_condition =
      TargetReachedCondition::Distance(Some(1.1));

    // Agent started within 1.1 units of the destination, so they are close
    // enough.
    assert!(agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      agent_position(&agent),
      (
        PathIndex::from_corridor_index(0, 1),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(1.0, 11.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 1.0, 0.0))
      ),
    ));

    // Agent is just outside of 1.1 units, so they still have not reached the
    // end.
    agent.position = transform.apply(Vec3::new(1.0, 2.0, 0.0));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      agent_position(&agent),
      (
        PathIndex::from_corridor_index(0, 1),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(1.0, 11.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 1.0, 0.0))
      ),
    ));
  }

  {
    agent.target_reached_condition =
      TargetReachedCondition::VisibleAtDistance(Some(15.0));

    // The agent cannot see the target and its path is still too long.
    agent.position = transform.apply(Vec3::new(1.0, 1.0, 0.0));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      agent_position(&agent),
      (
        PathIndex::from_corridor_index(0, 1),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(1.0, 11.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 1.0, 0.0))
      ),
    ));

    // The agent only has 12 units left to travel to the target, and yet the
    // agent still hasn't reached the target, since the target is not visible.
    agent.position = transform.apply(Vec3::new(1.0, 10.0, 0.0));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      agent_position(&agent),
      (
        PathIndex::from_corridor_index(0, 1),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(1.0, 11.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 1.0, 0.0))
      ),
    ));

    // The agent has now "rounded the corner", and so can see the target (and
    // is within the correct distance).
    agent.position = transform.apply(Vec3::new(2.0, 11.0, 0.0));
    assert!(agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      agent_position(&agent),
      (
        PathIndex::from_corridor_index(0, 2),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(2.0, 1.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 1.0, 0.0))
      ),
    ));

    // The agent can see the target but is still too far away.
    agent.position = transform.apply(Vec3::new(2.0, 20.0, 0.0));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      agent_position(&agent),
      (
        PathIndex::from_corridor_index(0, 2),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(2.0, 1.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 1.0, 0.0))
      ),
    ));
  }

  {
    agent.target_reached_condition =
      TargetReachedCondition::StraightPathDistance(Some(15.0));

    // The agent's path is too long (21 units).
    agent.position = transform.apply(Vec3::new(1.0, 1.0, 0.0));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      agent_position(&agent),
      (
        PathIndex::from_corridor_index(0, 1),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(1.0, 11.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 1.0, 0.0))
      ),
    ));

    // The agent only has 12 units left to travel to the target, so they have
    // reached the target.
    agent.position = transform.apply(Vec3::new(1.0, 10.0, 0.0));
    assert!(agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      agent_position(&agent),
      (
        PathIndex::from_corridor_index(0, 1),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(1.0, 11.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 1.0, 0.0))
      ),
    ));

    // The agent can see the target but is still too far away.
    agent.position = transform.apply(Vec3::new(2.0, 20.0, 0.0));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      agent_position(&agent),
      (
        PathIndex::from_corridor_index(0, 2),
        StraightPathStep::Waypoint(transform.apply(Vec3::new(2.0, 1.0, 0.0)))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 1.0, 0.0))
      ),
    ));
  }
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
fn using_animation_link_does_not_reach_target() {
  let mut archipelago =
    Archipelago::<XY>::new(ArchipelagoOptions::from_agent_radius(0.5));

  // +----+-+
  // |XXXX|X|
  // +----+-+
  //  L   |X|
  // +----+-+
  // |XXXX|X|
  // +----+-+
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec2::new(0.0, 0.0),
      Vec2::new(2.0, 0.0),
      Vec2::new(3.0, 0.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(2.0, 1.0),
      Vec2::new(3.0, 1.0),
      Vec2::new(0.0, 2.0),
      Vec2::new(2.0, 2.0),
      Vec2::new(3.0, 2.0),
      Vec2::new(0.0, 3.0),
      Vec2::new(2.0, 3.0),
      Vec2::new(3.0, 3.0),
    ],
    polygons: vec![
      vec![0, 1, 4, 3],
      vec![1, 2, 5, 4],
      vec![4, 5, 8, 7],
      vec![7, 8, 11, 10],
      vec![6, 7, 10, 9],
    ],
    polygon_type_indices: vec![0; 5],
    height_mesh: None,
  }
  .validate()
  .expect("nav mesh is valid");

  let island_id = archipelago.add_island(Island::new(
    Transform { translation: Vec2::new(10.0, 10.0), ..Default::default() },
    nav_mesh,
  ));

  let animation_link_id = archipelago.add_animation_link(AnimationLink {
    // Portals are in world space.
    start_edge: (Vec2::new(10.0, 11.0), Vec2::new(11.0, 11.0)),
    end_edge: (Vec2::new(10.0, 12.0), Vec2::new(11.0, 12.0)),
    cost: 1.0,
    kind: 0,
    bidirectional: false,
  });

  archipelago.update(1.0);

  let off_mesh_link =
    off_mesh_link_for_animation_link(&archipelago, animation_link_id);

  let mut agent = Agent::<XY>::create(
    /* position= */ Vec2::new(12.5, 10.5),
    /* velocity= */ Vec2::ZERO,
    /* radius= */ 0.0,
    /* desired_speed= */ 0.0,
    /* max_speed= */ 0.0,
  );

  let path = Path {
    island_segments: vec![
      IslandSegment {
        island_id,
        corridor: vec![1, 0],
        portal_edge_index: vec![3],
      },
      IslandSegment { island_id, corridor: vec![4], portal_edge_index: vec![] },
    ],
    off_mesh_link_segments: vec![OffMeshLinkSegment {
      off_mesh_link,
      starting_node: NodeRef { island_id, polygon_index: 0 },
      end_node: NodeRef { island_id, polygon_index: 4 },
    }],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
  };

  // Using the Distance condition should succeed even though we have an
  // animation link.
  agent.target_reached_condition = TargetReachedCondition::Distance(Some(10.0));
  assert!(agent.has_reached_target(
    &path,
    &archipelago.nav_data,
    agent_position(&agent),
    (
      PathIndex::from_corridor_index(0, 1),
      StraightPathStep::AnimationLink {
        start_point: Vec3::new(10.5, 11.0, 0.0),
        end_point: Vec3::new(10.5, 12.0, 0.0),
        link_id: animation_link_id,
        start_node: NodeRef { island_id, polygon_index: 0 },
        end_node: NodeRef { island_id, polygon_index: 4 },
      }
    ),
    (PathIndex::from_corridor_index(1, 0), Vec3::new(10.5, 12.5, 0.0)),
  ));

  // The animation link prevents reaching the target immediately for
  // VisibleAtDistance condition.
  agent.target_reached_condition =
    TargetReachedCondition::VisibleAtDistance(Some(10.0));
  assert!(!agent.has_reached_target(
    &path,
    &archipelago.nav_data,
    agent_position(&agent),
    (
      PathIndex::from_corridor_index(0, 1),
      StraightPathStep::AnimationLink {
        start_point: Vec3::new(10.5, 11.0, 0.0),
        end_point: Vec3::new(10.5, 12.0, 0.0),
        link_id: animation_link_id,
        start_node: NodeRef { island_id, polygon_index: 0 },
        end_node: NodeRef { island_id, polygon_index: 4 },
      }
    ),
    (PathIndex::from_corridor_index(1, 0), Vec3::new(10.5, 12.5, 0.0)),
  ));

  // The animation link prevents reaching the target immediately for
  // StraightPathDistance condition.
  agent.target_reached_condition =
    TargetReachedCondition::StraightPathDistance(Some(10.0));
  assert!(!agent.has_reached_target(
    &path,
    &archipelago.nav_data,
    agent_position(&agent),
    (
      PathIndex::from_corridor_index(0, 1),
      StraightPathStep::AnimationLink {
        start_point: Vec3::new(10.5, 11.0, 0.0),
        end_point: Vec3::new(10.5, 12.0, 0.0),
        link_id: animation_link_id,
        start_node: NodeRef { island_id, polygon_index: 0 },
        end_node: NodeRef { island_id, polygon_index: 4 },
      }
    ),
    (PathIndex::from_corridor_index(1, 0), Vec3::new(10.5, 12.5, 0.0)),
  ));

  // The animation link prevents reaching the target for StraightPathDistance
  // condition, even when it's not the next waypoint.
  agent.target_reached_condition =
    TargetReachedCondition::StraightPathDistance(Some(10.0));
  assert!(!agent.has_reached_target(
    &path,
    &archipelago.nav_data,
    agent_position(&agent),
    (
      PathIndex::from_corridor_index(0, 0),
      StraightPathStep::Waypoint(Vec3::new(12.5, 10.5, 0.0)),
    ),
    (PathIndex::from_corridor_index(1, 0), Vec3::new(10.5, 12.5, 0.0)),
  ));
}

#[googletest::test]
fn uses_sampled_point_for_reaching_target() {
  let nav_mesh = NavigationMesh {
    vertices: vec![],
    polygons: vec![],
    polygon_type_indices: vec![],
    height_mesh: None,
  }
  .validate()
  .expect("nav mesh is valid");
  let mut archipelago =
    Archipelago::<XYZ>::new(ArchipelagoOptions::from_agent_radius(0.5));
  let island_id =
    archipelago.add_island(Island::new(Transform::default(), nav_mesh));
  let mut agent = Agent::create(
    /* position= */ Vec3::new(1.0, 1.0, 1.0),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* desired_speed= */ 0.0,
    /* max_speed= */ 0.0,
  );

  let path = Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![0],
      portal_edge_index: vec![],
    }],
    off_mesh_link_segments: vec![],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
  };

  agent.target_reached_condition = TargetReachedCondition::Distance(Some(0.5));

  expect_true!(agent.has_reached_target(
    &path,
    &archipelago.nav_data,
    Vec3::new(1.0, 1.0, 0.25),
    (
      PathIndex::from_corridor_index(0, 0),
      StraightPathStep::Waypoint(Vec3::new(1.0, 1.0, 0.0))
    ),
    (PathIndex::from_corridor_index(0, 0), Vec3::new(1.0, 1.0, 0.0)),
  ))
}

#[test]
fn nothing_or_clear_path_for_no_target() {
  let mut agent = Agent::<XYZ>::create(
    /* position= */ Vec3::ZERO,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* desired_speed= */ 0.0,
    /* max_speed= */ 0.0,
  );

  assert_eq!(
    does_agent_need_repath(
      &agent,
      None,
      None,
      &HashSet::new(),
      &HashSet::new()
    ),
    RepathResult::DoNothing
  );

  agent.current_path = Some(Path {
    island_segments: vec![],
    off_mesh_link_segments: vec![],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
  });

  assert_eq!(
    does_agent_need_repath(
      &agent,
      None,
      None,
      &HashSet::new(),
      &HashSet::new()
    ),
    RepathResult::ClearPathNoTarget,
  );
}

#[test]
fn clears_path_for_missing_nodes() {
  let mut agent = Agent::<XYZ>::create(
    /* position= */ Vec3::ZERO,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* desired_speed= */ 0.0,
    /* max_speed= */ 0.0,
  );
  agent.current_target = Some(Vec3::ZERO);

  // Create an unused slotmap just to get `IslandId`s.
  let mut slotmap = HopSlotMap::<IslandId, _>::with_key();
  let island_id = slotmap.insert(0);

  assert_eq!(
    does_agent_need_repath(
      &agent,
      None,
      Some(NodeRef { island_id, polygon_index: 0 }),
      &HashSet::new(),
      &HashSet::new(),
    ),
    RepathResult::ClearPathBadAgent,
  );

  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id, polygon_index: 0 }),
      None,
      &HashSet::new(),
      &HashSet::new(),
    ),
    RepathResult::ClearPathBadTarget,
  );
}

#[test]
fn repaths_for_invalid_path_or_nodes_off_path() {
  let mut agent = Agent::<XYZ>::create(
    /* position= */ Vec3::ZERO,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* desired_speed= */ 0.0,
    /* max_speed= */ 0.0,
  );
  agent.current_target = Some(Vec3::ZERO);

  // Create an unused slotmap just to get `IslandId`s.
  let mut slotmap = HopSlotMap::<IslandId, _>::with_key();
  let island_id = slotmap.insert(0);
  let missing_island_id = slotmap.insert(0);

  // No path.
  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id, polygon_index: 1 }),
      Some(NodeRef { island_id, polygon_index: 3 }),
      &HashSet::new(),
      &HashSet::new(),
    ),
    RepathResult::NeedsRepath,
  );

  agent.current_path = Some(Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![2, 3, 4, 1, 0],
      portal_edge_index: vec![],
    }],
    off_mesh_link_segments: vec![],
    start_point: Vec3::ZERO,
    end_point: Vec3::ZERO,
  });

  // Invalidated island.
  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id, polygon_index: 3 }),
      Some(NodeRef { island_id, polygon_index: 1 }),
      &HashSet::new(),
      &HashSet::from([island_id]),
    ),
    RepathResult::NeedsRepath
  );

  // Missing agent node in path.
  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id, polygon_index: 5 }),
      Some(NodeRef { island_id, polygon_index: 1 }),
      &HashSet::new(),
      &HashSet::new(),
    ),
    RepathResult::NeedsRepath,
  );

  // Missing target node.
  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id, polygon_index: 3 }),
      Some(NodeRef { island_id, polygon_index: 6 }),
      &HashSet::new(),
      &HashSet::new(),
    ),
    RepathResult::NeedsRepath,
  );

  // Agent and target are in the wrong order.
  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id, polygon_index: 1 }),
      Some(NodeRef { island_id, polygon_index: 3 }),
      &HashSet::new(),
      &HashSet::new(),
    ),
    RepathResult::NeedsRepath,
  );

  // Following is now fine.
  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id, polygon_index: 3 }),
      Some(NodeRef { island_id, polygon_index: 1 }),
      &HashSet::new(),
      // This island is not involved in the path, so the path is still valid.
      &HashSet::from([missing_island_id]),
    ),
    RepathResult::FollowPath(
      PathIndex::from_corridor_index(0, 1),
      PathIndex::from_corridor_index(0, 3)
    ),
  );
}

#[googletest::test]
fn cannot_start_or_end_using_animation_link_without_reached_animation_link() {
  let mut agent = Agent::<XY>::create(Vec2::ZERO, Vec2::ZERO, 0.5, 1.0, 2.0);
  expect_that!(
    agent.start_animation_link(),
    err(matches_pattern!(NotReachedAnimationLinkError))
  );
  expect_that!(
    agent.end_animation_link(),
    err(matches_pattern!(NotUsingAnimationLinkError))
  );

  agent.current_animation_link = Some(ReachedAnimationLink {
    start_point: Vec2::new(0.0, 0.0),
    end_point: Vec2::new(1.0, 0.0),
    link_id: AnimationLinkId::default(),
  });

  expect_that!(
    agent.end_animation_link(),
    err(matches_pattern!(NotUsingAnimationLinkError))
  );
  expect_that!(agent.start_animation_link(), ok(()));
  expect_that!(agent.end_animation_link(), ok(()));
}
