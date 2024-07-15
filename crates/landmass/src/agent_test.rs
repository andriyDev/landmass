use std::{
  collections::{HashMap, HashSet},
  f32::consts::PI,
  sync::Arc,
};

use glam::Vec3;
use slotmap::HopSlotMap;

use crate::{
  agent::{does_agent_need_repath, RepathResult},
  coords::XYZ,
  nav_data::NodeRef,
  path::{IslandSegment, Path, PathIndex},
  Agent, Archipelago, IslandId, NavigationMesh, TargetReachedCondition,
  Transform,
};

#[test]
fn has_reached_target_at_end_node() {
  let nav_mesh = NavigationMesh {
    vertices: vec![],
    polygons: vec![],
    polygon_type_indices: vec![],
  }
  .validate()
  .expect("nav mesh is valid");
  let transform =
    Transform { translation: Vec3::new(2.0, 3.0, 4.0), rotation: PI * 0.85 };
  let mut archipelago = Archipelago::<XYZ>::new();
  let island_id = archipelago
    .add_island()
    .set_nav_mesh(transform.clone(), Arc::new(nav_mesh), HashMap::new())
    .id();
  let mut agent = Agent::create(
    /* position= */ transform.apply(Vec3::new(1.0, 0.0, 1.0)),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* max_velocity= */ 0.0,
  );

  let path = Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![0],
      portal_edge_index: vec![],
    }],
    boundary_link_segments: vec![],
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
      (first_path_index, transform.apply(Vec3::new(2.5, 0.0, 1.0))),
      (first_path_index, transform.apply(Vec3::new(2.5, 0.0, 1.0))),
    ));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (first_path_index, transform.apply(Vec3::new(3.5, 0.0, 1.0))),
      (first_path_index, transform.apply(Vec3::new(3.5, 0.0, 1.0))),
    ));
    assert!(agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (first_path_index, transform.apply(Vec3::new(2.0, 0.0, 2.0))),
      (first_path_index, transform.apply(Vec3::new(2.0, 0.0, 2.0))),
    ));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (first_path_index, transform.apply(Vec3::new(2.5, 0.0, 2.5))),
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
  }
  .validate()
  .expect("nav mesh is valid");

  let transform =
    Transform { translation: Vec3::new(2.0, 4.0, 3.0), rotation: PI * -0.85 };
  let mut archipelago = Archipelago::<XYZ>::new();
  let island_id = archipelago
    .add_island()
    .set_nav_mesh(transform.clone(), Arc::new(nav_mesh), HashMap::new())
    .id();

  let mut agent = Agent::create(
    /* position= */ Vec3::ZERO,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* max_velocity= */ 0.0,
  );

  let path = Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![0, 1, 2],
      portal_edge_index: vec![1, 2],
    }],
    boundary_link_segments: vec![],
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
      (
        PathIndex::from_corridor_index(0, 1),
        transform.apply(Vec3::new(1.0, 11.0, 0.0))
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
      (
        PathIndex::from_corridor_index(0, 1),
        transform.apply(Vec3::new(1.0, 11.0, 0.0))
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
      (
        PathIndex::from_corridor_index(0, 1),
        transform.apply(Vec3::new(1.0, 11.0, 0.0))
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
      (
        PathIndex::from_corridor_index(0, 1),
        transform.apply(Vec3::new(1.0, 11.0, 0.0))
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
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 1.0, 0.0))
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
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 1.0, 0.0))
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
      (
        PathIndex::from_corridor_index(0, 1),
        transform.apply(Vec3::new(1.0, 11.0, 0.0))
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
      (
        PathIndex::from_corridor_index(0, 1),
        transform.apply(Vec3::new(1.0, 11.0, 0.0))
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
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 1.0, 0.0))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 1.0, 0.0))
      ),
    ));
  }
}

#[test]
fn nothing_or_clear_path_for_no_target() {
  let mut agent = Agent::<XYZ>::create(
    /* position= */ Vec3::ZERO,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* max_velocity= */ 0.0,
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

  agent.current_path =
    Some(Path { island_segments: vec![], boundary_link_segments: vec![] });

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
    /* max_velocity= */ 0.0,
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
    /* max_velocity= */ 0.0,
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
    boundary_link_segments: vec![],
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
