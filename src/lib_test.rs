use std::sync::Arc;

use glam::Vec3;

use crate::{
  does_agent_need_repath,
  island::Island,
  nav_data::{NavigationData, NodeRef},
  path::Path,
  Agent, AgentId, AgentState, Archipelago, BoundingBox, IslandId,
  NavigationMesh, RepathResult, Transform, ValidNavigationMesh,
};

#[test]
fn nothing_or_clear_path_for_no_target() {
  let mut agent = Agent::create(
    /* position= */ Vec3::ZERO,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* max_velocity= */ 0.0,
  );

  let nav_data = NavigationData::new();

  assert_eq!(
    does_agent_need_repath(&agent, None, None, &nav_data),
    RepathResult::DoNothing
  );

  agent.current_path =
    Some(Path { corridor: vec![], portal_edge_index: vec![] });

  assert_eq!(
    does_agent_need_repath(&agent, None, None, &nav_data),
    RepathResult::ClearPathNoTarget,
  );
}

#[test]
fn clears_path_for_missing_nodes() {
  let mut agent = Agent::create(
    /* position= */ Vec3::ZERO,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* max_velocity= */ 0.0,
  );
  agent.current_target = Some(Vec3::ZERO);

  let nav_data = NavigationData::new();

  assert_eq!(
    does_agent_need_repath(
      &agent,
      None,
      Some(NodeRef { island_id: 0, polygon_index: 0 }),
      &nav_data,
    ),
    RepathResult::ClearPathBadAgent,
  );

  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id: 0, polygon_index: 0 }),
      None,
      &nav_data,
    ),
    RepathResult::ClearPathBadTarget,
  );
}

#[test]
fn repaths_for_invalid_path_or_nodes_off_path() {
  let mut agent = Agent::create(
    /* position= */ Vec3::ZERO,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* max_velocity= */ 0.0,
  );
  agent.current_target = Some(Vec3::ZERO);

  let mut nav_data = NavigationData::new();

  // No path.
  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id: 0, polygon_index: 1 }),
      Some(NodeRef { island_id: 0, polygon_index: 3 }),
      &nav_data,
    ),
    RepathResult::NeedsRepath,
  );

  agent.current_path = Some(Path {
    corridor: vec![
      NodeRef { island_id: 0, polygon_index: 2 },
      NodeRef { island_id: 0, polygon_index: 3 },
      NodeRef { island_id: 0, polygon_index: 4 },
      NodeRef { island_id: 0, polygon_index: 1 },
      NodeRef { island_id: 0, polygon_index: 0 },
    ],
    portal_edge_index: vec![],
  });

  // Missing island.
  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id: 0, polygon_index: 3 }),
      Some(NodeRef { island_id: 0, polygon_index: 1 }),
      &nav_data,
    ),
    RepathResult::NeedsRepath
  );

  nav_data.islands.insert(0, {
    let mut island = Island::new();
    island.set_nav_mesh(
      Transform::default(),
      Arc::new(
        NavigationMesh {
          mesh_bounds: None,
          vertices: vec![],
          polygons: vec![],
        }
        .validate()
        .expect("is valid"),
      ),
    );
    island
  });

  // Dirty island.
  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id: 0, polygon_index: 3 }),
      Some(NodeRef { island_id: 0, polygon_index: 1 }),
      &nav_data,
    ),
    RepathResult::NeedsRepath
  );

  nav_data.islands.get_mut(&0).unwrap().dirty = false;

  // Missing agent node.
  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id: 0, polygon_index: 5 }),
      Some(NodeRef { island_id: 0, polygon_index: 1 }),
      &nav_data,
    ),
    RepathResult::NeedsRepath,
  );

  // Missing target node.
  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id: 0, polygon_index: 3 }),
      Some(NodeRef { island_id: 0, polygon_index: 6 }),
      &nav_data,
    ),
    RepathResult::NeedsRepath,
  );

  // Agent and target are in the wrong order.
  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id: 0, polygon_index: 1 }),
      Some(NodeRef { island_id: 0, polygon_index: 3 }),
      &nav_data,
    ),
    RepathResult::NeedsRepath,
  );

  // Following is now fine.
  assert_eq!(
    does_agent_need_repath(
      &agent,
      Some(NodeRef { island_id: 0, polygon_index: 3 }),
      Some(NodeRef { island_id: 0, polygon_index: 1 }),
      &nav_data,
    ),
    RepathResult::FollowPath(1, 3),
  );
}

#[test]
fn add_and_remove_agents() {
  let mut archipelago = Archipelago::new();

  let agent_1 = archipelago.add_agent(Agent::create(
    /* position= */ Vec3::ZERO,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 1.0,
    /* max_velocity= */ 0.0,
  ));

  let agent_2 = archipelago.add_agent(Agent::create(
    /* position= */ Vec3::ZERO,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 2.0,
    /* max_velocity= */ 0.0,
  ));

  let agent_3 = archipelago.add_agent(Agent::create(
    /* position= */ Vec3::ZERO,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 3.0,
    /* max_velocity= */ 0.0,
  ));

  fn sorted(mut v: Vec<AgentId>) -> Vec<AgentId> {
    v.sort();
    v
  }

  assert_eq!(
    sorted(archipelago.get_agent_ids().collect::<Vec<_>>()),
    sorted(vec![agent_1, agent_2, agent_3]),
  );
  assert_eq!(
    [
      archipelago.get_agent(agent_1).radius,
      archipelago.get_agent(agent_2).radius,
      archipelago.get_agent(agent_3).radius,
    ],
    [1.0, 2.0, 3.0],
  );

  archipelago.remove_agent(agent_2);

  assert_eq!(
    sorted(archipelago.get_agent_ids().collect::<Vec<_>>()),
    sorted(vec![agent_1, agent_3]),
  );
  assert_eq!(
    [
      archipelago.get_agent(agent_1).radius,
      archipelago.get_agent(agent_3).radius,
    ],
    [1.0, 3.0],
  );

  archipelago.remove_agent(agent_3);

  assert_eq!(
    sorted(archipelago.get_agent_ids().collect::<Vec<_>>()),
    sorted(vec![agent_1]),
  );
  assert_eq!([archipelago.get_agent(agent_1).radius], [1.0]);

  archipelago.remove_agent(agent_1);

  assert_eq!(archipelago.get_agent_ids().collect::<Vec<_>>(), []);
}

#[test]
fn computes_and_follows_path() {
  let mut archipelago = Archipelago::new();
  let nav_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(1.0, 1.0, 1.0),
      Vec3::new(2.0, 1.0, 1.0),
      Vec3::new(3.0, 1.0, 1.0),
      Vec3::new(4.0, 1.0, 1.0),
      Vec3::new(4.0, 1.0, 2.0),
      Vec3::new(4.0, 1.0, 3.0),
      Vec3::new(4.0, 1.0, 4.0),
      Vec3::new(3.0, 1.0, 4.0),
      Vec3::new(3.0, 1.0, 3.0),
      Vec3::new(3.0, 1.0, 2.0),
      Vec3::new(2.0, 1.0, 2.0),
      Vec3::new(1.0, 1.0, 2.0),
    ],
    polygons: vec![
      vec![0, 1, 10, 11],
      vec![1, 2, 9, 10],
      vec![2, 3, 4, 9],
      vec![4, 5, 8, 9],
      vec![5, 6, 7, 8],
    ],
  }
  .validate()
  .expect("is valid");

  let island_id = archipelago.add_island();
  archipelago.get_island_mut(island_id).set_nav_mesh(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::new(nav_mesh),
  );

  archipelago.agent_options.neighbourhood = 0.0;
  archipelago.agent_options.obstacle_avoidance_time_horizon = 0.01;
  archipelago.agent_options.obstacle_avoidance_margin = 0.0;

  let agent_1 = archipelago.add_agent(Agent::create(
    /* position= */ Vec3::new(1.5, 1.09, 1.5),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.5,
    /* max_velocity= */ 2.0,
  ));
  let agent_2 = archipelago.add_agent(Agent::create(
    /* position= */ Vec3::new(3.5, 0.95, 3.5),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.5,
    /* max_velocity= */ 2.0,
  ));
  let agent_off_mesh = archipelago.add_agent(Agent::create(
    /* position= */ Vec3::new(1.5, 1.0, 2.5),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.5,
    /* max_velocity= */ 2.0,
  ));
  let agent_too_high_above_mesh = archipelago.add_agent(Agent::create(
    /* position= */ Vec3::new(1.5, 1.11, 1.5),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.5,
    /* max_velocity= */ 2.0,
  ));

  archipelago.get_agent_mut(agent_1).current_target =
    Some(Vec3::new(3.5, 0.95, 3.5));
  archipelago.get_agent_mut(agent_off_mesh).current_target =
    Some(Vec3::new(3.5, 0.95, 3.5));
  archipelago.get_agent_mut(agent_too_high_above_mesh).current_target =
    Some(Vec3::new(3.5, 0.95, 3.5));
  archipelago.get_agent_mut(agent_2).current_target =
    Some(Vec3::new(1.5, 1.09, 1.5));

  // Nothing has happened yet.
  assert_eq!(archipelago.get_agent(agent_1).state(), AgentState::Idle);
  assert_eq!(archipelago.get_agent(agent_2).state(), AgentState::Idle);
  assert_eq!(archipelago.get_agent(agent_off_mesh).state(), AgentState::Idle);
  assert_eq!(
    archipelago.get_agent(agent_too_high_above_mesh).state(),
    AgentState::Idle
  );

  assert_eq!(archipelago.get_agent(agent_1).get_desired_velocity(), Vec3::ZERO);
  assert_eq!(archipelago.get_agent(agent_2).get_desired_velocity(), Vec3::ZERO);
  assert_eq!(
    archipelago.get_agent(agent_off_mesh).get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(
    archipelago.get_agent(agent_too_high_above_mesh).get_desired_velocity(),
    Vec3::ZERO
  );

  archipelago.update(/* delta_time= */ 0.01);

  // These agents found a path and started following it.
  assert_eq!(archipelago.get_agent(agent_1).state(), AgentState::Moving);
  assert_eq!(archipelago.get_agent(agent_2).state(), AgentState::Moving);
  assert!(archipelago
    .get_agent(agent_1)
    .get_desired_velocity()
    .abs_diff_eq(Vec3::new(1.5, 0.0, 0.5).normalize() * 2.0, 1e-2));
  assert!(archipelago
    .get_agent(agent_2)
    .get_desired_velocity()
    .abs_diff_eq(Vec3::new(-0.5, 0.0, -1.5).normalize() * 2.0, 1e-2));
  // These agents are not on the nav mesh, so they don't do anything.
  assert_eq!(
    archipelago.get_agent(agent_off_mesh).state(),
    AgentState::AgentNotOnNavMesh
  );
  assert_eq!(
    archipelago.get_agent(agent_too_high_above_mesh).state(),
    AgentState::AgentNotOnNavMesh
  );
  assert_eq!(
    archipelago.get_agent(agent_off_mesh).get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(
    archipelago.get_agent(agent_too_high_above_mesh).get_desired_velocity(),
    Vec3::ZERO
  );

  // Move agent_1 forward.
  archipelago.get_agent_mut(agent_1).position = Vec3::new(2.5, 1.0, 1.5);
  archipelago.update(/* delta_time= */ 0.01);

  assert!(archipelago
    .get_agent(agent_1)
    .get_desired_velocity()
    .abs_diff_eq(Vec3::new(0.5, 0.0, 0.5).normalize() * 2.0, 1e-7));
  // These agents don't change.
  assert!(archipelago
    .get_agent(agent_2)
    .get_desired_velocity()
    .abs_diff_eq(Vec3::new(-0.5, 0.0, -1.5).normalize() * 2.0, 1e-2));
  assert_eq!(
    archipelago.get_agent(agent_off_mesh).get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(
    archipelago.get_agent(agent_too_high_above_mesh).get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(archipelago.get_agent(agent_1).state(), AgentState::Moving);
  assert_eq!(archipelago.get_agent(agent_2).state(), AgentState::Moving);
  assert_eq!(
    archipelago.get_agent(agent_off_mesh).state(),
    AgentState::AgentNotOnNavMesh
  );
  assert_eq!(
    archipelago.get_agent(agent_too_high_above_mesh).state(),
    AgentState::AgentNotOnNavMesh
  );

  // Move agent_1 close enough to destination and agent_2 forward.
  archipelago.get_agent_mut(agent_1).position = Vec3::new(3.4, 1.0, 3.4);
  archipelago.get_agent_mut(agent_2).position = Vec3::new(3.5, 1.0, 2.5);
  archipelago.update(/* delta_time= */ 0.01);

  assert_eq!(archipelago.get_agent(agent_1).state(), AgentState::ReachedTarget);
  assert_eq!(archipelago.get_agent(agent_2).state(), AgentState::Moving);
  assert_eq!(archipelago.get_agent(agent_1).get_desired_velocity(), Vec3::ZERO);
  assert!(archipelago
    .get_agent(agent_2)
    .get_desired_velocity()
    .abs_diff_eq(Vec3::new(-0.5, 0.0, -0.5).normalize() * 2.0, 1e-2));
  // These agents don't change.
  assert_eq!(
    archipelago.get_agent(agent_off_mesh).get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(
    archipelago.get_agent(agent_too_high_above_mesh).get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(
    archipelago.get_agent(agent_off_mesh).state(),
    AgentState::AgentNotOnNavMesh
  );
  assert_eq!(
    archipelago.get_agent(agent_too_high_above_mesh).state(),
    AgentState::AgentNotOnNavMesh
  );
}

#[test]
fn add_and_remove_islands() {
  let mut archipelago = Archipelago::new();

  let island_id_1 = archipelago.add_island();
  let island_id_2 = archipelago.add_island();
  let island_id_3 = archipelago.add_island();

  fn sorted(mut v: Vec<IslandId>) -> Vec<IslandId> {
    v.sort();
    v
  }

  assert_eq!(
    sorted(archipelago.get_island_ids().collect()),
    sorted(vec![island_id_1, island_id_2, island_id_3])
  );

  archipelago.remove_island(island_id_2);

  assert_eq!(
    sorted(archipelago.get_island_ids().collect()),
    sorted(vec![island_id_1, island_id_3])
  );
}

#[test]
fn new_or_changed_island_is_not_dirty_after_update() {
  let mut archipelago = Archipelago::new();

  let island_id = archipelago.add_island();

  assert!(archipelago.get_island(island_id).dirty);

  archipelago.update(/* delta_time= */ 0.01);

  assert!(!archipelago.get_island(island_id).dirty);

  archipelago.get_island_mut(island_id).set_nav_mesh(
    Transform::default(),
    Arc::new(ValidNavigationMesh {
      mesh_bounds: BoundingBox::Empty,
      boundary_edges: vec![],
      connectivity: vec![],
      polygons: vec![],
      vertices: vec![],
    }),
  );

  assert!(archipelago.get_island(island_id).dirty);

  archipelago.update(/* delta_time= */ 0.01);

  assert!(!archipelago.get_island(island_id).dirty);
}