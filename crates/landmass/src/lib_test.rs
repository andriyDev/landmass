use std::{collections::HashMap, sync::Arc};

use glam::{Vec2, Vec3};

use crate::{
  coords::{XY, XYZ},
  Agent, AgentId, AgentState, Archipelago, Character, CharacterId, IslandId,
  NavigationMesh, Transform,
};

#[test]
fn add_and_remove_agents() {
  let mut archipelago = Archipelago::<XYZ>::new();

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
      archipelago.get_agent(agent_1).unwrap().radius,
      archipelago.get_agent(agent_2).unwrap().radius,
      archipelago.get_agent(agent_3).unwrap().radius,
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
      archipelago.get_agent(agent_1).unwrap().radius,
      archipelago.get_agent(agent_3).unwrap().radius,
    ],
    [1.0, 3.0],
  );

  archipelago.remove_agent(agent_3);

  assert_eq!(
    sorted(archipelago.get_agent_ids().collect::<Vec<_>>()),
    sorted(vec![agent_1]),
  );
  assert_eq!([archipelago.get_agent(agent_1).unwrap().radius], [1.0]);

  archipelago.remove_agent(agent_1);

  assert_eq!(archipelago.get_agent_ids().collect::<Vec<_>>(), []);
}

#[test]
fn add_and_remove_characters() {
  let mut archipelago = Archipelago::<XYZ>::new();

  let character_1 =
    archipelago.add_character(Character { radius: 1.0, ..Default::default() });

  let character_2 =
    archipelago.add_character(Character { radius: 2.0, ..Default::default() });

  let character_3 =
    archipelago.add_character(Character { radius: 3.0, ..Default::default() });

  fn sorted(mut v: Vec<CharacterId>) -> Vec<CharacterId> {
    v.sort();
    v
  }

  assert_eq!(
    sorted(archipelago.get_character_ids().collect::<Vec<_>>()),
    sorted(vec![character_1, character_2, character_3]),
  );
  assert_eq!(
    [
      archipelago.get_character(character_1).unwrap().radius,
      archipelago.get_character(character_2).unwrap().radius,
      archipelago.get_character(character_3).unwrap().radius,
    ],
    [1.0, 2.0, 3.0],
  );

  archipelago.remove_character(character_2);

  assert_eq!(
    sorted(archipelago.get_character_ids().collect::<Vec<_>>()),
    sorted(vec![character_1, character_3]),
  );
  assert_eq!(
    [
      archipelago.get_character(character_1).unwrap().radius,
      archipelago.get_character(character_3).unwrap().radius,
    ],
    [1.0, 3.0],
  );

  archipelago.remove_character(character_3);

  assert_eq!(
    sorted(archipelago.get_character_ids().collect::<Vec<_>>()),
    sorted(vec![character_1]),
  );
  assert_eq!([archipelago.get_character(character_1).unwrap().radius], [1.0]);

  archipelago.remove_character(character_1);

  assert_eq!(archipelago.get_character_ids().collect::<Vec<_>>(), []);
}

#[test]
fn computes_and_follows_path() {
  let mut archipelago = Archipelago::<XYZ>::new();
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(1.0, 1.0, 1.0),
      Vec3::new(2.0, 1.0, 1.0),
      Vec3::new(3.0, 1.0, 1.0),
      Vec3::new(4.0, 1.0, 1.0),
      Vec3::new(4.0, 2.0, 1.0),
      Vec3::new(4.0, 3.0, 1.0),
      Vec3::new(4.0, 4.0, 1.0),
      Vec3::new(3.0, 4.0, 1.0),
      Vec3::new(3.0, 3.0, 1.0),
      Vec3::new(3.0, 2.0, 1.0),
      Vec3::new(2.0, 2.0, 1.0),
      Vec3::new(1.0, 2.0, 1.0),
    ],
    polygons: vec![
      vec![0, 1, 10, 11],
      vec![1, 2, 9, 10],
      vec![2, 3, 4, 9],
      vec![4, 5, 8, 9],
      vec![5, 6, 7, 8],
    ],
    polygon_type_indices: vec![0, 0, 0, 0, 0],
  }
  .validate()
  .expect("is valid");

  archipelago.add_island().set_nav_mesh(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::new(nav_mesh),
    HashMap::new(),
  );

  archipelago.agent_options.neighbourhood = 0.0;
  archipelago.agent_options.obstacle_avoidance_time_horizon = 0.01;

  let agent_1 = archipelago.add_agent(Agent::create(
    /* position= */ Vec3::new(1.5, 1.5, 1.09),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.5,
    /* max_velocity= */ 2.0,
  ));
  let agent_2 = archipelago.add_agent(Agent::create(
    /* position= */ Vec3::new(3.5, 3.5, 0.95),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.5,
    /* max_velocity= */ 2.0,
  ));
  let agent_off_mesh = archipelago.add_agent(Agent::create(
    /* position= */ Vec3::new(1.5, 2.5, 1.0),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.5,
    /* max_velocity= */ 2.0,
  ));
  let agent_too_high_above_mesh = archipelago.add_agent(Agent::create(
    /* position= */ Vec3::new(1.5, 1.5, 1.11),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.5,
    /* max_velocity= */ 2.0,
  ));

  archipelago.get_agent_mut(agent_1).unwrap().current_target =
    Some(Vec3::new(3.5, 3.5, 0.95));
  archipelago.get_agent_mut(agent_off_mesh).unwrap().current_target =
    Some(Vec3::new(3.5, 3.5, 0.95));
  archipelago
    .get_agent_mut(agent_too_high_above_mesh)
    .unwrap()
    .current_target = Some(Vec3::new(3.5, 3.5, 0.95));
  archipelago.get_agent_mut(agent_2).unwrap().current_target =
    Some(Vec3::new(1.5, 1.5, 1.09));

  // Nothing has happened yet.
  assert_eq!(archipelago.get_agent(agent_1).unwrap().state(), AgentState::Idle);
  assert_eq!(archipelago.get_agent(agent_2).unwrap().state(), AgentState::Idle);
  assert_eq!(
    archipelago.get_agent(agent_off_mesh).unwrap().state(),
    AgentState::Idle
  );
  assert_eq!(
    archipelago.get_agent(agent_too_high_above_mesh).unwrap().state(),
    AgentState::Idle
  );

  assert_eq!(
    *archipelago.get_agent(agent_1).unwrap().get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(
    *archipelago.get_agent(agent_2).unwrap().get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(
    *archipelago.get_agent(agent_off_mesh).unwrap().get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(
    *archipelago
      .get_agent(agent_too_high_above_mesh)
      .unwrap()
      .get_desired_velocity(),
    Vec3::ZERO
  );

  archipelago.update(/* delta_time= */ 0.01);

  // These agents found a path and started following it.
  assert_eq!(
    archipelago.get_agent(agent_1).unwrap().state(),
    AgentState::Moving
  );
  assert_eq!(
    archipelago.get_agent(agent_2).unwrap().state(),
    AgentState::Moving
  );
  assert!(archipelago
    .get_agent(agent_1)
    .unwrap()
    .get_desired_velocity()
    .abs_diff_eq(Vec3::new(1.5, 0.5, 0.0).normalize() * 2.0, 1e-2));
  assert!(archipelago
    .get_agent(agent_2)
    .unwrap()
    .get_desired_velocity()
    .abs_diff_eq(Vec3::new(-0.5, -1.5, 0.0).normalize() * 2.0, 1e-2));
  // These agents are not on the nav mesh, so they don't do anything.
  assert_eq!(
    archipelago.get_agent(agent_off_mesh).unwrap().state(),
    AgentState::AgentNotOnNavMesh
  );
  assert_eq!(
    archipelago.get_agent(agent_too_high_above_mesh).unwrap().state(),
    AgentState::AgentNotOnNavMesh
  );
  assert_eq!(
    *archipelago.get_agent(agent_off_mesh).unwrap().get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(
    *archipelago
      .get_agent(agent_too_high_above_mesh)
      .unwrap()
      .get_desired_velocity(),
    Vec3::ZERO
  );

  assert_eq!(archipelago.get_pathing_results().len(), 2);
  let path_result_1 = archipelago.get_pathing_results()[0];
  let path_result_2 = archipelago.get_pathing_results()[1];
  assert_eq!(path_result_1.success, true);
  assert_eq!(path_result_2.success, true);
  assert!(path_result_1.explored_nodes > 0);
  assert!(path_result_2.explored_nodes > 0);

  // Move agent_1 forward.
  archipelago.get_agent_mut(agent_1).unwrap().position =
    Vec3::new(2.5, 1.5, 1.0);
  archipelago.update(/* delta_time= */ 0.01);

  assert!(archipelago
    .get_agent(agent_1)
    .unwrap()
    .get_desired_velocity()
    .abs_diff_eq(Vec3::new(0.5, 0.5, 0.0).normalize() * 2.0, 1e-7));
  // These agents don't change.
  assert!(archipelago
    .get_agent(agent_2)
    .unwrap()
    .get_desired_velocity()
    .abs_diff_eq(Vec3::new(-0.5, -1.5, 0.0).normalize() * 2.0, 1e-2));
  assert_eq!(
    *archipelago.get_agent(agent_off_mesh).unwrap().get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(
    *archipelago
      .get_agent(agent_too_high_above_mesh)
      .unwrap()
      .get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(
    archipelago.get_agent(agent_1).unwrap().state(),
    AgentState::Moving
  );
  assert_eq!(
    archipelago.get_agent(agent_2).unwrap().state(),
    AgentState::Moving
  );
  assert_eq!(
    archipelago.get_agent(agent_off_mesh).unwrap().state(),
    AgentState::AgentNotOnNavMesh
  );
  assert_eq!(
    archipelago.get_agent(agent_too_high_above_mesh).unwrap().state(),
    AgentState::AgentNotOnNavMesh
  );

  // Move agent_1 close enough to destination and agent_2 forward.
  archipelago.get_agent_mut(agent_1).unwrap().position =
    Vec3::new(3.4, 3.4, 1.0);
  archipelago.get_agent_mut(agent_2).unwrap().position =
    Vec3::new(3.5, 2.5, 1.0);
  archipelago.update(/* delta_time= */ 0.01);

  assert_eq!(
    archipelago.get_agent(agent_1).unwrap().state(),
    AgentState::ReachedTarget
  );
  assert_eq!(
    archipelago.get_agent(agent_2).unwrap().state(),
    AgentState::Moving
  );
  assert_eq!(
    *archipelago.get_agent(agent_1).unwrap().get_desired_velocity(),
    Vec3::ZERO
  );
  assert!(archipelago
    .get_agent(agent_2)
    .unwrap()
    .get_desired_velocity()
    .abs_diff_eq(Vec3::new(-0.5, -0.5, 0.0).normalize() * 2.0, 1e-2));
  // These agents don't change.
  assert_eq!(
    *archipelago.get_agent(agent_off_mesh).unwrap().get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(
    *archipelago
      .get_agent(agent_too_high_above_mesh)
      .unwrap()
      .get_desired_velocity(),
    Vec3::ZERO
  );
  assert_eq!(
    archipelago.get_agent(agent_off_mesh).unwrap().state(),
    AgentState::AgentNotOnNavMesh
  );
  assert_eq!(
    archipelago.get_agent(agent_too_high_above_mesh).unwrap().state(),
    AgentState::AgentNotOnNavMesh
  );
}

#[test]
fn add_and_remove_islands() {
  let mut archipelago = Archipelago::<XYZ>::new();

  let island_id_1 = archipelago.add_island().id();
  let island_id_2 = archipelago.add_island().id();
  let island_id_3 = archipelago.add_island().id();

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
fn changed_island_is_not_dirty_after_update() {
  let mut archipelago = Archipelago::<XYZ>::new();

  let island_id = archipelago.add_island().id();

  assert!(!archipelago.get_island(island_id).unwrap().dirty);

  archipelago.get_island_mut(island_id).unwrap().set_nav_mesh(
    Transform::default(),
    Arc::new(
      NavigationMesh {
        vertices: vec![],
        polygons: vec![],
        polygon_type_indices: vec![],
      }
      .validate()
      .unwrap(),
    ),
    HashMap::new(),
  );

  assert!(archipelago.get_island(island_id).unwrap().dirty);

  archipelago.update(/* delta_time= */ 0.01);

  assert!(!archipelago.get_island(island_id).unwrap().dirty);
}

#[test]
fn samples_point() {
  let mut archipelago = Archipelago::<XY>::new();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let offset = Vec2::new(10.0, 10.0);
  archipelago.add_island().set_nav_mesh(
    Transform { translation: offset, rotation: 0.0 },
    nav_mesh,
    HashMap::new(),
  );
  archipelago.update(1.0);

  assert_eq!(
    archipelago
      .sample_point(
        /* point= */ offset + Vec2::new(-0.5, 0.5),
        /* distance_to_node= */ 0.6
      )
      .map(|p| p.point()),
    Ok(offset + Vec2::new(0.0, 0.5))
  );
  assert_eq!(
    archipelago
      .sample_point(
        /* point= */ offset + Vec2::new(0.5, 0.5),
        /* distance_to_node= */ 0.6
      )
      .map(|p| p.point()),
    Ok(offset + Vec2::new(0.5, 0.5))
  );
  assert_eq!(
    archipelago
      .sample_point(
        /* point= */ offset + Vec2::new(1.2, 1.2),
        /* distance_to_node= */ 0.6
      )
      .map(|p| p.point()),
    Ok(offset + Vec2::new(1.0, 1.0))
  );
}

#[test]
fn finds_path() {
  let mut archipelago = Archipelago::<XY>::new();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let offset = Vec2::new(10.0, 10.0);
  archipelago.add_island().set_nav_mesh(
    Transform { translation: offset, rotation: 0.0 },
    nav_mesh.clone(),
    HashMap::new(),
  );
  archipelago.add_island().set_nav_mesh(
    Transform { translation: offset + Vec2::new(1.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
    HashMap::new(),
  );
  archipelago.add_island().set_nav_mesh(
    Transform { translation: offset + Vec2::new(2.0, 0.5), rotation: 0.0 },
    nav_mesh,
    HashMap::new(),
  );
  archipelago.update(1.0);

  let start_point = archipelago
    .sample_point(offset + Vec2::new(0.5, 0.5), 1e-5)
    .expect("point is on nav mesh.");
  let end_point = archipelago
    .sample_point(offset + Vec2::new(2.5, 1.25), 1e-5)
    .expect("point is on nav mesh.");
  assert_eq!(
    archipelago.find_path(&start_point, &end_point),
    Ok(vec![
      offset + Vec2::new(0.5, 0.5),
      offset + Vec2::new(2.0, 1.0),
      offset + Vec2::new(2.5, 1.25)
    ])
  );
}
