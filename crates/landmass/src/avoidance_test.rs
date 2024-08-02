use std::{collections::HashMap, sync::Arc};

use glam::{Vec2, Vec3};
use slotmap::HopSlotMap;

use crate::{
  avoidance::apply_avoidance_to_agents,
  coords::{XY, XYZ},
  nav_data::NodeRef,
  Agent, AgentId, AgentOptions, Archipelago, Character, CharacterId, Island,
  NavigationData, NavigationMesh, Transform,
};

use super::nav_mesh_borders_to_dodgy_obstacles;

fn obstacle_matches(
  left: &dodgy_2d::Obstacle,
  right: &dodgy_2d::Obstacle,
) -> bool {
  match (left, right) {
    (
      dodgy_2d::Obstacle::Closed { vertices: left_vertices },
      dodgy_2d::Obstacle::Closed { vertices: right_vertices },
    ) => {
      for left_offset in 0..left_vertices.len() {
        if left_vertices[left_offset..]
          .iter()
          .cloned()
          .chain(left_vertices[..left_offset].iter().cloned())
          .collect::<Vec<_>>()
          == *right_vertices
        {
          return true;
        }
      }
      false
    }
    (
      dodgy_2d::Obstacle::Open { vertices: left_vertices },
      dodgy_2d::Obstacle::Open { vertices: right_vertices },
    ) => left_vertices == right_vertices,
    _ => false,
  }
}

macro_rules! assert_obstacles_match {
  ($left: expr, $right: expr) => {{
    let left = $left;
    let mut right = $right;

    'outer: for (left_index, left_obstacle) in left.iter().enumerate() {
      for (right_index, right_obstacle) in right.iter().enumerate() {
        if obstacle_matches(left_obstacle, right_obstacle) {
          right.remove(right_index);
          continue 'outer;
        }
      }
      panic!("Failed to match left obstacle: index={} obstacle={:?}\n\nleft_obstacles={:?}\n\nremaining_obstacles={:?}\n",
        left_index, left_obstacle, left, right);
    }

    if !right.is_empty() {
      panic!("Failed to match right obstacles:\n\nleft_obstacles={:?}\n\nremaining_obstacles={:?}\n", left, right);
    }
  }};
}

#[test]
fn computes_obstacle_for_box() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(2.0, 1.0, 0.0),
      Vec3::new(2.0, 2.0, 0.0),
      Vec3::new(1.0, 2.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
    polygon_type_indices: vec![0],
  }
  .validate()
  .expect("Validation succeeds");

  let mut nav_data = NavigationData::<XYZ>::new();

  let island_offset = Vec3::new(130.0, -50.0, 20.0);
  let island_offset_dodgy =
    dodgy_2d::Vec2::new(island_offset.x, island_offset.y);

  let island_id = nav_data.add_island(Island::new(
    Transform { translation: island_offset, rotation: 0.0 },
    Arc::new(nav_mesh),
    HashMap::new(),
  ));

  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (
        Vec3::new(1.5, 1.5, 0.0) + island_offset,
        NodeRef { island_id, polygon_index: 0 }
      ),
      &nav_data,
      /* distance_limit= */ 10.0,
    ),
    vec![dodgy_2d::Obstacle::Closed {
      vertices: vec![
        dodgy_2d::Vec2::new(1.0, 1.0) + island_offset_dodgy,
        dodgy_2d::Vec2::new(1.0, 2.0) + island_offset_dodgy,
        dodgy_2d::Vec2::new(2.0, 2.0) + island_offset_dodgy,
        dodgy_2d::Vec2::new(2.0, 1.0) + island_offset_dodgy
      ]
    }]
  );
}

#[test]
fn dead_end_makes_open_obstacle() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(2.0, 1.0, 0.0),
      Vec3::new(2.0, 2.0, 0.0),
      Vec3::new(1.0, 2.0, 0.0),
      Vec3::new(3.0, 1.0, 0.0),
      Vec3::new(3.0, 2.0, 0.0),
      Vec3::new(4.0, 1.0, 0.0),
      Vec3::new(4.0, 2.0, 0.0),
      Vec3::new(4.0, 3.0, 0.0),
      Vec3::new(3.0, 3.0, 0.0),
      Vec3::new(4.0, 4.0, 0.0),
      Vec3::new(3.0, 4.0, 0.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3],
      vec![2, 1, 4, 5],
      vec![5, 4, 6, 7],
      vec![5, 7, 8, 9],
      vec![9, 8, 10, 11],
    ],
    polygon_type_indices: vec![0, 0, 0, 0, 0],
  }
  .validate()
  .expect("Validation succeeds");

  let mut nav_data = NavigationData::<XYZ>::new();
  let island_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::new(nav_mesh),
    HashMap::new(),
  ));

  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (Vec3::new(1.5, 1.5, 0.0), NodeRef { island_id, polygon_index: 0 }),
      &nav_data,
      /* distance_limit= */ 10.0,
    ),
    vec![dodgy_2d::Obstacle::Open {
      vertices: vec![
        dodgy_2d::Vec2::new(4.0, 3.0),
        dodgy_2d::Vec2::new(4.0, 2.0),
        dodgy_2d::Vec2::new(4.0, 1.0),
        dodgy_2d::Vec2::new(3.0, 1.0),
        dodgy_2d::Vec2::new(2.0, 1.0),
        dodgy_2d::Vec2::new(1.0, 1.0),
        dodgy_2d::Vec2::new(1.0, 2.0),
        dodgy_2d::Vec2::new(2.0, 2.0),
        dodgy_2d::Vec2::new(3.0, 2.0),
      ]
    }]
  );

  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (Vec3::new(3.5, 3.5, 0.0), NodeRef { island_id, polygon_index: 4 }),
      &nav_data,
      /* distance_limit= */ 10.0,
    ),
    vec![dodgy_2d::Obstacle::Open {
      vertices: vec![
        dodgy_2d::Vec2::new(3.0, 2.0),
        dodgy_2d::Vec2::new(3.0, 3.0),
        dodgy_2d::Vec2::new(3.0, 4.0),
        dodgy_2d::Vec2::new(4.0, 4.0),
        dodgy_2d::Vec2::new(4.0, 3.0),
        dodgy_2d::Vec2::new(4.0, 2.0),
        dodgy_2d::Vec2::new(4.0, 1.0),
        dodgy_2d::Vec2::new(3.0, 1.0),
        dodgy_2d::Vec2::new(2.0, 1.0),
      ]
    }]
  );

  // Decrease the distance limit to limit the size of the open obstacle.
  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (Vec3::new(3.5, 3.5, 0.0), NodeRef { island_id, polygon_index: 4 }),
      &nav_data,
      /* distance_limit= */ 1.0,
    ),
    vec![dodgy_2d::Obstacle::Open {
      vertices: vec![
        dodgy_2d::Vec2::new(3.0, 2.0),
        dodgy_2d::Vec2::new(3.0, 3.0),
        dodgy_2d::Vec2::new(3.0, 4.0),
        dodgy_2d::Vec2::new(4.0, 4.0),
        dodgy_2d::Vec2::new(4.0, 3.0),
        dodgy_2d::Vec2::new(4.0, 2.0),
      ]
    }]
  );

  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (Vec3::new(3.5, 1.5, 0.0), NodeRef { island_id, polygon_index: 2 }),
      &nav_data,
      /* distance_limit= */ 10.0,
    ),
    vec![dodgy_2d::Obstacle::Closed {
      vertices: vec![
        dodgy_2d::Vec2::new(1.0, 1.0),
        dodgy_2d::Vec2::new(1.0, 2.0),
        dodgy_2d::Vec2::new(2.0, 2.0),
        dodgy_2d::Vec2::new(3.0, 2.0),
        dodgy_2d::Vec2::new(3.0, 3.0),
        dodgy_2d::Vec2::new(3.0, 4.0),
        dodgy_2d::Vec2::new(4.0, 4.0),
        dodgy_2d::Vec2::new(4.0, 3.0),
        dodgy_2d::Vec2::new(4.0, 2.0),
        dodgy_2d::Vec2::new(4.0, 1.0),
        dodgy_2d::Vec2::new(3.0, 1.0),
        dodgy_2d::Vec2::new(2.0, 1.0),
      ]
    }]
  );
}

#[test]
fn split_borders() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(2.0, 1.0, 0.0),
      Vec3::new(3.0, 1.0, 0.0),
      Vec3::new(4.0, 1.0, 0.0),
      Vec3::new(5.0, 1.0, 0.0),
      Vec3::new(5.0, 2.0, 0.0),
      Vec3::new(5.0, 3.0, 0.0),
      Vec3::new(5.0, 4.0, 0.0),
      Vec3::new(6.0, 4.0, 0.0),
      Vec3::new(6.0, 3.0, 0.0),
      Vec3::new(6.0, 2.0, 0.0),
      Vec3::new(6.0, 1.0, 0.0),
      Vec3::new(6.0, 0.0, 0.0),
      Vec3::new(5.0, 0.0, 0.0),
      Vec3::new(4.0, 0.0, 0.0),
      Vec3::new(3.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
      Vec3::new(0.0, 2.0, 0.0),
      Vec3::new(0.0, 3.0, 0.0),
      Vec3::new(0.0, 4.0, 0.0),
      Vec3::new(1.0, 4.0, 0.0),
      Vec3::new(1.0, 3.0, 0.0),
      Vec3::new(1.0, 2.0, 0.0),
    ],
    polygons: vec![
      vec![0, 17, 16, 1],
      vec![1, 16, 15, 2],
      vec![2, 15, 14, 3],
      vec![3, 14, 13, 4],
      vec![4, 13, 12, 11],
      vec![4, 11, 10, 5],
      vec![5, 10, 9, 6],
      vec![6, 9, 8, 7],
      vec![0, 19, 18, 17],
      vec![25, 20, 19, 0],
      vec![24, 21, 20, 25],
      vec![23, 22, 21, 24],
    ],
    polygon_type_indices: vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  }
  .validate()
  .expect("Validation succeeds");

  let mut nav_data = NavigationData::<XYZ>::new();
  let island_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::new(nav_mesh),
    HashMap::new(),
  ));

  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (Vec3::new(3.0, 0.9, 0.0), NodeRef { island_id, polygon_index: 0 }),
      &nav_data,
      /* distance_limit= */ 10.0,
    ),
    vec![
      dodgy_2d::Obstacle::Open {
        vertices: vec![
          dodgy_2d::Vec2::new(1.0, 1.0),
          dodgy_2d::Vec2::new(2.0, 1.0),
          dodgy_2d::Vec2::new(3.0, 1.0),
          dodgy_2d::Vec2::new(4.0, 1.0),
          dodgy_2d::Vec2::new(5.0, 1.0),
        ]
      },
      dodgy_2d::Obstacle::Open {
        vertices: vec![
          dodgy_2d::Vec2::new(6.0, 2.0),
          dodgy_2d::Vec2::new(6.0, 1.0),
          dodgy_2d::Vec2::new(6.0, 0.0),
          dodgy_2d::Vec2::new(5.0, 0.0),
          dodgy_2d::Vec2::new(4.0, 0.0),
          dodgy_2d::Vec2::new(3.0, 0.0),
          dodgy_2d::Vec2::new(2.0, 0.0),
          dodgy_2d::Vec2::new(1.0, 0.0),
          dodgy_2d::Vec2::new(0.0, 0.0),
          dodgy_2d::Vec2::new(0.0, 1.0),
          dodgy_2d::Vec2::new(0.0, 2.0),
        ]
      }
    ]
  );
}

#[test]
fn creates_obstacles_across_boundary_link() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(2.0, 1.0, 1.0),
        Vec3::new(2.0, 2.0, 1.0),
        Vec3::new(1.0, 2.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("Validation succeeds"),
  );

  let mut nav_data = NavigationData::<XYZ>::new();
  nav_data.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::clone(&nav_mesh),
    HashMap::new(),
  ));
  let island_id_2 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(1.0, 0.0, 0.0), rotation: 0.0 },
    nav_mesh,
    HashMap::new(),
  ));

  nav_data.update(0.01);

  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (
        Vec3::new(2.5, 1.5, 1.0),
        NodeRef { island_id: island_id_2, polygon_index: 0 }
      ),
      &nav_data,
      /* distance_limit= */ 10.0,
    ),
    vec![
      dodgy_2d::Obstacle::Open {
        vertices: vec![
          dodgy_2d::Vec2::new(2.0, 1.0),
          dodgy_2d::Vec2::new(1.0, 1.0),
          dodgy_2d::Vec2::new(1.0, 2.0),
          dodgy_2d::Vec2::new(2.0, 2.0),
        ]
      },
      dodgy_2d::Obstacle::Open {
        vertices: vec![
          dodgy_2d::Vec2::new(2.0, 2.0),
          dodgy_2d::Vec2::new(3.0, 2.0),
          dodgy_2d::Vec2::new(3.0, 1.0),
          dodgy_2d::Vec2::new(2.0, 1.0),
        ]
      }
    ]
  );
}

#[test]
fn applies_no_avoidance_for_far_agents() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(-1.0, -1.0, 0.0),
      Vec3::new(13.0, -1.0, 0.0),
      Vec3::new(13.0, 3.0, 0.0),
      Vec3::new(-1.0, 3.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
    polygon_type_indices: vec![0],
  }
  .validate()
  .expect("Validation succeeded.");

  let mut nav_data = NavigationData::<XYZ>::new();
  let island_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::new(nav_mesh),
    HashMap::new(),
  ));

  let mut agents = HopSlotMap::<AgentId, _>::with_key();
  let agent_1 = agents.insert({
    let mut agent = Agent::create(
      /* position= */ Vec3::new(1.0, 1.0, 0.0),
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.01,
      /* desired_speed= */ 1.0,
      /* max_speed= */ 1.0,
    );
    agent.current_desired_move = Vec3::new(1.0, 0.0, 0.0);
    agent
  });
  let agent_2 = agents.insert({
    let mut agent = Agent::create(
      /* position= */ Vec3::new(11.0, 1.0, 0.0),
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.01,
      /* desired_speed= */ 1.0,
      /* max_speed= */ 1.0,
    );
    agent.current_desired_move = Vec3::new(-1.0, 0.0, 0.0);
    agent
  });
  let agent_3 = agents.insert({
    let mut agent = Agent::create(
      /* position= */ Vec3::new(5.0, 4.0, 0.0),
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.01,
      /* desired_speed= */ 1.0,
      /* max_speed= */ 1.0,
    );
    agent.current_desired_move = Vec3::new(0.0, 1.0, 0.0);
    agent
  });

  let mut agent_id_to_agent_node = HashMap::new();
  agent_id_to_agent_node.insert(
    agent_1,
    (
      agents.get(agent_1).unwrap().position,
      NodeRef { island_id, polygon_index: 0 },
    ),
  );
  agent_id_to_agent_node.insert(
    agent_2,
    (
      agents.get(agent_2).unwrap().position,
      NodeRef { island_id, polygon_index: 0 },
    ),
  );
  // `agent_3` is not on a node.

  apply_avoidance_to_agents(
    &mut agents,
    &agent_id_to_agent_node,
    /* characters= */ &HopSlotMap::with_key(),
    /* character_id_to_nav_mesh_point= */ &HashMap::new(),
    &nav_data,
    &AgentOptions { neighbourhood: 5.0, ..Default::default() },
    /* delta_time= */ 0.01,
  );

  assert_eq!(
    *agents.get(agent_1).unwrap().get_desired_velocity(),
    Vec3::new(1.0, 0.0, 0.0)
  );
  assert_eq!(
    *agents.get(agent_2).unwrap().get_desired_velocity(),
    Vec3::new(-1.0, 0.0, 0.0)
  );
  assert_eq!(
    *agents.get(agent_3).unwrap().get_desired_velocity(),
    Vec3::new(0.0, 1.0, 0.0)
  );
}

#[test]
fn applies_avoidance_for_two_agents() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(-1.0, -1.0, 0.0),
      Vec3::new(13.0, -1.0, 0.0),
      Vec3::new(13.0, 3.0, 0.0),
      Vec3::new(-1.0, 3.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
    polygon_type_indices: vec![0],
  }
  .validate()
  .expect("Validation succeeded.");

  let mut nav_data = NavigationData::<XYZ>::new();
  let island_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::new(nav_mesh),
    HashMap::new(),
  ));

  let mut agents = HopSlotMap::<AgentId, _>::with_key();
  let agent_1 = agents.insert({
    let mut agent = Agent::create(
      /* position= */ Vec3::new(1.0, 1.0, 0.0),
      /* velocity= */ Vec3::new(1.0, 0.0, 0.0),
      /* radius= */ 1.0,
      /* desired_speed= */ 1.0,
      /* max_speed= */ 1.0,
    );
    agent.current_desired_move = Vec3::new(1.0, 0.0, 0.0);
    agent
  });
  let agent_2 = agents.insert({
    let mut agent = Agent::create(
      /* position= */ Vec3::new(11.0, 1.01, 0.0),
      /* velocity= */ Vec3::new(-1.0, 0.0, 0.0),
      /* radius= */ 1.0,
      /* desired_speed= */ 1.0,
      /* max_speed= */ 1.0,
    );
    agent.current_desired_move = Vec3::new(-1.0, 0.0, 0.0);
    agent
  });

  let mut agent_id_to_agent_node = HashMap::new();
  agent_id_to_agent_node.insert(
    agent_1,
    (
      agents.get(agent_1).unwrap().position,
      NodeRef { island_id, polygon_index: 0 },
    ),
  );
  agent_id_to_agent_node.insert(
    agent_2,
    (
      agents.get(agent_2).unwrap().position,
      NodeRef { island_id, polygon_index: 0 },
    ),
  );

  apply_avoidance_to_agents(
    &mut agents,
    &agent_id_to_agent_node,
    /* characters= */ &HopSlotMap::with_key(),
    /* character_id_to_nav_mesh_point= */ &HashMap::new(),
    &nav_data,
    &AgentOptions {
      neighbourhood: 15.0,
      avoidance_time_horizon: 15.0,
      ..Default::default()
    },
    /* delta_time= */ 0.01,
  );

  // The agents each have a radius of 1, and they are separated by a distance
  // of 10 (they start at (1,0) and (11,0)). So in order to pass each other, one
  // agent must go to (6,1) and the other agent must go to (6,-1). That's a rise
  // over run of 1/5 or 0.2, which is our expected Z velocity. We derive the X
  // velocity by just making the length of the vector 1 (the agent's max speed).
  let agent_1_desired_velocity =
    *agents.get(agent_1).unwrap().get_desired_velocity();
  assert!(
    agent_1_desired_velocity.abs_diff_eq(Vec3::new(0.98, -0.2, 0.0), 0.05),
    "left={agent_1_desired_velocity}, right=Vec3(0.98, -0.2, 0.0)"
  );
  let agent_2_desired_velocity =
    *agents.get(agent_2).unwrap().get_desired_velocity();
  assert!(
    agent_2_desired_velocity.abs_diff_eq(Vec3::new(-0.98, 0.2, 0.0), 0.05),
    "left={agent_2_desired_velocity}, right=Vec3(-0.98, 0.2, 0.0)"
  );
}

#[test]
fn agent_avoids_character() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(-1.0, -1.0, 0.0),
      Vec3::new(13.0, -1.0, 0.0),
      Vec3::new(13.0, 3.0, 0.0),
      Vec3::new(-1.0, 3.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
    polygon_type_indices: vec![0],
  }
  .validate()
  .expect("Validation succeeded.");

  let mut nav_data = NavigationData::<XYZ>::new();
  let island_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::new(nav_mesh),
    HashMap::new(),
  ));

  let mut agents = HopSlotMap::<AgentId, _>::with_key();
  let agent = agents.insert({
    let mut agent = Agent::create(
      /* position= */ Vec3::new(1.0, 1.0, 0.0),
      /* velocity= */ Vec3::new(1.0, 0.0, 0.0),
      /* radius= */ 1.0,
      /* desired_speed= */ 1.0,
      /* max_speed= */ 1.0,
    );
    agent.current_desired_move = Vec3::new(1.0, 0.0, 0.0);
    agent
  });
  let mut characters = HopSlotMap::<CharacterId, _>::with_key();
  let character = characters.insert(Character {
    position: Vec3::new(11.0, 1.01, 0.0),
    velocity: Vec3::new(-1.0, 0.0, 0.0),
    radius: 1.0,
  });

  let mut agent_id_to_agent_node = HashMap::new();
  agent_id_to_agent_node.insert(
    agent,
    (
      agents.get(agent).unwrap().position,
      NodeRef { island_id, polygon_index: 0 },
    ),
  );
  let mut character_id_to_nav_mesh_point = HashMap::new();
  character_id_to_nav_mesh_point
    .insert(character, characters.get(character).unwrap().position);

  apply_avoidance_to_agents(
    &mut agents,
    &agent_id_to_agent_node,
    &characters,
    &character_id_to_nav_mesh_point,
    &nav_data,
    &AgentOptions {
      neighbourhood: 15.0,
      avoidance_time_horizon: 15.0,
      ..Default::default()
    },
    /* delta_time= */ 0.01,
  );

  // The agent+character each have a radius of 1, and they are separated by a
  // distance of 10 (they start at (1,0) and (11,0)). Only the agent is
  // managed by landmass, so it must go to (6,2), since the character will go to
  // (6,0). That's a rise over run of 2/5 or 0.4, which is our expected Z
  // velocity. We derive the X velocity by just making the length of the
  // vector 1 (the agent's max speed).
  let agent_desired_velocity =
    agents.get(agent).unwrap().get_desired_velocity();
  assert!(
    agent_desired_velocity
      .abs_diff_eq(Vec3::new((1.0f32 - 0.4 * 0.4).sqrt(), -0.4, 0.0), 0.05),
    "left={agent_desired_velocity}, right=Vec3(0.9165..., -0.4, 0.0)"
  );
}

#[test]
fn agent_speeds_up_to_avoid_character() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec2::new(-10.0, -10.0),
      Vec2::new(10.0, -10.0),
      Vec2::new(10.0, 10.0),
      Vec2::new(-10.0, 10.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
    polygon_type_indices: vec![0],
  }
  .validate()
  .expect("Validation succeeded.");

  let mut nav_data = NavigationData::<XY>::new();
  let island_id = nav_data.add_island(Island::new(
    Transform { translation: Vec2::ZERO, rotation: 0.0 },
    Arc::new(nav_mesh),
    HashMap::new(),
  ));

  let mut agents = HopSlotMap::<AgentId, _>::with_key();
  let agent = agents.insert({
    let mut agent = Agent::<XY>::create(
      /* position= */ Vec2::new(5.0, 0.0),
      /* velocity= */ Vec2::new(-1.0, 0.0),
      /* radius= */ 0.5,
      /* desired_speed= */ 1.0,
      /* max_speed= */ 2.0,
    );
    agent.current_desired_move = Vec2::new(1.0, 0.0);
    agent
  });

  let mut agent_id_to_agent_node = HashMap::new();
  agent_id_to_agent_node.insert(
    agent,
    (
      agents.get(agent).unwrap().position.extend(0.0),
      NodeRef { island_id, polygon_index: 0 },
    ),
  );

  apply_avoidance_to_agents(
    &mut agents,
    &agent_id_to_agent_node,
    &HopSlotMap::with_key(),
    &HashMap::new(),
    &nav_data,
    &AgentOptions {
      neighbourhood: 15.0,
      avoidance_time_horizon: 15.0,
      ..Default::default()
    },
    /* delta_time= */ 0.01,
  );
  // The agent sticks to its desired velocity.
  assert_eq!(
    *agents.get(agent).unwrap().get_desired_velocity(),
    Vec2::new(1.0, 0.0)
  );

  let mut characters = HopSlotMap::<CharacterId, _>::with_key();
  let character = characters.insert(Character::<XY> {
    // Just slightly closer to the agent so it prefers to "speed up".
    position: Vec2::new(0.0, 5.0),
    velocity: Vec2::new(0.0, -1.0),
    radius: 0.5,
  });
  let mut character_id_to_nav_mesh_point = HashMap::new();
  character_id_to_nav_mesh_point
    .insert(character, characters.get(character).unwrap().position.extend(0.0));

  apply_avoidance_to_agents(
    &mut agents,
    &agent_id_to_agent_node,
    &characters,
    &character_id_to_nav_mesh_point,
    &nav_data,
    &AgentOptions {
      neighbourhood: 15.0,
      avoidance_time_horizon: 15.0,
      ..Default::default()
    },
    /* delta_time= */ 0.01,
  );

  let agent_desired_velocity =
    *agents.get(agent).unwrap().get_desired_velocity();
  // Check the agent has sped up to avoid the character.
  assert!(
    agent_desired_velocity.length() > 1.1,
    "actual={agent_desired_velocity} actual_length={} expected=greater than 1.0",
    agent_desired_velocity.length(),
  );
}

#[test]
fn reached_target_agent_has_different_avoidance() {
  let mut archipelago = Archipelago::<XY>::new();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(-10.0, -10.0),
        Vec2::new(10.0, -10.0),
        Vec2::new(10.0, 10.0),
        Vec2::new(-10.0, 10.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .unwrap(),
  );

  archipelago.add_island(Island::new(
    Transform::default(),
    nav_mesh,
    HashMap::new(),
  ));

  let agent_1 = archipelago.add_agent({
    let mut agent = Agent::create(
      /* position= */ Vec2::new(0.0, 0.0),
      /* velocity= */ Vec2::ZERO,
      /* radius= */ 0.5,
      /* desired_speed= */ 1.0,
      /* max_speed= */ 1.0,
    );
    agent.current_target = Some(Vec2::new(0.0, 0.0));
    agent
  });

  let agent_2 = archipelago.add_agent({
    let mut agent = Agent::create(
      /* position= */ Vec2::new(0.0, -3.0),
      /* velocity= */ Vec2::new(0.0, 1.0),
      /* radius= */ 0.5,
      /* desired_speed= */ 1.0,
      /* max_speed= */ 1.0,
    );
    agent.current_target = Some(Vec2::new(0.0, 3.0));
    agent
  });

  archipelago.agent_options.avoidance_time_horizon = 100.0;
  archipelago.agent_options.obstacle_avoidance_time_horizon = 0.1;
  // Use a responsibility of one third, so that agent_2 has 3/4 responsibility
  // and agent_1 has 1/4 responsibility.
  archipelago.agent_options.reached_destination_avoidance_responsibility =
    1.0 / 3.0;

  // 35 was chosen by just running until the second agent crosses y=0 (roughly).
  // This is probably easy to break, but I couldn't think of another way to get
  // the right value here...
  for _ in 0..35 {
    archipelago.update(0.1);
    // Update the velocities to match the desired velocities.
    let agent_1 = archipelago.get_agent_mut(agent_1).unwrap();
    agent_1.velocity = *agent_1.get_desired_velocity();
    agent_1.position += agent_1.velocity * 0.1;
    dbg!(agent_1.position);
    let agent_2 = archipelago.get_agent_mut(agent_2).unwrap();
    agent_2.velocity = *agent_2.get_desired_velocity();
    agent_2.position += agent_2.velocity * 0.1;
    dbg!(agent_2.position);
  }

  let agent_1 = archipelago.get_agent(agent_1).unwrap();
  let agent_2 = archipelago.get_agent(agent_2).unwrap();

  // Since agent_1 takes 1/4 responsibility, it moves away by 0.25.
  assert!(
    (agent_1.position.x - 0.25) < 0.01,
    "left={}, right={}",
    agent_1.position.x,
    0.25
  );
  // Since agent_2 takes 3/4 responsibility, it moves away by 0.75.
  assert!(
    (agent_2.position.x - 0.75) < 0.01,
    "left={}, right={}",
    agent_2.position.x,
    0.75
  );
}
