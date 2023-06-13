mod agent;
mod archipelago_pathfinding;
mod nav_mesh;
mod path;
mod pathfinding;
mod util;

use glam::Vec3;
use kdtree::{distance::squared_euclidean, KdTree};
use rand::Rng;
use std::collections::HashMap;

use nav_mesh::MeshNodeRef;

pub use agent::{Agent, AgentId, TargetReachedCondition};
pub use nav_mesh::{NavigationMesh, ValidNavigationMesh};
pub use util::BoundingBox;

pub struct Archipelago {
  pub agent_options: AgentOptions,
  nav_data: NavigationData,
  agents: HashMap<AgentId, Agent>,
}

struct NavigationData {
  nav_mesh: ValidNavigationMesh,
}

// Options that apply to all agents
pub struct AgentOptions {
  // The distance that an agent will consider avoiding another agent.
  pub neighbourhood: f32,
  // The time into the future that collisions with other agents should be
  // avoided.
  pub avoidance_time_horizon: f32,
  // The time into the future that collisions with obstacles should be avoided.
  pub obstacle_avoidance_time_horizon: f32,
}

impl Default for AgentOptions {
  fn default() -> Self {
    Self {
      neighbourhood: 5.0,
      avoidance_time_horizon: 1.0,
      obstacle_avoidance_time_horizon: 0.5,
    }
  }
}

impl Archipelago {
  pub fn create_from_navigation_mesh(
    navigation_mesh: ValidNavigationMesh,
  ) -> Self {
    Self {
      nav_data: NavigationData { nav_mesh: navigation_mesh },
      agent_options: AgentOptions::default(),
      agents: HashMap::new(),
    }
  }

  pub fn add_agent(&mut self, agent: Agent) -> AgentId {
    let mut rng = rand::thread_rng();

    let agent_id: AgentId = rng.gen();
    assert!(self.agents.insert(agent_id, agent).is_none());

    agent_id
  }

  pub fn remove_agent(&mut self, agent_id: AgentId) {
    self
      .agents
      .remove(&agent_id)
      .expect("Agent should be present in the archipelago");
  }

  pub fn get_agent(&self, agent_id: AgentId) -> &Agent {
    self.agents.get(&agent_id).unwrap()
  }

  pub fn get_agent_mut(&mut self, agent_id: AgentId) -> &mut Agent {
    self.agents.get_mut(&agent_id).unwrap()
  }

  pub fn get_agent_ids(&self) -> impl ExactSizeIterator<Item = AgentId> + '_ {
    self.agents.keys().copied()
  }

  pub fn update(&mut self, delta_time: f32) {
    const NODE_DISTANCE: f32 = 0.1;

    let mut agent_id_to_agent_node = HashMap::new();
    let mut agent_id_to_target_node = HashMap::new();

    let mut agent_id_to_dodgy_agent = HashMap::new();
    let mut agent_kdtree = KdTree::new(/* dimensions= */ 3);
    let mut agent_max_radius = 0.0f32;

    for (agent_id, agent) in self.agents.iter() {
      let agent_node_and_point = match self
        .nav_data
        .nav_mesh
        .sample_point(agent.position, NODE_DISTANCE)
      {
        None => continue,
        Some(node_and_point) => node_and_point,
      };
      let inserted = agent_id_to_agent_node
        .insert(*agent_id, agent_node_and_point.clone())
        .is_none();
      debug_assert!(inserted);

      agent_id_to_dodgy_agent.insert(
        *agent_id,
        dodgy::Agent {
          position: glam::Vec2::new(
            agent_node_and_point.0.x,
            agent_node_and_point.0.y,
          ),
          velocity: glam::Vec2::new(agent.velocity.x, agent.velocity.y),
          radius: agent.radius,
          max_velocity: agent.max_velocity,
          avoidance_responsibility: 1.0,
        },
      );
      agent_kdtree
        .add(
          [
            agent_node_and_point.0.x,
            agent_node_and_point.0.y,
            agent_node_and_point.0.z,
          ],
          *agent_id,
        )
        .expect("Agent point is finite");
      agent_max_radius = agent_max_radius.max(agent.radius);

      if let Some(target) = agent.current_target {
        let target_node_and_point =
          match self.nav_data.nav_mesh.sample_point(target, NODE_DISTANCE) {
            None => continue,
            Some(node_and_point) => node_and_point,
          };

        let inserted = agent_id_to_target_node
          .insert(*agent_id, target_node_and_point)
          .is_none();
        debug_assert!(inserted);
      }
    }

    let mut agent_id_to_follow_path_indices = HashMap::new();

    for (agent_id, agent) in self.agents.iter_mut() {
      let agent_node = agent_id_to_agent_node
        .get(agent_id)
        .map(|node_and_point| node_and_point.1.clone());
      let target_node = agent_id_to_target_node
        .get(agent_id)
        .map(|node_and_point| node_and_point.1.clone());
      match does_agent_need_repath(
        agent,
        agent_node.clone(),
        target_node.clone(),
      ) {
        RepathResult::DoNothing => {}
        RepathResult::FollowPath(
          agent_node_in_corridor,
          target_node_in_corridor,
        ) => {
          agent_id_to_follow_path_indices.insert(
            *agent_id,
            (agent_node_in_corridor, target_node_in_corridor),
          );
        }
        RepathResult::ClearPath => agent.current_path = None,
        RepathResult::NeedsRepath => {
          agent.current_path = None;

          let new_path = match archipelago_pathfinding::find_path(
            &self.nav_data,
            agent_node.unwrap(),
            target_node.unwrap(),
          ) {
            Err(_) => continue,
            Ok(archipelago_pathfinding::PathResult { path, .. }) => path,
          };

          agent_id_to_follow_path_indices
            .insert(*agent_id, (0, new_path.corridor.len() - 1));
          agent.current_path = Some(new_path);
        }
      }
    }

    for (agent_id, agent) in self.agents.iter_mut() {
      let path = match &agent.current_path {
        None => {
          agent.current_desired_move = Vec3::ZERO;
          continue;
        }
        Some(path) => path,
      };

      let agent_point = agent_id_to_agent_node
        .get(agent_id)
        .expect("Agent has a path, so should have a valid start node")
        .0;
      let target_point = agent_id_to_target_node
        .get(agent_id)
        .expect("Agent has a path, so should have a valid target node")
        .0;

      let &(agent_node_index_in_corridor, target_node_index_in_corridor) =
        agent_id_to_follow_path_indices.get(agent_id).expect(
          "Any agent with a path must have its follow path indices filled out.",
        );

      let next_waypoint = path.find_next_point_in_straight_path(
        &self.nav_data,
        agent_node_index_in_corridor,
        agent_point,
        target_node_index_in_corridor,
        target_point,
      );

      if agent.has_reached_target(
        path,
        &self.nav_data,
        next_waypoint,
        (target_node_index_in_corridor, target_point),
      ) {
        agent.current_desired_move = Vec3::ZERO;
      } else {
        agent.current_desired_move = (next_waypoint.1 - agent.position)
          .normalize_or_zero()
          * agent.max_velocity;
      }
    }

    for (agent_id, agent) in self.agents.iter_mut() {
      let agent_point = match agent_id_to_agent_node.get(agent_id) {
        None => continue,
        Some(agent_node) => agent_node.0,
      };
      let nearby_agents = agent_kdtree
        .within(
          &[agent_point.x, agent_point.y, agent_point.z],
          agent_max_radius + self.agent_options.neighbourhood,
          &squared_euclidean,
        )
        .unwrap();

      let nearby_agents = nearby_agents
        .iter()
        .filter_map(|&(distance, neighbour_id)| {
          if neighbour_id == agent_id {
            return None;
          }

          let dodgy_agent = agent_id_to_dodgy_agent.get(neighbour_id).unwrap();

          if distance < self.agent_options.neighbourhood + dodgy_agent.radius {
            Some(dodgy_agent)
          } else {
            None
          }
        })
        .collect::<Vec<_>>();

      let dodgy_agent = agent_id_to_dodgy_agent.get(agent_id).unwrap();
      let desired_move = dodgy_agent.compute_avoiding_velocity(
        &nearby_agents,
        &[],
        glam::Vec2::new(
          agent.current_desired_move.x,
          agent.current_desired_move.z,
        ),
        self.agent_options.avoidance_time_horizon,
        self.agent_options.obstacle_avoidance_time_horizon,
        delta_time,
      );

      agent.current_desired_move =
        glam::Vec3::new(desired_move.x, 0.0, desired_move.y);
    }
  }
}

enum RepathResult {
  DoNothing,
  FollowPath(usize, usize),
  ClearPath,
  NeedsRepath,
}

fn does_agent_need_repath(
  agent: &Agent,
  agent_node: Option<MeshNodeRef>,
  target_node: Option<MeshNodeRef>,
) -> RepathResult {
  if let None = agent.current_target {
    if agent.current_path.is_some() {
      return RepathResult::ClearPath;
    } else {
      return RepathResult::DoNothing;
    }
  }

  let agent_node = match agent_node {
    None => return RepathResult::ClearPath,
    Some(result) => result,
  };
  let target_node = match target_node {
    None => return RepathResult::ClearPath,
    Some(result) => result,
  };

  let current_path = match &agent.current_path {
    None => return RepathResult::NeedsRepath,
    Some(current_path) => current_path,
  };

  let agent_node_index_in_corridor =
    match current_path.corridor.iter().position(|x| x == &agent_node) {
      None => return RepathResult::NeedsRepath,
      Some(index) => index,
    };

  let target_node_index_in_corridor =
    match current_path.corridor.iter().rev().position(|x| x == &target_node) {
      None => return RepathResult::NeedsRepath,
      Some(index) => current_path.corridor.len() - 1 - index,
    };

  match agent_node_index_in_corridor <= target_node_index_in_corridor {
    true => RepathResult::FollowPath(
      agent_node_index_in_corridor,
      target_node_index_in_corridor,
    ),
    false => RepathResult::NeedsRepath,
  }
}
