use std::collections::HashMap;

use glam::Vec3;
use kdtree::{distance::squared_euclidean, KdTree};

use crate::{nav_mesh::MeshNodeRef, Agent, AgentId, AgentOptions};

pub(crate) fn apply_avoidance_to_agents(
  agents: &mut HashMap<AgentId, Agent>,
  agent_id_to_agent_node: &HashMap<AgentId, (Vec3, MeshNodeRef)>,
  agent_options: &AgentOptions,
  delta_time: f32,
) {
  let mut agent_id_to_dodgy_agent = HashMap::new();
  let mut agent_kdtree = KdTree::new(/* dimensions= */ 3);
  let mut agent_max_radius = 0.0f32;

  for (agent_id, agent) in agents.iter() {
    let agent_point = match agent_id_to_agent_node.get(agent_id) {
      None => continue,
      Some(agent_point_and_node) => agent_point_and_node.0,
    };

    agent_id_to_dodgy_agent.insert(
      *agent_id,
      dodgy::Agent {
        position: glam::Vec2::new(agent_point.x, agent_point.z),
        velocity: glam::Vec2::new(agent.velocity.x, agent.velocity.z),
        radius: agent.radius,
        max_velocity: agent.max_velocity,
        avoidance_responsibility: 1.0,
      },
    );
    agent_kdtree
      .add([agent_point.x, agent_point.y, agent_point.z], *agent_id)
      .expect("Agent point is finite");
    agent_max_radius = agent_max_radius.max(agent.radius);
  }

  for (agent_id, agent) in agents.iter_mut() {
    let agent_point = match agent_id_to_agent_node.get(agent_id) {
      None => continue,
      Some(agent_node) => agent_node.0,
    };
    let nearby_agents = agent_kdtree
      .within(
        &[agent_point.x, agent_point.y, agent_point.z],
        agent_max_radius + agent_options.neighbourhood,
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

        if distance < agent_options.neighbourhood + dodgy_agent.radius {
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
      agent_options.avoidance_time_horizon,
      agent_options.obstacle_avoidance_time_horizon,
      delta_time,
    );

    agent.current_desired_move =
      glam::Vec3::new(desired_move.x, 0.0, desired_move.y);
  }
}
