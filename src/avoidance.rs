use std::collections::{BinaryHeap, HashMap, HashSet};

use dodgy::VisibilitySet;
use glam::{Vec3, Vec3Swizzles};
use kdtree::{distance::squared_euclidean, KdTree};

use crate::{nav_data::NodeRef, Agent, AgentId, AgentOptions, NavigationData};

/// Adjusts the velocity of `agents` to apply local avoidance. `delta_time` must
/// be positive.
pub(crate) fn apply_avoidance_to_agents(
  agents: &mut HashMap<AgentId, Agent>,
  agent_id_to_agent_node: &HashMap<AgentId, (Vec3, NodeRef)>,
  nav_data: &NavigationData,
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
        position: to_dodgy_vec2(agent_point.xz()),
        velocity: to_dodgy_vec2(agent.velocity.xz()),
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

  let neighbourhood = agent_max_radius + agent_options.neighbourhood;
  let neighbourhood_squared = neighbourhood * neighbourhood;
  for (agent_id, agent) in agents.iter_mut() {
    let agent_node = match agent_id_to_agent_node.get(agent_id) {
      None => continue,
      Some(agent_node) => agent_node,
    };
    let agent_point = agent_node.0;
    let nearby_agents = agent_kdtree
      .within(
        &[agent_point.x, agent_point.y, agent_point.z],
        neighbourhood_squared,
        &squared_euclidean,
      )
      .unwrap();

    let nearby_agents = nearby_agents
      .iter()
      .filter_map(|&(distance_squared, neighbour_id)| {
        if neighbour_id == agent_id {
          return None;
        }

        let dodgy_agent = agent_id_to_dodgy_agent.get(neighbour_id).unwrap();

        let neighbourhood = agent_options.neighbourhood + dodgy_agent.radius;
        if distance_squared < neighbourhood * neighbourhood {
          Some(dodgy_agent)
        } else {
          None
        }
      })
      .collect::<Vec<_>>();

    let nearby_obstacles = nav_mesh_borders_to_dodgy_obstacles(
      agent_node.clone(),
      nav_data,
      agent_options.neighbourhood,
    );

    let dodgy_agent = agent_id_to_dodgy_agent.get(agent_id).unwrap();
    let desired_move = dodgy_agent.compute_avoiding_velocity(
      &nearby_agents,
      &nearby_obstacles.iter().map(|obstacle| obstacle).collect::<Vec<_>>(),
      to_dodgy_vec2(agent.current_desired_move.xz()),
      delta_time,
      &dodgy::AvoidanceOptions {
        obstacle_margin: agent_options.obstacle_avoidance_margin,
        time_horizon: agent_options.avoidance_time_horizon,
        obstacle_time_horizon: agent_options.obstacle_avoidance_time_horizon,
      },
    );

    agent.current_desired_move =
      glam::Vec3::new(desired_move.x, 0.0, desired_move.y);
  }
}

fn to_dodgy_vec2(v: glam::Vec2) -> dodgy::Vec2 {
  dodgy::Vec2 { x: v.x, y: v.y }
}

/// Computes the dodgy obstacles corresponding to the navigation mesh borders.
/// These obstacles are from the perspective of `agent_node` (to avoid problems
/// with obstacles above/below the agent). `distance_limit` is the distance from
/// the agent to include obstacles.
fn nav_mesh_borders_to_dodgy_obstacles(
  agent_node: (Vec3, NodeRef),
  nav_data: &NavigationData,
  distance_limit: f32,
) -> Vec<dodgy::Obstacle> {
  let distance_limit = distance_limit * distance_limit;

  let mut visibility_set = VisibilitySet::new();
  let mut border_edges = HashMap::new();

  struct ExploreNode {
    node: usize,
    score: f32,
  }
  impl PartialEq for ExploreNode {
    fn eq(&self, other: &Self) -> bool {
      self.score == other.score
    }
  }
  impl Eq for ExploreNode {}
  impl PartialOrd for ExploreNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
      other.score.partial_cmp(&self.score)
    }
  }
  impl Ord for ExploreNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
      self.partial_cmp(other).unwrap()
    }
  }

  let mut explored_nodes = HashSet::new();
  let mut next_nodes = BinaryHeap::new();
  next_nodes.push(ExploreNode { node: agent_node.1.polygon_index, score: 0.0 });

  let island_data = nav_data
    .islands
    .get(&agent_node.1.island_id)
    .unwrap()
    .nav_data
    .as_ref()
    .unwrap();

  while !next_nodes.is_empty() {
    let node = next_nodes.pop().unwrap().node;
    if !explored_nodes.insert(node) {
      // This node has already been explored. Skip to the next one.
      continue;
    }

    let polygon = &island_data.nav_mesh.polygons[node];
    let connectivity = &island_data.nav_mesh.connectivity[node];

    let mut remaining_edges: HashSet<usize> =
      HashSet::from_iter(0..polygon.vertices.len());
    for connectivity in connectivity.iter() {
      remaining_edges.remove(&connectivity.edge_index);

      let (vertex_1, vertex_2) =
        polygon.get_edge_indices(connectivity.edge_index);
      let (vertex_1, vertex_2) = (
        island_data.nav_mesh.vertices[vertex_1].xz(),
        island_data.nav_mesh.vertices[vertex_2].xz(),
      );
      let (vertex_1, vertex_2) = (
        to_dodgy_vec2(vertex_1 - agent_node.0.xz()),
        to_dodgy_vec2(vertex_2 - agent_node.0.xz()),
      );

      if !visibility_set.is_line_visible(vertex_1, vertex_2) {
        continue;
      }

      let node = ExploreNode {
        node: connectivity.polygon_index,
        score: vertex_1.length_squared().min(vertex_2.length_squared()),
      };

      if node.score < distance_limit {
        next_nodes.push(node);
      }
    }

    for border_edge in remaining_edges {
      let (border_vertex_1, border_vertex_2) =
        polygon.get_edge_indices(border_edge);
      let (vertex_1, vertex_2) = (
        island_data.nav_mesh.vertices[border_vertex_1].xz(),
        island_data.nav_mesh.vertices[border_vertex_2].xz(),
      );
      let (vertex_1, vertex_2) = (
        to_dodgy_vec2(vertex_1 - agent_node.0.xz()),
        to_dodgy_vec2(vertex_2 - agent_node.0.xz()),
      );

      if let Some(line_index) = visibility_set.add_line(vertex_1, vertex_2) {
        border_edges.insert(line_index, (border_vertex_1, border_vertex_2));
      }
    }
  }

  let mut finished_loops = Vec::new();
  let mut unfinished_loops = Vec::<Vec<usize>>::new();
  for line_id in visibility_set.get_visible_line_ids() {
    let edge = border_edges.get(&line_id).unwrap();

    let mut left_loop = None;
    let mut right_loop = None;

    for (loop_index, edge_loop) in unfinished_loops.iter().enumerate() {
      if left_loop.is_none() && edge_loop[edge_loop.len() - 1] == edge.0 {
        left_loop = Some(loop_index);
        if right_loop.is_some() {
          break;
        }
      }
      if right_loop.is_none() && edge_loop[0] == edge.1 {
        right_loop = Some(loop_index);
        if left_loop.is_some() {
          break;
        }
      }
    }

    match (left_loop, right_loop) {
      (None, None) => {
        unfinished_loops.push(vec![edge.0, edge.1]);
      }
      (Some(left_loop), None) => {
        unfinished_loops[left_loop].push(edge.1);
      }
      (None, Some(right_loop)) => {
        unfinished_loops[right_loop].insert(0, edge.0);
      }
      (Some(left_loop_index), Some(right_loop_index)) => {
        if left_loop_index == right_loop_index {
          finished_loops.push(unfinished_loops.remove(left_loop_index));
        } else {
          let mut left_loop = unfinished_loops.swap_remove(left_loop_index);
          let mut right_loop = unfinished_loops.swap_remove(
            if right_loop_index == unfinished_loops.len() {
              left_loop_index
            } else {
              right_loop_index
            },
          );
          unfinished_loops
            .push(left_loop.drain(..).chain(right_loop.drain(..)).collect());
        }
      }
    }
  }

  finished_loops
    .drain(..)
    .map(|looop| dodgy::Obstacle::Closed {
      vertices: looop
        .iter()
        .rev()
        .map(|vert| island_data.nav_mesh.vertices[*vert].xz())
        .map(to_dodgy_vec2)
        .collect(),
    })
    .chain(unfinished_loops.drain(..).map(|looop| {
      dodgy::Obstacle::Open {
        vertices: looop
          .iter()
          .rev()
          .map(|vert| island_data.nav_mesh.vertices[*vert].xz())
          .map(to_dodgy_vec2)
          .collect(),
      }
    }))
    .collect()
}

#[cfg(test)]
#[path = "avoidance_test.rs"]
mod test;
