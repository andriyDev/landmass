use std::collections::{BinaryHeap, HashMap, HashSet};

use dodgy::VisibilitySet;
use glam::{Vec3, Vec3Swizzles};
use kdtree::{distance::squared_euclidean, KdTree};

use crate::{
  nav_mesh::MeshNodeRef, Agent, AgentId, AgentOptions, NavigationData,
};

pub(crate) fn apply_avoidance_to_agents(
  agents: &mut HashMap<AgentId, Agent>,
  agent_id_to_agent_node: &HashMap<AgentId, (Vec3, MeshNodeRef)>,
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
    let agent_node = match agent_id_to_agent_node.get(agent_id) {
      None => continue,
      Some(agent_node) => agent_node,
    };
    let agent_point = agent_node.0;
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

    let nearby_obstacles = nav_mesh_borders_to_dodgy_obstacles(
      agent_node.clone(),
      nav_data,
      agent_options.neighbourhood,
    );

    let dodgy_agent = agent_id_to_dodgy_agent.get(agent_id).unwrap();
    let desired_move = dodgy_agent.compute_avoiding_velocity(
      &nearby_agents,
      &nearby_obstacles.iter().map(|obstacle| obstacle).collect::<Vec<_>>(),
      glam::Vec2::new(
        agent.current_desired_move.x,
        agent.current_desired_move.z,
      ),
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

fn nav_mesh_borders_to_dodgy_obstacles(
  agent_node: (Vec3, MeshNodeRef),
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

  while !next_nodes.is_empty() {
    let node = next_nodes.pop().unwrap().node;
    if !explored_nodes.insert(node) {
      // This node has already been explored. Skip to the next one.
      continue;
    }

    let polygon = &nav_data.nav_mesh.polygons[node];
    let connectivity = &nav_data.nav_mesh.connectivity[node];

    let mut remaining_edges: HashSet<usize> =
      HashSet::from_iter(0..polygon.vertices.len());
    for connectivity in connectivity.iter() {
      remaining_edges.remove(&connectivity.edge_index);

      let (vertex_1, vertex_2) =
        polygon.get_edge_indices(connectivity.edge_index);
      let (vertex_1, vertex_2) = (
        nav_data.nav_mesh.vertices[vertex_1].xz(),
        nav_data.nav_mesh.vertices[vertex_2].xz(),
      );
      let (vertex_1, vertex_2) =
        (vertex_1 - agent_node.0.xz(), vertex_2 - agent_node.0.xz());

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
        nav_data.nav_mesh.vertices[border_vertex_1].xz(),
        nav_data.nav_mesh.vertices[border_vertex_2].xz(),
      );
      let (vertex_1, vertex_2) =
        (vertex_1 - agent_node.0.xz(), vertex_2 - agent_node.0.xz());

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
        .map(|vert| nav_data.nav_mesh.vertices[*vert].xz())
        .collect(),
    })
    .chain(unfinished_loops.drain(..).map(|looop| {
      dodgy::Obstacle::Open {
        vertices: looop
          .iter()
          .rev()
          .map(|vert| nav_data.nav_mesh.vertices[*vert].xz())
          .collect(),
      }
    }))
    .collect()
}

#[cfg(test)]
mod tests {
  use glam::{Vec2, Vec3};

  use crate::{nav_mesh::MeshNodeRef, NavigationData, NavigationMesh};

  use super::nav_mesh_borders_to_dodgy_obstacles;

  fn obstacle_matches(left: &dodgy::Obstacle, right: &dodgy::Obstacle) -> bool {
    match (left, right) {
      (
        dodgy::Obstacle::Closed { vertices: left_vertices },
        dodgy::Obstacle::Closed { vertices: right_vertices },
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
        dodgy::Obstacle::Open { vertices: left_vertices },
        dodgy::Obstacle::Open { vertices: right_vertices },
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
        panic!("Failed to match left obstacle: index={} obstacle={:?} left_obstacles={:?} remaining_obstacles={:?}",
          left_index, left_obstacle, left, right);
      }

      if !right.is_empty() {
        panic!("Failed to match right obstacles: left_obstacles={:?} remaining_obstacles={:?}", left, right);
      }
    }};
  }

  #[test]
  fn computes_obstacle_for_box() {
    let nav_data = NavigationData {
      nav_mesh: NavigationMesh {
        mesh_bounds: None,
        vertices: vec![
          Vec3::new(1.0, 0.0, 1.0),
          Vec3::new(2.0, 0.0, 1.0),
          Vec3::new(2.0, 0.0, 2.0),
          Vec3::new(1.0, 0.0, 2.0),
        ],
        polygons: vec![vec![0, 1, 2, 3]],
      }
      .validate()
      .expect("Validation succeeds"),
    };

    assert_obstacles_match!(
      nav_mesh_borders_to_dodgy_obstacles(
        (Vec3::new(1.5, 0.0, 1.5), MeshNodeRef { polygon_index: 0 }),
        &nav_data,
        /* distance_limit= */ 10.0,
      ),
      vec![dodgy::Obstacle::Closed {
        vertices: vec![
          Vec2::new(1.0, 1.0),
          Vec2::new(1.0, 2.0),
          Vec2::new(2.0, 2.0),
          Vec2::new(2.0, 1.0)
        ]
      }]
    );
  }

  #[test]
  fn dead_end_makes_open_obstacle() {
    let nav_data = NavigationData {
      nav_mesh: NavigationMesh {
        mesh_bounds: None,
        vertices: vec![
          Vec3::new(1.0, 0.0, 1.0),
          Vec3::new(2.0, 0.0, 1.0),
          Vec3::new(2.0, 0.0, 2.0),
          Vec3::new(1.0, 0.0, 2.0),
          Vec3::new(3.0, 0.0, 1.0),
          Vec3::new(3.0, 0.0, 2.0),
          Vec3::new(4.0, 0.0, 1.0),
          Vec3::new(4.0, 0.0, 2.0),
          Vec3::new(4.0, 0.0, 3.0),
          Vec3::new(3.0, 0.0, 3.0),
          Vec3::new(4.0, 0.0, 4.0),
          Vec3::new(3.0, 0.0, 4.0),
        ],
        polygons: vec![
          vec![0, 1, 2, 3],
          vec![2, 1, 4, 5],
          vec![5, 4, 6, 7],
          vec![5, 7, 8, 9],
          vec![9, 8, 10, 11],
        ],
      }
      .validate()
      .expect("Validation succeeds"),
    };

    assert_obstacles_match!(
      nav_mesh_borders_to_dodgy_obstacles(
        (Vec3::new(1.5, 0.0, 1.5), MeshNodeRef { polygon_index: 0 }),
        &nav_data,
        /* distance_limit= */ 10.0,
      ),
      vec![dodgy::Obstacle::Open {
        vertices: vec![
          Vec2::new(4.0, 3.0),
          Vec2::new(4.0, 2.0),
          Vec2::new(4.0, 1.0),
          Vec2::new(3.0, 1.0),
          Vec2::new(2.0, 1.0),
          Vec2::new(1.0, 1.0),
          Vec2::new(1.0, 2.0),
          Vec2::new(2.0, 2.0),
          Vec2::new(3.0, 2.0),
        ]
      }]
    );

    assert_obstacles_match!(
      nav_mesh_borders_to_dodgy_obstacles(
        (Vec3::new(3.5, 0.0, 3.5), MeshNodeRef { polygon_index: 4 }),
        &nav_data,
        /* distance_limit= */ 10.0,
      ),
      vec![dodgy::Obstacle::Open {
        vertices: vec![
          Vec2::new(3.0, 2.0),
          Vec2::new(3.0, 3.0),
          Vec2::new(3.0, 4.0),
          Vec2::new(4.0, 4.0),
          Vec2::new(4.0, 3.0),
          Vec2::new(4.0, 2.0),
          Vec2::new(4.0, 1.0),
          Vec2::new(3.0, 1.0),
          Vec2::new(2.0, 1.0),
        ]
      }]
    );

    // Decrease the distance limit to limit the size of the open obstacle.
    assert_obstacles_match!(
      nav_mesh_borders_to_dodgy_obstacles(
        (Vec3::new(3.5, 0.0, 3.5), MeshNodeRef { polygon_index: 4 }),
        &nav_data,
        /* distance_limit= */ 1.0,
      ),
      vec![dodgy::Obstacle::Open {
        vertices: vec![
          Vec2::new(3.0, 2.0),
          Vec2::new(3.0, 3.0),
          Vec2::new(3.0, 4.0),
          Vec2::new(4.0, 4.0),
          Vec2::new(4.0, 3.0),
          Vec2::new(4.0, 2.0),
        ]
      }]
    );

    assert_obstacles_match!(
      nav_mesh_borders_to_dodgy_obstacles(
        (Vec3::new(3.5, 0.0, 1.5), MeshNodeRef { polygon_index: 2 }),
        &nav_data,
        /* distance_limit= */ 10.0,
      ),
      vec![dodgy::Obstacle::Closed {
        vertices: vec![
          Vec2::new(1.0, 1.0),
          Vec2::new(1.0, 2.0),
          Vec2::new(2.0, 2.0),
          Vec2::new(3.0, 2.0),
          Vec2::new(3.0, 3.0),
          Vec2::new(3.0, 4.0),
          Vec2::new(4.0, 4.0),
          Vec2::new(4.0, 3.0),
          Vec2::new(4.0, 2.0),
          Vec2::new(4.0, 1.0),
          Vec2::new(3.0, 1.0),
          Vec2::new(2.0, 1.0),
        ]
      }]
    );
  }

  #[test]
  fn split_borders() {
    let nav_data = NavigationData {
      nav_mesh: NavigationMesh {
        mesh_bounds: None,
        vertices: vec![
          Vec3::new(1.0, 0.0, 1.0),
          Vec3::new(2.0, 0.0, 1.0),
          Vec3::new(3.0, 0.0, 1.0),
          Vec3::new(4.0, 0.0, 1.0),
          Vec3::new(5.0, 0.0, 1.0),
          Vec3::new(5.0, 0.0, 2.0),
          Vec3::new(5.0, 0.0, 3.0),
          Vec3::new(5.0, 0.0, 4.0),
          Vec3::new(6.0, 0.0, 4.0),
          Vec3::new(6.0, 0.0, 3.0),
          Vec3::new(6.0, 0.0, 2.0),
          Vec3::new(6.0, 0.0, 1.0),
          Vec3::new(6.0, 0.0, 0.0),
          Vec3::new(5.0, 0.0, 0.0),
          Vec3::new(4.0, 0.0, 0.0),
          Vec3::new(3.0, 0.0, 0.0),
          Vec3::new(2.0, 0.0, 0.0),
          Vec3::new(1.0, 0.0, 0.0),
          Vec3::new(0.0, 0.0, 0.0),
          Vec3::new(0.0, 0.0, 1.0),
          Vec3::new(0.0, 0.0, 2.0),
          Vec3::new(0.0, 0.0, 3.0),
          Vec3::new(0.0, 0.0, 4.0),
          Vec3::new(1.0, 0.0, 4.0),
          Vec3::new(1.0, 0.0, 3.0),
          Vec3::new(1.0, 0.0, 2.0),
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
      }
      .validate()
      .expect("Validation succeeds"),
    };

    assert_obstacles_match!(
      nav_mesh_borders_to_dodgy_obstacles(
        (Vec3::new(3.0, 0.0, 0.9), MeshNodeRef { polygon_index: 0 }),
        &nav_data,
        /* distance_limit= */ 10.0,
      ),
      vec![
        dodgy::Obstacle::Open {
          vertices: vec![
            Vec2::new(1.0, 1.0),
            Vec2::new(2.0, 1.0),
            Vec2::new(3.0, 1.0),
            Vec2::new(4.0, 1.0),
            Vec2::new(5.0, 1.0),
          ]
        },
        dodgy::Obstacle::Open {
          vertices: vec![
            Vec2::new(6.0, 2.0),
            Vec2::new(6.0, 1.0),
            Vec2::new(6.0, 0.0),
            Vec2::new(5.0, 0.0),
            Vec2::new(4.0, 0.0),
            Vec2::new(3.0, 0.0),
            Vec2::new(2.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(0.0, 0.0),
            Vec2::new(0.0, 1.0),
            Vec2::new(0.0, 2.0),
          ]
        }
      ]
    );
  }
}
