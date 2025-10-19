use std::collections::{BinaryHeap, HashMap, HashSet};

use dodgy_2d::VisibilitySet;
use glam::{Vec3, Vec3Swizzles};
use kdtree::{KdTree, distance::squared_euclidean};
use slotmap::HopSlotMap;

use crate::{
  Agent, AgentId, AgentState, ArchipelagoOptions, CharacterId,
  CoordinateSystem, IslandId, NavigationData,
  character::CoreCharacter,
  island::CoreIsland,
  nav_data::{KindedOffMeshLink, ModifiedNode, NodeRef},
};

/// Adjusts the velocity of `agents` to apply local avoidance. `delta_time` must
/// be positive.
pub(crate) fn apply_avoidance_to_agents<CS: CoordinateSystem>(
  agents: &mut HopSlotMap<AgentId, Agent<CS>>,
  agent_id_to_agent_node: &HashMap<AgentId, (Vec3, NodeRef)>,
  characters: &HopSlotMap<CharacterId, CoreCharacter>,
  character_id_to_nav_mesh_point: &HashMap<CharacterId, Vec3>,
  nav_data: &NavigationData,
  agent_options: &ArchipelagoOptions<CS>,
  mut delta_time: f32,
) {
  if delta_time == 0.0 {
    // Make sure that our delta time is positive. Since the delta time is 0.0,
    // the desired velocity doesn't really matter.
    delta_time = 1.0;
  }

  let mut agent_id_to_dodgy_agent = HashMap::new();
  let mut agent_kdtree = KdTree::new(/* dimensions= */ 3);
  let mut agent_max_radius = 0.0f32;

  for (agent_id, agent) in agents.iter() {
    let agent_point = match agent_id_to_agent_node.get(&agent_id) {
      None => continue,
      Some(agent_point_and_node) => agent_point_and_node.0,
    };

    agent_id_to_dodgy_agent.insert(
      agent_id,
      dodgy_2d::Agent {
        position: to_dodgy_vec2(agent_point.xy()),
        velocity: to_dodgy_vec2(CS::to_landmass(&agent.velocity).xy()),
        radius: agent.radius,
        avoidance_responsibility: if agent.state == AgentState::ReachedTarget {
          agent_options.reached_destination_avoidance_responsibility
        } else {
          1.0
        },
      },
    );
    agent_kdtree
      .add([agent_point.x, agent_point.y, agent_point.z], agent_id)
      .expect("Agent point is finite");
    agent_max_radius = agent_max_radius.max(agent.radius);
  }

  let mut character_kdtree = KdTree::new(/* dimensions= */ 3);
  for (character_id, character) in characters.iter() {
    let Some(character_point) =
      character_id_to_nav_mesh_point.get(&character_id)
    else {
      continue;
    };
    character_kdtree
      .add(
        [character_point.x, character_point.y, character_point.z],
        dodgy_2d::Agent {
          position: to_dodgy_vec2(character_point.xy()),
          velocity: to_dodgy_vec2(character.velocity.xy()),
          radius: character.radius,
          // Characters are not responsible for any avoidance since landmass has
          // no control over them.
          avoidance_responsibility: 0.0,
        },
      )
      .expect("Character point is finite");
  }

  let neighbourhood = agent_max_radius + agent_options.neighbourhood;
  let neighbourhood_squared = neighbourhood * neighbourhood;
  for (agent_id, agent) in agents.iter_mut() {
    let agent_node = match agent_id_to_agent_node.get(&agent_id) {
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
    let nearby_characters = character_kdtree
      .within(
        &[agent_point.x, agent_point.y, agent_point.z],
        neighbourhood_squared,
        &squared_euclidean,
      )
      .unwrap();

    let nearby_agents = nearby_agents
      .iter()
      .filter_map(|&(distance_squared, neighbour_id)| {
        if *neighbour_id == agent_id {
          return None;
        }

        let dodgy_agent = agent_id_to_dodgy_agent.get(neighbour_id).unwrap();

        let neighbourhood = agent_options.neighbourhood + dodgy_agent.radius;
        if distance_squared < neighbourhood * neighbourhood {
          Some(std::borrow::Cow::Borrowed(dodgy_agent))
        } else {
          None
        }
      })
      .chain(nearby_characters.iter().filter_map(
        |&(distance_squared, dodgy_agent)| {
          if distance_squared < neighbourhood_squared {
            Some(std::borrow::Cow::Borrowed(dodgy_agent))
          } else {
            None
          }
        },
      ))
      .collect::<Vec<_>>();

    let mut nearby_obstacles = nav_mesh_borders_to_dodgy_obstacles(
      *agent_node,
      nav_data,
      agent_options.neighbourhood,
    );
    let nearby_obstacles = nearby_obstacles
      .drain(..)
      .map(std::borrow::Cow::Owned)
      .collect::<Vec<_>>();
    let preferred_velocity =
      to_dodgy_vec2(CS::to_landmass(&agent.current_desired_move).xy());
    let avoidance_options = dodgy_2d::AvoidanceOptions {
      // Always use an avoidance margin of zero since we assume the nav mesh
      // is the "valid" region.
      obstacle_margin: 0.0,
      time_horizon: agent_options.avoidance_time_horizon,
      obstacle_time_horizon: agent_options.obstacle_avoidance_time_horizon,
    };

    let dodgy_agent = agent_id_to_dodgy_agent.get(&agent_id).unwrap();
    #[cfg(not(feature = "debug-avoidance"))]
    let desired_move = dodgy_agent.compute_avoiding_velocity(
      &nearby_agents,
      &nearby_obstacles,
      preferred_velocity,
      agent.max_speed,
      delta_time,
      &avoidance_options,
    );
    #[cfg(feature = "debug-avoidance")]
    let desired_move = {
      let (desired_move, debug_data) = dodgy_agent
        .compute_avoiding_velocity_with_debug(
          &nearby_agents,
          &nearby_obstacles,
          preferred_velocity,
          agent.max_speed,
          delta_time,
          &avoidance_options,
        );
      agent.avoidance_data = agent.keep_avoidance_data.then_some(debug_data);
      desired_move
    };

    agent.current_desired_move =
      CS::from_landmass(&glam::Vec3::new(desired_move.x, desired_move.y, 0.0));
  }
}

fn to_dodgy_vec2(v: glam::Vec2) -> dodgy_2d::Vec2 {
  dodgy_2d::Vec2 { x: v.x, y: v.y }
}

/// Computes the dodgy obstacles corresponding to the navigation mesh borders.
/// These obstacles are from the perspective of `agent_node` (to avoid problems
/// with obstacles above/below the agent). `distance_limit` is the distance from
/// the agent to include obstacles.
fn nav_mesh_borders_to_dodgy_obstacles(
  agent_node: (Vec3, NodeRef),
  nav_data: &NavigationData,
  distance_limit: f32,
) -> Vec<dodgy_2d::Obstacle> {
  let distance_limit = distance_limit * distance_limit;

  let mut visibility_set = VisibilitySet::new();
  let mut border_edges = HashMap::new();
  let mut new_vertices = Vec::new();

  struct ExploreNode {
    node: NodeRef,
    score: f32,
  }
  impl PartialEq for ExploreNode {
    fn eq(&self, other: &Self) -> bool {
      self.score == other.score
    }
  }
  impl Eq for ExploreNode {}
  // Since we are comparing floats which are not Ord, it is more meaningful to
  // impl PartialOrd, then unwrap in Ord.
  #[allow(clippy::non_canonical_partial_ord_impl)]
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
  next_nodes.push(ExploreNode { node: agent_node.1, score: 0.0 });

  let agent_point = agent_node.0.xy();

  fn vertex_index_to_dodgy_vec(
    island: &CoreIsland,
    index: usize,
    relative_point: glam::Vec2,
  ) -> dodgy_2d::Vec2 {
    to_dodgy_vec2(
      island.transform.apply(island.nav_mesh.vertices[index]).xy()
        - relative_point,
    )
  }

  while !next_nodes.is_empty() {
    let node = next_nodes.pop().unwrap().node;
    if !explored_nodes.insert(node) {
      // This node has already been explored. Skip to the next one.
      continue;
    }

    let island = nav_data.get_island(node.island_id).unwrap();

    let polygon = &island.nav_mesh.polygons[node.polygon_index];
    let off_mesh_links = nav_data.node_to_off_mesh_link_ids.get(&node);
    let modified_node = nav_data.modified_nodes.get(&node);

    let mut remaining_edges: HashSet<usize> =
      HashSet::from_iter(0..polygon.vertices.len());
    for (vertex_1, vertex_2, node_ref) in polygon
      .connectivity
      .iter()
      .enumerate()
      .filter_map(|(edge_index, conn)| {
        conn.as_ref().map(|conn| (edge_index, conn))
      })
      .map(|(edge_index, connectivity)| {
        remaining_edges.remove(&edge_index);

        let (vertex_1, vertex_2) = polygon.get_edge_indices(edge_index);
        (
          // dodgy represents the inside of an obstacle as counter-clockwise,
          // but we actually want to treat the "solid" part as the outside of
          // the node. So reverse the order of the vertices so we get a
          // clockwise polygon.
          vertex_index_to_dodgy_vec(island, vertex_2, agent_point),
          vertex_index_to_dodgy_vec(island, vertex_1, agent_point),
          NodeRef {
            island_id: node.island_id,
            polygon_index: connectivity.polygon_index,
          },
        )
      })
      .chain(off_mesh_links.iter().flat_map(|off_mesh_links| {
        off_mesh_links
          .iter()
          .map(|link_id| nav_data.off_mesh_links.get(*link_id).unwrap())
          // Only boundary links should be considered for visibility.
          .filter(|link| {
            matches!(&link.kinded, KindedOffMeshLink::BoundaryLink { .. })
          })
          .map(|link| {
            (
              // dodgy represents the inside of an obstacle as
              // counter-clockwise, but we actually want to treat the "solid"
              // part as the outside of the node. So reverse the order of the
              // vertices so we get a clockwise polygon.
              to_dodgy_vec2(link.portal.1.xy() - agent_point),
              to_dodgy_vec2(link.portal.0.xy() - agent_point),
              link.destination_node,
            )
          })
      }))
    {
      if !visibility_set.is_line_visible(vertex_1, vertex_2) {
        continue;
      }

      let node = ExploreNode {
        node: node_ref,
        score: vertex_1.length_squared().min(vertex_2.length_squared()),
      };

      if node.score < distance_limit {
        next_nodes.push(node);
      }
    }

    if let Some(modified_node) = modified_node {
      for &(left, right) in modified_node.new_boundary.iter() {
        fn index_to_vertex_and_index(
          index: usize,
          island_id: IslandId,
          agent_point: glam::Vec2,
          modified_node: &ModifiedNode,
          island: &CoreIsland,
          new_vertices: &mut Vec<dodgy_2d::Vec2>,
        ) -> (dodgy_2d::Vec2, (Option<IslandId>, usize)) {
          if index >= island.nav_mesh.vertices.len() {
            let new_vertex = modified_node.new_vertices
              [index - island.nav_mesh.vertices.len()];
            let new_index = new_vertices.len();
            new_vertices.push(to_dodgy_vec2(new_vertex));
            return (
              to_dodgy_vec2(new_vertex - agent_point),
              (None, new_index),
            );
          }

          let vertex = vertex_index_to_dodgy_vec(island, index, agent_point);
          (vertex, (Some(island_id), index))
        }

        let (left_point, left_index) = index_to_vertex_and_index(
          left,
          node.island_id,
          agent_point,
          modified_node,
          island,
          &mut new_vertices,
        );
        let (right_point, right_index) = index_to_vertex_and_index(
          right,
          node.island_id,
          agent_point,
          modified_node,
          island,
          &mut new_vertices,
        );

        if let Some(line_index) =
          visibility_set.add_line(left_point, right_point)
        {
          border_edges.insert(line_index, (left_index, right_index));
        }
      }
    } else {
      for border_edge in remaining_edges {
        // dodgy represents the inside of an obstacle as counter-clockwise,
        // but we actually want to treat the "solid" part as the outside of
        // the node. So reverse the order of the vertices so we get a
        // clockwise polygon.
        let (border_vertex_2, border_vertex_1) =
          polygon.get_edge_indices(border_edge);
        let (vertex_1, vertex_2) = (
          vertex_index_to_dodgy_vec(island, border_vertex_1, agent_point),
          vertex_index_to_dodgy_vec(island, border_vertex_2, agent_point),
        );

        if let Some(line_index) = visibility_set.add_line(vertex_1, vertex_2) {
          border_edges.insert(
            line_index,
            (
              (Some(node.island_id), border_vertex_1),
              (Some(node.island_id), border_vertex_2),
            ),
          );
        }
      }
    }
  }

  let mut finished_loops = Vec::new();
  let mut unfinished_loops = Vec::<Vec<(Option<IslandId>, usize)>>::new();
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

  let island_and_vertex_index_to_dodgy_vec =
    |(island_id, index)| match island_id {
      None => new_vertices[index],
      Some(island_id) => {
        let island_data = nav_data.get_island(island_id).unwrap();
        vertex_index_to_dodgy_vec(island_data, index, glam::Vec2::ZERO)
      }
    };

  finished_loops
    .drain(..)
    .map(|looop| dodgy_2d::Obstacle::Closed {
      vertices: looop
        .iter()
        .rev()
        .copied()
        .map(island_and_vertex_index_to_dodgy_vec)
        .collect(),
    })
    .chain(unfinished_loops.drain(..).map(|looop| {
      dodgy_2d::Obstacle::Open {
        vertices: looop
          .iter()
          .rev()
          .copied()
          .map(island_and_vertex_index_to_dodgy_vec)
          .collect(),
      }
    }))
    .collect()
}

#[cfg(test)]
#[path = "avoidance_test.rs"]
mod test;
