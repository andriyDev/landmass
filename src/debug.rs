use glam::Vec3;

use crate::{AgentId, Archipelago};

/// The type of debug points.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum PointType {
  /// The position of an agent.
  AgentPosition(AgentId),
  /// The target of an agent.
  TargetPosition(AgentId),
  /// The waypoint of an agent.
  Waypoint(AgentId),
}

/// The type of debug lines.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum LineType {
  /// An edge of a node that is the boundary of the nav mesh.
  BoundaryEdge,
  /// An edge of a node that is connected to another node.
  ConnectivityEdge,
  /// Part of an agent's current path. The corridor follows the path along
  /// nodes, not the actual path the agent will travel.
  AgentCorridor(AgentId),
  /// Line from an agent to its target.
  Target(AgentId),
  /// Line to the waypoint of an agent.
  Waypoint(AgentId),
}

/// The type of debug triangles.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum TriangleType {
  /// Part of a node/polygon in a nav mesh.
  Node,
}

/// Trait to "draw" Archipelago state to. Users should implement this to
/// visualize the state of their Archipelago.
pub trait DebugDrawer {
  fn add_point(&mut self, point_type: PointType, point: Vec3);
  fn add_line(&mut self, line_type: LineType, line: [Vec3; 2]);
  fn add_triangle(&mut self, triangle_type: TriangleType, triangle: [Vec3; 3]);
}

/// Draws all parts of `archipelago` to `debug_drawer`.
pub fn draw_archipelago_debug(
  archipelago: &Archipelago,
  debug_drawer: &mut impl DebugDrawer,
) {
  for island in archipelago.nav_data.islands.values() {
    if island.dirty {
      panic!("Drawing an archipelago while things are dirty is unsafe! Update the archipelago first.");
    }
    let Some(nav_data) = island.nav_data.as_ref() else {
      continue;
    };

    for (polygon_index, (polygon, connectivity)) in nav_data
      .nav_mesh
      .polygons
      .iter()
      .zip(nav_data.nav_mesh.connectivity.iter())
      .enumerate()
    {
      let center_point = nav_data.transform.apply(polygon.center);
      for i in 0..polygon.vertices.len() {
        let j = (i + 1) % polygon.vertices.len();

        let i = polygon.vertices[i];
        let j = polygon.vertices[j];

        debug_drawer.add_triangle(
          TriangleType::Node,
          [
            nav_data.transform.apply(nav_data.nav_mesh.vertices[i]),
            nav_data.transform.apply(nav_data.nav_mesh.vertices[j]),
            center_point,
          ],
        );
      }

      for connection in connectivity.iter() {
        // Ignore connections where the connected polygon has a greater index.
        // This prevents drawing the same edge multiple times by picking one of
        // the edges to draw.
        if polygon_index > connection.polygon_index {
          continue;
        }

        let i = connection.edge_index;
        let j = (i + 1) % polygon.vertices.len();

        let i = polygon.vertices[i];
        let j = polygon.vertices[j];

        debug_drawer.add_line(
          LineType::ConnectivityEdge,
          [
            nav_data.transform.apply(nav_data.nav_mesh.vertices[i]),
            nav_data.transform.apply(nav_data.nav_mesh.vertices[j]),
          ],
        );
      }
    }

    for boundary_edge in nav_data.nav_mesh.boundary_edges.iter() {
      let polygon_vertices =
        &nav_data.nav_mesh.polygons[boundary_edge.polygon_index].vertices;
      let i = boundary_edge.edge_index;
      let j = (i + 1) % polygon_vertices.len();

      let i = polygon_vertices[i];
      let j = polygon_vertices[j];

      debug_drawer.add_line(
        LineType::BoundaryEdge,
        [
          nav_data.transform.apply(nav_data.nav_mesh.vertices[i]),
          nav_data.transform.apply(nav_data.nav_mesh.vertices[j]),
        ],
      );
    }
  }

  for (&agent_id, agent) in archipelago.agents.iter() {
    debug_drawer.add_point(PointType::AgentPosition(agent_id), agent.position);
    if let Some(target) = agent.current_target {
      debug_drawer
        .add_line(LineType::Target(agent_id), [agent.position, target]);
      debug_drawer.add_point(PointType::TargetPosition(agent_id), target);
    }
    if let Some(path) = agent.current_path.as_ref() {
      let target = agent
        .current_target
        .expect("The path is valid, so the target is valid.");

      let corridor_points = path
        .corridor
        .iter()
        .map(|node_ref| {
          let island = archipelago
            .nav_data
            .islands
            .get(&node_ref.island_id)
            .expect("Island in corridor should be valid.");
          let nav_data = island
            .nav_data
            .as_ref()
            .expect("Island nav data in corridor should be valid.");
          nav_data
            .transform
            .apply(nav_data.nav_mesh.polygons[node_ref.polygon_index].center)
        })
        .collect::<Vec<_>>();
      for pair in corridor_points.windows(2) {
        debug_drawer
          .add_line(LineType::AgentCorridor(agent_id), [pair[0], pair[1]]);
      }

      let (agent_sample_point, agent_node_ref) = archipelago
        .nav_data
        .sample_point(
          agent.position,
          archipelago.agent_options.node_sample_distance,
        )
        .expect("Path exists, so sampling the agent should be fine.");
      let (target_sample_point, target_node_ref) = archipelago
        .nav_data
        .sample_point(target, archipelago.agent_options.node_sample_distance)
        .expect("Path exists, so sampling the agent should be fine.");

      let agent_corridor_index = path
        .corridor
        .iter()
        .position(|node_ref| *node_ref == agent_node_ref)
        .expect("Path exists, so the agent's node must be in the corridor.");
      let target_corridor_index = path
        .corridor
        .iter()
        .position(|node_ref| *node_ref == target_node_ref)
        .expect("Path exists, so the target's node must be in the corridor.");

      let waypoint = path
        .find_next_point_in_straight_path(
          &archipelago.nav_data,
          agent_corridor_index,
          agent_sample_point,
          target_corridor_index,
          target_sample_point,
        )
        .1;
      debug_drawer
        .add_line(LineType::Waypoint(agent_id), [agent.position, waypoint]);
      debug_drawer.add_point(PointType::Waypoint(agent_id), waypoint);
    }
  }
}

#[cfg(test)]
#[path = "debug_test.rs"]
mod test;
