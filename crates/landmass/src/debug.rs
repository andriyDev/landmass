use crate::{
  island::IslandNavigationData, nav_data::NodeRef, path::Path, Agent, AgentId,
  Archipelago, CoordinateSystem,
};

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
  /// An edge of a node that is the boundary of a nav mesh.
  BoundaryEdge,
  /// An edge of a node that is connected to another node.
  ConnectivityEdge,
  /// A link between two islands along their boundary edge.
  BoundaryLink,
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
pub trait DebugDrawer<CS: CoordinateSystem> {
  fn add_point(&mut self, point_type: PointType, point: CS::Coordinate);
  fn add_line(&mut self, line_type: LineType, line: [CS::Coordinate; 2]);
  fn add_triangle(
    &mut self,
    triangle_type: TriangleType,
    triangle: [CS::Coordinate; 3],
  );
}

/// An error resulting from trying to debug draw an archipelago.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DebugDrawError {
  /// The navigation data of the archipelago has been mutated since the last
  /// update. Drawing requires an up-to-date archipelago.
  NavDataDirty,
}

/// Draws all parts of `archipelago` to `debug_drawer`.
pub fn draw_archipelago_debug<CS: CoordinateSystem>(
  archipelago: &Archipelago<CS>,
  debug_drawer: &mut impl DebugDrawer<CS>,
) -> Result<(), DebugDrawError> {
  if archipelago.nav_data.dirty {
    return Err(DebugDrawError::NavDataDirty);
  }

  fn index_to_vertex<CS: CoordinateSystem>(
    index: usize,
    nav_data: &IslandNavigationData<CS>,
  ) -> CS::Coordinate {
    CS::from_landmass(
      &nav_data.transform.apply(nav_data.nav_mesh.vertices[index]),
    )
  }

  for island_id in archipelago.get_island_ids() {
    let island = archipelago.get_island(island_id).unwrap();
    assert!(!island.dirty, "Drawing an archipelago while things are dirty is unsafe! Update the archipelago first.");
    let Some(island_nav_data) = island.nav_data.as_ref() else {
      continue;
    };

    for (polygon_index, polygon) in
      island_nav_data.nav_mesh.polygons.iter().enumerate()
    {
      let center_point = island_nav_data.transform.apply(polygon.center);
      for i in 0..polygon.vertices.len() {
        let j = (i + 1) % polygon.vertices.len();

        let i = polygon.vertices[i];
        let j = polygon.vertices[j];

        debug_drawer.add_triangle(
          TriangleType::Node,
          [
            index_to_vertex(i, island_nav_data),
            index_to_vertex(j, island_nav_data),
            CS::from_landmass(&center_point),
          ],
        );
      }

      for (edge_index, connection) in polygon.connectivity.iter().enumerate() {
        let line_type = match connection.as_ref() {
          None => LineType::BoundaryEdge,
          Some(connection) => {
            // Ignore connections where the connected polygon has a greater
            // index. This prevents drawing the same edge multiple
            // times by picking one of the edges to draw.
            if polygon_index > connection.polygon_index {
              continue;
            }
            LineType::ConnectivityEdge
          }
        };

        let i = edge_index;
        let j = (i + 1) % polygon.vertices.len();

        let i = polygon.vertices[i];
        let j = polygon.vertices[j];

        debug_drawer.add_line(
          line_type,
          [
            index_to_vertex(i, island_nav_data),
            index_to_vertex(j, island_nav_data),
          ],
        );
      }

      let node_ref = NodeRef { island_id, polygon_index };
      if let Some(boundary_link_ids) =
        archipelago.nav_data.node_to_boundary_link_ids.get(&node_ref)
      {
        for &boundary_link_id in boundary_link_ids.iter() {
          let boundary_link = archipelago
            .nav_data
            .boundary_links
            .get(boundary_link_id)
            .expect("Boundary links are present.");
          // Ignore links where the connected node has a greater node_ref. This
          // prevents drawing the same link multiple times by picking one of the
          // links to draw.
          if node_ref > boundary_link.destination_node {
            continue;
          }

          debug_drawer.add_line(
            LineType::BoundaryLink,
            [
              CS::from_landmass(&boundary_link.portal.0),
              CS::from_landmass(&boundary_link.portal.1),
            ],
          );
        }
      }
    }
  }

  for (agent_id, agent) in archipelago.agents.iter() {
    debug_drawer
      .add_point(PointType::AgentPosition(agent_id), agent.position.clone());
    if let Some(target) = &agent.current_target {
      debug_drawer.add_line(
        LineType::Target(agent_id),
        [agent.position.clone(), target.clone()],
      );
      debug_drawer
        .add_point(PointType::TargetPosition(agent_id), target.clone());
    }
    if let Some(path) = agent.current_path.as_ref() {
      draw_path(path, agent_id, agent, archipelago, debug_drawer);
    }
  }

  Ok(())
}

/// Draws `path` to `debug_drawer`. The path belongs to `agent` and both belong
/// to `archipelago`.
fn draw_path<CS: CoordinateSystem>(
  path: &Path,
  agent_id: AgentId,
  agent: &Agent<CS>,
  archipelago: &Archipelago<CS>,
  debug_drawer: &mut impl DebugDrawer<CS>,
) {
  let target = agent
    .current_target
    .clone()
    .expect("The path is valid, so the target is valid.");

  let corridor_points = path
    .island_segments
    .iter()
    .flat_map(|island_segment| {
      island_segment.corridor.iter().copied().map(|polygon_index| {
        let island = archipelago
          .nav_data
          .get_island(island_segment.island_id)
          .expect("Island in corridor should be valid.");
        let nav_data = island
          .nav_data
          .as_ref()
          .expect("Island nav data in corridor should be valid.");
        nav_data
          .transform
          .apply(nav_data.nav_mesh.polygons[polygon_index].center)
      })
    })
    .collect::<Vec<_>>();
  for pair in corridor_points.windows(2) {
    debug_drawer.add_line(
      LineType::AgentCorridor(agent_id),
      [CS::from_landmass(&pair[0]), CS::from_landmass(&pair[1])],
    );
  }

  let (agent_sample_point, agent_node_ref) = archipelago
    .nav_data
    .sample_point(
      CS::to_landmass(&agent.position),
      archipelago.agent_options.node_sample_distance,
    )
    .expect("Path exists, so sampling the agent should be fine.");
  let (target_sample_point, target_node_ref) = archipelago
    .nav_data
    .sample_point(
      CS::to_landmass(&target),
      archipelago.agent_options.node_sample_distance,
    )
    .expect("Path exists, so sampling the agent should be fine.");

  let agent_corridor_index = path
    .find_index_of_node(agent_node_ref)
    .expect("Path exists, so the agent's node must be in the corridor.");
  let target_corridor_index = path
    .find_index_of_node_rev(target_node_ref)
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
  debug_drawer.add_line(
    LineType::Waypoint(agent_id),
    [agent.position.clone(), CS::from_landmass(&waypoint)],
  );
  debug_drawer
    .add_point(PointType::Waypoint(agent_id), CS::from_landmass(&waypoint));
}

#[cfg(test)]
#[path = "debug_test.rs"]
mod test;
