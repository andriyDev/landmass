use thiserror::Error;

use crate::{
  AgentId, Archipelago, CoordinateSystem, CoreAgent,
  coords::CorePointSampleDistance,
  island::CoreIsland,
  link::AnimationLinkId,
  nav_data::{KindedOffMeshLink, NodeRef},
  nav_mesh::MeshEdgeRef,
  path::{Path, StraightPathStep},
};

#[cfg(feature = "debug-avoidance")]
use glam::Vec2;

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
  /// An edge of a triangle in the detail mesh.
  HeightEdge,
  /// A link between two islands along their boundary edge.
  BoundaryLink,
  /// The start edge of an animation link.
  AnimationLinkStart(AnimationLinkId),
  /// The end edge of an animation link.
  AnimationLinkEnd(AnimationLinkId),
  /// The connection between the start and end edge of the animation link.
  AnimationLinkConnection(AnimationLinkId),
  /// Part of an agent's current path. The corridor follows the path along
  /// nodes, not the actual path the agent will travel.
  AgentCorridor(AgentId),
  /// Part of an agent's current path that uses an animation link. The corridor
  /// follows the path along nodes, not the actual path the agent will travel.
  CorridorAnimationLink(AgentId, AnimationLinkId),
  /// Line from an agent to its target.
  Target(AgentId),
  /// Line to the waypoint of an agent.
  Waypoint(AgentId),
  /// The estimated line of travel along an animation link that the agent will
  /// take on its path next. While [`LineType::CorridorAnimationLink`] refers to
  /// the corridor, this is a more accurate line that relates to the current
  /// position of the agent.
  PathAnimationLink(AgentId, AnimationLinkId),
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
pub enum DebugDrawError {
  #[error(
    "The navigation data of the archipelago has been mutated since the last update."
  )]
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
    island: &CoreIsland,
  ) -> CS::Coordinate {
    CS::from_landmass(&island.transform.apply(island.nav_mesh.vertices[index]))
  }

  for island_id in archipelago.get_island_ids() {
    let island = archipelago.get_island(island_id).unwrap().island;
    assert!(
      !island.dirty,
      "Drawing an archipelago while things are dirty is unsafe! Update the archipelago first."
    );
    for (polygon_index, polygon) in island.nav_mesh.polygons.iter().enumerate()
    {
      let center_point = island.transform.apply(polygon.center);
      for i in 0..polygon.vertices.len() {
        let j = (i + 1) % polygon.vertices.len();

        let i = polygon.vertices[i];
        let j = polygon.vertices[j];

        debug_drawer.add_triangle(
          TriangleType::Node,
          [
            index_to_vertex::<CS>(i, island),
            index_to_vertex::<CS>(j, island),
            CS::from_landmass(&center_point),
          ],
        );
      }

      if let Some(height_mesh) = island.nav_mesh.height_mesh.as_ref() {
        let height_polygon = &height_mesh.polygons[polygon_index];

        for triangle_index in height_polygon.triangle_range() {
          let [a, b, c] = height_mesh.triangles[triangle_index];
          let triangle = (
            CS::from_landmass(
              &island
                .transform
                .apply(height_mesh.vertices[height_polygon.vertex(a)]),
            ),
            CS::from_landmass(
              &island
                .transform
                .apply(height_mesh.vertices[height_polygon.vertex(b)]),
            ),
            CS::from_landmass(
              &island
                .transform
                .apply(height_mesh.vertices[height_polygon.vertex(c)]),
            ),
          );
          debug_drawer.add_line(
            LineType::HeightEdge,
            [triangle.0.clone(), triangle.1.clone()],
          );
          debug_drawer.add_line(
            LineType::HeightEdge,
            [triangle.1.clone(), triangle.2.clone()],
          );
          debug_drawer.add_line(
            LineType::HeightEdge,
            [triangle.2.clone(), triangle.0.clone()],
          );
        }
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
          [index_to_vertex::<CS>(i, island), index_to_vertex::<CS>(j, island)],
        );
      }

      let node_ref = NodeRef { island_id, polygon_index };
      if let Some(off_mesh_link_ids) =
        archipelago.nav_data.node_to_off_mesh_link_ids.get(&node_ref)
      {
        for &off_mesh_link_id in off_mesh_link_ids.iter() {
          let off_mesh_link = archipelago
            .nav_data
            .off_mesh_links
            .get(off_mesh_link_id)
            .expect("Boundary links are present.");

          match &off_mesh_link.kinded {
            KindedOffMeshLink::BoundaryLink { .. } => {
              // Ignore links where the connected node has a greater node_ref.
              // This prevents drawing the same link multiple times by
              // picking one of the links to draw.
              if node_ref > off_mesh_link.destination_node {
                continue;
              }
              debug_drawer.add_line(
                LineType::BoundaryLink,
                [
                  CS::from_landmass(&off_mesh_link.portal.0),
                  CS::from_landmass(&off_mesh_link.portal.1),
                ],
              );
            }
            KindedOffMeshLink::AnimationLink {
              destination_portal,
              animation_link,
              ..
            } => {
              debug_drawer.add_line(
                LineType::AnimationLinkStart(*animation_link),
                [
                  CS::from_landmass(&off_mesh_link.portal.0),
                  CS::from_landmass(&off_mesh_link.portal.1),
                ],
              );
              debug_drawer.add_line(
                LineType::AnimationLinkEnd(*animation_link),
                [
                  CS::from_landmass(&destination_portal.0),
                  CS::from_landmass(&destination_portal.1),
                ],
              );
              let start_midpoint =
                off_mesh_link.portal.0.midpoint(off_mesh_link.portal.1);
              let end_midpoint =
                destination_portal.0.midpoint(destination_portal.1);
              debug_drawer.add_line(
                LineType::AnimationLinkConnection(*animation_link),
                [
                  CS::from_landmass(&start_midpoint),
                  CS::from_landmass(&end_midpoint),
                ],
              );
            }
          }
        }
      }
    }
  }

  for (agent_id, agent) in archipelago.agents.iter() {
    if agent.paused || agent.using_animation_link {
      // Don't render paused agents.
      continue;
    }

    let agent_position = CS::from_landmass(&agent.position);
    debug_drawer
      .add_point(PointType::AgentPosition(agent_id), agent_position.clone());
    if let Some(target) = &agent.current_target {
      let target = CS::from_landmass(&target);
      debug_drawer
        .add_line(LineType::Target(agent_id), [agent_position, target.clone()]);
      debug_drawer.add_point(PointType::TargetPosition(agent_id), target);
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
  agent: &CoreAgent,
  archipelago: &Archipelago<CS>,
  debug_drawer: &mut impl DebugDrawer<CS>,
) {
  let target = agent
    .current_target
    .clone()
    .expect("The path is valid, so the target is valid.");

  let mut last_point = CS::from_landmass(&path.start_point);

  for (segment_index, island_segment) in path.island_segments.iter().enumerate()
  {
    let island = archipelago
      .nav_data
      .get_island(island_segment.island_id)
      .expect("Island in corridor should be valid");
    for (&polygon_index, &edge_index) in island_segment
      .corridor
      .iter()
      .zip(island_segment.portal_edge_index.iter())
    {
      let (left, right) = island
        .nav_mesh
        .get_edge_points(MeshEdgeRef { polygon_index, edge_index });

      let next_point =
        CS::from_landmass(&island.transform.apply(left.midpoint(right)));
      debug_drawer.add_line(
        LineType::AgentCorridor(agent_id),
        [last_point.clone(), next_point.clone()],
      );
      last_point = next_point;
    }

    if segment_index >= path.off_mesh_link_segments.len() {
      debug_drawer.add_line(
        LineType::AgentCorridor(agent_id),
        [last_point.clone(), CS::from_landmass(&path.end_point)],
      );
    } else {
      let link_segment = &path.off_mesh_link_segments[segment_index];
      let link =
        &archipelago.nav_data.off_mesh_links[link_segment.off_mesh_link];
      let (left, right) = link.portal;

      let next_point = CS::from_landmass(&left.midpoint(right));
      debug_drawer.add_line(
        LineType::AgentCorridor(agent_id),
        [last_point.clone(), next_point.clone()],
      );

      match &link.kinded {
        KindedOffMeshLink::BoundaryLink { .. } => {
          last_point = next_point;
        }
        KindedOffMeshLink::AnimationLink {
          destination_portal,
          animation_link,
          ..
        } => {
          let destination_point = CS::from_landmass(
            &destination_portal.0.midpoint(destination_portal.1),
          );
          debug_drawer.add_line(
            LineType::CorridorAnimationLink(agent_id, *animation_link),
            [next_point.clone(), destination_point.clone()],
          );
          last_point = destination_point;
        }
      }
    }
  }

  let (agent_sample_point, agent_node_ref) = archipelago
    .nav_data
    .sample_point(
      agent.position,
      &CorePointSampleDistance::new(
        &archipelago.archipelago_options.point_sample_distance,
      ),
    )
    .expect("Path exists, so sampling the agent should be fine.");
  let (target_sample_point, target_node_ref) = archipelago
    .nav_data
    .sample_point(
      target,
      &CorePointSampleDistance::new(
        &archipelago.archipelago_options.point_sample_distance,
      ),
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
  // Convert the path step into the point the agent is walking towards.
  let waypoint = match waypoint {
    StraightPathStep::Waypoint(point) => point,
    StraightPathStep::AnimationLink {
      start_point, end_point, link_id, ..
    } => {
      debug_drawer.add_line(
        LineType::PathAnimationLink(agent_id, link_id),
        [CS::from_landmass(&start_point), CS::from_landmass(&end_point)],
      );
      start_point
    }
  };
  debug_drawer.add_line(
    LineType::Waypoint(agent_id),
    [CS::from_landmass(&agent.position), CS::from_landmass(&waypoint)],
  );
  debug_drawer
    .add_point(PointType::Waypoint(agent_id), CS::from_landmass(&waypoint));
}

#[cfg(feature = "debug-avoidance")]
/// A constraint in velocity-space for an agent's velocity for local collision
/// avoidance. The constraint restricts the velocity to lie on one side of a
/// line (aka., only a half-plane is considered valid).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConstraintLine {
  /// A point on the line separating the valid and invalid velocities.
  pub point: Vec2,
  /// The normal of the line separating the valid and invalid velocities. The
  /// normal always points towards the valid velocities.
  pub normal: Vec2,
}

#[cfg(feature = "debug-avoidance")]
/// The kinds of constraint during avoidance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ConstraintKind {
  /// The constraints from the original algorithm. This is either just all the
  /// constraints if the algorithm succeeded, or it is the constraints that
  /// failed, and there are also fallback constraints.
  Original,
  /// The constraints after the algorithm has fallen back to ensure a valid
  /// avoidance direction.
  Fallback,
}

#[cfg(feature = "debug-avoidance")]
/// A trait for reporting agent local collision avoidance constraints.
pub trait AvoidanceDrawer {
  /// Reports a single avoidance constraint.
  fn add_constraint(
    &mut self,
    agent: AgentId,
    constraint: ConstraintLine,
    kind: ConstraintKind,
  );
}

#[cfg(feature = "debug-avoidance")]
impl ConstraintLine {
  fn from_dodgy(line: &dodgy_2d::debug::Line) -> Self {
    Self {
      point: Vec2::new(line.point.x, line.point.y),
      normal: Vec2::new(-line.direction.y, line.direction.x),
    }
  }
}

#[cfg(feature = "debug-avoidance")]
pub fn draw_avoidance_data<CS: CoordinateSystem>(
  archipelago: &Archipelago<CS>,
  avoidance_drawer: &mut impl AvoidanceDrawer,
) {
  for (agent_id, agent) in archipelago.agents.iter() {
    let Some(avoidance_data) = agent.avoidance_data.as_ref() else {
      continue;
    };

    let (original_constraints, fallback_constraints) = match avoidance_data {
      dodgy_2d::debug::DebugData::Satisfied { constraints } => {
        (constraints.as_slice(), [].as_slice())
      }
      dodgy_2d::debug::DebugData::Fallback {
        original_constraints,
        fallback_constraints,
      } => (original_constraints.as_slice(), fallback_constraints.as_slice()),
    };

    for constraint in original_constraints {
      avoidance_drawer.add_constraint(
        agent_id,
        ConstraintLine::from_dodgy(constraint),
        ConstraintKind::Original,
      );
    }

    for constraint in fallback_constraints {
      avoidance_drawer.add_constraint(
        agent_id,
        ConstraintLine::from_dodgy(constraint),
        ConstraintKind::Fallback,
      );
    }
  }
}

#[cfg(test)]
#[path = "debug_test.rs"]
mod test;
