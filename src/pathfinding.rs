use std::{borrow::Cow, collections::HashMap};

use glam::Vec3;

use crate::{
  astar::{self, AStarProblem, PathStats},
  nav_data::{BoundaryLinkId, NodeRef},
  path::{BoundaryLinkSegment, IslandSegment, Path},
  NavigationData,
};

/// A concrete A* problem specifically for [`crate::Archipelago`]s.
struct ArchipelagoPathProblem<'a> {
  /// The navigation data to search.
  nav_data: &'a NavigationData,
  /// The node the agent is starting from.
  start_node: NodeRef,
  /// The node the target is in.
  end_node: NodeRef,
  /// The center of the end_node. This is just a cached point for easy access.
  end_point: Vec3,
}

/// An action taken in the path.
#[derive(Clone, Copy)]
enum PathStep {
  /// Take the node connection at the specified index in the current node.
  NodeConnection(usize),
  /// Take the boundary link with the specified ID in the current node.
  BoundaryLink(BoundaryLinkId),
}

impl AStarProblem for ArchipelagoPathProblem<'_> {
  type ActionType = PathStep;

  type StateType = NodeRef;

  fn initial_state(&self) -> Self::StateType {
    self.start_node.clone()
  }

  fn successors(
    &self,
    state: &Self::StateType,
  ) -> Vec<(f32, Self::ActionType, Self::StateType)> {
    let island = self.nav_data.islands.get(&state.island_id).unwrap();
    let nav_data = island.nav_data.as_ref().unwrap();
    let polygon = &nav_data.nav_mesh.polygons[state.polygon_index];
    let connectivity = &nav_data.nav_mesh.connectivity[state.polygon_index];
    let boundary_links = self
      .nav_data
      .boundary_links
      .get(state)
      .map_or(Cow::Owned(HashMap::new()), |links| Cow::Borrowed(links));

    connectivity
      .iter()
      .enumerate()
      .map(|(conn_index, conn)| {
        let next_polygon = &nav_data.nav_mesh.polygons[conn.polygon_index];
        let edge = polygon.get_edge_indices(conn.edge_index);
        let edge_point = (nav_data.nav_mesh.vertices[edge.0]
          + nav_data.nav_mesh.vertices[edge.1])
          / 2.0;
        let cost = polygon.center.distance(edge_point)
          + next_polygon.center.distance(edge_point);

        (
          cost,
          PathStep::NodeConnection(conn_index),
          NodeRef {
            island_id: state.island_id,
            polygon_index: conn.polygon_index,
          },
        )
      })
      .chain(boundary_links.iter().map(|(&link_id, link)| {
        let next_island =
          self.nav_data.islands.get(&link.destination_node.island_id).unwrap();
        let next_nav_data = next_island.nav_data.as_ref().unwrap();
        let next_polygon =
          &next_nav_data.nav_mesh.polygons[link.destination_node.polygon_index];

        let polygon_center = nav_data.transform.apply(polygon.center);
        let next_polygon_center =
          next_nav_data.transform.apply(next_polygon.center);

        let edge_point = (link.portal.0 + link.portal.1) / 2.0;

        let cost = polygon_center.distance(edge_point)
          + next_polygon_center.distance(edge_point);
        (cost, PathStep::BoundaryLink(link_id), link.destination_node)
      }))
      .collect()
  }

  fn heuristic(&self, state: &Self::StateType) -> f32 {
    let island_nav_data = self
      .nav_data
      .islands
      .get(&state.island_id)
      .unwrap()
      .nav_data
      .as_ref()
      .unwrap();
    island_nav_data
      .transform
      .apply(island_nav_data.nav_mesh.polygons[state.polygon_index].center)
      .distance(self.end_point)
  }

  fn is_goal_state(&self, state: &Self::StateType) -> bool {
    *state == self.end_node
  }
}

/// The results of pathfinding.
pub(crate) struct PathResult {
  /// Statistics about the pathfinding process.
  pub(crate) stats: PathStats,
  /// The resulting path.
  pub(crate) path: Path,
}

/// Finds a path in `nav_data` from `start_node` to `end_node`. Returns an `Err`
/// if no path was found.
pub(crate) fn find_path(
  nav_data: &NavigationData,
  start_node: NodeRef,
  end_node: NodeRef,
) -> Result<PathResult, PathStats> {
  let path_problem = ArchipelagoPathProblem {
    nav_data,
    start_node: start_node.clone(),
    end_node,
    end_point: {
      let island_nav_data = nav_data
        .islands
        .get(&end_node.island_id)
        .unwrap()
        .nav_data
        .as_ref()
        .unwrap();
      island_nav_data
        .transform
        .apply(island_nav_data.nav_mesh.polygons[end_node.polygon_index].center)
    },
  };

  let path_result = astar::find_path(&path_problem)?;

  let mut output_path =
    Path { island_segments: vec![], boundary_link_segments: vec![] };

  output_path.island_segments.push(IslandSegment {
    island_id: start_node.island_id,
    corridor: vec![start_node.polygon_index],
    portal_edge_index: vec![],
  });

  for path_step in path_result.path {
    let last_segment = output_path.island_segments.last_mut().unwrap();

    let previous_node = *last_segment.corridor.last().unwrap();

    match path_step {
      PathStep::NodeConnection(conn_index) => {
        let nav_mesh = &nav_data
          .islands
          .get(&last_segment.island_id)
          .unwrap()
          .nav_data
          .as_ref()
          .unwrap()
          .nav_mesh;
        let connectivity = &nav_mesh.connectivity[previous_node][conn_index];
        last_segment.corridor.push(connectivity.polygon_index);
        last_segment.portal_edge_index.push(connectivity.edge_index);
      }
      PathStep::BoundaryLink(boundary_link) => {
        let previous_node = NodeRef {
          island_id: last_segment.island_id,
          polygon_index: previous_node,
        };

        output_path.boundary_link_segments.push(BoundaryLinkSegment {
          starting_node: previous_node,
          boundary_link,
        });

        let boundary_links =
          nav_data.boundary_links.get(&previous_node).unwrap();

        let boundary_link = boundary_links.get(&boundary_link).unwrap();
        output_path.island_segments.push(IslandSegment {
          island_id: boundary_link.destination_node.island_id,
          corridor: vec![boundary_link.destination_node.polygon_index],
          portal_edge_index: vec![],
        });
      }
    }
  }

  Ok(PathResult { stats: path_result.stats, path: output_path })
}

#[cfg(test)]
#[path = "pathfinding_test.rs"]
mod test;
