use std::{borrow::Cow, collections::HashSet};

use glam::Vec3;

use crate::{
  astar::{self, AStarProblem, PathStats},
  nav_data::{BoundaryLinkId, NodeRef},
  path::{BoundaryLinkSegment, IslandSegment, Path},
  CoordinateSystem, NavigationData,
};

/// A concrete A* problem specifically for [`crate::Archipelago`]s.
struct ArchipelagoPathProblem<'a, CS: CoordinateSystem> {
  /// The navigation data to search.
  nav_data: &'a NavigationData<CS>,
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
  /// Take the node connection at the specified edge index in the current node.
  NodeConnection(usize),
  /// Take the boundary link with the specified ID in the current node.
  BoundaryLink(BoundaryLinkId),
}

impl<CS: CoordinateSystem> AStarProblem for ArchipelagoPathProblem<'_, CS> {
  type ActionType = PathStep;

  type StateType = NodeRef;

  fn initial_state(&self) -> Self::StateType {
    self.start_node
  }

  fn successors(
    &self,
    state: &Self::StateType,
  ) -> Vec<(f32, Self::ActionType, Self::StateType)> {
    let island = self.nav_data.islands.get(state.island_id).unwrap();
    let nav_data = island.nav_data.as_ref().unwrap();
    let polygon = &nav_data.nav_mesh.polygons[state.polygon_index];
    let boundary_links = self
      .nav_data
      .node_to_boundary_link_ids
      .get(state)
      .map_or(Cow::Owned(HashSet::new()), Cow::Borrowed);

    polygon
      .connectivity
      .iter()
      .enumerate()
      .filter_map(|(edge_index, conn)| {
        conn.as_ref().map(|conn| (edge_index, conn))
      })
      .map(|(edge_index, conn)| {
        (
          conn.cost,
          PathStep::NodeConnection(edge_index),
          NodeRef {
            island_id: state.island_id,
            polygon_index: conn.polygon_index,
          },
        )
      })
      .chain(boundary_links.iter().map(|link_id| {
        let link = self.nav_data.boundary_links.get(*link_id).unwrap();
        (link.cost, PathStep::BoundaryLink(*link_id), link.destination_node)
      }))
      .collect()
  }

  fn heuristic(&self, state: &Self::StateType) -> f32 {
    let island_nav_data = self
      .nav_data
      .islands
      .get(state.island_id)
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
#[derive(Debug)]
pub(crate) struct PathResult {
  /// Statistics about the pathfinding process.
  pub(crate) stats: PathStats,
  /// The path if one was found.
  pub(crate) path: Option<Path>,
}

/// Finds a path in `nav_data` from `start_node` to `end_node`. Returns an `Err`
/// if no path was found.
pub(crate) fn find_path<CS: CoordinateSystem>(
  nav_data: &NavigationData<CS>,
  start_node: NodeRef,
  end_node: NodeRef,
) -> PathResult {
  if !nav_data.are_nodes_connected(start_node, end_node) {
    return PathResult { stats: PathStats { explored_nodes: 0 }, path: None };
  }

  let path_problem = ArchipelagoPathProblem {
    nav_data,
    start_node,
    end_node,
    end_point: {
      let island_nav_data = nav_data
        .islands
        .get(end_node.island_id)
        .unwrap()
        .nav_data
        .as_ref()
        .unwrap();
      island_nav_data
        .transform
        .apply(island_nav_data.nav_mesh.polygons[end_node.polygon_index].center)
    },
  };

  let path_result = astar::find_path(&path_problem);
  let Some(astar_path) = path_result.path else {
    return PathResult { stats: path_result.stats, path: None };
  };

  let mut output_path =
    Path { island_segments: vec![], boundary_link_segments: vec![] };

  output_path.island_segments.push(IslandSegment {
    island_id: start_node.island_id,
    corridor: vec![start_node.polygon_index],
    portal_edge_index: vec![],
  });

  for path_step in astar_path {
    let last_segment = output_path.island_segments.last_mut().unwrap();

    let previous_node = *last_segment.corridor.last().unwrap();

    match path_step {
      PathStep::NodeConnection(edge_index) => {
        let nav_mesh = &nav_data
          .islands
          .get(last_segment.island_id)
          .unwrap()
          .nav_data
          .as_ref()
          .unwrap()
          .nav_mesh;
        let connectivity = nav_mesh.polygons[previous_node].connectivity
          [edge_index]
          .as_ref()
          .unwrap();
        last_segment.corridor.push(connectivity.polygon_index);
        last_segment.portal_edge_index.push(edge_index);
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

        let boundary_link = nav_data.boundary_links.get(boundary_link).unwrap();
        output_path.island_segments.push(IslandSegment {
          island_id: boundary_link.destination_node.island_id,
          corridor: vec![boundary_link.destination_node.polygon_index],
          portal_edge_index: vec![],
        });
      }
    }
  }

  PathResult { stats: path_result.stats, path: Some(output_path) }
}

#[cfg(test)]
#[path = "pathfinding_test.rs"]
mod test;
