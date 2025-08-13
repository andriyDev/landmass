use std::{
  borrow::Cow,
  collections::{HashMap, HashSet},
};

use glam::Vec3;

use crate::{
  CoordinateSystem, NavigationData,
  astar::{self, AStarProblem, PathStats},
  nav_data::{BoundaryLinkId, NodeRef},
  nav_mesh::MeshEdgeRef,
  path::{BoundaryLinkSegment, IslandSegment, Path},
  util::FloatOrd,
};

/// A concrete A* problem specifically for [`crate::Archipelago`]s.
struct ArchipelagoPathProblem<'a, CS: CoordinateSystem> {
  /// The navigation data to search.
  nav_data: &'a NavigationData<CS>,
  /// The node the agent is starting from.
  start_node: NodeRef,
  /// The center of the start_node. This is just a cached point for easy
  /// access.
  start_point: Vec3,
  /// The node the target is in.
  end_node: NodeRef,
  /// The center of the end_node. This is just a cached point for easy access.
  end_point: Vec3,
  /// The cheapest type index cost in [`Self::nav_data`]. This is cached once
  /// since it is constant for the whole problem.
  cheapest_type_index_cost: f32,
  /// Replacement costs for the `nav_data.type_index_to_cost`.
  override_type_index_to_cost: &'a HashMap<usize, f32>,
}

/// An action taken in the path.
#[derive(Clone, Copy)]
enum PathStep {
  /// Just head directly to the end. This is only valid when inside the end
  /// node.
  GoToEnd,
  /// Take the node connection at the specified edge index in the current node.
  NodeConnection(usize),
  /// Take the boundary link with the specified ID in the current node.
  BoundaryLink(BoundaryLinkId),
}

/// A node in the path. This generally corresponds to an edge in the navigation
/// data.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
enum PathNode {
  /// The start of the path.
  Start,
  /// The end of the path.
  End,
  /// An edge of a node in the navigation data.
  NodeEdge {
    /// The node that we are now in.
    node: NodeRef,
    /// The edge that we start at inside this node.
    start_edge: usize,
  },
  /// A boundary link.
  BoundaryLink(BoundaryLinkId),
}

impl<CS: CoordinateSystem> ArchipelagoPathProblem<'_, CS> {
  /// Determines the cost of `type_index`.
  fn type_index_to_cost(&self, type_index: usize) -> f32 {
    self.override_type_index_to_cost.get(&type_index).copied().unwrap_or_else(
      || self.nav_data.get_type_index_cost(type_index).unwrap_or(1.0),
    )
  }
}

impl<CS: CoordinateSystem> AStarProblem for ArchipelagoPathProblem<'_, CS> {
  type ActionType = PathStep;

  type StateType = PathNode;

  fn initial_state(&self) -> Self::StateType {
    PathNode::Start
  }

  fn successors(
    &self,
    state: &Self::StateType,
  ) -> Vec<(f32, Self::ActionType, Self::StateType)> {
    let (node_ref, island, polygon, point, ignore_step) = match state {
      PathNode::Start => {
        let island =
          self.nav_data.get_island(self.start_node.island_id).unwrap();
        let polygon = &island.nav_mesh.polygons[self.start_node.polygon_index];
        (self.start_node, island, polygon, self.start_point, None)
      }
      PathNode::NodeEdge { node, start_edge: edge } => {
        let island = self.nav_data.get_island(node.island_id).unwrap();
        let polygon = &island.nav_mesh.polygons[node.polygon_index];

        let (i, j) = polygon.get_edge_indices(*edge);
        let local_midpoint =
          island.nav_mesh.vertices[i].midpoint(island.nav_mesh.vertices[j]);

        (
          *node,
          island,
          polygon,
          island.transform.apply(local_midpoint),
          Some(PathStep::NodeConnection(*edge)),
        )
      }
      PathNode::BoundaryLink(link) => {
        let link = self.nav_data.boundary_links.get(*link).unwrap();
        let island =
          self.nav_data.get_island(link.destination_node.island_id).unwrap();
        let polygon =
          &island.nav_mesh.polygons[link.destination_node.polygon_index];

        (
          link.destination_node,
          island,
          polygon,
          link.portal.0.midpoint(link.portal.1),
          Some(PathStep::BoundaryLink(link.reverse_link)),
        )
      }
      PathNode::End => {
        unreachable!("we never need the successors of the goal node")
      }
    };
    let boundary_links = self
      .nav_data
      .node_to_boundary_link_ids
      .get(&node_ref)
      .map_or(Cow::Owned(HashSet::new()), Cow::Borrowed);

    let current_node_cost = self.type_index_to_cost(polygon.type_index);

    if node_ref == self.end_node {
      let cost = point.distance(self.end_point) * current_node_cost;
      return vec![(cost, PathStep::GoToEnd, PathNode::End)];
    }

    polygon
      .connectivity
      .iter()
      .enumerate()
      .filter_map(|(edge_index, conn)| {
        conn.as_ref().map(|conn| (edge_index, conn))
      })
      .filter_map(|(edge_index, conn)| {
        if let Some(PathStep::NodeConnection(ignore_edge)) = ignore_step
          && edge_index == ignore_edge
        {
          return None;
        }

        let target_node_cost = self.type_index_to_cost(
          island.nav_mesh.polygons[conn.polygon_index].type_index,
        );
        if !target_node_cost.is_finite() {
          return None;
        }

        let (i, j) = polygon.get_edge_indices(edge_index);
        let local_midpoint =
          island.nav_mesh.vertices[i].midpoint(island.nav_mesh.vertices[j]);
        let cost = point.distance(island.transform.apply(local_midpoint))
          * current_node_cost;

        Some((
          cost,
          PathStep::NodeConnection(edge_index),
          PathNode::NodeEdge {
            node: NodeRef {
              island_id: node_ref.island_id,
              polygon_index: conn.polygon_index,
            },
            start_edge: conn.reverse_edge,
          },
        ))
      })
      .chain(boundary_links.iter().filter_map(|link_id| {
        if let Some(PathStep::BoundaryLink(ignore_link)) = ignore_step
          && *link_id == ignore_link
        {
          return None;
        }

        let link = self.nav_data.boundary_links.get(*link_id).unwrap();
        let destination_node_cost =
          self.type_index_to_cost(link.destination_type_index);
        if !destination_node_cost.is_finite() {
          return None;
        }

        let cost = point.distance(link.portal.0.midpoint(link.portal.1))
          * current_node_cost;
        Some((
          cost,
          PathStep::BoundaryLink(*link_id),
          PathNode::BoundaryLink(*link_id),
        ))
      }))
      .collect()
  }

  fn heuristic(&self, state: &Self::StateType) -> f32 {
    let world_point = match state {
      PathNode::Start => self.start_point,
      PathNode::End => return 0.0,
      PathNode::NodeEdge { node, start_edge: edge } => {
        let island = self.nav_data.get_island(node.island_id).unwrap();
        let edge = island.get_nav_mesh().get_edge_points(MeshEdgeRef {
          polygon_index: node.polygon_index,
          edge_index: *edge,
        });
        island.transform.apply(edge.0.midpoint(edge.1))
      }
      PathNode::BoundaryLink(link) => {
        let boundary_link = self.nav_data.boundary_links.get(*link).unwrap();
        boundary_link.portal.0.midpoint(boundary_link.portal.1)
      }
    };
    world_point.distance(self.end_point) * self.cheapest_type_index_cost
  }

  fn is_goal_state(&self, state: &Self::StateType) -> bool {
    matches!(state, PathNode::End)
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

/// Finds a path in `nav_data` from `start_node` to `end_node`. Type index costs
/// are overriden with `override_type_index_to_cost`. Returns an `Err` if no
/// path was found. `start_point` and `end_point` are assumed to be in the
/// corresponding nodes, and in world space.
pub(crate) fn find_path<CS: CoordinateSystem>(
  nav_data: &NavigationData<CS>,
  start_node: NodeRef,
  start_point: Vec3,
  end_node: NodeRef,
  end_point: Vec3,
  override_type_index_to_cost: &HashMap<usize, f32>,
) -> PathResult {
  if !nav_data.are_nodes_connected(start_node, end_node) {
    return PathResult { stats: PathStats { explored_nodes: 0 }, path: None };
  }

  let path_problem = ArchipelagoPathProblem {
    nav_data,
    start_node,
    end_node,
    start_point,
    end_point,
    cheapest_type_index_cost: *nav_data
      .get_type_index_costs()
      .map(|(type_index, cost)| {
        (
          type_index,
          // Replace any type indices with their overriden value, but only if
          // it was overriden.
          override_type_index_to_cost.get(&type_index).copied().unwrap_or(cost),
        )
      })
      .filter(|pair| pair.1.is_finite())
      .map(|pair| FloatOrd(pair.1))
      .chain(std::iter::once(FloatOrd(1.0)))
      .min()
      .unwrap(),
    override_type_index_to_cost,
  };

  let path_result = astar::find_path(&path_problem);
  let Some(astar_path) = path_result.path else {
    return PathResult { stats: path_result.stats, path: None };
  };

  let mut output_path = Path {
    island_segments: vec![],
    boundary_link_segments: vec![],
    start_point,
    end_point,
  };

  output_path.island_segments.push(IslandSegment {
    island_id: start_node.island_id,
    corridor: vec![start_node.polygon_index],
    portal_edge_index: vec![],
  });

  for path_step in astar_path {
    let last_segment = output_path.island_segments.last_mut().unwrap();

    let previous_node = *last_segment.corridor.last().unwrap();

    match path_step {
      PathStep::GoToEnd => {
        // Do nothing. The previous step already inserted the end node, so
        // there's nothing left to do. We could possibly assert here that this
        // is the last step, but it's not easy to do that, so no need.
      }
      PathStep::NodeConnection(edge_index) => {
        let nav_mesh =
          &nav_data.get_island(last_segment.island_id).unwrap().nav_mesh;
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
