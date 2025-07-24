use std::{
  borrow::Cow,
  collections::{HashMap, HashSet},
};

use glam::Vec3;
use ord_subset::OrdVar;

use crate::{
  astar::{self, AStarProblem, PathStats},
  nav_data::{BoundaryLinkId, NodeRef},
  path::{BoundaryLinkSegment, IslandSegment, Path},
  CoordinateSystem, Island, NavigationData, NodeType,
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
  /// The cheapest node type cost in [`Self::nav_data`]. This is cached once
  /// since it is constant for the whole problem.
  cheapest_node_type_cost: f32,
  /// Replacement costs for the `nav_data.node_type_to_cost`.
  override_node_type_to_cost: &'a HashMap<NodeType, f32>,
}

/// An action taken in the path.
#[derive(Clone, Copy)]
enum PathStep {
  /// Take the node connection at the specified edge index in the current node.
  NodeConnection(usize),
  /// Take the boundary link with the specified ID in the current node.
  BoundaryLink(BoundaryLinkId),
}

impl<CS: CoordinateSystem> ArchipelagoPathProblem<'_, CS> {
  /// Determines the cost of the node type corresponding to `type_index` in
  /// `island`.
  fn type_index_to_cost(&self, island: &Island<CS>, type_index: usize) -> f32 {
    self.node_type_to_cost(
      island.type_index_to_node_type.get(&type_index).copied(),
    )
  }

  /// Returns the cost associated with `node_type`. Returns 1.0 if the node_type
  /// is unset.
  fn node_type_to_cost(&self, node_type: Option<NodeType>) -> f32 {
    let Some(node_type) = node_type else {
      return 1.0;
    };
    self.override_node_type_to_cost.get(&node_type).copied().unwrap_or_else(
      || {
        self
          .nav_data
          .get_node_type_cost(node_type)
          .expect("The node type is valid in the NavigationData.")
      },
    )
  }
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
    let island = self.nav_data.get_island(state.island_id).unwrap();
    let polygon = &island.nav_mesh.polygons[state.polygon_index];
    let boundary_links = self
      .nav_data
      .node_to_boundary_link_ids
      .get(state)
      .map_or(Cow::Owned(HashSet::new()), Cow::Borrowed);

    let current_node_cost = self.type_index_to_cost(island, polygon.type_index);

    polygon
      .connectivity
      .iter()
      .enumerate()
      .filter_map(|(edge_index, conn)| {
        conn.as_ref().map(|conn| (edge_index, conn))
      })
      .filter_map(|(edge_index, conn)| {
        let target_node_cost = self.type_index_to_cost(
          island,
          island.nav_mesh.polygons[conn.polygon_index].type_index,
        );
        if !target_node_cost.is_finite() {
          return None;
        }

        let cost = todo!();

        Some((
          cost,
          PathStep::NodeConnection(edge_index),
          NodeRef {
            island_id: state.island_id,
            polygon_index: conn.polygon_index,
          },
        ))
      })
      .chain(boundary_links.iter().filter_map(|link_id| {
        let link = self.nav_data.boundary_links.get(*link_id).unwrap();
        let destination_node_cost =
          self.node_type_to_cost(link.destination_node_type);
        if !destination_node_cost.is_finite() {
          return None;
        }
        let cost = todo!();
        Some((cost, PathStep::BoundaryLink(*link_id), link.destination_node))
      }))
      .collect()
  }

  fn heuristic(&self, state: &Self::StateType) -> f32 {
    let island = self.nav_data.get_island(state.island_id).unwrap();
    island
      .transform
      .apply(island.nav_mesh.polygons[state.polygon_index].center)
      .distance(self.end_point)
      * self.cheapest_node_type_cost
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

/// Finds a path in `nav_data` from `start_node` to `end_node`. Node costs are
/// overriden with `override_node_type_to_cost`. Returns an `Err` if no path was
/// found.
pub(crate) fn find_path<CS: CoordinateSystem>(
  nav_data: &NavigationData<CS>,
  start_node: NodeRef,
  end_node: NodeRef,
  override_node_type_to_cost: &HashMap<NodeType, f32>,
) -> PathResult {
  if !nav_data.are_nodes_connected(start_node, end_node) {
    return PathResult { stats: PathStats { explored_nodes: 0 }, path: None };
  }

  let path_problem = ArchipelagoPathProblem {
    nav_data,
    start_node,
    end_node,
    end_point: {
      let island = nav_data.get_island(end_node.island_id).unwrap();
      island
        .transform
        .apply(island.nav_mesh.polygons[end_node.polygon_index].center)
    },
    cheapest_node_type_cost: *nav_data
      .get_node_types()
      .map(|(node_type, cost)| {
        (
          node_type,
          // Replace any node types with their overriden value, but only if it
          // was overriden.
          override_node_type_to_cost.get(&node_type).copied().unwrap_or(cost),
        )
      })
      .filter(|pair| pair.1.is_finite())
      .map(|pair| OrdVar::new_unchecked(pair.1))
      .chain(std::iter::once(OrdVar::new_unchecked(1.0)))
      .min()
      .unwrap(),
    override_node_type_to_cost,
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
