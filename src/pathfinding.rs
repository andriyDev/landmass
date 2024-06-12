use crate::{
  astar::{self, AStarProblem, PathStats},
  nav_data::NodeRef,
  path::{IslandSegment, Path},
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
}

impl AStarProblem for ArchipelagoPathProblem<'_> {
  type ActionType = usize;

  type StateType = NodeRef;

  fn initial_state(&self) -> Self::StateType {
    self.start_node.clone()
  }

  fn successors(
    &self,
    state: &Self::StateType,
  ) -> Vec<(f32, Self::ActionType, Self::StateType)> {
    let island = self.nav_data.islands.get(&state.island_id).unwrap();
    let nav_mesh = &island.nav_data.as_ref().unwrap().nav_mesh;
    let polygon = &nav_mesh.polygons[state.polygon_index];
    let connectivity = &nav_mesh.connectivity[state.polygon_index];

    connectivity
      .iter()
      .enumerate()
      .map(|(conn_index, conn)| {
        let next_polygon = &nav_mesh.polygons[conn.polygon_index];
        let edge = polygon.get_edge_indices(conn.edge_index);
        let edge_point =
          (nav_mesh.vertices[edge.0] + nav_mesh.vertices[edge.1]) / 2.0;
        let cost = polygon.center.distance(edge_point)
          + next_polygon.center.distance(edge_point);

        (
          cost,
          conn_index,
          NodeRef {
            island_id: state.island_id,
            polygon_index: conn.polygon_index,
          },
        )
      })
      .collect()
  }

  fn heuristic(&self, state: &Self::StateType) -> f32 {
    let island = self.nav_data.islands.get(&state.island_id).unwrap();
    let nav_mesh = &island.nav_data.as_ref().unwrap().nav_mesh;
    nav_mesh.polygons[state.polygon_index]
      .center
      .distance(nav_mesh.polygons[self.end_node.polygon_index].center)
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
  };

  let path_result = astar::find_path(&path_problem)?;

  let mut corridor = Vec::with_capacity(path_result.path.len() + 1);
  let mut portal_edge_index = Vec::with_capacity(path_result.path.len());
  corridor.push(start_node.polygon_index);

  for conn_index in path_result.path {
    let previous_node = *corridor.last().unwrap();
    let nav_mesh = &nav_data
      .islands
      .get(&start_node.island_id)
      .unwrap()
      .nav_data
      .as_ref()
      .unwrap()
      .nav_mesh;
    let connectivity = &nav_mesh.connectivity[previous_node][conn_index];
    portal_edge_index.push(connectivity.edge_index);
    corridor.push(connectivity.polygon_index);
  }

  Ok(PathResult {
    stats: path_result.stats,
    path: Path {
      island_segments: vec![IslandSegment {
        island_id: start_node.island_id,
        corridor,
        portal_edge_index,
      }],
      boundary_link_segments: vec![],
    },
  })
}

#[cfg(test)]
#[path = "pathfinding_test.rs"]
mod test;
