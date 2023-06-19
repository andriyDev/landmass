use crate::{
  astar::{self, AStarProblem, PathStats},
  nav_mesh::MeshNodeRef,
  path::Path,
  NavigationData,
};

struct ArchipelagoPathProblem<'a> {
  nav_data: &'a NavigationData,
  start_node: MeshNodeRef,
  end_node: MeshNodeRef,
}

impl AStarProblem for ArchipelagoPathProblem<'_> {
  type ActionType = usize;

  type StateType = MeshNodeRef;

  fn initial_state(&self) -> Self::StateType {
    self.start_node.clone()
  }

  fn successors(
    &self,
    state: &Self::StateType,
  ) -> Vec<(f32, Self::ActionType, Self::StateType)> {
    let polygon = &self.nav_data.nav_mesh.polygons[state.polygon_index];
    let connectivity =
      &self.nav_data.nav_mesh.connectivity[state.polygon_index];

    connectivity
      .iter()
      .enumerate()
      .map(|(conn_index, conn)| {
        let next_polygon = &self.nav_data.nav_mesh.polygons[conn.polygon_index];
        let edge = polygon.get_edge_indices(conn.edge_index);
        let edge_point = (self.nav_data.nav_mesh.vertices[edge.0]
          + self.nav_data.nav_mesh.vertices[edge.1])
          / 2.0;
        let cost = polygon.center.distance(edge_point)
          + next_polygon.center.distance(edge_point);

        (cost, conn_index, MeshNodeRef { polygon_index: conn.polygon_index })
      })
      .collect()
  }

  fn heuristic(&self, state: &Self::StateType) -> f32 {
    self.nav_data.nav_mesh.polygons[state.polygon_index].center.distance(
      self.nav_data.nav_mesh.polygons[self.end_node.polygon_index].center,
    )
  }

  fn is_goal_state(&self, state: &Self::StateType) -> bool {
    *state == self.end_node
  }
}

pub(crate) struct PathResult {
  pub(crate) stats: PathStats,
  pub(crate) path: Path,
}

pub(crate) fn find_path(
  nav_data: &NavigationData,
  start_node: MeshNodeRef,
  end_node: MeshNodeRef,
) -> Result<PathResult, PathStats> {
  let path_problem = ArchipelagoPathProblem {
    nav_data,
    start_node: start_node.clone(),
    end_node,
  };

  let path_result = astar::find_path(&path_problem)?;

  let mut corridor = Vec::with_capacity(path_result.path.len() + 1);
  let mut portal_edge_index = Vec::with_capacity(path_result.path.len());
  corridor.push(start_node);

  for conn_index in path_result.path {
    let connectivity = &nav_data.nav_mesh.connectivity
      [corridor.last().unwrap().polygon_index][conn_index];
    portal_edge_index.push(connectivity.edge_index);
    corridor.push(MeshNodeRef { polygon_index: connectivity.polygon_index });
  }

  Ok(PathResult {
    stats: path_result.stats,
    path: Path { corridor, portal_edge_index },
  })
}

#[cfg(test)]
mod tests {
  use glam::Vec3;

  use crate::{
    nav_mesh::{MeshNodeRef, NavigationMesh},
    path::Path,
    Archipelago,
  };

  use super::find_path;

  #[test]
  fn finds_path_in_archipelago() {
    let mesh = NavigationMesh {
      mesh_bounds: None,
      vertices: vec![
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(4.0, 0.0, 1.0),
        Vec3::new(4.0, 0.0, 2.0),
        Vec3::new(2.0, 0.0, 3.0),
        Vec3::new(1.0, 0.0, 3.0),
        Vec3::new(0.0, 0.0, 2.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 5.0),
        Vec3::new(2.0, 0.0, 5.0),
        Vec3::new(2.0, 0.0, 4.0),
        Vec3::new(3.0, 1.0, 5.0),
        Vec3::new(3.0, 1.0, 4.0),
        Vec3::new(3.0, -2.0, 4.0),
        Vec3::new(3.0, -2.0, 3.0),
      ],
      polygons: vec![
        vec![0, 1, 2, 3, 4, 5, 6, 7],
        vec![5, 4, 10, 9, 8],
        vec![9, 10, 12, 11],
        vec![10, 4, 14, 13],
      ],
    }
    .validate()
    .expect("Mesh is valid.");

    let archipelago = Archipelago::create_from_navigation_mesh(mesh);
    let nav_data = &archipelago.nav_data;

    let path_result = find_path(
      nav_data,
      MeshNodeRef { polygon_index: 0 },
      MeshNodeRef { polygon_index: 2 },
    )
    .expect("found path");

    assert_eq!(
      path_result.path,
      Path {
        corridor: vec![
          MeshNodeRef { polygon_index: 0 },
          MeshNodeRef { polygon_index: 1 },
          MeshNodeRef { polygon_index: 2 }
        ],
        portal_edge_index: vec![4, 2],
      }
    );

    let path_result = find_path(
      nav_data,
      MeshNodeRef { polygon_index: 2 },
      MeshNodeRef { polygon_index: 0 },
    )
    .expect("found path");

    assert_eq!(
      path_result.path,
      Path {
        corridor: vec![
          MeshNodeRef { polygon_index: 2 },
          MeshNodeRef { polygon_index: 1 },
          MeshNodeRef { polygon_index: 0 }
        ],
        portal_edge_index: vec![0, 0],
      }
    );

    let path_result = find_path(
      nav_data,
      MeshNodeRef { polygon_index: 3 },
      MeshNodeRef { polygon_index: 0 },
    )
    .expect("found path");

    assert_eq!(
      path_result.path,
      Path {
        corridor: vec![
          MeshNodeRef { polygon_index: 3 },
          MeshNodeRef { polygon_index: 1 },
          MeshNodeRef { polygon_index: 0 }
        ],
        portal_edge_index: vec![0, 0],
      }
    );
  }
}
