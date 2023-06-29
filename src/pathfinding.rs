use crate::{
  astar::{self, AStarProblem, PathStats},
  nav_data::NodeRef,
  path::Path,
  NavigationData,
};

struct ArchipelagoPathProblem<'a> {
  nav_data: &'a NavigationData,
  start_node: NodeRef,
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

pub(crate) struct PathResult {
  pub(crate) stats: PathStats,
  pub(crate) path: Path,
}

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
  corridor.push(start_node);

  for conn_index in path_result.path {
    let previous_node = corridor.last().unwrap();
    let nav_mesh = &nav_data
      .islands
      .get(&previous_node.island_id)
      .unwrap()
      .nav_data
      .as_ref()
      .unwrap()
      .nav_mesh;
    let connectivity =
      &nav_mesh.connectivity[previous_node.polygon_index][conn_index];
    portal_edge_index.push(connectivity.edge_index);
    corridor.push(NodeRef {
      island_id: previous_node.island_id,
      polygon_index: connectivity.polygon_index,
    });
  }

  Ok(PathResult {
    stats: path_result.stats,
    path: Path { corridor, portal_edge_index },
  })
}

#[cfg(test)]
mod tests {
  use std::f32::consts::PI;

  use glam::Vec3;

  use crate::{
    nav_data::NodeRef, nav_mesh::NavigationMesh, path::Path, Archipelago,
    Transform,
  };

  use super::find_path;

  #[test]
  fn finds_path_in_archipelago() {
    let nav_mesh = NavigationMesh {
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

    let mut archipelago = Archipelago::new();
    let island_id = archipelago.add_island(nav_mesh.mesh_bounds);
    archipelago.get_island_mut(island_id).set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      nav_mesh,
      /* linkable_distance_to_region_edge= */ 0.01,
    );

    let nav_data = &archipelago.nav_data;

    let path_result = find_path(
      nav_data,
      NodeRef { island_id, polygon_index: 0 },
      NodeRef { island_id, polygon_index: 2 },
    )
    .expect("found path");

    assert_eq!(
      path_result.path,
      Path {
        corridor: vec![
          NodeRef { island_id, polygon_index: 0 },
          NodeRef { island_id, polygon_index: 1 },
          NodeRef { island_id, polygon_index: 2 }
        ],
        portal_edge_index: vec![4, 2],
      }
    );

    let path_result = find_path(
      nav_data,
      NodeRef { island_id, polygon_index: 2 },
      NodeRef { island_id, polygon_index: 0 },
    )
    .expect("found path");

    assert_eq!(
      path_result.path,
      Path {
        corridor: vec![
          NodeRef { island_id, polygon_index: 2 },
          NodeRef { island_id, polygon_index: 1 },
          NodeRef { island_id, polygon_index: 0 }
        ],
        portal_edge_index: vec![0, 0],
      }
    );

    let path_result = find_path(
      nav_data,
      NodeRef { island_id, polygon_index: 3 },
      NodeRef { island_id, polygon_index: 0 },
    )
    .expect("found path");

    assert_eq!(
      path_result.path,
      Path {
        corridor: vec![
          NodeRef { island_id, polygon_index: 3 },
          NodeRef { island_id, polygon_index: 1 },
          NodeRef { island_id, polygon_index: 0 }
        ],
        portal_edge_index: vec![0, 0],
      }
    );
  }

  #[test]
  fn finds_paths_on_two_islands() {
    let nav_mesh = NavigationMesh {
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

    let mut archipelago = Archipelago::new();
    let island_id_1 = archipelago.add_island(nav_mesh.mesh_bounds);
    archipelago.get_island_mut(island_id_1).set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      nav_mesh.clone(),
      /* linkable_distance_to_region_edge= */ 0.01,
    );

    let island_id_2 = archipelago.add_island(nav_mesh.mesh_bounds);
    archipelago.get_island_mut(island_id_2).set_nav_mesh(
      Transform { translation: Vec3::new(6.0, 0.0, 0.0), rotation: PI * 0.5 },
      nav_mesh,
      /* linkable_distance_to_region_edge= */ 0.01,
    );

    let nav_data = &archipelago.nav_data;

    let path_result = find_path(
      nav_data,
      NodeRef { island_id: island_id_1, polygon_index: 0 },
      NodeRef { island_id: island_id_1, polygon_index: 2 },
    )
    .expect("found path");

    assert_eq!(
      path_result.path,
      Path {
        corridor: vec![
          NodeRef { island_id: island_id_1, polygon_index: 0 },
          NodeRef { island_id: island_id_1, polygon_index: 1 },
          NodeRef { island_id: island_id_1, polygon_index: 2 }
        ],
        portal_edge_index: vec![4, 2],
      }
    );

    let path_result = find_path(
      nav_data,
      NodeRef { island_id: island_id_2, polygon_index: 0 },
      NodeRef { island_id: island_id_2, polygon_index: 2 },
    )
    .expect("found path");

    assert_eq!(
      path_result.path,
      Path {
        corridor: vec![
          NodeRef { island_id: island_id_2, polygon_index: 0 },
          NodeRef { island_id: island_id_2, polygon_index: 1 },
          NodeRef { island_id: island_id_2, polygon_index: 2 }
        ],
        portal_edge_index: vec![4, 2],
      }
    );
  }

  #[test]
  fn no_path_between_disconnected_islands() {
    let nav_mesh = NavigationMesh {
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

    let mut archipelago = Archipelago::new();
    let island_id_1 = archipelago.add_island(nav_mesh.mesh_bounds);
    archipelago.get_island_mut(island_id_1).set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      nav_mesh.clone(),
      /* linkable_distance_to_region_edge= */ 0.01,
    );

    let island_id_2 = archipelago.add_island(nav_mesh.mesh_bounds);
    archipelago.get_island_mut(island_id_2).set_nav_mesh(
      Transform { translation: Vec3::new(6.0, 0.0, 0.0), rotation: PI * 0.5 },
      nav_mesh,
      /* linkable_distance_to_region_edge= */ 0.01,
    );

    let nav_data = &archipelago.nav_data;

    assert!(find_path(
      nav_data,
      NodeRef { island_id: island_id_1, polygon_index: 0 },
      NodeRef { island_id: island_id_2, polygon_index: 0 },
    )
    .is_err());

    assert!(find_path(
      nav_data,
      NodeRef { island_id: island_id_2, polygon_index: 0 },
      NodeRef { island_id: island_id_1, polygon_index: 0 },
    )
    .is_err());
  }
}
