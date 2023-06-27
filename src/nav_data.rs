use glam::Vec3;

use crate::ValidNavigationMesh;

pub struct NavigationData {
  pub nav_mesh: ValidNavigationMesh,
}

#[derive(PartialEq, Eq, Debug, Clone, Hash)]
pub struct NodeRef {
  // The index of the polygon in the navigation mesh.
  pub polygon_index: usize,
}

impl NavigationData {
  pub fn sample_point(
    &self,
    point: Vec3,
    distance_to_node: f32,
  ) -> Option<(Vec3, NodeRef)> {
    self
      .nav_mesh
      .sample_point(point, distance_to_node)
      .map(|(point, polygon_index)| (point, NodeRef { polygon_index }))
  }
}
