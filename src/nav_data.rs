use glam::Vec3;

use crate::{nav_mesh::MeshNodeRef, ValidNavigationMesh};

pub struct NavigationData {
  pub nav_mesh: ValidNavigationMesh,
}

impl NavigationData {
  pub fn sample_point(
    &self,
    point: Vec3,
    distance_to_node: f32,
  ) -> Option<(Vec3, MeshNodeRef)> {
    self.nav_mesh.sample_point(point, distance_to_node)
  }
}
