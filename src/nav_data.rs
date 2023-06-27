use std::collections::HashMap;

use glam::Vec3;

use crate::island::{Island, IslandId};

pub struct NavigationData {
  pub islands: HashMap<IslandId, Island>,
}

#[derive(PartialEq, Eq, Debug, Clone, Hash)]
pub struct NodeRef {
  // The island of the node.
  pub island_id: IslandId,
  // The index of the node in the island.
  pub polygon_index: usize,
}

impl NavigationData {
  pub fn sample_point(
    &self,
    point: Vec3,
    distance_to_node: f32,
  ) -> Option<(Vec3, NodeRef)> {
    let mut best_point = None;
    for (island_id, island) in self.islands.iter() {
      let nav_data = match &island.nav_data {
        None => continue,
        Some(data) => data,
      };

      let relative_point = nav_data.transform.apply_inverse(point);
      if !nav_data
        .nav_mesh
        .mesh_bounds
        .expand_by_size(Vec3::ONE * distance_to_node)
        .contains_point(relative_point)
      {
        continue;
      }

      let (sampled_point, sampled_node) =
        match nav_data.nav_mesh.sample_point(point, distance_to_node) {
          Some(sampled) => sampled,
          None => continue,
        };

      let distance = relative_point.distance_squared(sampled_point);
      match best_point {
        Some((best_distance, _)) if distance >= best_distance => continue,
        _ => {}
      }

      best_point = Some((
        distance,
        (
          sampled_point,
          NodeRef { island_id: *island_id, polygon_index: sampled_node },
        ),
      ));
    }
    best_point.map(|(_, b)| b)
  }
}
