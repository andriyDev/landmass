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

      let (sampled_point, sampled_node) = match nav_data
        .nav_mesh
        .sample_point(relative_point, distance_to_node)
      {
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
          nav_data.transform.apply(sampled_point),
          NodeRef { island_id: *island_id, polygon_index: sampled_node },
        ),
      ));
    }
    best_point.map(|(_, b)| b)
  }
}

#[cfg(test)]
mod tests {
  use std::{collections::HashMap, f32::consts::PI, sync::Arc};

  use glam::Vec3;

  use crate::{
    island::Island, nav_data::NodeRef, nav_mesh::NavigationMesh, BoundingBox,
    Transform,
  };

  use super::NavigationData;

  #[test]
  fn samples_points() {
    let nav_mesh = NavigationMesh {
      mesh_bounds: None,
      vertices: vec![
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(2.0, 1.0, 1.0),
        Vec3::new(3.0, 1.0, 1.0),
        Vec3::new(4.0, 1.0, 1.0),
        Vec3::new(4.0, 1.0, 2.0),
        Vec3::new(3.0, 1.0, 2.0),
        Vec3::new(2.0, 1.0, 2.0),
        Vec3::new(1.0, 1.0, 2.0),
      ],
      polygons: vec![vec![0, 1, 6, 7], vec![1, 2, 5, 6], vec![2, 3, 4, 5]],
    }
    .validate()
    .expect("is valid");
    let nav_mesh = Arc::new(nav_mesh);

    let mut islands = HashMap::new();

    let mut island_1 = Island::new(BoundingBox::new_box(Vec3::ZERO, Vec3::ONE));
    island_1.set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::clone(&nav_mesh),
      /* linkable_distance_to_region_edge= */ 0.01,
    );

    let mut island_2 = Island::new(BoundingBox::new_box(Vec3::ZERO, Vec3::ONE));
    island_2.set_nav_mesh(
      Transform { translation: Vec3::new(5.0, 0.1, 0.0), rotation: PI * -0.5 },
      Arc::clone(&nav_mesh),
      /* linkable_distance_to_region_edge= */ 0.01,
    );

    islands.insert(1, island_1);
    islands.insert(2, island_2);

    let nav_data = NavigationData { islands };
    // Just above island 1 node.
    assert_eq!(
      nav_data.sample_point(
        Vec3::new(1.5, 1.09, 1.5),
        /* distance_to_node= */ 0.1,
      ),
      Some((
        Vec3::new(1.5, 1.0, 1.5),
        NodeRef { island_id: 1, polygon_index: 0 }
      )),
    );
    // Just outside island 1 node.
    assert_eq!(
      nav_data.sample_point(
        Vec3::new(0.95, 0.95, 0.95),
        /* distance_to_node= */ 0.1,
      ),
      Some((
        Vec3::new(1.0, 1.0, 1.0),
        NodeRef { island_id: 1, polygon_index: 0 }
      )),
    );
    // At overlap, but closer to island 1.
    assert_eq!(
      nav_data.sample_point(
        Vec3::new(3.5, 1.04, 1.5),
        /* distance_to_node= */ 0.1,
      ),
      Some((
        Vec3::new(3.5, 1.0, 1.5),
        NodeRef { island_id: 1, polygon_index: 2 }
      )),
    );
    // At overlap, but closer to island 2.
    assert_eq!(
      nav_data
        .sample_point(
          Vec3::new(3.5, 1.06, 1.5),
          /* distance_to_node= */ 0.1,
        )
        .map(|(p, n)| ((p * 1e6).round() / 1e6, n)),
      Some((
        Vec3::new(3.5, 1.1, 1.5),
        NodeRef { island_id: 2, polygon_index: 0 }
      )),
    );
  }
}
