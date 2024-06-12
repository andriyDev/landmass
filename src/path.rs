use std::collections::HashSet;

use glam::{Vec3, Vec3Swizzles};

use crate::{nav_data::NodeRef, NavigationData};

/// A path computed on the navigation data.
#[derive(PartialEq, Eq, Clone, Debug)]
pub struct Path {
  /// The nodes belonging to the path. Must have at least one element.
  pub(crate) corridor: Vec<NodeRef>,
  /// The "portals" used between each node in [`Path::corridor`]. The portals
  /// are the edges that the agent must cross along its path. Must have
  /// exactly one less element than [`Path::corridor`].
  pub(crate) portal_edge_index: Vec<usize>,
}

impl Path {
  /// Determines the endpoints of the portal at `portal_index` in `nav_data`.
  fn get_portal_endpoints(
    &self,
    portal_index: usize,
    nav_data: &NavigationData,
  ) -> (Vec3, Vec3) {
    let node_ref = self.corridor[portal_index].clone();
    let edge = self.portal_edge_index[portal_index];

    let island_data = nav_data
      .islands
      .get(&node_ref.island_id)
      .expect("only called if path is still valid")
      .nav_data
      .as_ref()
      .expect("only called if path is still valid");
    let (left_vertex, right_vertex) = island_data.nav_mesh.polygons
      [node_ref.polygon_index]
      .get_edge_indices(edge);

    (
      island_data.transform.apply(island_data.nav_mesh.vertices[left_vertex]),
      island_data.transform.apply(island_data.nav_mesh.vertices[right_vertex]),
    )
  }

  /// Determines the next point along `self` that the agent can walk straight
  /// towards, starting at the node `start_index` at `start_point` and ending at
  /// the node `end_index` at `end_point`. `start_index` and `end_index` are
  /// indices into `self`. Returns the index of the node in the path where the
  /// next point is, and that next point. Note this can be called repeatedly by
  /// passing in the returned tuple as the `start_index` and `start_point` to
  /// generate the full straight path.
  pub(crate) fn find_next_point_in_straight_path(
    &self,
    nav_data: &NavigationData,
    start_index: usize,
    start_point: Vec3,
    end_index: usize,
    end_point: Vec3,
  ) -> (usize, Vec3) {
    let apex = start_point;
    let (mut left_index, mut right_index) = (start_index, start_index);

    let (mut current_left, mut current_right) = if start_index == end_index {
      (end_point, end_point)
    } else {
      self.get_portal_endpoints(start_index, nav_data)
    };

    fn triangle_area_2(point_0: Vec3, point_1: Vec3, point_2: Vec3) -> f32 {
      return (point_1.xz() - point_0.xz())
        .perp_dot(point_2.xz() - point_0.xz());
    }

    for portal_index in (start_index + 1)..=end_index {
      let (portal_left, portal_right) = if portal_index == end_index {
        (end_point, end_point)
      } else {
        self.get_portal_endpoints(portal_index, nav_data)
      };

      if triangle_area_2(apex, current_right, portal_right) <= 0.0 {
        if triangle_area_2(apex, current_left, portal_right) >= 0.0 {
          right_index = portal_index;
          current_right = portal_right;
        } else {
          return (left_index, current_left);
        }
      }

      if triangle_area_2(apex, current_left, portal_left) >= 0.0 {
        if triangle_area_2(apex, current_right, portal_left) <= 0.0 {
          left_index = portal_index;
          current_left = portal_left;
        } else {
          return (right_index, current_right);
        }
      }
    }

    (end_index, end_point)
  }

  /// Determines if a path is valid. A path may be invalid if an island it
  /// travelled across was updated or removed.
  pub(crate) fn is_valid(&self, nav_data: &NavigationData) -> bool {
    let islands_in_path =
      self.corridor.iter().map(|n| n.island_id).collect::<HashSet<u32>>();

    for island_id in islands_in_path {
      match nav_data.islands.get(&island_id) {
        None => return false,
        Some(island) if island.dirty => return false,
        _ => {}
      }
    }

    true
  }

  /// Finds the index of `node` in the path.
  pub(crate) fn find_index_of_node(&self, node: NodeRef) -> Option<usize> {
    self.corridor.iter().position(|x| x == &node)
  }

  /// Finds the index of `node` in the path, iterating backwards. This is
  /// slightly more efficient than [`Path::find_index_of_node`] for the target
  /// node, since most of the time the target node will be near the end of the
  /// path.
  pub(crate) fn find_index_of_node_rev(&self, node: NodeRef) -> Option<usize> {
    self
      .corridor
      .iter()
      .rev()
      .position(|x| x == &node)
      .map(|rev_index| self.corridor.len() - 1 - rev_index)
  }
}

#[cfg(test)]
#[path = "path_test.rs"]
mod test;
