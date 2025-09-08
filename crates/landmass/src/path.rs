use std::{cmp::Ordering, collections::HashSet};

use glam::{Vec3, Vec3Swizzles};

use crate::{
  CoordinateSystem, IslandId, NavigationData,
  geometry::project_point_to_line_segment,
  link::AnimationLinkId,
  nav_data::{KindedOffMeshLink, NodeRef, OffMeshLinkId},
};

/// A path computed on the navigation data.
#[derive(PartialEq, Clone, Debug)]
pub struct Path {
  /// The segments of this path on islands. These are joined together with
  /// [`Path::off_mesh_link_segments`]. Note even if an island is only used to
  /// take another off mesh link, it must be included with the relevant node.
  pub(crate) island_segments: Vec<IslandSegment>,
  /// The off mesh links that connect the island segments. Must have exactly
  /// one less element than [`Path::island_segments`].
  pub(crate) off_mesh_link_segments: Vec<OffMeshLinkSegment>,
  /// The point where this path was started from.
  ///
  /// This is not used, but is good for debugging.
  pub(crate) start_point: Vec3,
  /// The point where this path was originally targetting.
  ///
  /// This is not used, but is good for debugging.
  pub(crate) end_point: Vec3,
}

/// Part of a path entirely along a single island.
#[derive(PartialEq, Eq, Clone, Debug)]
pub(crate) struct IslandSegment {
  /// The island that the nodes belong to.
  pub(crate) island_id: IslandId,
  /// The nodes belonging to the path as their polygon index. Must have at
  /// least one element.
  pub(crate) corridor: Vec<usize>,
  /// The "portals" used between each node in [`IslandSegment::corridor`]. The
  /// portals are the edges that the agent must cross along its path. Must
  /// have exactly one less element than [`IslandSegment::corridor`].
  pub(crate) portal_edge_index: Vec<usize>,
}

/// Part of a path taking an off mesh link.
#[derive(PartialEq, Eq, Clone, Debug)]
pub(crate) struct OffMeshLinkSegment {
  /// The node that the off mesh link starts from.
  pub(crate) starting_node: NodeRef,
  /// The node that the off mesh link ends at.
  pub(crate) end_node: NodeRef,
  /// The link to be used.
  pub(crate) off_mesh_link: OffMeshLinkId,
}

/// A portal retrieved from a segment. All portals are in world-space.
enum Portal {
  /// The portal is walkable by an agent. This portal must be oriented
  /// left-to-right wrt the node it comes from.
  Walkable(Vec3, Vec3),
  /// The portal is from an animation link.
  AnimationLink {
    /// The portal to reach in order to initiate the animation link. The order
    /// is not defined.
    start_portal: (Vec3, Vec3),
    /// The portal that the agent gets sent to after using the animation link.
    /// The orientation must match the start_portal.
    end_portal: (Vec3, Vec3),
    /// The ID of the animation link that this portal came from.
    link_id: AnimationLinkId,
    /// The node that the animation link starts from.
    start_node: NodeRef,
    /// The node that the animation link ends at.
    end_node: NodeRef,
  },
}

impl IslandSegment {
  /// Determines the endpoints of the portal at `portal_index` in `nav_data`.
  fn get_portal_endpoints<CS: CoordinateSystem>(
    &self,
    portal_index: usize,
    nav_data: &NavigationData<CS>,
  ) -> Portal {
    let polygon_index = self.corridor[portal_index];
    let edge = self.portal_edge_index[portal_index];

    let island_data = nav_data
      .get_island(self.island_id)
      .expect("only called if path is still valid");
    let (left_vertex, right_vertex) =
      island_data.nav_mesh.polygons[polygon_index].get_edge_indices(edge);

    Portal::Walkable(
      island_data.transform.apply(island_data.nav_mesh.vertices[left_vertex]),
      island_data.transform.apply(island_data.nav_mesh.vertices[right_vertex]),
    )
  }
}

impl OffMeshLinkSegment {
  /// Gets the endpoints of the portal for this off mesh link in `nav_data`.
  fn get_portal_endpoints<CS: CoordinateSystem>(
    &self,
    nav_data: &NavigationData<CS>,
  ) -> Portal {
    let off_mesh_link = nav_data
      .off_mesh_links
      .get(self.off_mesh_link)
      .expect("only called if path is still valid");
    let portal = off_mesh_link.portal;
    match &off_mesh_link.kinded {
      KindedOffMeshLink::BoundaryLink { .. } => {
        Portal::Walkable(portal.0, portal.1)
      }
      KindedOffMeshLink::AnimationLink {
        destination_portal,
        animation_link,
        ..
      } => Portal::AnimationLink {
        start_portal: portal,
        end_portal: *destination_portal,
        link_id: *animation_link,
        start_node: self.starting_node,
        end_node: self.end_node,
      },
    }
  }
}

/// An index in a path.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub(crate) struct PathIndex {
  /// The index of the segment this belongs to.
  segment_index: usize,
  /// The index of the portal in the corridor. The last index in a corrider
  /// either corresponds to the off mesh link to the next segment or the target
  /// node.
  portal_index: usize,
}

impl PathIndex {
  /// Creates a `PathIndex` starting at the `island_segment_index` at the
  /// `corridor_index`.
  pub(crate) fn from_corridor_index(
    island_segment_index: usize,
    corridor_index: usize,
  ) -> Self {
    Self { segment_index: island_segment_index, portal_index: corridor_index }
  }

  fn next(&self, path: &Path) -> Self {
    let new_portal_index = self.portal_index + 1;
    match new_portal_index
      .cmp(&path.island_segments[self.segment_index].portal_edge_index.len())
    {
      Ordering::Less | Ordering::Equal => Self {
        segment_index: self.segment_index,
        portal_index: new_portal_index,
      },
      Ordering::Greater => {
        if self.segment_index + 1 < path.island_segments.len() {
          Self { segment_index: self.segment_index + 1, portal_index: 0 }
        } else {
          // Only the last segment can go past the end of the path to allow
          // being inclusive over the end index if the end index is
          // after the last portal.
          Self {
            segment_index: self.segment_index,
            portal_index: new_portal_index,
          }
        }
      }
    }
  }
}

/// A step in a straight-line path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum StraightPathStep {
  /// Walk directly to this point.
  Waypoint(Vec3),
  /// Use this animation link (which includes walking to its start point).
  AnimationLink {
    /// The point to walk to in order to use the animation link.
    start_point: Vec3,
    /// The point that using the animation link *should* lead to. There is no
    /// actual guarantee that the animation link will take you here, but it is
    /// a good approximation for straight-line paths.
    end_point: Vec3,
    /// The ID of the animation link to use.
    link_id: AnimationLinkId,
    /// The node that the start point is in.
    start_node: NodeRef,
    /// The node that the end point is in.
    end_node: NodeRef,
  },
}

impl Path {
  /// Determines the endpoints of the portal at `segment_index` at
  /// `portal_index` in `nav_data`.
  fn get_portal_endpoints<CS: CoordinateSystem>(
    &self,
    path_index: PathIndex,
    nav_data: &NavigationData<CS>,
  ) -> Portal {
    if path_index.portal_index
      == self.island_segments[path_index.segment_index].portal_edge_index.len()
    {
      self.off_mesh_link_segments[path_index.segment_index]
        .get_portal_endpoints(nav_data)
    } else {
      self.island_segments[path_index.segment_index]
        .get_portal_endpoints(path_index.portal_index, nav_data)
    }
  }

  /// Determines the next point along `self` that the agent can walk straight
  /// towards, starting at the node `start_index` at `start_point` and ending at
  /// the node `end_index` at `end_point`. `start_index` and `end_index` are
  /// indices into `self`. Returns the index of the node in the path where the
  /// next point is, and that next point. Note this can be called repeatedly by
  /// passing in the returned tuple as the `start_index` and `start_point` to
  /// generate the full straight path.
  pub(crate) fn find_next_point_in_straight_path<CS: CoordinateSystem>(
    &self,
    nav_data: &NavigationData<CS>,
    start_index: PathIndex,
    start_point: Vec3,
    mut end_index: PathIndex,
    end_point: Vec3,
  ) -> (PathIndex, StraightPathStep) {
    let apex = start_point;
    let (mut left_index, mut right_index) = (start_index, start_index);

    enum EndOfPath {
      Point(Vec3),
      AnimationLink {
        start_point: Vec3,
        end_point: Vec3,
        link_id: AnimationLinkId,
        start_node: NodeRef,
        end_node: NodeRef,
      },
    }
    let mut end_of_path = EndOfPath::Point(end_point);

    let (mut current_left, mut current_right) = if start_index == end_index {
      (end_point, end_point)
    } else {
      match self.get_portal_endpoints(start_index, nav_data) {
        Portal::Walkable(left, right) => (left, right),
        Portal::AnimationLink {
          start_portal,
          end_portal,
          link_id,
          start_node,
          end_node,
        } => {
          let (start_point, fraction) =
            project_point_to_line_segment(apex, start_portal);
          let end_point = end_portal.0.lerp(end_portal.1, fraction);
          return (
            // Skip to the next index after you take the animation link.
            start_index.next(self),
            StraightPathStep::AnimationLink {
              start_point,
              end_point,
              link_id,
              start_node,
              end_node,
            },
          );
        }
      }
    };

    fn triangle_area_2(point_0: Vec3, point_1: Vec3, point_2: Vec3) -> f32 {
      (point_1.xy() - point_0.xy()).perp_dot(point_2.xy() - point_0.xy())
    }

    let mut portal_index = start_index.next(self);
    while portal_index <= end_index {
      let (portal_left, portal_right) = if portal_index == end_index {
        (end_point, end_point)
      } else {
        match self.get_portal_endpoints(portal_index, nav_data) {
          Portal::Walkable(left, right) => (left, right),
          Portal::AnimationLink {
            start_portal,
            end_portal,
            link_id,
            start_node,
            end_node,
          } => {
            let (start_point, fraction) =
              project_point_to_line_segment(apex, start_portal);
            let end_point = end_portal.0.lerp(end_portal.1, fraction);

            // Change the end of the path (for this step) to be the animation
            // link.
            end_of_path = EndOfPath::AnimationLink {
              start_point,
              end_point,
              link_id,
              start_node,
              end_node,
            };
            // At the end of this iteration, we will increment the portal index
            // which will be less than this, ending the loop. Note, we don't
            // want to just break here, since we need to see if the portal is
            // visible from the current funnel, or not (in which case we should
            // return a point instead).
            end_index = portal_index;
            // Don't treat the animation link as a portal, since we won't be
            // adding any more portals. Treat it as an endpoint.
            (start_point, start_point)
          }
        }
      };

      if triangle_area_2(apex, current_right, portal_right) >= 0.0 {
        if triangle_area_2(apex, current_left, portal_right) <= 0.0 {
          right_index = portal_index;
          current_right = portal_right;
        } else {
          return (left_index, StraightPathStep::Waypoint(current_left));
        }
      }

      if triangle_area_2(apex, current_left, portal_left) <= 0.0 {
        if triangle_area_2(apex, current_right, portal_left) >= 0.0 {
          left_index = portal_index;
          current_left = portal_left;
        } else {
          return (right_index, StraightPathStep::Waypoint(current_right));
        }
      }

      portal_index = portal_index.next(self);
    }

    match end_of_path {
      EndOfPath::Point(end_point) => {
        (end_index, StraightPathStep::Waypoint(end_point))
      }
      EndOfPath::AnimationLink {
        start_point,
        end_point,
        link_id,
        start_node,
        end_node,
      } => (
        // We want to skip over this animation link, so use `portal_index`
        // (which had `next` called on it) instead of `end_index`.
        portal_index,
        StraightPathStep::AnimationLink {
          start_point,
          end_point,
          link_id,
          start_node,
          end_node,
        },
      ),
    }
  }

  pub(crate) fn last_index(&self) -> PathIndex {
    let segment_index = self.island_segments.len() - 1;
    PathIndex {
      segment_index,
      portal_index: self.island_segments[segment_index].portal_edge_index.len(),
    }
  }

  /// Determines if a path is valid. A path may be invalid if an island it
  /// travelled across was invalidared, or a off mesh link it used was
  /// invalidated.
  pub(crate) fn is_valid(
    &self,
    invalidated_off_mesh_links: &HashSet<OffMeshLinkId>,
    invalidated_islands: &HashSet<IslandId>,
  ) -> bool {
    for island_segment in self.island_segments.iter() {
      if invalidated_islands.contains(&island_segment.island_id) {
        return false;
      }
    }
    for off_mesh_link_segment in self.off_mesh_link_segments.iter() {
      if invalidated_off_mesh_links
        .contains(&off_mesh_link_segment.off_mesh_link)
      {
        return false;
      }
    }

    true
  }

  /// Finds the index of `node` in the path.
  pub(crate) fn find_index_of_node(&self, node: NodeRef) -> Option<PathIndex> {
    for (segment_index, island_segment) in
      self.island_segments.iter().enumerate()
    {
      if node.island_id != island_segment.island_id {
        continue;
      }
      for (portal_index, corridor_node) in
        island_segment.corridor.iter().enumerate()
      {
        if node.polygon_index == *corridor_node {
          return Some(PathIndex { segment_index, portal_index });
        }
      }
    }

    None
  }

  /// Finds the index of `node` in the path, iterating backwards. This is
  /// slightly more efficient than [`Path::find_index_of_node`] for the target
  /// node, since most of the time the target node will be near the end of the
  /// path.
  pub(crate) fn find_index_of_node_rev(
    &self,
    node: NodeRef,
  ) -> Option<PathIndex> {
    for (segment_index, island_segment) in
      self.island_segments.iter().enumerate().rev()
    {
      if node.island_id != island_segment.island_id {
        continue;
      }
      for (portal_index, corridor_node) in
        island_segment.corridor.iter().enumerate().rev()
      {
        if node.polygon_index == *corridor_node {
          return Some(PathIndex { segment_index, portal_index });
        }
      }
    }

    None
  }
}

#[cfg(test)]
#[path = "path_test.rs"]
mod test;
