use std::{collections::HashMap, marker::PhantomData};

use thiserror::Error;

use crate::{
  Archipelago, CoordinateSystem, IslandId,
  agent::PermittedAnimationLinks,
  coords::CorePointSampleDistance,
  link::AnimationLinkId,
  nav_data::NodeRef,
  path::{PathIndex, StraightPathStep},
  pathfinding,
};

/// A point on the navigation meshes.
pub struct SampledPoint<'archipelago, CS: CoordinateSystem> {
  /// The point on the navigation meshes.
  point: CS::Coordinate,
  /// The node that the point is on.
  node_ref: NodeRef,
  /// The type index for `node_ref`.
  type_index: usize,
  /// Marker to prevent this object from out-living a borrow to the
  /// archipelago.
  marker: PhantomData<&'archipelago ()>,
}

// Manual Clone impl for `SampledPoint` to avoid the Clone bound on CS.
impl<CS: CoordinateSystem> Clone for SampledPoint<'_, CS> {
  fn clone(&self) -> Self {
    Self {
      point: self.point.clone(),
      node_ref: self.node_ref,
      type_index: self.type_index,
      marker: self.marker,
    }
  }
}

impl<CS: CoordinateSystem> SampledPoint<'_, CS> {
  /// Gets the point on the navigation meshes.
  pub fn point(&self) -> CS::Coordinate {
    self.point.clone()
  }

  /// Gets the island the sampled point is on.
  pub fn island(&self) -> IslandId {
    self.node_ref.island_id
  }

  /// Gets the type index the sampled point is on.
  pub fn type_index(&self) -> usize {
    self.type_index
  }
}

/// An error while sampling a point.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
pub enum SamplePointError {
  #[error("The sample point is too far from any island.")]
  OutOfRange,
  #[error(
    "The navigation data of the archipelago has been mutated since the last update."
  )]
  NavDataDirty,
}

/// Finds the nearest point on the navigation meshes to (and within
/// `distance_to_node` of) `point`.
pub(crate) fn sample_point<'archipelago, CS: CoordinateSystem>(
  archipelago: &'archipelago Archipelago<CS>,
  point: CS::Coordinate,
  point_sample_distance: &'_ CorePointSampleDistance,
) -> Result<SampledPoint<'archipelago, CS>, SamplePointError> {
  if archipelago.nav_data.dirty {
    return Err(SamplePointError::NavDataDirty);
  }
  let Some((point, node_ref)) = archipelago
    .nav_data
    .sample_point(CS::to_landmass(&point), point_sample_distance)
  else {
    return Err(SamplePointError::OutOfRange);
  };

  let island = archipelago.nav_data.get_island(node_ref.island_id).unwrap();
  let type_index =
    island.island.nav_mesh.polygons[node_ref.polygon_index].type_index;

  Ok(SampledPoint {
    point: CS::from_landmass(&point),
    node_ref,
    type_index,
    marker: PhantomData,
  })
}

/// An error from finding a path between two sampled points.
#[derive(Clone, Copy, Debug, PartialEq, Error)]
pub enum FindPathError {
  #[error("The type index {0:?} had a cost of {1}, which is non-positive.")]
  NonPositiveTypeIndexCost(usize, f32),
  #[error("No path was found between the start and end points.")]
  NoPathFound,
}

/// A single step in a path.
pub enum PathStep<CS: CoordinateSystem> {
  /// A point along the path that can be walked directly towards.
  Waypoint(CS::Coordinate),
  /// An animation link that should be used. The path should move directly
  /// towards the `start_point`, then use the animation link, which should
  /// result in the path being at `end_point`.
  AnimationLink {
    /// The point that needs to be reached to use the animation link.
    start_point: CS::Coordinate,
    /// The point that using the animation link should take the path to.
    end_point: CS::Coordinate,
    /// The animation link that will be used here.
    link_id: AnimationLinkId,
  },
}

impl<CS: CoordinateSystem<Coordinate: std::fmt::Debug>> std::fmt::Debug
  for PathStep<CS>
{
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      Self::Waypoint(arg0) => f.debug_tuple("Waypoint").field(arg0).finish(),
      Self::AnimationLink { start_point, end_point, link_id } => f
        .debug_struct("AnimationLink")
        .field("start_point", start_point)
        .field("end_point", end_point)
        .field("link_id", link_id)
        .finish(),
    }
  }
}

// Manual implementations of derived traits so we don't require the CS to have
// the trait.

impl<CS: CoordinateSystem<Coordinate: Clone>> Clone for PathStep<CS> {
  fn clone(&self) -> Self {
    match self {
      Self::Waypoint(arg0) => Self::Waypoint(arg0.clone()),
      Self::AnimationLink { start_point, end_point, link_id } => {
        Self::AnimationLink {
          start_point: start_point.clone(),
          end_point: end_point.clone(),
          link_id: *link_id,
        }
      }
    }
  }
}

impl<CS: CoordinateSystem<Coordinate: Copy>> Copy for PathStep<CS> {}

impl<CS: CoordinateSystem<Coordinate: PartialEq>> PartialEq for PathStep<CS> {
  fn eq(&self, other: &Self) -> bool {
    match (self, other) {
      (Self::Waypoint(l0), Self::Waypoint(r0)) => l0 == r0,
      (
        Self::AnimationLink {
          start_point: l_start_point,
          end_point: l_end_point,
          link_id: l_link_id,
        },
        Self::AnimationLink {
          start_point: r_start_point,
          end_point: r_end_point,
          link_id: r_link_id,
        },
      ) => {
        l_start_point == r_start_point
          && l_end_point == r_end_point
          && l_link_id == r_link_id
      }
      _ => false,
    }
  }
}

/// Finds a straight-line path across the navigation meshes from `start_point`
/// to `end_point`.
pub(crate) fn find_path<'a, CS: CoordinateSystem>(
  archipelago: &'a Archipelago<CS>,
  start_point: &SampledPoint<'a, CS>,
  end_point: &SampledPoint<'a, CS>,
  override_type_index_costs: &HashMap<usize, f32>,
  permitted_animation_links: PermittedAnimationLinks,
) -> Result<Vec<PathStep<CS>>, FindPathError> {
  // This assert can actually be triggered. This can happen if a user samples
  // points from one archipelago, but finds a path in a **different**
  // archipelago. This seems almost malicious though, so I don't think we should
  // handle it at all. I'd rather the "wins" we get from avoiding
  // double-sampling (in cases where the user samples a point to check for
  // validity and then finds a path).
  assert!(
    !archipelago.nav_data.dirty,
    "The navigation data has been mutated, but we have SampledPoints, so this should be impossible."
  );

  for (type_index, cost) in override_type_index_costs.iter() {
    if *cost <= 0.0 {
      return Err(FindPathError::NonPositiveTypeIndexCost(*type_index, *cost));
    }
  }

  let Some(path) = pathfinding::find_path(
    &archipelago.nav_data,
    start_point.node_ref,
    CS::to_landmass(&start_point.point),
    end_point.node_ref,
    CS::to_landmass(&end_point.point),
    override_type_index_costs,
    permitted_animation_links,
  )
  .path
  else {
    return Err(FindPathError::NoPathFound);
  };

  let mut current_index = PathIndex::from_corridor_index(0, 0);
  let mut current_point = CS::to_landmass(&start_point.point());

  let last_index = path.last_index();
  let last_point = CS::to_landmass(&end_point.point());

  let mut path_points = vec![PathStep::Waypoint(start_point.point())];
  if current_index == last_index {
    path_points.push(PathStep::Waypoint(end_point.point.clone()));
    return Ok(path_points);
  }

  // Keep looping until we reach the end index. If it's the last index, but the
  // previous step was an animation link, run once more to get the waypoint to
  // the end point.
  while current_index != last_index
    || matches!(path_points.last().unwrap(), PathStep::AnimationLink { .. })
  {
    let next_step;
    (current_index, next_step) = path.find_next_point_in_straight_path(
      &archipelago.nav_data,
      current_index,
      current_point,
      last_index,
      last_point,
    );
    let next_path_step;
    (current_point, next_path_step) = match next_step {
      StraightPathStep::Waypoint(point) => {
        (point, PathStep::Waypoint(CS::from_landmass(&point)))
      }
      StraightPathStep::AnimationLink {
        start_point,
        end_point,
        link_id,
        ..
      } => (
        // Using this animation link leads to the end point of the link.
        end_point,
        PathStep::AnimationLink {
          start_point: CS::from_landmass(&start_point),
          end_point: CS::from_landmass(&end_point),
          link_id,
        },
      ),
    };
    path_points.push(next_path_step);
  }

  Ok(path_points)
}

#[cfg(test)]
#[path = "query_test.rs"]
mod test;
