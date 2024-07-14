use std::marker::PhantomData;

use thiserror::Error;

use crate::{nav_data::NodeRef, Archipelago, CoordinateSystem};

/// A point on the navigation meshes.
pub struct SampledPoint<'archipelago, CS: CoordinateSystem> {
  /// The point on the navigation meshes.
  point: CS::Coordinate,
  /// The node that the point is on.
  node_ref: NodeRef,
  /// Marker to prevent this object from out-living a borrow to the
  /// archipelago.
  marker: PhantomData<&'archipelago ()>,
}

// Manual Clone impl for `SampledPoint` to avoid the Clone bound on CS.
impl<'archipelago, CS: CoordinateSystem> Clone
  for SampledPoint<'archipelago, CS>
{
  fn clone(&self) -> Self {
    Self {
      point: self.point.clone(),
      node_ref: self.node_ref,
      marker: self.marker,
    }
  }
}

impl<CS: CoordinateSystem> SampledPoint<'_, CS> {
  /// Gets the point on the navigation meshes.
  pub fn point(&self) -> CS::Coordinate {
    self.point.clone()
  }
}

/// An error while sampling a point.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
pub enum SamplePointError {
  #[error("The sample point is too far from any island.")]
  OutOfRange,
  #[error("The navigation data of the archipelago has been mutated since the last update.")]
  NavDataDirty,
}

/// Finds the nearest point on the navigation meshes to (and within
/// `distance_to_node` of) `point`.
pub(crate) fn sample_point<CS: CoordinateSystem>(
  archipelago: &Archipelago<CS>,
  point: CS::Coordinate,
  distance_to_node: f32,
) -> Result<SampledPoint<'_, CS>, SamplePointError> {
  if archipelago.nav_data.dirty {
    return Err(SamplePointError::NavDataDirty);
  }
  let Some((point, node_ref)) = archipelago
    .nav_data
    .sample_point(CS::to_landmass(&point), distance_to_node)
  else {
    return Err(SamplePointError::OutOfRange);
  };

  Ok(SampledPoint {
    point: CS::from_landmass(&point),
    node_ref,
    marker: PhantomData,
  })
}

#[cfg(test)]
#[path = "query_test.rs"]
mod test;
