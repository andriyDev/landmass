use thiserror::Error;

use crate::{Archipelago, CoordinateSystem};

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
) -> Result<CS::Coordinate, SamplePointError> {
  if archipelago.nav_data.dirty {
    return Err(SamplePointError::NavDataDirty);
  }
  let Some((point, _)) = archipelago
    .nav_data
    .sample_point(CS::to_landmass(&point), distance_to_node)
  else {
    return Err(SamplePointError::OutOfRange);
  };

  Ok(CS::from_landmass(&point))
}

#[cfg(test)]
#[path = "query_test.rs"]
mod test;
