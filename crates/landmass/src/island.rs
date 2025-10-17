use std::sync::Arc;

use slotmap::new_key_type;

use crate::{
  CoordinateSystem, ValidNavigationMesh,
  nav_mesh::CoreValidNavigationMesh,
  util::{BoundingBox, Transform},
};

new_key_type! {
  /// The ID of an island.
  pub struct IslandId;
}

/// An Island in an Archipelago. Each island holds a navigation mesh.
pub struct Island<CS: CoordinateSystem> {
  /// The transform from the Island's frame to the Archipelago's frame.
  pub(crate) transform: Transform<CS>,
  /// The navigation mesh for the island.
  pub(crate) nav_mesh: Arc<CoreValidNavigationMesh>,

  /// The bounds of `nav_mesh` after being transformed by `transform`.
  pub(crate) transformed_bounds: BoundingBox,
  /// Whether the island has been updated recently.
  pub(crate) dirty: bool,
}

impl<CS: CoordinateSystem> Island<CS> {
  /// Creates a new island.
  pub fn new(
    transform: Transform<CS>,
    nav_mesh: ValidNavigationMesh<CS>,
  ) -> Self {
    let nav_mesh = nav_mesh.to_core();
    Self {
      transformed_bounds: nav_mesh.get_bounds().transform(&transform),
      transform,
      nav_mesh,
      dirty: true,
    }
  }

  /// Gets the current transform of the island.
  pub fn get_transform(&self) -> &Transform<CS> {
    &self.transform
  }

  /// Sets the current transform of the island.
  pub fn set_transform(&mut self, transform: Transform<CS>) {
    self.transform = transform;
    self.dirty = true;

    self.transformed_bounds =
      self.nav_mesh.get_bounds().transform(&self.transform);
  }

  /// Gets the current navigation mesh used by the island.
  pub fn get_nav_mesh(&self) -> ValidNavigationMesh<CS> {
    ValidNavigationMesh::new(self.nav_mesh.clone())
  }

  /// Sets the navigation mesh of the island.
  pub fn set_nav_mesh(&mut self, nav_mesh: ValidNavigationMesh<CS>) {
    self.nav_mesh = nav_mesh.to_core();
    self.dirty = true;

    self.transformed_bounds =
      self.nav_mesh.get_bounds().transform(&self.transform);
  }
}
