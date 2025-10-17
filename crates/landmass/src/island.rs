use std::sync::Arc;

use slotmap::new_key_type;

use crate::{
  nav_mesh::CoreValidNavigationMesh,
  util::{BoundingBox, CoreTransform},
};

new_key_type! {
  /// The ID of an island.
  pub struct IslandId;
}

/// An island in an Archipelago. Each island holds a navigation mesh.
///
/// This is a "core" type meaning it has no generics. This type expresses
/// everything in the standard coordinate system.
pub struct CoreIsland {
  /// The transform from the Island's frame to the Archipelago's frame.
  pub(crate) transform: CoreTransform,
  /// The navigation mesh for the island.
  pub(crate) nav_mesh: Arc<CoreValidNavigationMesh>,

  /// The bounds of `nav_mesh` after being transformed by `transform`.
  pub(crate) transformed_bounds: BoundingBox,
  /// Whether the island has been updated recently.
  pub(crate) dirty: bool,
}

impl CoreIsland {
  pub fn new(
    transform: CoreTransform,
    nav_mesh: Arc<CoreValidNavigationMesh>,
  ) -> Self {
    Self {
      transformed_bounds: nav_mesh.get_bounds().transform(&transform),
      transform,
      nav_mesh,
      dirty: true,
    }
  }

  /// Gets the current transform of the island.
  pub fn get_transform(&self) -> &CoreTransform {
    &self.transform
  }

  /// Sets the current transform of the island.
  pub(crate) fn set_transform(&mut self, transform: CoreTransform) {
    self.transform = transform;
    self.dirty = true;

    self.transformed_bounds =
      self.nav_mesh.get_bounds().transform(&self.transform);
  }

  /// Gets the current navigation mesh used by the island.
  pub fn get_nav_mesh(&self) -> Arc<CoreValidNavigationMesh> {
    self.nav_mesh.clone()
  }

  /// Sets the navigation mesh of the island.
  pub(crate) fn set_nav_mesh(
    &mut self,
    nav_mesh: Arc<CoreValidNavigationMesh>,
  ) {
    self.nav_mesh = nav_mesh;
    self.dirty = true;

    self.transformed_bounds =
      self.nav_mesh.get_bounds().transform(&self.transform);
  }
}
