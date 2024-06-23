use std::sync::Arc;

use crate::{util::Transform, BoundingBox, ValidNavigationMesh};

/// The ID of an island.
#[derive(PartialEq, Eq, PartialOrd, Ord, Clone, Copy, Hash, Debug)]
pub struct IslandId(pub(crate) u32);

/// An Island in an Archipelago. Islands are the region that an navigation mesh
/// can be put into.
pub struct Island {
  /// The navigation data, if present. May be missing for islands that have not
  /// finished loading yet, or are having their data updated.
  pub(crate) nav_data: Option<IslandNavigationData>,
  /// Whether the island has been updated recently.
  pub(crate) dirty: bool,
}

/// The navigation data of an island.
pub(crate) struct IslandNavigationData {
  /// The transform from the Island's frame to the Archipelago's frame.
  pub transform: Transform,
  /// The navigation mesh for the island.
  pub nav_mesh: Arc<ValidNavigationMesh>,

  // The bounds of `nav_mesh` after being transformed by `transform`.
  pub transformed_bounds: BoundingBox,
}

impl Island {
  /// Creates a new island.
  pub(crate) fn new() -> Self {
    Self {
      nav_data: None,
      // The island is dirty in case an island is removed then added back.
      dirty: true,
    }
  }

  /// Gets the current transform of the island.
  pub fn get_transform(&self) -> Option<Transform> {
    self.nav_data.as_ref().map(|d| d.transform)
  }

  /// Gets the current navigation mesh used by the island.
  pub fn get_nav_mesh(&self) -> Option<Arc<ValidNavigationMesh>> {
    self.nav_data.as_ref().map(|d| Arc::clone(&d.nav_mesh))
  }

  /// Sets the navigation mesh and the transform of the island.
  pub fn set_nav_mesh(
    &mut self,
    transform: Transform,
    nav_mesh: Arc<ValidNavigationMesh>,
  ) {
    self.nav_data = Some(IslandNavigationData {
      transformed_bounds: nav_mesh.get_bounds().transform(transform),
      transform,
      nav_mesh,
    });
    self.dirty = true;
  }

  /// Clears the navigation mesh from the island, making the island completely
  /// empty.
  pub fn clear_nav_mesh(&mut self) {
    self.nav_data = None;
    self.dirty = true;
  }
}
