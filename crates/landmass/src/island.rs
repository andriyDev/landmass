use std::{collections::HashMap, sync::Arc};

use slotmap::new_key_type;

use crate::{
  util::{BoundingBox, Transform},
  CoordinateSystem, NodeType, ValidNavigationMesh,
};

new_key_type! {
  /// The ID of an island.
  pub struct IslandId;
}

/// An Island in an Archipelago. Islands are the region that an navigation mesh
/// can be put into.
pub struct Island<CS: CoordinateSystem> {
  /// The navigation data, if present. May be missing for islands that have not
  /// finished loading yet, or are having their data updated.
  pub(crate) nav_data: Option<IslandNavigationData<CS>>,
  /// Whether the island has been updated recently.
  pub(crate) dirty: bool,
}

/// The navigation data of an island.
pub(crate) struct IslandNavigationData<CS: CoordinateSystem> {
  /// The transform from the Island's frame to the Archipelago's frame.
  pub(crate) transform: Transform<CS>,
  /// The navigation mesh for the island.
  pub(crate) nav_mesh: Arc<ValidNavigationMesh<CS>>,
  /// A map from the type indices used by [`Self::nav_mesh`] to the
  /// [`NodeType`]s used in the [`crate::Archipelago`].
  pub(crate) type_index_to_node_type: HashMap<usize, NodeType>,

  // The bounds of `nav_mesh` after being transformed by `transform`.
  pub(crate) transformed_bounds: BoundingBox,
}

impl<CS: CoordinateSystem> Island<CS> {
  /// Creates a new island.
  pub(crate) fn new() -> Self {
    Self { nav_data: None, dirty: false }
  }

  /// Gets the current transform of the island.
  pub fn get_transform(&self) -> Option<&Transform<CS>> {
    self.nav_data.as_ref().map(|d| &d.transform)
  }

  /// Gets the current navigation mesh used by the island.
  pub fn get_nav_mesh(&self) -> Option<Arc<ValidNavigationMesh<CS>>> {
    self.nav_data.as_ref().map(|d| Arc::clone(&d.nav_mesh))
  }

  /// Gets the current `type_index_to_node_type` used by the island.
  pub fn get_type_index_to_node_type(
    &self,
  ) -> Option<&HashMap<usize, NodeType>> {
    self.nav_data.as_ref().map(|d| &d.type_index_to_node_type)
  }

  /// Sets the navigation mesh and the transform of the island.
  /// `type_index_to_node_type` translates the type indices used in `nav_mesh`
  /// into [`NodeType`]s from the [`crate::Archipelago`]. Type indices without a
  /// corresponding node type will be treated as the "default" node type, which
  /// has a cost of 1.0. See [`crate::Archipelago::add_node_type`] for
  /// details on cost. [`NodeType`]s not present in the corresponding
  /// [`crate::Archipelago`] will cause a panic, so do not mix [`NodeType`]s
  /// across [`crate::Archipelago`]s.
  pub fn set_nav_mesh(
    &mut self,
    transform: Transform<CS>,
    nav_mesh: Arc<ValidNavigationMesh<CS>>,
    type_index_to_node_type: HashMap<usize, NodeType>,
  ) {
    self.nav_data = Some(IslandNavigationData {
      transformed_bounds: nav_mesh.get_bounds().transform(&transform),
      transform,
      nav_mesh,
      type_index_to_node_type,
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
