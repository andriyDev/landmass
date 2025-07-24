use std::{collections::HashMap, sync::Arc};

use slotmap::new_key_type;

use crate::{
  CoordinateSystem, NodeType, ValidNavigationMesh,
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
  pub(crate) nav_mesh: Arc<ValidNavigationMesh<CS>>,
  /// A map from the type indices used by [`Self::nav_mesh`] to the
  /// [`NodeType`]s used in the [`crate::Archipelago`].
  pub(crate) type_index_to_node_type: HashMap<usize, NodeType>,

  /// The bounds of `nav_mesh` after being transformed by `transform`.
  pub(crate) transformed_bounds: BoundingBox,
  /// Whether the island has been updated recently.
  pub(crate) dirty: bool,
}

impl<CS: CoordinateSystem> Island<CS> {
  /// Creates a new island. For details on the `type_index_to_node_type`
  /// argument, see [`Self::set_type_index_to_node_type`].
  pub fn new(
    transform: Transform<CS>,
    nav_mesh: Arc<ValidNavigationMesh<CS>>,
    type_index_to_node_type: HashMap<usize, NodeType>,
  ) -> Self {
    Self {
      transformed_bounds: nav_mesh.get_bounds().transform(&transform),
      transform,
      nav_mesh,
      type_index_to_node_type,
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
  pub fn get_nav_mesh(&self) -> Arc<ValidNavigationMesh<CS>> {
    self.nav_mesh.clone()
  }

  /// Sets the navigation mesh of the island.
  pub fn set_nav_mesh(&mut self, nav_mesh: Arc<ValidNavigationMesh<CS>>) {
    self.nav_mesh = nav_mesh;
    self.dirty = true;

    self.transformed_bounds =
      self.nav_mesh.get_bounds().transform(&self.transform);
  }

  /// Gets the current `type_index_to_node_type` used by the island.
  pub fn get_type_index_to_node_type(&self) -> &HashMap<usize, NodeType> {
    &self.type_index_to_node_type
  }

  /// Sets the "translation" from the type indices used in the navigation mesh
  /// into [`NodeType`]s from the [`crate::Archipelago`]. Type indices without a
  /// corresponding node type will be treated as the "default" node type, which
  /// has a cost of 1.0. See [`crate::Archipelago::add_node_type`] for
  /// details on cost. [`NodeType`]s not present in the corresponding
  /// [`crate::Archipelago`] will cause a panic, so do not mix [`NodeType`]s
  /// across [`crate::Archipelago`]s.
  pub fn set_type_index_to_node_type(
    &mut self,
    type_index_to_node_type: HashMap<usize, NodeType>,
  ) {
    self.type_index_to_node_type = type_index_to_node_type;
    self.dirty = true;
  }
}
