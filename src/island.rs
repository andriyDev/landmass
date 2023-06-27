use crate::{
  nav_mesh::MeshEdgeRef, util::Transform, BoundingBox, ValidNavigationMesh,
};

pub type IslandId = u32;

// An Island in an Archipelago. Islands are the region that an navigation mesh
// can be put into.
pub struct Island {
  // The bounding box that this island is responsible for. For tiled worlds,
  // this should be the bounding box of the tile. In other cases, it is usually
  // the bounding box of the navigation mesh.
  pub(crate) region_bounds: BoundingBox,
  // The navigation data, if present. May be missing for islands that have not
  // finished loading yet, or are having their data updated.
  pub(crate) nav_data: Option<IslandNavigationData>,
  // Whether the island has been updated recently.
  pub(crate) dirty: bool,
}

pub(crate) struct IslandNavigationData {
  // The transform from the Island's frame to the Archipelago's frame.
  pub transform: Transform,
  // The navigation mesh for the island.
  pub nav_mesh: ValidNavigationMesh,
  // The edges in `nav_mesh` that can be linked with other adjacent islands.
  pub linkable_edges: [Vec<MeshEdgeRef>; 6],
}

impl Island {
  pub(crate) fn new(region_bounds: BoundingBox) -> Self {
    Self {
      region_bounds,
      nav_data: None,
      // The island is dirty in case an island is removed then added back.
      dirty: true,
    }
  }

  pub fn set_nav_mesh(
    &mut self,
    transform: Transform,
    nav_mesh: ValidNavigationMesh,
    linkable_distance_to_region_edge: f32,
  ) {
    self.nav_data = Some(IslandNavigationData {
      transform,
      linkable_edges: nav_mesh.find_linkable_edges(
        self.region_bounds,
        linkable_distance_to_region_edge,
      ),
      nav_mesh,
    });
    self.dirty = true;
  }

  pub fn clear_nav_mesh(&mut self) {
    self.nav_data = None;
    self.dirty = true;
  }
}
