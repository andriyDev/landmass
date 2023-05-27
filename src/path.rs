use crate::nav_mesh::MeshNodeRef;

pub struct Path {
  pub(crate) corridor: Vec<MeshNodeRef>,
}
