use std::{
  collections::{HashMap, HashSet},
  mem::swap,
  ops::{Deref, DerefMut},
  sync::{Arc, Mutex},
};

use disjoint::DisjointSet;
use geo::{BooleanOps, Coord, LineString, LinesIter, MultiPolygon, Polygon};
use glam::{Vec2, Vec3, Vec3Swizzles};
use kdtree::{distance::squared_euclidean, KdTree};
use slotmap::{new_key_type, HopSlotMap, SlotMap};

use crate::{
  geometry::edge_intersection,
  island::{Island, IslandId, IslandNavigationData},
  nav_mesh::MeshEdgeRef,
  util::{BoundingBox, BoundingBoxHierarchy},
  CoordinateSystem, Transform, ValidNavigationMesh,
};

/// The navigation data of a whole [`crate::Archipelago`]. This only includes
/// "static" features.
pub(crate) struct NavigationData<CS: CoordinateSystem> {
  /// The islands in the [`crate::Archipelago`].
  islands: HopSlotMap<IslandId, Island<CS>>,
  /// The "default" cost of each node type. This also defines the node types
  /// (excluding the `None` type which has an implicit cost of 0.0).
  node_type_to_cost: HopSlotMap<NodeType, f32>,
  /// Whether the navigation data has been mutated since the last update.
  /// Reading should not occur unless the navigation data is not dirty.
  pub(crate) dirty: bool,
  /// Maps a "region id" (consisting of the IslandId and the region in that
  /// island's nav mesh) to its "region number" (the number used in
  /// [`Self::region_connections`]).
  pub(crate) region_id_to_number: HashMap<(IslandId, usize), usize>,
  /// Connectedness of regions based on their "region number" in
  /// [`Self::region_id_to_number`].
  pub(crate) region_connections: Mutex<DisjointSet>,
  /// The links to other islands by [`crate::NodeRef`]
  pub(crate) boundary_links: SlotMap<BoundaryLinkId, BoundaryLink>,
  /// The links that can be taken from a particular node ref.
  pub(crate) node_to_boundary_link_ids:
    HashMap<NodeRef, HashSet<BoundaryLinkId>>,
  /// The nodes that have been modified.
  pub(crate) modified_nodes: HashMap<NodeRef, ModifiedNode>,
  /// The islands that have been deleted since the last update.
  pub(crate) deleted_islands: HashSet<IslandId>,
}

new_key_type! {
  /// A unique type of node.
  pub struct NodeType;
}

/// A reference to a node in the navigation data.
#[derive(PartialEq, Eq, Debug, Clone, Copy, Hash, PartialOrd, Ord)]
pub(crate) struct NodeRef {
  /// The island of the node.
  pub(crate) island_id: IslandId,
  /// The index of the node in the island.
  pub(crate) polygon_index: usize,
}

new_key_type! {
  /// The ID of a boundary link.
  pub struct BoundaryLinkId;
}

/// A single link between two nodes on the boundary of an island.
#[derive(PartialEq, Debug, Clone)]
pub(crate) struct BoundaryLink {
  /// The node that taking this link leads to.
  pub(crate) destination_node: NodeRef,
  /// The portal that this link occupies on the boundary of the source node.
  /// This is essentially the intersection of the linked islands' linkable
  /// edges.
  pub(crate) portal: (Vec3, Vec3),
  /// The cost of travelling across this link.
  pub(crate) cost: f32,
}

/// A node that has been modified (e.g., by being connected with a boundary link
/// to another island).
#[derive(PartialEq, Debug, Clone)]
pub(crate) struct ModifiedNode {
  /// The new (2D) edges that make up the boundary of this node. These are
  /// indices in the nav mesh this corresponds to. Indices larger than the nav
  /// mesh vertices refer to [`ModifiedNode::new_vertices`]. Note the boundary
  /// winds in the same direction as nav mesh polygons (CCW).
  pub(crate) new_boundary: Vec<(usize, usize)>,
  /// The "new" vertices (in world space) that are needed for the modified
  /// node. These are not vertices in the original nav mesh and should only
  /// be used by a single boundary edge.
  pub(crate) new_vertices: Vec<Vec2>,
}

impl<CS: CoordinateSystem> NavigationData<CS> {
  /// Creates new navigation data.
  pub(crate) fn new() -> Self {
    Self {
      islands: HopSlotMap::with_key(),
      node_type_to_cost: HopSlotMap::with_key(),
      // The navigation data is empty, so there's nothing to update (so not
      // dirty).
      dirty: false,
      region_id_to_number: HashMap::new(),
      region_connections: Mutex::new(DisjointSet::new()),
      boundary_links: SlotMap::with_key(),
      node_to_boundary_link_ids: HashMap::new(),
      modified_nodes: HashMap::new(),
      deleted_islands: HashSet::new(),
    }
  }

  /// Creates a new node type with the specified `cost`. The cost is a
  /// multiplier on the distance travelled along this node (essentially the cost
  /// per meter). Agents will prefer to travel along low-cost terrain. This node
  /// type is distinct from all other node types.
  pub(crate) fn create_node_type(&mut self, cost: f32) -> NodeType {
    self.node_type_to_cost.insert(cost)
  }

  /// Sets the cost of `node_type` to `cost`. See
  /// [`NavigationData::create_node_type`] for the meaning of cost. Returns
  /// false if the node type does not exist in this nav data. Otherwise,
  /// returns true.
  pub(crate) fn set_node_type_cost(
    &mut self,
    node_type: NodeType,
    cost: f32,
  ) -> bool {
    let Some(node_type_cost) = self.node_type_to_cost.get_mut(node_type) else {
      return false;
    };
    *node_type_cost = cost;
    true
  }

  /// Gets the cost of `node_type`. Returns [`None`] if `node_type` is not in
  /// this nav data.
  pub(crate) fn get_node_type_cost(&self, node_type: NodeType) -> Option<f32> {
    self.node_type_to_cost.get(node_type).copied()
  }

  /// Removes the node type from the navigation data. Returns false if any
  /// islands still use this node type (so the node type cannot be removed).
  /// Otherwise, returns true.
  pub(crate) fn remove_node_type(&mut self, node_type: NodeType) -> bool {
    if !self.node_type_to_cost.contains_key(node_type) {
      return false;
    }

    for island in self.islands.values() {
      let Some(nav_data) = island.nav_data.as_ref() else {
        continue;
      };

      for type_index in nav_data.nav_mesh.used_type_indices.iter() {
        let Some(island_node_type) =
          nav_data.type_index_to_node_type.get(type_index)
        else {
          continue;
        };

        if *island_node_type == node_type {
          return false;
        }
      }
    }

    self.node_type_to_cost.remove(node_type);
    true
  }

  /// Gets the current node types and their costs.
  pub(crate) fn get_node_types(
    &self,
  ) -> impl Iterator<Item = (NodeType, f32)> + '_ {
    self.node_type_to_cost.iter().map(|(node_type, &cost)| (node_type, cost))
  }

  /// Adds a new (empty) island to the navigation data.
  pub(crate) fn add_island(&mut self) -> IslandMut<'_, CS> {
    // Don't set the dirty flag since a new island doesn't need to be updated.
    let island_id = self.islands.insert(Island::new());
    IslandMut {
      id: island_id,
      island: self.islands.get_mut(island_id).unwrap(),
      dirty_flag: &mut self.dirty,
    }
  }

  /// Gets a borrow to the island with `id`.
  pub(crate) fn get_island(&self, id: IslandId) -> Option<&Island<CS>> {
    self.islands.get(id)
  }

  /// Gets a mutable borrow to the island with `id`.
  pub(crate) fn get_island_mut(
    &mut self,
    id: IslandId,
  ) -> Option<IslandMut<'_, CS>> {
    self.islands.get_mut(id).map(|island| IslandMut {
      id,
      island,
      dirty_flag: &mut self.dirty,
    })
  }

  pub(crate) fn get_island_ids(
    &self,
  ) -> impl ExactSizeIterator<Item = IslandId> + '_ {
    self.islands.keys()
  }

  /// Removes the island with `island_id`. Panics if the island ID is not in the
  /// navigation data.
  pub(crate) fn remove_island(&mut self, island_id: IslandId) {
    self.dirty = true;
    self
      .islands
      .remove(island_id)
      .expect("Island should be present in the Archipelago");
    self.deleted_islands.insert(island_id);
  }

  /// Finds the node nearest to (and within `distance_to_node` of) `point`.
  /// Returns the point on the nav data nearest to `point` and the reference to
  /// the corresponding node.
  pub(crate) fn sample_point(
    &self,
    point: Vec3,
    distance_to_node: f32,
  ) -> Option<(Vec3, NodeRef)> {
    let mut best_point = None;
    for (island_id, island) in self.islands.iter() {
      let nav_data = match &island.nav_data {
        None => continue,
        Some(data) => data,
      };

      let relative_point = nav_data.transform.apply_inverse(point);
      if !nav_data
        .nav_mesh
        .mesh_bounds
        .expand_by_size(Vec3::ONE * distance_to_node)
        .contains_point(relative_point)
      {
        continue;
      }

      let (sampled_point, sampled_node) = match nav_data
        .nav_mesh
        .sample_point(relative_point, distance_to_node)
      {
        Some(sampled) => sampled,
        None => continue,
      };

      let distance = relative_point.distance_squared(sampled_point);
      match best_point {
        Some((best_distance, _)) if distance >= best_distance => continue,
        _ => {}
      }

      best_point = Some((
        distance,
        (
          nav_data.transform.apply(sampled_point),
          NodeRef { island_id, polygon_index: sampled_node },
        ),
      ));
    }
    best_point.map(|(_, b)| b)
  }

  fn update_islands(
    &mut self,
    edge_link_distance: f32,
  ) -> (HashSet<BoundaryLinkId>, HashSet<IslandId>, HashSet<NodeRef>) {
    let mut dirty_islands = HashSet::new();
    for (island_id, island) in self.islands.iter_mut() {
      if island.dirty {
        island.dirty = false;
        dirty_islands.insert(island_id);
      }
    }

    let changed_islands = self
      .deleted_islands
      .union(&dirty_islands)
      .copied()
      .collect::<HashSet<_>>();

    let mut dropped_links = HashSet::new();
    let mut modified_node_refs_to_update = HashSet::new();
    if !self.deleted_islands.is_empty() || !dirty_islands.is_empty() {
      self.node_to_boundary_link_ids.retain(|node_ref, links| {
        if changed_islands.contains(&node_ref.island_id) {
          for &link in links.iter() {
            self.boundary_links.remove(link);
          }
          modified_node_refs_to_update.insert(*node_ref);
          dropped_links.extend(links.iter().copied());
          return false;
        }

        let links_before = links.len();

        links.retain(|&link_id| {
          let link = self.boundary_links.get(link_id).unwrap();
          let retain =
            !changed_islands.contains(&link.destination_node.island_id);
          if !retain {
            self.boundary_links.remove(link_id);
            dropped_links.insert(link_id);
          }
          retain
        });

        if links_before != links.len() {
          // If a node has a different set of boundary links, we need to
          // recompute that node.
          modified_node_refs_to_update.insert(*node_ref);
        }

        !links.is_empty()
      });
    }

    self.deleted_islands.clear();

    if dirty_islands.is_empty() {
      // No new or changed islands, so no need to check for new links.
      return (dropped_links, changed_islands, modified_node_refs_to_update);
    }

    let mut island_bounds = self
      .islands
      .iter()
      .filter_map(|(key, value)| {
        value
          .nav_data
          .as_ref()
          .filter(|nav_data| !nav_data.nav_mesh.mesh_bounds.is_empty())
          .map(|nav_data| (nav_data.transformed_bounds, Some(key)))
      })
      .collect::<Vec<_>>();
    // There are no islands with nav data, so no islands to link and prevents a
    // panic.
    if island_bounds.is_empty() {
      return (dropped_links, changed_islands, modified_node_refs_to_update);
    }
    let island_bbh = BoundingBoxHierarchy::new(&mut island_bounds);

    for &dirty_island_id in dirty_islands.iter() {
      let dirty_island = self.islands.get(dirty_island_id).unwrap();
      let dirty_nav_data = match &dirty_island.nav_data {
        None => continue,
        Some(n) => n,
      };
      if dirty_nav_data.nav_mesh.boundary_edges.is_empty() {
        continue;
      }
      let query = dirty_nav_data
        .transformed_bounds
        .expand_by_size(Vec3::ONE * edge_link_distance);
      let candidate_islands = island_bbh.query_box(query);
      // If the only candidate island is the dirty island itself, skip the rest.
      if candidate_islands.len() <= 1 {
        continue;
      }
      let dirty_island_edge_bbh = island_edges_bbh(dirty_nav_data);
      for &candidate_island_id in candidate_islands {
        if candidate_island_id == dirty_island_id {
          continue;
        }
        // `link_edges_between_islands` links forwards and backwards. This means
        // for a pair of dirty islands, we must only process them once, so only
        // execute `link_edges_between_islands` for one ordering of the island
        // ids and not the other.
        if dirty_islands.contains(&candidate_island_id)
          && candidate_island_id < dirty_island_id
        {
          continue;
        }
        let candidate_island = self.islands.get(candidate_island_id).unwrap();
        link_edges_between_islands(
          (dirty_island_id, dirty_island),
          (candidate_island_id, candidate_island),
          &dirty_island_edge_bbh,
          edge_link_distance,
          &mut self.boundary_links,
          &mut self.node_to_boundary_link_ids,
          &mut modified_node_refs_to_update,
        );
      }
    }

    (dropped_links, changed_islands, modified_node_refs_to_update)
  }

  fn update_modified_node(
    &mut self,
    node_ref: NodeRef,
    edge_link_distance: f32,
  ) {
    // Any node from an island that doesn't exist (deleted), or one without nav
    // data (the nav mesh was removed), should be removed.
    let Some(island_nav_data) = self
      .islands
      .get(node_ref.island_id)
      .and_then(|island| island.nav_data.as_ref())
    else {
      self.modified_nodes.remove(&node_ref);
      return;
    };
    // Any nodes without boundary links don't need to be modified.
    let Some(boundary_links) = self.node_to_boundary_link_ids.get(&node_ref)
    else {
      self.modified_nodes.remove(&node_ref);
      return;
    };

    let polygon = &island_nav_data.nav_mesh.polygons[node_ref.polygon_index];

    fn vec2_to_coord(v: Vec2) -> Coord<f32> {
      Coord::from((v.x, v.y))
    }

    fn coord_to_vec2(c: Coord<f32>) -> Vec2 {
      Vec2::new(c.x, c.y)
    }

    fn push_vertex<CS: CoordinateSystem>(
      vertex: usize,
      nav_data: &IslandNavigationData<CS>,
      line_string: &mut Vec<Coord<f32>>,
    ) {
      let vertex = nav_data.transform.apply(nav_data.nav_mesh.vertices[vertex]);
      line_string.push(vec2_to_coord(vertex.xy()));
    }

    let mut multi_line_string = vec![];
    let mut current_line_string = vec![];

    for (i, vertex) in polygon.vertices.iter().copied().enumerate() {
      if polygon.connectivity[i].is_some() {
        if !current_line_string.is_empty() {
          let mut line_string = vec![];
          swap(&mut current_line_string, &mut line_string);

          // Add the right vertex of the previous edge (the current vertex).
          push_vertex(vertex, island_nav_data, &mut line_string);

          multi_line_string.push(LineString(line_string));
        }
        continue;
      }

      // Add the left vertex. The right vertex will be added when we finish the
      // line string.
      push_vertex(vertex, island_nav_data, &mut current_line_string);
    }

    if !current_line_string.is_empty() {
      // The last "closing" edge must not be connected, so add the first vertex
      // to finish that edge and push it into the `multi_line_string`.
      push_vertex(
        polygon.vertices[0],
        island_nav_data,
        &mut current_line_string,
      );

      multi_line_string.push(LineString(current_line_string));
    }

    let boundary_edges = geo::MultiLineString(multi_line_string);

    fn boundary_link_to_clip_polygon(
      link: &BoundaryLink,
      edge_link_distance: f32,
    ) -> MultiPolygon<f32> {
      let flat_portal = (link.portal.0.xy(), link.portal.1.xy());
      let portal_forward =
        (flat_portal.1 - flat_portal.0).normalize().perp() * edge_link_distance;
      MultiPolygon::new(vec![Polygon::new(
        LineString(vec![
          vec2_to_coord(flat_portal.0 + portal_forward),
          vec2_to_coord(flat_portal.0 - portal_forward),
          vec2_to_coord(flat_portal.1 - portal_forward),
          vec2_to_coord(flat_portal.1 + portal_forward),
        ]),
        vec![],
      )])
    }

    let mut clip_polygons = boundary_links
      .iter()
      .map(|link_id| self.boundary_links.get(*link_id).unwrap())
      .map(|link| boundary_link_to_clip_polygon(link, edge_link_distance));

    let mut link_clip = clip_polygons.next().unwrap();
    for clip_polygon in clip_polygons {
      link_clip = link_clip.union(&clip_polygon);
    }

    let clipped_boundary_edges =
      link_clip.clip(&boundary_edges, /* invert= */ true);

    let mut original_vertices = KdTree::new(/* dimensions= */ 2);
    for &index in polygon.vertices.iter() {
      let vertex = island_nav_data
        .transform
        .apply(island_nav_data.nav_mesh.vertices[index]);

      original_vertices
        .add([vertex.x, vertex.y], index)
        .expect("Vertex is valid");
    }

    let modified_node =
      self.modified_nodes.entry(node_ref).or_insert_with(|| ModifiedNode {
        new_boundary: Vec::new(),
        new_vertices: Vec::new(),
      });

    for line_string in clipped_boundary_edges.iter() {
      for edge in line_string.lines_iter() {
        let start_index = original_vertices
          .nearest(
            &[edge.start.x, edge.start.y],
            /* num= */ 1,
            &squared_euclidean,
          )
          .unwrap()
          .first()
          .filter(|&(distance, _)| *distance < 0.01)
          .map(|&(_, &index)| index);

        let end_index = original_vertices
          .nearest(
            &[edge.end.x, edge.end.y],
            /* num= */ 1,
            &squared_euclidean,
          )
          .unwrap()
          .first()
          .filter(|&(distance, _)| *distance < 0.01)
          .map(|&(_, &index)| index);

        if let (Some(start_index), Some(end_index)) = (start_index, end_index) {
          // We don't want degenerate edges, so ignore edges with equal indices.
          if start_index == end_index {
            continue;
          }
        }

        let mut start_index = start_index.unwrap_or_else(|| {
          modified_node.new_vertices.push(coord_to_vec2(edge.start));
          island_nav_data.nav_mesh.vertices.len()
            + modified_node.new_vertices.len()
            - 1
        });

        let mut end_index = end_index.unwrap_or_else(|| {
          modified_node.new_vertices.push(coord_to_vec2(edge.end));
          island_nav_data.nav_mesh.vertices.len()
            + modified_node.new_vertices.len()
            - 1
        });

        let polygon_center =
          island_nav_data.transform.apply(polygon.center).xy();

        // Ensure the winding order of the modified node boundary matches the
        // polygon edges.
        if (coord_to_vec2(edge.start) - polygon_center)
          .perp_dot(coord_to_vec2(edge.end) - polygon_center)
          < 0.0
        {
          swap(&mut start_index, &mut end_index);
        }

        modified_node.new_boundary.push((start_index, end_index));
      }
    }
  }

  fn node_to_region_id(&self, node_ref: NodeRef) -> (IslandId, usize) {
    let region = self
      .islands
      .get(node_ref.island_id)
      .unwrap()
      .nav_data
      .as_ref()
      .unwrap()
      .nav_mesh
      .polygons[node_ref.polygon_index]
      .region;
    (node_ref.island_id, region)
  }

  /// Determines whether `node_1` and `node_2` can be connected by some path.
  pub(crate) fn are_nodes_connected(
    &self,
    node_1: NodeRef,
    node_2: NodeRef,
  ) -> bool {
    let region_id_1 = self.node_to_region_id(node_1);
    let region_id_2 = self.node_to_region_id(node_2);
    if region_id_1 == region_id_2 {
      // The regions are the same, so they are definitely connected. Skip all
      // the rest of the work.
      return true;
    }

    let Some(&region_number_1) = self.region_id_to_number.get(&region_id_1)
    else {
      // If the requested region is not in the `region_id_to_number` map, it is
      // not connected to any other region by a boundary link. Therefore, the
      // regions are unconnected.
      return false;
    };

    let Some(&region_number_2) = self.region_id_to_number.get(&region_id_2)
    else {
      // Same reasoning as above.
      return false;
    };

    self
      .region_connections
      .lock()
      .unwrap()
      .is_joined(region_number_1, region_number_2)
  }

  fn update_regions(&mut self) {
    self.region_id_to_number.clear();
    let mut region_connections = self.region_connections.lock().unwrap();
    region_connections.clear();

    for (node_ref, link) in
      self.node_to_boundary_link_ids.iter().flat_map(|(&node_ref, links)| {
        links
          .iter()
          .map(|link_id| self.boundary_links.get(*link_id).unwrap())
          .map(move |link| (node_ref, link))
      })
    {
      let start_region = self.node_to_region_id(node_ref);
      let end_region = self.node_to_region_id(link.destination_node);

      let start_region = *self
        .region_id_to_number
        .entry(start_region)
        .or_insert_with(|| region_connections.add_singleton());

      let end_region = *self
        .region_id_to_number
        .entry(end_region)
        .or_insert_with(|| region_connections.add_singleton());

      region_connections.join(start_region, end_region);
    }
  }

  pub(crate) fn update(
    &mut self,
    edge_link_distance: f32,
  ) -> (HashSet<BoundaryLinkId>, HashSet<IslandId>) {
    if !self.dirty {
      return (HashSet::new(), HashSet::new());
    }
    self.dirty = false;

    let (dropped_links, changed_islands, modified_node_refs_to_update) =
      self.update_islands(edge_link_distance);
    for node_ref in modified_node_refs_to_update {
      self.update_modified_node(node_ref, edge_link_distance);
    }
    if !changed_islands.is_empty() {
      self.update_regions();
    }
    (dropped_links, changed_islands)
  }
}

/// A mutable borrow to an island.
pub struct IslandMut<'nav_data, CS: CoordinateSystem> {
  /// The ID of the island.
  id: IslandId,
  /// The borrow.
  island: &'nav_data mut Island<CS>,
  /// A borrow to the navigation data's dirty flag. Mutating the island should
  /// set this flag.
  dirty_flag: &'nav_data mut bool,
}

impl<CS: CoordinateSystem> Deref for IslandMut<'_, CS> {
  type Target = Island<CS>;

  fn deref(&self) -> &Self::Target {
    self.island
  }
}

impl<CS: CoordinateSystem> DerefMut for IslandMut<'_, CS> {
  fn deref_mut(&mut self) -> &mut Self::Target {
    *self.dirty_flag = true;
    self.island
  }
}

impl<CS: CoordinateSystem> IslandMut<'_, CS> {
  /// Returns the ID of the borrowed island.
  pub fn id(&self) -> IslandId {
    self.id
  }

  /// Sets the navigation mesh and the transform of the island.
  /// `type_index_to_node_type` translates the type indices used in `nav_mesh`
  /// into [`NodeType`]s from the [`crate::Archipelago`]. Type indices without a
  /// corresponding node type will be treated as the "default" node type, which
  /// has a cost of 1.0. See [`crate::Archipelago::create_node_type`] for
  /// details on cost. [`NodeType`]s not present in the corresponding
  /// [`crate::Archipelago`] will cause a panic, so do not mix [`NodeType`]s
  /// across [`crate::Archipelago`]s. This matches
  /// [`crate::Island::set_nav_mesh`], but returns self for convenience.
  pub fn set_nav_mesh(
    &mut self,
    transform: Transform<CS>,
    nav_mesh: Arc<ValidNavigationMesh<CS>>,
    type_index_to_node_type: HashMap<usize, NodeType>,
  ) -> &mut Self {
    // Deref so we automatically trigger any change detection stuff.
    self.deref_mut().set_nav_mesh(transform, nav_mesh, type_index_to_node_type);
    self
  }
}

fn edge_ref_to_world_edge<CS: CoordinateSystem>(
  edge: MeshEdgeRef,
  nav_data: &IslandNavigationData<CS>,
) -> (Vec3, Vec3) {
  let (a, b) = nav_data.nav_mesh.get_edge_points(edge);
  (nav_data.transform.apply(a), nav_data.transform.apply(b))
}

fn edge_to_bbox(edge: (Vec3, Vec3)) -> BoundingBox {
  BoundingBox::new_box(edge.0, edge.0).expand_to_point(edge.1)
}

fn island_edges_bbh<CS: CoordinateSystem>(
  island_nav_data: &IslandNavigationData<CS>,
) -> BoundingBoxHierarchy<MeshEdgeRef> {
  let mut edge_bounds = island_nav_data
    .nav_mesh
    .boundary_edges
    .iter()
    .map(|edge| {
      (
        edge_to_bbox(edge_ref_to_world_edge(edge.clone(), island_nav_data)),
        Some(edge.clone()),
      )
    })
    .collect::<Vec<_>>();
  BoundingBoxHierarchy::new(&mut edge_bounds)
}

fn link_edges_between_islands<CS: CoordinateSystem>(
  island_1: (IslandId, &Island<CS>),
  island_2: (IslandId, &Island<CS>),
  island_1_edge_bbh: &BoundingBoxHierarchy<MeshEdgeRef>,
  edge_link_distance: f32,
  boundary_links: &mut SlotMap<BoundaryLinkId, BoundaryLink>,
  node_to_boundary_link_ids: &mut HashMap<NodeRef, HashSet<BoundaryLinkId>>,
  modified_node_refs_to_update: &mut HashSet<NodeRef>,
) {
  let island_1_nav_data = island_1.1.nav_data.as_ref().unwrap();
  let island_2_nav_data = island_2.1.nav_data.as_ref().unwrap();

  let edge_link_distance_squared = edge_link_distance * edge_link_distance;

  for island_2_edge_ref in island_2_nav_data.nav_mesh.boundary_edges.iter() {
    let island_2_edge =
      edge_ref_to_world_edge(island_2_edge_ref.clone(), island_2_nav_data);
    let island_2_edge_bbox = edge_to_bbox(island_2_edge)
      .expand_by_size(Vec3::ONE * edge_link_distance);

    for island_1_edge_ref in island_1_edge_bbh.query_box(island_2_edge_bbox) {
      let island_1_edge =
        edge_ref_to_world_edge(island_1_edge_ref.clone(), island_1_nav_data);
      if let Some(portal) = edge_intersection(
        island_1_edge,
        island_2_edge,
        edge_link_distance_squared,
      ) {
        if portal.0.distance_squared(portal.1) < edge_link_distance_squared {
          continue;
        }

        let node_1 = NodeRef {
          island_id: island_1.0,
          polygon_index: island_1_edge_ref.polygon_index,
        };
        let node_2 = NodeRef {
          island_id: island_2.0,
          polygon_index: island_2_edge_ref.polygon_index,
        };

        let polygon_center_1 = island_1_nav_data.transform.apply(
          island_1_nav_data.nav_mesh.polygons[island_1_edge_ref.polygon_index]
            .center,
        );
        let polygon_center_2 = island_2_nav_data.transform.apply(
          island_2_nav_data.nav_mesh.polygons[island_2_edge_ref.polygon_index]
            .center,
        );
        let portal_center = (portal.0 + portal.1) / 2.0;

        let cost = polygon_center_1.distance(portal_center)
          + polygon_center_2.distance(portal_center);

        let id_1 = boundary_links.insert(BoundaryLink {
          destination_node: node_2,
          portal,
          cost,
        });
        let id_2 = boundary_links.insert(BoundaryLink {
          destination_node: node_1,
          portal: (portal.1, portal.0),
          cost,
        });

        node_to_boundary_link_ids.entry(node_1).or_default().insert(id_1);
        node_to_boundary_link_ids.entry(node_2).or_default().insert(id_2);

        modified_node_refs_to_update.insert(node_1);
        modified_node_refs_to_update.insert(node_2);
      }
    }
  }
}

#[cfg(test)]
#[path = "nav_data_test.rs"]
mod test;
