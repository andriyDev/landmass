use std::{
  collections::{HashMap, HashSet, VecDeque},
  mem::swap,
  ops::{Deref, DerefMut},
  sync::Mutex,
};

use disjoint::DisjointSet;
use geo::{BooleanOps, Coord, LineString, LinesIter, MultiPolygon, Polygon};
use glam::{Vec2, Vec3, Vec3Swizzles};
use kdtree::{KdTree, distance::squared_euclidean};
use slotmap::{HopSlotMap, SlotMap, new_key_type};
use thiserror::Error;

use crate::{
  CoordinateSystem,
  coords::CorePointSampleDistance,
  geometry::edge_intersection,
  island::{Island, IslandId},
  link::{AnimationLink, AnimationLinkId, AnimationLinkState, NodePortal},
  nav_mesh::{MeshEdgeRef, nav_mesh_node_bbh},
  util::{BoundingBox, BoundingBoxHierarchy, RaySegment},
};

/// The navigation data of a whole [`crate::Archipelago`]. This only includes
/// "static" features.
pub(crate) struct NavigationData<CS: CoordinateSystem> {
  /// The islands in the [`crate::Archipelago`].
  islands: HopSlotMap<IslandId, Island<CS>>,
  /// The animation links in the [`crate::AnimationLink`].
  animation_links: HopSlotMap<AnimationLinkId, AnimationLinkState<CS>>,
  /// The "default" cost of each type index. Missing type indices default to a
  /// cost of 1.0.
  type_index_to_cost: HashMap<usize, f32>,
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
  /// Maps a region number (the key of [`Self::region_connections`]) to the set
  /// of links that **could** connect the regions if agents are allowed to use
  /// them.
  pub(crate) region_number_to_possible_links:
    HashMap<usize, Vec<(AnimationLinkId, usize)>>,
  /// The links that go off the mesh, connecting two node refs.
  pub(crate) off_mesh_links: SlotMap<OffMeshLinkId, OffMeshLink>,
  /// The links that can be taken from a particular node ref.
  pub(crate) node_to_off_mesh_link_ids:
    HashMap<NodeRef, HashSet<OffMeshLinkId>>,
  /// The nodes that have been modified.
  pub(crate) modified_nodes: HashMap<NodeRef, ModifiedNode>,
  /// The islands that have been deleted since the last update.
  pub(crate) deleted_islands: HashSet<IslandId>,
  /// The set of animation links created since the last update.
  new_animation_links: HashSet<AnimationLinkId>,
  /// The set of animation links deleted since the last update.
  deleted_animation_links: HashSet<AnimationLinkId>,
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
  /// The ID of an off mesh link.
  pub(crate) struct OffMeshLinkId;
}

/// A single link between two nodes that does not exist in the nav meshes.
#[derive(PartialEq, Debug, Clone)]
pub(crate) struct OffMeshLink {
  /// The node that taking this link leads to.
  pub(crate) destination_node: NodeRef,
  /// The type index of the destination node. This is stored for convenience
  /// since it is stable once the link is created.
  pub(crate) destination_type_index: usize,
  /// The portal that this link occupies on the source node. The portal is in
  /// world-space. For [`KindedOffMeshLink::BoundaryLink`], the portal points
  /// are written "left-to-right" wrt the start node center. For
  /// [`KindedOffMeshLink::AnimationLink`], this is written in no particular
  /// order and **must** be manually made left-to-right.
  pub(crate) portal: (Vec3, Vec3),
  /// The kind-specific data for the off mesh link.
  pub(crate) kinded: KindedOffMeshLink,
}

#[derive(PartialEq, Debug, Clone)]
pub(crate) enum KindedOffMeshLink {
  /// A single link between two nodes on the boundary of an island.
  BoundaryLink {
    /// The ID of the boundary link that goes back to the original node.
    reverse_link: OffMeshLinkId,
  },
  /// A link where the agent will play some animation (or otherwise handle the
  /// motion outside landmass) to get to a new location.
  AnimationLink {
    /// The portal that this link leads to. The portal is in world-space. This
    /// portal must be in the same orientation as [`OffMeshLink::portal`] (so
    /// portal.0 corresponds to destination_portal.0).
    destination_portal: (Vec3, Vec3),
    /// The cost to take this link.
    cost: f32,
    /// The animation link that produced this off-mesh link.
    animation_link: AnimationLinkId,
  },
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
      animation_links: HopSlotMap::with_key(),
      type_index_to_cost: HashMap::new(),
      // The navigation data is empty, so there's nothing to update (so not
      // dirty).
      dirty: false,
      region_id_to_number: HashMap::new(),
      region_connections: Mutex::new(DisjointSet::new()),
      region_number_to_possible_links: HashMap::default(),
      off_mesh_links: SlotMap::with_key(),
      node_to_off_mesh_link_ids: HashMap::new(),
      modified_nodes: HashMap::new(),
      deleted_islands: HashSet::new(),
      new_animation_links: HashSet::new(),
      deleted_animation_links: HashSet::new(),
    }
  }

  /// Sets the cost of `type_index` to `cost`. The cost is a multiplier on the
  /// distance travelled along this node (essentially the cost per meter).
  /// Agents will prefer to travel along low-cost terrain.
  pub(crate) fn set_type_index_cost(
    &mut self,
    type_index: usize,
    cost: f32,
  ) -> Result<(), SetTypeIndexCostError> {
    if cost <= 0.0 {
      return Err(SetTypeIndexCostError::NonPositiveCost(cost));
    }
    self.type_index_to_cost.insert(type_index, cost);
    Ok(())
  }

  /// Gets the cost of `type_index`.
  pub(crate) fn get_type_index_cost(&self, type_index: usize) -> Option<f32> {
    self.type_index_to_cost.get(&type_index).copied()
  }

  /// Gets the current type indices and their costs.
  pub(crate) fn get_type_index_costs(
    &self,
  ) -> impl Iterator<Item = (usize, f32)> + '_ {
    self
      .type_index_to_cost
      .iter()
      .map(|(&type_index, &cost)| (type_index, cost))
  }

  /// Adds a new island to the navigation data.
  pub(crate) fn add_island(&mut self, island: Island<CS>) -> IslandId {
    // A new island means a new nav mesh - so mark it dirty.
    self.dirty = true;
    self.islands.insert(island)
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

  pub(crate) fn add_animation_link(
    &mut self,
    link: AnimationLink<CS>,
  ) -> AnimationLinkId {
    self.dirty = true;
    let link_id = self.animation_links.insert(AnimationLinkState::new(link));
    self.new_animation_links.insert(link_id);
    link_id
  }

  pub(crate) fn remove_animation_link(&mut self, link_id: AnimationLinkId) {
    self.dirty = true;
    self.new_animation_links.remove(&link_id);
    self.deleted_animation_links.insert(link_id);
    self.animation_links.remove(link_id);
  }

  pub(crate) fn get_animation_link(
    &self,
    link_id: AnimationLinkId,
  ) -> Option<&AnimationLink<CS>> {
    self.animation_links.get(link_id).map(|state| &state.main_link)
  }

  pub fn get_animation_link_ids(
    &self,
  ) -> impl ExactSizeIterator<Item = AnimationLinkId> {
    self.animation_links.keys()
  }

  /// Finds the node nearest to (and within `distance_to_node` of) `point`.
  /// Returns the point on the nav data nearest to `point` and the reference to
  /// the corresponding node.
  pub(crate) fn sample_point(
    &self,
    point: Vec3,
    point_sample_distance: &CorePointSampleDistance,
  ) -> Option<(Vec3, NodeRef)> {
    let mut best_point = None;
    for (island_id, island) in self.islands.iter() {
      let relative_point = island.transform.apply_inverse(point);
      if !island
        .nav_mesh
        .mesh_bounds
        .add_to_corners(
          // Note we flip the distance_above and distance_below since we are
          // expanding the nav mesh bounding boxes. From the query point's
          // perspective, we need to sample up by distance_above, which is the
          // same as expanding the bounding box down by distance_above.
          Vec3::new(
            -point_sample_distance.horizontal_distance,
            -point_sample_distance.horizontal_distance,
            -point_sample_distance.distance_above,
          ),
          Vec3::new(
            point_sample_distance.horizontal_distance,
            point_sample_distance.horizontal_distance,
            point_sample_distance.distance_below,
          ),
        )
        .contains_point(relative_point)
      {
        continue;
      }

      let (sampled_point, sampled_node) = match island
        .nav_mesh
        .sample_point(relative_point, point_sample_distance)
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
          island.transform.apply(sampled_point),
          NodeRef { island_id, polygon_index: sampled_node },
        ),
      ));
    }
    best_point.map(|(_, b)| b)
  }

  fn update_islands(
    &mut self,
    edge_link_distance: f32,
    animation_link_distance: f32,
  ) -> (HashSet<OffMeshLinkId>, HashSet<IslandId>, HashSet<NodeRef>) {
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
    let mut changed_animation_links = HashSet::new();
    let mut modified_node_refs_to_update = HashSet::new();
    if !self.deleted_islands.is_empty()
      || !dirty_islands.is_empty()
      || !self.deleted_animation_links.is_empty()
    {
      self.node_to_off_mesh_link_ids.retain(|node_ref, links| {
        let mut has_dropped_boundary_link = false;

        if changed_islands.contains(&node_ref.island_id) {
          for &link in links.iter() {
            match self.off_mesh_links.remove(link).unwrap().kinded {
              KindedOffMeshLink::BoundaryLink { .. } => {
                has_dropped_boundary_link = true;
              }
              KindedOffMeshLink::AnimationLink { animation_link, .. } => {
                changed_animation_links.insert(animation_link);
              }
            }
          }
          if has_dropped_boundary_link {
            modified_node_refs_to_update.insert(*node_ref);
          }
          dropped_links.extend(links.iter().copied());
          return false;
        }

        links.retain(|&link_id| {
          let link = self.off_mesh_links.get(link_id).unwrap();
          let mut retain =
            !changed_islands.contains(&link.destination_node.island_id);
          match &link.kinded {
            KindedOffMeshLink::AnimationLink { animation_link, .. } => {
              retain = retain
                && !self.deleted_animation_links.contains(animation_link);
            }
            KindedOffMeshLink::BoundaryLink { .. } => {}
          };
          if !retain {
            match self.off_mesh_links.remove(link_id).unwrap().kinded {
              KindedOffMeshLink::BoundaryLink { .. } => {
                has_dropped_boundary_link = true;
              }
              KindedOffMeshLink::AnimationLink { animation_link, .. } => {
                changed_animation_links.insert(animation_link);
              }
            }

            dropped_links.insert(link_id);
          }
          retain
        });

        if has_dropped_boundary_link {
          // If a node has a different set of boundary links, we need to
          // recompute that node.
          modified_node_refs_to_update.insert(*node_ref);
        }

        !links.is_empty()
      });

      for &animation_link_id in changed_animation_links.iter() {
        let Some(animation_link) =
          self.animation_links.get_mut(animation_link_id)
        else {
          // The animation link has been deleted, so we don't care about any
          // changes.
          continue;
        };
        animation_link.start_portals.retain(|node_portal| {
          !changed_islands.contains(&node_portal.node.island_id)
        });
        animation_link.end_portals.retain(|node_portal| {
          !changed_islands.contains(&node_portal.node.island_id)
        });
      }
    }

    self.deleted_islands.clear();
    self.deleted_animation_links.clear();

    if dirty_islands.is_empty() && self.new_animation_links.is_empty() {
      // No changed islands or animation links, so no need to check for new
      // links.
      return (dropped_links, changed_islands, modified_node_refs_to_update);
    }

    // Consume any new animation links so no matter what happens now, the links
    // won't be considered new anymore.
    let new_animation_links = std::mem::take(&mut self.new_animation_links);

    let mut island_bounds = self
      .islands
      .iter()
      .filter(|(_, island)| !island.transformed_bounds.is_empty())
      .map(|(island_id, island)| (island.transformed_bounds, Some(island_id)))
      .collect::<Vec<_>>();
    // There are no islands with nav data, so no islands to link and prevents a
    // panic.
    if island_bounds.is_empty() {
      return (dropped_links, changed_islands, modified_node_refs_to_update);
    }
    let island_bbh = BoundingBoxHierarchy::new(&mut island_bounds);

    self.create_boundary_links(
      &island_bbh,
      &dirty_islands,
      edge_link_distance,
      &mut modified_node_refs_to_update,
    );

    self.create_off_mesh_links_for_animation_links(
      new_animation_links,
      &changed_islands,
      &island_bbh,
      animation_link_distance,
    );

    (dropped_links, changed_islands, modified_node_refs_to_update)
  }

  fn create_boundary_links(
    &mut self,
    island_bbh: &BoundingBoxHierarchy<IslandId>,
    dirty_islands: &HashSet<IslandId>,
    edge_link_distance: f32,
    modified_node_refs_to_update: &mut HashSet<NodeRef>,
  ) {
    for &dirty_island_id in dirty_islands.iter() {
      let dirty_island = self.islands.get(dirty_island_id).unwrap();
      if dirty_island.nav_mesh.boundary_edges.is_empty() {
        continue;
      }
      let query = dirty_island
        .transformed_bounds
        .expand_by_size(Vec3::ONE * edge_link_distance);
      let candidate_islands = island_bbh.query_box(query);
      // If the only candidate island is the dirty island itself, skip the rest.
      if candidate_islands.len() <= 1 {
        continue;
      }
      let dirty_island_edge_bbh = island_edges_bbh(dirty_island);
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
          &mut self.off_mesh_links,
          &mut self.node_to_off_mesh_link_ids,
          modified_node_refs_to_update,
        );
      }
    }
  }

  fn create_off_mesh_links_for_animation_links(
    &mut self,
    new_animation_links: HashSet<AnimationLinkId>,
    changed_islands: &HashSet<IslandId>,
    island_bbh: &BoundingBoxHierarchy<IslandId>,
    max_vertical_distance: f32,
  ) {
    fn portal_segment(
      portal: (Vec3, Vec3),
      interval: (f32, f32),
    ) -> (Vec3, Vec3) {
      (portal.0.lerp(portal.1, interval.0), portal.0.lerp(portal.1, interval.1))
    }

    #[expect(clippy::too_many_arguments)]
    fn create_link<CS: CoordinateSystem>(
      start_portal: &NodePortal,
      end_portal: &NodePortal,
      start_edge: (Vec3, Vec3),
      end_edge: (Vec3, Vec3),
      animation_link_id: AnimationLinkId,
      link: &AnimationLink<CS>,
      islands: &HopSlotMap<IslandId, Island<CS>>,
      off_mesh_links: &mut SlotMap<OffMeshLinkId, OffMeshLink>,
      node_to_off_mesh_link_ids: &mut HashMap<NodeRef, HashSet<OffMeshLinkId>>,
    ) {
      let intersection = (
        start_portal.interval.0.max(end_portal.interval.0),
        start_portal.interval.1.min(end_portal.interval.1),
      );
      if intersection.1 <= intersection.0 {
        return;
      }

      let link = off_mesh_links.insert(OffMeshLink {
        portal: portal_segment(start_edge, intersection),
        destination_node: end_portal.node,
        destination_type_index: islands
          .get(end_portal.node.island_id)
          .unwrap()
          .nav_mesh
          .polygons[end_portal.node.polygon_index]
          .type_index,
        kinded: KindedOffMeshLink::AnimationLink {
          destination_portal: portal_segment(end_edge, intersection),
          cost: link.cost,
          animation_link: animation_link_id,
        },
      });
      node_to_off_mesh_link_ids
        .entry(start_portal.node)
        .or_default()
        .insert(link);
    }

    let mut island_to_node_bbh = Default::default();
    for &animation_link_id in &new_animation_links {
      let state = self.animation_links.get_mut(animation_link_id).unwrap();
      let link = &state.main_link;

      let start_edge = (
        CS::to_landmass(&link.start_edge.0),
        CS::to_landmass(&link.start_edge.1),
      );
      let mut end_edge =
        (CS::to_landmass(&link.end_edge.0), CS::to_landmass(&link.end_edge.1));
      if start_edge.0 == start_edge.1 {
        // In case the user has a point start edge but a full end edge, turn the
        // end edge into a single point at the midpoint. This is kinda an error,
        // but there's a fairly reasonable fallback, so we just do that.
        let end_midpoint = end_edge.0.midpoint(end_edge.1);
        end_edge = (end_midpoint, end_midpoint);
      }
      let start_portals = world_portal_to_node_portals(
        start_edge,
        island_bbh,
        &self.islands,
        &mut island_to_node_bbh,
        max_vertical_distance,
      );

      if start_portals.is_empty() {
        continue;
      }

      let end_portals = world_portal_to_node_portals(
        end_edge,
        island_bbh,
        &self.islands,
        &mut island_to_node_bbh,
        max_vertical_distance,
      );

      if end_portals.is_empty() {
        continue;
      }

      state.start_portals = start_portals;
      state.end_portals = end_portals;

      // Connect the start portals to the end portals.
      for start_portal in state.start_portals.iter() {
        for end_portal in state.end_portals.iter() {
          create_link(
            start_portal,
            end_portal,
            start_edge,
            end_edge,
            animation_link_id,
            link,
            &self.islands,
            &mut self.off_mesh_links,
            &mut self.node_to_off_mesh_link_ids,
          );
        }
      }
    }

    for &island_id in changed_islands {
      let Some(island) = self.islands.get(island_id) else {
        // The island was deleted, resulting in being "changed".
        continue;
      };
      if island.nav_mesh.polygons.is_empty() {
        // Ignore empty islands.
        continue;
      }

      let mut node_bbh = None;
      for (animation_link_id, state) in self.animation_links.iter_mut() {
        if new_animation_links.contains(&animation_link_id) {
          // We already handled the new animation links above. Doing this here
          // would result in duplicate links.
          continue;
        }
        let link = &state.main_link;

        let start_edge = (
          CS::to_landmass(&link.start_edge.0),
          CS::to_landmass(&link.start_edge.1),
        );
        let mut end_edge = (
          CS::to_landmass(&link.end_edge.0),
          CS::to_landmass(&link.end_edge.1),
        );
        if start_edge.0 == start_edge.1 {
          // In case the user has a point start edge but a full end edge, turn
          // the end edge into a single point at the midpoint. This is kinda an
          // error, but there's a fairly reasonable fallback, so we just do
          // that.
          let midpoint = end_edge.0.midpoint(end_edge.1);
          end_edge = (midpoint, midpoint);
        }

        fn intersects(portal: (Vec3, Vec3), bounds: &BoundingBox) -> bool {
          if portal.0 == portal.1 {
            bounds.contains_point(portal.0)
          } else {
            bounds.intersects_ray_segment(&RaySegment::new(portal.0, portal.1))
          }
        }

        if node_bbh.is_none() {
          if !intersects(start_edge, &island.transformed_bounds)
            && !intersects(end_edge, &island.transformed_bounds)
          {
            // Neither the start or end edges intersects the island bounds, so
            // bail out early.
            continue;
          }
          node_bbh = Some(nav_mesh_node_bbh(
            island.nav_mesh.as_ref(),
            Vec3::new(0.0, 0.0, max_vertical_distance),
          ));
        }

        let node_bbh = node_bbh.as_ref().unwrap();
        fn sample_portal_edge<CS: CoordinateSystem>(
          edge: (Vec3, Vec3),
          island_id: IslandId,
          island: &Island<CS>,
          node_bbh: &BoundingBoxHierarchy<usize>,
          max_vertical_distance: f32,
        ) -> Vec<NodePortal> {
          if edge.0 == edge.1 {
            let point = island.transform.apply_inverse(edge.0);
            island
              .nav_mesh
              .sample_point(
                point,
                &CorePointSampleDistance {
                  distance_above: max_vertical_distance,
                  distance_below: max_vertical_distance,
                  horizontal_distance: 0.0,
                  vertical_preference_ratio: 1.0,
                },
              )
              .map(|(_, node)| NodePortal {
                interval: (0.0, 1.0),
                node: NodeRef { island_id, polygon_index: node },
              })
              .into_iter()
              .collect()
          } else {
            let edge = (
              island.transform.apply_inverse(edge.0),
              island.transform.apply_inverse(edge.1),
            );
            island
              .nav_mesh
              .sample_edge(edge, node_bbh, max_vertical_distance)
              .into_iter()
              .map(|edge| NodePortal {
                interval: edge.interval,
                node: NodeRef { island_id, polygon_index: edge.node },
              })
              .collect()
          }
        }
        let new_start_portals = sample_portal_edge(
          start_edge,
          island_id,
          island,
          node_bbh,
          max_vertical_distance,
        );
        let new_end_portals = sample_portal_edge(
          end_edge,
          island_id,
          island,
          node_bbh,
          max_vertical_distance,
        );

        for start_portal in new_start_portals.iter() {
          for end_portal in
            state.end_portals.iter().chain(new_end_portals.iter())
          {
            create_link(
              start_portal,
              end_portal,
              start_edge,
              end_edge,
              animation_link_id,
              link,
              &self.islands,
              &mut self.off_mesh_links,
              &mut self.node_to_off_mesh_link_ids,
            );
          }
        }

        for end_portal in new_end_portals.iter() {
          for start_portal in state.start_portals.iter() {
            create_link(
              start_portal,
              end_portal,
              start_edge,
              end_edge,
              animation_link_id,
              link,
              &self.islands,
              &mut self.off_mesh_links,
              &mut self.node_to_off_mesh_link_ids,
            );
          }
        }
        state.start_portals.extend(new_start_portals.into_iter());
        state.end_portals.extend(new_end_portals.into_iter());
      }
    }
  }

  fn update_modified_node(
    &mut self,
    node_ref: NodeRef,
    edge_link_distance: f32,
  ) {
    // Any node from an island that doesn't exist (deleted), or one without nav
    // data (the nav mesh was removed), should be removed.
    let Some(island) = self.islands.get(node_ref.island_id) else {
      self.modified_nodes.remove(&node_ref);
      return;
    };
    // Any nodes without off mesh links don't need to be modified.
    let Some(off_mesh_links) = self.node_to_off_mesh_link_ids.get(&node_ref)
    else {
      self.modified_nodes.remove(&node_ref);
      return;
    };

    // If none of the off mesh links for this node are boundary links, the node
    // doesn't need to be modified.
    'has_boundary_link: {
      for off_mesh_link in off_mesh_links {
        let link = self.off_mesh_links.get(*off_mesh_link).unwrap();
        if let KindedOffMeshLink::BoundaryLink { .. } = &link.kinded {
          break 'has_boundary_link;
        }
      }
      self.modified_nodes.remove(&node_ref);
      return;
    }

    let polygon = &island.nav_mesh.polygons[node_ref.polygon_index];

    fn vec2_to_coord(v: Vec2) -> Coord<f32> {
      Coord::from((v.x, v.y))
    }

    fn coord_to_vec2(c: Coord<f32>) -> Vec2 {
      Vec2::new(c.x, c.y)
    }

    fn push_vertex<CS: CoordinateSystem>(
      vertex: usize,
      island: &Island<CS>,
      line_string: &mut Vec<Coord<f32>>,
    ) {
      let vertex = island.transform.apply(island.nav_mesh.vertices[vertex]);
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
          push_vertex(vertex, island, &mut line_string);

          multi_line_string.push(LineString(line_string));
        }
        continue;
      }

      // Add the left vertex. The right vertex will be added when we finish the
      // line string.
      push_vertex(vertex, island, &mut current_line_string);
    }

    if !current_line_string.is_empty() {
      // The last "closing" edge must not be connected, so add the first vertex
      // to finish that edge and push it into the `multi_line_string`.
      push_vertex(polygon.vertices[0], island, &mut current_line_string);

      multi_line_string.push(LineString(current_line_string));
    }

    let boundary_edges = geo::MultiLineString(multi_line_string);

    fn boundary_link_to_clip_polygon(
      link: &OffMeshLink,
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

    let mut clip_polygons = off_mesh_links
      .iter()
      .map(|link_id| self.off_mesh_links.get(*link_id).unwrap())
      .map(|link| boundary_link_to_clip_polygon(link, edge_link_distance));

    let mut link_clip = clip_polygons.next().unwrap();
    for clip_polygon in clip_polygons {
      link_clip = link_clip.union(&clip_polygon);
    }

    let clipped_boundary_edges =
      link_clip.clip(&boundary_edges, /* invert= */ true);

    let mut original_vertices = KdTree::new(/* dimensions= */ 2);
    for &index in polygon.vertices.iter() {
      let vertex = island.transform.apply(island.nav_mesh.vertices[index]);

      original_vertices
        .add([vertex.x, vertex.y], index)
        .expect("Vertex is valid");
    }

    let mut modified_node =
      self.modified_nodes.entry(node_ref).insert_entry(ModifiedNode {
        new_boundary: Vec::new(),
        new_vertices: Vec::new(),
      });
    let modified_node = modified_node.get_mut();

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
          island.nav_mesh.vertices.len() + modified_node.new_vertices.len() - 1
        });

        let mut end_index = end_index.unwrap_or_else(|| {
          modified_node.new_vertices.push(coord_to_vec2(edge.end));
          island.nav_mesh.vertices.len() + modified_node.new_vertices.len() - 1
        });

        let polygon_center = island.transform.apply(polygon.center).xy();

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
    let region =
      self.islands.get(node_ref.island_id).unwrap().nav_mesh.polygons
        [node_ref.polygon_index]
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

    let region_connections = self.region_connections.lock().unwrap();
    // If the regions are already connected, we're done!
    if region_connections.is_joined(region_number_1, region_number_2) {
      return true;
    }

    // Otherwise, we may need to use some possible links to get to the correct
    // region. Use a BFS to see if we can find the region.
    let mut seen_regions = HashSet::new();
    let mut region_queue = VecDeque::new();
    seen_regions.insert(region_number_1);
    region_queue.push_back(region_number_1);
    while let Some(region_number) = region_queue.pop_front() {
      if region_number == region_number_2 {
        return true;
      }

      let Some(possible_links) =
        self.region_number_to_possible_links.get(&region_number)
      else {
        continue;
      };

      for (_possible_link, destination_region) in possible_links {
        if !seen_regions.insert(*destination_region) {
          continue;
        }
        // TODO: Check if we are allowed to use this link.
        region_queue.push_back(*destination_region);
      }
    }

    false
  }

  fn update_regions(&mut self) {
    self.region_id_to_number.clear();
    let mut region_connections = self.region_connections.lock().unwrap();
    region_connections.clear();

    let links =
      self.node_to_off_mesh_link_ids.iter().flat_map(|(&node_ref, links)| {
        links
          .iter()
          .map(|link_id| self.off_mesh_links.get(*link_id).unwrap())
          .map(move |link| (node_ref, link))
      });

    // First, join all the regions where we know for sure they will be
    // connected.
    for (node_ref, link) in links.clone() {
      match &link.kinded {
        KindedOffMeshLink::BoundaryLink { .. } => {}
        KindedOffMeshLink::AnimationLink { .. } => continue,
      }
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

    // Then, any links that could potentially link two regions (but not always)
    // are stored in `self.region_number_to_animation_links`.
    for (node_ref, link) in links {
      let animation_link_id = match &link.kinded {
        KindedOffMeshLink::BoundaryLink { .. } => continue,
        KindedOffMeshLink::AnimationLink { animation_link, .. } => {
          *animation_link
        }
      };

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
      // If both regions are already connected, there's no sense in recording
      // the link for potential connections.
      if region_connections.is_joined(start_region, end_region) {
        continue;
      }

      self
        .region_number_to_possible_links
        .entry(start_region)
        .or_default()
        .push((animation_link_id, end_region));
    }
  }

  pub(crate) fn update(
    &mut self,
    edge_link_distance: f32,
    animation_link_distance: f32,
  ) -> (HashSet<OffMeshLinkId>, HashSet<IslandId>) {
    if !self.dirty {
      return (HashSet::new(), HashSet::new());
    }
    self.dirty = false;

    let animation_links_changed = !self.new_animation_links.is_empty()
      || !self.deleted_animation_links.is_empty();

    let (dropped_links, changed_islands, modified_node_refs_to_update) =
      self.update_islands(edge_link_distance, animation_link_distance);
    for node_ref in modified_node_refs_to_update {
      self.update_modified_node(node_ref, edge_link_distance);
    }
    if animation_links_changed || !changed_islands.is_empty() {
      self.update_regions();
    }
    (dropped_links, changed_islands)
  }
}

/// An error for settings the cost of a type index.
#[derive(Clone, Copy, PartialEq, Error, Debug)]
pub enum SetTypeIndexCostError {
  #[error(
    "The provided cost {0} is non-positive. Type index costs must be positive."
  )]
  NonPositiveCost(f32),
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
}

fn edge_ref_to_world_edge<CS: CoordinateSystem>(
  edge: MeshEdgeRef,
  island: &Island<CS>,
) -> (Vec3, Vec3) {
  let (a, b) = island.nav_mesh.get_edge_points(edge);
  (island.transform.apply(a), island.transform.apply(b))
}

fn edge_to_bbox(edge: (Vec3, Vec3)) -> BoundingBox {
  BoundingBox::new_box(edge.0, edge.0).expand_to_point(edge.1)
}

fn island_edges_bbh<CS: CoordinateSystem>(
  island: &Island<CS>,
) -> BoundingBoxHierarchy<MeshEdgeRef> {
  let mut edge_bounds = island
    .nav_mesh
    .boundary_edges
    .iter()
    .map(|edge| {
      (
        edge_to_bbox(edge_ref_to_world_edge(edge.clone(), island)),
        Some(edge.clone()),
      )
    })
    .collect::<Vec<_>>();
  BoundingBoxHierarchy::new(&mut edge_bounds)
}

fn link_edges_between_islands<CS: CoordinateSystem>(
  (island_id_1, island_1): (IslandId, &Island<CS>),
  (island_id_2, island_2): (IslandId, &Island<CS>),
  island_1_edge_bbh: &BoundingBoxHierarchy<MeshEdgeRef>,
  edge_link_distance: f32,
  off_mesh_links: &mut SlotMap<OffMeshLinkId, OffMeshLink>,
  node_to_off_mesh_link_ids: &mut HashMap<NodeRef, HashSet<OffMeshLinkId>>,
  modified_node_refs_to_update: &mut HashSet<NodeRef>,
) {
  let edge_link_distance_squared = edge_link_distance * edge_link_distance;

  for island_2_edge_ref in island_2.nav_mesh.boundary_edges.iter() {
    let island_2_edge =
      edge_ref_to_world_edge(island_2_edge_ref.clone(), island_2);
    let island_2_edge_bbox = edge_to_bbox(island_2_edge)
      .expand_by_size(Vec3::ONE * edge_link_distance);

    for island_1_edge_ref in island_1_edge_bbh.query_box(island_2_edge_bbox) {
      let island_1_edge =
        edge_ref_to_world_edge(island_1_edge_ref.clone(), island_1);
      if let Some(portal) = edge_intersection(
        island_1_edge,
        island_2_edge,
        edge_link_distance_squared,
      ) {
        if portal.0.distance_squared(portal.1) < edge_link_distance_squared {
          continue;
        }

        let node_1 = NodeRef {
          island_id: island_id_1,
          polygon_index: island_1_edge_ref.polygon_index,
        };
        let node_2 = NodeRef {
          island_id: island_id_2,
          polygon_index: island_2_edge_ref.polygon_index,
        };

        let polygon_1 =
          &island_1.nav_mesh.polygons[island_1_edge_ref.polygon_index];
        let polygon_2 =
          &island_2.nav_mesh.polygons[island_2_edge_ref.polygon_index];

        let id_1 = off_mesh_links.insert(OffMeshLink {
          destination_node: node_2,
          destination_type_index: polygon_2.type_index,
          portal,
          // Set the reverse link to the default and we'll replace it with the
          // correct ID later.
          kinded: KindedOffMeshLink::BoundaryLink {
            reverse_link: OffMeshLinkId::default(),
          },
        });
        let id_2 = off_mesh_links.insert(OffMeshLink {
          destination_node: node_1,
          destination_type_index: polygon_1.type_index,
          portal: (portal.1, portal.0),
          kinded: KindedOffMeshLink::BoundaryLink { reverse_link: id_1 },
        });
        let KindedOffMeshLink::BoundaryLink { ref mut reverse_link } =
          off_mesh_links.get_mut(id_1).unwrap().kinded
        else {
          unreachable!();
        };
        *reverse_link = id_2;

        node_to_off_mesh_link_ids.entry(node_1).or_default().insert(id_1);
        node_to_off_mesh_link_ids.entry(node_2).or_default().insert(id_2);

        modified_node_refs_to_update.insert(node_1);
        modified_node_refs_to_update.insert(node_2);
      }
    }
  }
}

/// For a single portal in world-space, finds the corresponding portals on nodes
/// in nav meshes.
///
/// The order of portal points is not defined.
fn world_portal_to_node_portals<CS: CoordinateSystem>(
  portal: (Vec3, Vec3),
  island_bbh: &BoundingBoxHierarchy<IslandId>,
  islands: &HopSlotMap<IslandId, Island<CS>>,
  island_to_node_bbh: &mut HashMap<IslandId, BoundingBoxHierarchy<usize>>,
  max_vertical_distance: f32,
) -> Vec<NodePortal> {
  if portal.0 == portal.1 {
    return sample_animation_link_point(
      portal.0,
      island_bbh,
      islands,
      max_vertical_distance,
    )
    .into_iter()
    .map(|node| NodePortal { node, interval: (0.0, 1.0) })
    .collect();
  }

  let mut node_portals = vec![];
  let query_box = BoundingBox::Empty
    .expand_to_point(portal.0)
    .expand_to_point(portal.1)
    .expand_by_size(Vec3::new(0.0, 0.0, max_vertical_distance));
  for &island_id in island_bbh.query_box(query_box) {
    let island = islands.get(island_id).unwrap();
    if island.nav_mesh.polygons.is_empty() {
      // Ignore empty islands to prevent panics below.
      continue;
    }

    let node_bbh = island_to_node_bbh.entry(island_id).or_insert_with(|| {
      nav_mesh_node_bbh(
        island.nav_mesh.as_ref(),
        Vec3::new(0.0, 0.0, max_vertical_distance),
      )
    });
    let local_portal = (
      island.transform.apply_inverse(portal.0),
      island.transform.apply_inverse(portal.1),
    );

    node_portals.extend(
      island
        .nav_mesh
        .sample_edge(local_portal, node_bbh, max_vertical_distance)
        .into_iter()
        .map(|edge| NodePortal {
          node: NodeRef { island_id, polygon_index: edge.node },
          interval: edge.interval,
        }),
    );
  }
  node_portals
}

fn sample_animation_link_point<CS: CoordinateSystem>(
  point: Vec3,
  island_bbh: &BoundingBoxHierarchy<IslandId>,
  islands: &HopSlotMap<IslandId, Island<CS>>,
  max_vertical_distance: f32,
) -> Option<NodeRef> {
  let query_box = BoundingBox::new_box(
    point - Vec3::new(0.0, 0.0, max_vertical_distance),
    point + Vec3::new(0.0, 0.0, max_vertical_distance),
  );

  let mut best_point = None;
  for &island_id in island_bbh.query_box(query_box) {
    let island = islands.get(island_id).unwrap();
    let relative_point = island.transform.apply_inverse(point);
    let Some((sampled_point, sampled_node)) = island.nav_mesh.sample_point(
      relative_point,
      &CorePointSampleDistance {
        horizontal_distance: 0.0,
        distance_above: max_vertical_distance,
        distance_below: max_vertical_distance,
        vertical_preference_ratio: 1.0,
      },
    ) else {
      continue;
    };
    let distance = relative_point.distance_squared(sampled_point);
    match best_point {
      Some((best_distance, _)) if distance >= best_distance => continue,
      _ => {}
    }

    best_point =
      Some((distance, NodeRef { island_id, polygon_index: sampled_node }));
  }
  best_point.map(|(_, node)| node)
}

#[cfg(test)]
#[path = "nav_data_test.rs"]
mod test;
