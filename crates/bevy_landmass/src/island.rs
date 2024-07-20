use std::sync::Arc;

use bevy::{
  asset::{Assets, Handle},
  math::EulerRot,
  prelude::{Bundle, Component, Entity, Query, Res, With},
  transform::components::GlobalTransform,
  utils::{HashMap, HashSet},
};

use crate::{
  coords::{CoordinateSystem, ThreeD, TwoD},
  Archipelago, ArchipelagoRef, NavMesh,
};

/// A bundle to create islands. The GlobalTransform component is omitted, since
/// this is commonly added in other bundles (which is redundant and can
/// override previous bundles).
#[derive(Bundle)]
pub struct IslandBundle<CS: CoordinateSystem> {
  /// An island marker component.
  pub island: Island,
  /// A reference pointing to the Archipelago to associate this entity with.
  pub archipelago_ref: ArchipelagoRef<CS>,
  /// A handle to the nav mesh that this island needs.
  pub nav_mesh: Handle<NavMesh<CS>>,
}

pub type Island2dBundle = IslandBundle<TwoD>;
pub type Island3dBundle = IslandBundle<ThreeD>;

/// A marker component that an entity is an island.
#[derive(Component)]
pub struct Island;

/// Ensures every Bevy island has a corresponding `landmass` island.
pub(crate) fn add_islands_to_archipelago<CS: CoordinateSystem>(
  mut archipelago_query: Query<(Entity, &mut Archipelago<CS>)>,
  island_query: Query<(Entity, &ArchipelagoRef<CS>), With<Island>>,
) {
  let mut archipelago_to_islands = HashMap::<_, HashSet<_>>::new();
  for (entity, archipleago_ref) in island_query.iter() {
    archipelago_to_islands
      .entry(archipleago_ref.entity)
      .or_default()
      .insert(entity);
  }

  for (archipelago_entity, mut archipelago) in archipelago_query.iter_mut() {
    let mut new_islands = archipelago_to_islands
      .remove(&archipelago_entity)
      .unwrap_or_else(HashSet::new);
    let archipelago = archipelago.as_mut();

    // Remove any islands that aren't in the `new_islands`. Also remove any
    // islands from the `new_islands` that are in the archipelago.
    archipelago.islands.retain(|island_entity, island_id| {
      match new_islands.remove(island_entity) {
        false => {
          archipelago.archipelago.remove_island(*island_id);
          false
        }
        true => true,
      }
    });

    for new_island_entity in new_islands.drain() {
      let island_id = archipelago.archipelago.add_island().id();
      archipelago.islands.insert(new_island_entity, island_id);
    }
  }
}

/// Ensures that the island transform and nav mesh are up to date.
pub(crate) fn sync_island_nav_mesh<CS: CoordinateSystem>(
  mut archipelago_query: Query<&mut Archipelago<CS>>,
  island_query: Query<
    (
      Entity,
      Option<&Handle<NavMesh<CS>>>,
      Option<&GlobalTransform>,
      &ArchipelagoRef<CS>,
    ),
    With<Island>,
  >,
  nav_meshes: Res<Assets<NavMesh<CS>>>,
) {
  for (island_entity, island_nav_mesh, island_transform, archipelago_ref) in
    island_query.iter()
  {
    let mut archipelago =
      match archipelago_query.get_mut(archipelago_ref.entity) {
        Err(_) => continue,
        Ok(arch) => arch,
      };

    let mut landmass_island = match archipelago.get_island_mut(island_entity) {
      None => continue,
      Some(island) => island,
    };

    let island_nav_mesh = match island_nav_mesh {
      None => {
        if landmass_island.get_nav_mesh().is_some() {
          landmass_island.clear_nav_mesh();
        }
        continue;
      }
      Some(nav_mesh) => nav_mesh,
    };

    let island_nav_mesh = match nav_meshes.get(island_nav_mesh) {
      None => {
        if landmass_island.get_nav_mesh().is_some() {
          landmass_island.clear_nav_mesh();
        }
        continue;
      }
      Some(nav_mesh) => nav_mesh,
    };

    let island_transform = match island_transform {
      None => {
        if landmass_island.get_nav_mesh().is_some() {
          landmass_island.clear_nav_mesh();
        }
        continue;
      }
      Some(transform) => {
        let transform = transform.compute_transform();
        landmass::Transform {
          translation: CS::from_transform_position(transform.translation),
          rotation: transform.rotation.to_euler(EulerRot::YXZ).0,
        }
      }
    };

    let set_nav_mesh = match landmass_island.get_transform().map(|transform| {
      (
        transform,
        landmass_island.get_nav_mesh().unwrap(),
        landmass_island.get_type_index_to_node_type().unwrap(),
      )
    }) {
      None => true,
      Some((
        current_transform,
        current_nav_mesh,
        current_type_index_to_node_type,
      )) => {
        current_transform != &island_transform
          || !Arc::ptr_eq(&current_nav_mesh, &island_nav_mesh.nav_mesh)
          // TODO: This check is a little too expensive to do every frame.
          || current_type_index_to_node_type
            != &island_nav_mesh.type_index_to_node_type
      }
    };

    if set_nav_mesh {
      landmass_island.set_nav_mesh(
        island_transform,
        Arc::clone(&island_nav_mesh.nav_mesh),
        island_nav_mesh.type_index_to_node_type.clone(),
      );
    }
  }
}
