use std::sync::Arc;

use bevy_asset::Assets;
use bevy_ecs::{
  bundle::Bundle,
  component::Component,
  entity::Entity,
  query::With,
  system::{Query, Res},
};
use bevy_platform_support::collections::{HashMap, HashSet};
use bevy_transform::{components::Transform, helper::TransformHelper};

use crate::{
  coords::{CoordinateSystem, ThreeD, TwoD},
  Archipelago, ArchipelagoRef, NavMesh, NavMeshHandle,
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
  pub nav_mesh: NavMeshHandle<CS>,
}

pub type Island2dBundle = IslandBundle<TwoD>;
pub type Island3dBundle = IslandBundle<ThreeD>;

/// A marker component that an entity is an island.
#[derive(Component)]
#[require(Transform)]
pub struct Island;

/// Ensures that the island transform and nav mesh are up to date.
pub(crate) fn sync_islands_to_archipelago<CS: CoordinateSystem>(
  mut archipelagos: Query<(Entity, &mut Archipelago<CS>)>,
  islands: Query<
    (Entity, &NavMeshHandle<CS>, &ArchipelagoRef<CS>),
    (With<Island>, With<Transform>),
  >,
  transform_helper: TransformHelper,
  nav_meshes: Res<Assets<NavMesh<CS>>>,
) {
  let mut archipelago_to_islands = HashMap::<_, HashSet<_>>::default();
  for (island_entity, island_nav_mesh, archipelago_ref) in islands.iter() {
    let mut archipelago = match archipelagos.get_mut(archipelago_ref.entity) {
      Err(_) => continue,
      Ok((_, arch)) => arch,
    };

    let Some(island_nav_mesh) = nav_meshes.get(&island_nav_mesh.0) else {
      continue;
    };

    let Ok(island_transform) =
      transform_helper.compute_global_transform(island_entity)
    else {
      continue;
    };

    archipelago_to_islands
      .entry(archipelago_ref.entity)
      .or_default()
      .insert(island_entity);

    let island_transform = island_transform.compute_transform();
    let landmass_transform = landmass::Transform {
      translation: CS::from_bevy_position(island_transform.translation),
      rotation: CS::from_bevy_rotation(&island_transform.rotation),
    };

    match archipelago.get_island_mut(island_entity) {
      None => {
        let island_id =
          archipelago.archipelago.add_island(landmass::Island::new(
            landmass_transform,
            island_nav_mesh.nav_mesh.clone(),
            island_nav_mesh.type_index_to_node_type.clone(),
          ));
        archipelago.islands.insert(island_entity, island_id);
        archipelago.reverse_islands.insert(island_id, island_entity);
      }
      Some(mut island) => {
        if island.get_transform() != &landmass_transform {
          island.set_transform(landmass_transform);
        }
        if !Arc::ptr_eq(&island.get_nav_mesh(), &island_nav_mesh.nav_mesh) {
          island.set_nav_mesh(island_nav_mesh.nav_mesh.clone());
        }
        if island.get_type_index_to_node_type()
          != &island_nav_mesh.type_index_to_node_type
        {
          island.set_type_index_to_node_type(
            island_nav_mesh.type_index_to_node_type.clone(),
          );
        }
      }
    };
  }

  for (entity, mut archipelago) in archipelagos.iter_mut() {
    let islands = archipelago_to_islands.get(&entity);
    let archipelago = archipelago.as_mut();
    archipelago.islands.retain(|entity, id| {
      if islands.map(|islands| islands.contains(entity)).unwrap_or(false) {
        return true;
      }
      archipelago.archipelago.remove_island(*id);
      false
    });
    archipelago
      .reverse_islands
      .retain(|_, entity| archipelago.islands.contains_key(entity));
  }
}
