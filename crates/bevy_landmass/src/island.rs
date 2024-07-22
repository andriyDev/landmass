use std::sync::Arc;

use bevy::{
  asset::{Assets, Handle},
  math::EulerRot,
  prelude::{Bundle, Component, Entity, Query, Res, With},
  transform::components::GlobalTransform,
  utils::hashbrown::{HashMap, HashSet},
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

/// Ensures that the island transform and nav mesh are up to date.
pub(crate) fn sync_islands_to_archipelago<CS: CoordinateSystem>(
  mut archipelagos: Query<&mut Archipelago<CS>>,
  islands: Query<
    (Entity, &Handle<NavMesh<CS>>, &GlobalTransform, &ArchipelagoRef<CS>),
    With<Island>,
  >,
  nav_meshes: Res<Assets<NavMesh<CS>>>,
) {
  let mut archipelago_to_islands = HashMap::<_, HashSet<_>>::new();
  for (island_entity, island_nav_mesh, island_transform, archipelago_ref) in
    islands.iter()
  {
    let mut archipelago = match archipelagos.get_mut(archipelago_ref.entity) {
      Err(_) => continue,
      Ok(arch) => arch,
    };

    let Some(island_nav_mesh) = nav_meshes.get(island_nav_mesh) else {
      continue;
    };

    archipelago_to_islands
      .entry(archipelago_ref.entity)
      .or_default()
      .insert(island_entity);

    let island_transform = island_transform.compute_transform();
    let landmass_transform = landmass::Transform {
      translation: CS::from_transform_position(island_transform.translation),
      rotation: island_transform.rotation.to_euler(EulerRot::YXZ).0,
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

  for (archipelago, islands) in archipelago_to_islands {
    let mut archipelago = archipelagos.get_mut(archipelago).unwrap();
    let archipelago = archipelago.as_mut();
    archipelago.islands.retain(|entity, id| {
      if islands.contains(entity) {
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
