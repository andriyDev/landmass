use bevy_ecs::{
  bundle::Bundle,
  component::Component,
  entity::Entity,
  event::EntityEvent,
  lifecycle::{Remove, Replace},
  observer::On,
  query::{Changed, With},
  system::Query,
};

use crate::{
  Archipelago, ArchipelagoRef,
  coords::{CoordinateSystem, ThreeD, TwoD},
};

/// A bundle to create animation links.
#[derive(Bundle)]
pub struct AnimationLinkBundle<CS: CoordinateSystem> {
  /// The link itself.
  pub link: AnimationLink<CS>,
  /// A reference pointing to the Archipelago to associate this entity with.
  pub archipelago_ref: ArchipelagoRef<CS>,
}

pub type AnimationLink2dBundle = AnimationLinkBundle<TwoD>;
pub type AnimationLink3dBundle = AnimationLinkBundle<ThreeD>;

/// A link connecting two edges where an agent must perform some action (or
/// animation) to use the link.
///
/// This is often referred to as an off-mesh link in other navigation systems.
#[derive(Component)]
pub struct AnimationLink<CS: CoordinateSystem> {
  /// The edge that the agent must reach to use the animation link.
  ///
  /// The order of the edge is arbitrary.
  pub start_edge: (CS::Coordinate, CS::Coordinate),
  /// The edge that the agent will be sent to after using the animation link.
  ///
  /// The order of the edge must match the order of `start_edge`. So
  /// `start_edge.0` will take the agent to `end_edge.0` and the same for `.1`.
  pub end_edge: (CS::Coordinate, CS::Coordinate),
  /// The kind of the animation link.
  ///
  /// This is an arbitrary number that can be filtered on.
  pub kind: usize,
  /// The cost of taking this animation link.
  pub cost: f32,
  /// Whether the link can be traversed in either direction.
  ///
  /// This is a convenience to avoid needing to create two links to go in both
  /// directions.
  pub bidirectional: bool,
}

pub type AnimationLink2d = AnimationLink<TwoD>;
pub type AnimationLink3d = AnimationLink<ThreeD>;

impl<CS: CoordinateSystem> AnimationLink<CS> {
  /// Converts from the `bevy_landmass` animation link to the `landmass`
  /// version.
  pub(crate) fn to_landmass(&self) -> landmass::AnimationLink<CS> {
    landmass::AnimationLink {
      start_edge: self.start_edge.clone(),
      end_edge: self.end_edge.clone(),
      kind: self.kind,
      cost: self.cost,
      bidirectional: self.bidirectional,
    }
  }
}

/// Handles removing an [`AnimationLink`] component by trying to remove it from
/// the corresponding archipelago.
pub(crate) fn on_remove_animation_link<CS: CoordinateSystem>(
  event: On<Remove, AnimationLink<CS>>,
  archipelago_ref: Query<&ArchipelagoRef<CS>, With<AnimationLink<CS>>>,
  archipelago: Query<&mut Archipelago<CS>>,
) {
  try_remove_animation_link(event.event_target(), archipelago_ref, archipelago);
}

/// Handles replacing an [`ArchipelagoRef`] component by trying to remove an
/// animation link from the corresponding archipelago.
pub(crate) fn on_replace_archipelago_ref_from_animation_link<
  CS: CoordinateSystem,
>(
  trigger: On<Replace, ArchipelagoRef<CS>>,
  archipelago_ref: Query<&ArchipelagoRef<CS>, With<AnimationLink<CS>>>,
  archipelago: Query<&mut Archipelago<CS>>,
) {
  try_remove_animation_link(
    trigger.event_target(),
    archipelago_ref,
    archipelago,
  );
}

/// Tries to remove an animation link from the archipelago referenced on
/// `link_entity`. Does nothing if the link doesn't exist on the archipelago
/// (however that may be).
fn try_remove_animation_link<CS: CoordinateSystem>(
  link_entity: Entity,
  archipelago_ref: Query<&ArchipelagoRef<CS>, With<AnimationLink<CS>>>,
  mut archipelago: Query<&mut Archipelago<CS>>,
) {
  let Ok(archipelago_ref) = archipelago_ref.get(link_entity) else {
    return;
  };
  let Ok(mut archipelago) = archipelago.get_mut(archipelago_ref.entity) else {
    return;
  };
  let Some(link_id) = archipelago.animation_links.remove(&link_entity) else {
    return;
  };
  archipelago.archipelago.remove_animation_link(link_id);
  archipelago.reverse_animation_links.remove(&link_id);
}

/// Adds or changes animation links to match between the `bevy_landmass` version
/// and the underlying archipelago.
pub(crate) fn update_animation_links_to_archipelagos<CS: CoordinateSystem>(
  animation_links: Query<
    (Entity, &AnimationLink<CS>, &ArchipelagoRef<CS>),
    Changed<AnimationLink<CS>>,
  >,
  mut archipelagos: Query<&mut Archipelago<CS>>,
) {
  for (entity, animation_link, archipelago_ref) in animation_links.iter() {
    let Ok(mut archipelago) = archipelagos.get_mut(archipelago_ref.entity)
    else {
      continue;
    };
    // Try to remove the link first, so we don't have a stale version.
    if let Some(link_id) = archipelago.animation_links.remove(&entity) {
      archipelago.archipelago.remove_animation_link(link_id);
      archipelago.reverse_animation_links.remove(&link_id);
    }

    let link_id =
      archipelago.archipelago.add_animation_link(animation_link.to_landmass());
    archipelago.animation_links.insert(entity, link_id);
    archipelago.reverse_animation_links.insert(link_id, entity);
  }
}
