use std::marker::PhantomData;

use glam::Vec3;
use slotmap::new_key_type;

use crate::CoordinateSystem;

new_key_type! {
  /// The ID of a character.
  pub struct CharacterId;
}

/// A non-agent character. While agents are "managed" by the archipelago,
/// characters are only as obstacles to be avoided by agents.
#[derive(Debug, Default)]
pub struct CoreCharacter {
  /// The current position of the character.
  pub position: Vec3,
  /// The current velocity of the character.
  pub velocity: Vec3,
  /// The radius of the character.
  pub radius: f32,
}

#[derive(Clone, Copy)]
pub struct CharacterRef<'a, CS: CoordinateSystem> {
  pub(crate) character: &'a CoreCharacter,
  pub(crate) marker: PhantomData<CS>,
}

impl<CS: CoordinateSystem> CharacterRef<'_, CS> {
  /// The current position of the character.
  pub fn position(&self) -> CS::Coordinate {
    CS::from_landmass(&self.character.position)
  }

  /// The current velocity of the character.
  pub fn velocity(&self) -> CS::Coordinate {
    CS::from_landmass(&self.character.velocity)
  }

  /// The radius of the character.
  pub fn radius(&self) -> f32 {
    self.character.radius
  }
}

pub struct CharacterMut<'a, CS: CoordinateSystem> {
  pub(crate) character: &'a mut CoreCharacter,
  pub(crate) marker: PhantomData<CS>,
}

impl<CS: CoordinateSystem> CharacterMut<'_, CS> {
  /// The current position of the character.
  pub fn position(&self) -> CS::Coordinate {
    CS::from_landmass(&self.character.position)
  }

  /// Sets the position of the character.
  pub fn set_position(&mut self, position: CS::Coordinate) {
    self.character.position = CS::to_landmass(&position);
  }

  /// The current velocity of the character.
  pub fn velocity(&self) -> CS::Coordinate {
    CS::from_landmass(&self.character.velocity)
  }

  /// Sets the velocity of the character.
  pub fn set_velocity(&mut self, velocity: CS::Coordinate) {
    self.character.velocity = CS::to_landmass(&velocity);
  }

  /// The radius of the character.
  pub fn radius(&self) -> f32 {
    self.character.radius
  }

  /// Sets the radius of the character.
  pub fn set_radius(&mut self, radius: f32) {
    self.character.radius = radius;
  }
}
