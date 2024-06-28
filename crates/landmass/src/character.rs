use glam::Vec3;
use slotmap::new_key_type;

new_key_type! {
  /// The ID of a character.
  pub struct CharacterId;
}

/// A non-agent character. While agents are "managed" by the archipelago,
/// characters are only as obstacles to be avoided by agents.
#[derive(Default, Debug)]
pub struct Character {
  /// The current position of the character.
  pub position: Vec3,
  /// The current velocity of the character.
  pub velocity: Vec3,
  /// The radius of the character.
  pub radius: f32,
}
