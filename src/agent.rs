use glam::Vec3;

use crate::path::Path;

pub struct Agent {
  position: Vec3,
  velocity: Vec3,
  radius: f32,
  max_velocity: f32,
  current_path: Option<Path>,
}

impl Agent {
  pub fn create(
    position: Vec3,
    velocity: Vec3,
    radius: f32,
    max_velocity: f32,
  ) -> Self {
    Self { position, velocity, radius, max_velocity, current_path: None }
  }
}
