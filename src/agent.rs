use glam::Vec3;

use crate::path::Path;

pub type AgentId = u32;

pub struct Agent {
  pub position: Vec3,
  pub velocity: Vec3,
  pub radius: f32,
  pub max_velocity: f32,
  pub current_target: Option<Vec3>,
  pub(crate) current_path: Option<Path>,
  pub(crate) current_desired_move: Vec3,
}

impl Agent {
  pub fn create(
    position: Vec3,
    velocity: Vec3,
    radius: f32,
    max_velocity: f32,
  ) -> Self {
    Self {
      position,
      velocity,
      radius,
      max_velocity,
      current_target: None,
      current_path: None,
      current_desired_move: Vec3::ZERO,
    }
  }

  pub fn get_desired_velocity(&self) -> Vec3 {
    self.current_desired_move
  }
}
