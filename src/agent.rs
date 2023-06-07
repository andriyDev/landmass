use glam::Vec3;

use crate::path::Path;

pub type AgentId = u32;

pub struct Agent {
  pub(crate) position: Vec3,
  pub(crate) velocity: Vec3,
  pub(crate) radius: f32,
  pub(crate) max_velocity: f32,
  pub(crate) current_target: Option<Vec3>,
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

  pub fn get_position(&self) -> Vec3 {
    self.position
  }

  pub fn get_velocity(&self) -> Vec3 {
    self.velocity
  }

  pub fn get_desired_velocity(&self) -> Vec3 {
    self.current_desired_move
  }

  pub fn set_position(&mut self, position: Vec3) {
    self.position = position;
  }

  pub fn set_velocity(&mut self, velocity: Vec3) {
    self.velocity = velocity;
  }

  pub fn set_target(&mut self, target: Option<Vec3>) {
    self.current_target = target;
  }
}
