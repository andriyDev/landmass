use glam::Vec3;

use crate::path::Path;

pub type AgentId = u32;

pub struct Agent {
  position: Vec3,
  velocity: Vec3,
  radius: f32,
  max_velocity: f32,
  current_target: Option<Vec3>,
  current_path: Option<Path>,
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
    }
  }

  pub fn get_position(&self) -> Vec3 {
    self.position
  }

  pub fn get_velocity(&self) -> Vec3 {
    self.velocity
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
