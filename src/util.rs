pub fn bevy_vec3_to_glam_vec3(bevy_vec: bevy::prelude::Vec3) -> glam::Vec3 {
  glam::Vec3::new(bevy_vec.x, bevy_vec.y, bevy_vec.z)
}

pub fn glam_vec3_to_bevy_vec3(glam_vec: glam::Vec3) -> bevy::prelude::Vec3 {
  bevy::prelude::Vec3::new(glam_vec.x, glam_vec.y, glam_vec.z)
}
