use bevy::{math::Vec3Swizzles, reflect::TypePath};
pub use landmass::CoordinateSystem as LandmassCoordinateSystem;
use landmass::PointSampleDistance3d;

/// A [`landmass::CoordinateSystem`] compatible with `bevy_landmass`.
pub trait CoordinateSystem:
  LandmassCoordinateSystem<
    Coordinate: Default + Send + Sync + PartialEq,
    SampleDistance: Send + Sync,
  > + TypePath
  + Send
  + Sync
{
  /// Converts a vertex from a mesh into this system's coordinate.
  fn from_mesh_vertex(v: &[f32; 3]) -> Self::Coordinate;

  /// Converts a position in Bevy into this system's coordinate.
  fn from_bevy_position(v: bevy::math::Vec3) -> Self::Coordinate;

  /// Converts a [`bevy::math::Quat`] into the corresponding rotation (in
  /// radians) in this system.
  fn from_bevy_rotation(rotation: &bevy::math::Quat) -> f32;

  /// Converts this system's coordinate into a world position.
  fn to_world_position(c: &Self::Coordinate) -> bevy::math::Vec3;
}

/// A 3D coordinate system where X is right, Y is up, and -Z is forward.
#[derive(TypePath)]
pub struct ThreeD;

impl LandmassCoordinateSystem for ThreeD {
  type Coordinate = bevy::math::Vec3;
  type SampleDistance = PointSampleDistance3d;

  fn to_landmass(v: &Self::Coordinate) -> landmass::Vec3 {
    landmass::Vec3::new(v.x, -v.z, v.y)
  }

  fn from_landmass(v: &landmass::Vec3) -> Self::Coordinate {
    bevy::math::Vec3::new(v.x, v.z, -v.y)
  }
}

impl CoordinateSystem for ThreeD {
  fn from_mesh_vertex(v: &[f32; 3]) -> Self::Coordinate {
    bevy::math::Vec3::new(v[0], v[1], v[2])
  }

  fn from_bevy_position(v: bevy::math::Vec3) -> Self::Coordinate {
    v
  }

  fn from_bevy_rotation(rotation: &bevy::math::Quat) -> f32 {
    rotation.to_euler(bevy::math::EulerRot::YXZ).0
  }

  fn to_world_position(c: &Self::Coordinate) -> bevy::math::Vec3 {
    *c
  }
}

/// A 2D coordinate system, where XY form the 2D plane.
#[derive(TypePath)]
pub struct TwoD;

impl LandmassCoordinateSystem for TwoD {
  type Coordinate = bevy::math::Vec2;
  type SampleDistance = f32;

  fn to_landmass(v: &Self::Coordinate) -> landmass::Vec3 {
    landmass::Vec3::new(v.x, v.y, 0.0)
  }

  fn from_landmass(v: &landmass::Vec3) -> Self::Coordinate {
    bevy::math::Vec2::new(v.x, v.y)
  }
}

impl CoordinateSystem for TwoD {
  fn from_mesh_vertex(v: &[f32; 3]) -> Self::Coordinate {
    bevy::math::Vec2::new(v[0], v[1])
  }

  fn from_bevy_position(v: bevy::math::Vec3) -> Self::Coordinate {
    v.xy()
  }

  fn from_bevy_rotation(rotation: &bevy::math::Quat) -> f32 {
    rotation.to_euler(bevy::math::EulerRot::ZXY).0
  }

  fn to_world_position(c: &Self::Coordinate) -> bevy::math::Vec3 {
    c.extend(0.0)
  }
}
