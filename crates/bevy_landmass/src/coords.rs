use bevy::math::Vec3Swizzles;
pub use landmass::CoordinateSystem as LandmassCoordinateSystem;

/// A [`landmass::CoordinateSystem`] compatible with `bevy_landmass`.
pub trait CoordinateSystem: LandmassCoordinateSystem {
  /// Converts a vertex from a mesh into this system's coordinate.
  fn from_mesh_vertex(v: &[f32; 3]) -> Self::Coordinate;
}

/// A 3D coordinate system where X is right, Y is up, and -Z is forward.
pub struct ThreeD;

impl LandmassCoordinateSystem for ThreeD {
  type Coordinate = bevy::math::Vec3;

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
}

/// A 2D coordinate system, where XY form the 2D plane.
pub struct TwoD;

impl LandmassCoordinateSystem for TwoD {
  type Coordinate = bevy::math::Vec2;

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
}
