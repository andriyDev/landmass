use glam::{Vec2, Vec3};

/// A coordinate system used to convert from a user-facing coordinate system
/// into landmass's standard coordinate system. The standard coordinate system
/// is [`crate::coords::XYZ`].
pub trait CoordinateSystem {
  /// The user-facing coordinate type.
  type Coordinate: Clone;

  /// Converts a coordinate in this system to the standard coordinate system.
  fn to_landmass(v: &Self::Coordinate) -> Vec3;

  /// Converts a standard coordinate into this system's coordinate.
  fn from_landmass(v: &Vec3) -> Self::Coordinate;
}

/// The standard coordinate system, where X points right, Y points forward, and
/// Z points up.
pub struct XYZ;

impl CoordinateSystem for XYZ {
  type Coordinate = Vec3;

  fn to_landmass(v: &Self::Coordinate) -> Vec3 {
    *v
  }

  fn from_landmass(v: &Vec3) -> Self::Coordinate {
    *v
  }
}

/// A 2D coordinate system, where X points right, and Y points forward.
pub struct XY;

impl CoordinateSystem for XY {
  type Coordinate = Vec2;

  fn to_landmass(v: &Self::Coordinate) -> Vec3 {
    Vec3::new(v.x, v.y, 0.0)
  }

  fn from_landmass(v: &Vec3) -> Self::Coordinate {
    Vec2::new(v.x, v.z)
  }
}
