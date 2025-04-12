use glam::{Vec2, Vec3};

/// A coordinate system used to convert from a user-facing coordinate system
/// into landmass's standard coordinate system. The standard coordinate system
/// is [`crate::coords::XYZ`].
pub trait CoordinateSystem {
  /// The user-facing coordinate type.
  type Coordinate: Clone;
  /// The type to use for point sampling options.
  type SampleDistance: PointSampleDistance;

  /// Converts a coordinate in this system to the standard coordinate system.
  fn to_landmass(v: &Self::Coordinate) -> Vec3;

  /// Converts a standard coordinate into this system's coordinate.
  fn from_landmass(v: &Vec3) -> Self::Coordinate;
}

/// A configuration of how a type relates to distances when sampling points. See
/// [`crate::Archipelago::sample_point`] for more.
pub trait PointSampleDistance {
  /// The horizontal distance that a node may be sampled. If a sample point is
  /// further than this distance away horizontally, it will be ignored.
  fn horizontal_distance(&self) -> f32;

  /// The vertical distance above the query point that a node may be sampled.
  ///
  /// If a sample point is further above than this distance, it will be ignored.
  /// For 2D coordinate systems, this value is not really used, but a value of
  /// 1.0 is good to avoid floating point errors. This value must be greater
  /// than negative [`PointSampleDistance::distance_below`] (and preferably
  /// positive).
  fn distance_above(&self) -> f32;

  /// The vertical distance below the query point that a node may be sampled.
  ///
  /// If a sample point is further below than this distance, it will be ignored.
  /// For 2D coordinate systems, this value is not really used, but a value of
  /// 1.0 is good to avoid floating point errors. This value must be less than
  /// than negative [`PointSampleDistance::distance_above`] (and preferably
  /// positive).
  fn distance_below(&self) -> f32;

  /// The ratio between the vertical and the horizontal distances to prefer. For
  /// example, if this value is 2.0, then a sample point directly below the
  /// query point 1.9 units away will be selected over a sample point 1.0 unit
  /// away horizontally. This value must be positive. For 2D coordinate systems,
  /// the value is irrelevant (so 1.0 is preferred).
  fn vertical_preference_ratio(&self) -> f32;
}

/// The standard coordinate system, where X points right, Y points forward, and
/// Z points up.
pub struct XYZ;

impl CoordinateSystem for XYZ {
  type Coordinate = Vec3;
  type SampleDistance = PointSampleDistance3d;

  fn to_landmass(v: &Self::Coordinate) -> Vec3 {
    *v
  }

  fn from_landmass(v: &Vec3) -> Self::Coordinate {
    *v
  }
}

/// A [`PointSampleDistance`] type for 3D coordinate systems.
#[derive(Debug, PartialEq, Clone)]
pub struct PointSampleDistance3d {
  /// The horizontal distance that a node may be sampled. If a sample point is
  /// further than this distance away horizontally, it will be ignored.
  pub horizontal_distance: f32,

  /// The vertical distance above the query point that a node may be sampled.
  ///
  /// If a sample point is further above than this distance, it will be
  /// ignored. This value must be greater than [`Self::distance_below`].
  pub distance_above: f32,

  /// The vertical distance below the query point that a node may be sampled.
  ///
  /// If a sample point is further below than this distance, it will be
  /// ignored. This value must be greater than [`Self::distance_above`].
  pub distance_below: f32,

  /// The ratio between the vertical and the horizontal distances to prefer.
  /// For example, if this value is 2.0, then a sample point directly below
  /// the query point 1.9 units away will be selected over a sample point 1.0
  /// unit away horizontally. This value must be positive.
  pub vertical_preference_ratio: f32,
}

impl PointSampleDistance for PointSampleDistance3d {
  fn horizontal_distance(&self) -> f32 {
    self.horizontal_distance
  }
  fn distance_above(&self) -> f32 {
    self.distance_above
  }
  fn distance_below(&self) -> f32 {
    self.distance_below
  }
  fn vertical_preference_ratio(&self) -> f32 {
    self.vertical_preference_ratio
  }
}

/// A 2D coordinate system, where X points right, and Y points forward.
pub struct XY;

impl CoordinateSystem for XY {
  type Coordinate = Vec2;
  type SampleDistance = f32;

  fn to_landmass(v: &Self::Coordinate) -> Vec3 {
    Vec3::new(v.x, v.y, 0.0)
  }

  fn from_landmass(v: &Vec3) -> Self::Coordinate {
    Vec2::new(v.x, v.y)
  }
}

impl PointSampleDistance for f32 {
  fn horizontal_distance(&self) -> f32 {
    *self
  }

  fn distance_above(&self) -> f32 {
    1.0
  }

  fn distance_below(&self) -> f32 {
    1.0
  }

  fn vertical_preference_ratio(&self) -> f32 {
    1.0
  }
}
