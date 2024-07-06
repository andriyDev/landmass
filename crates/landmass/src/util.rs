use std::mem::swap;

use glam::{Quat, Vec3};
use ord_subset::OrdVar;

use crate::CoordinateSystem;

/// A bounding box.
#[derive(PartialEq, Clone, Copy, Debug)]
pub enum BoundingBox {
  /// The bounding box has no points in it.
  Empty,
  /// The bounding box has some points in it.
  Box {
    /// The minimum bounds of the bounding box.
    min: Vec3,
    /// The maximum bounds of the bounding box. Must be component-wise greather
    /// than or equal to `min`.
    max: Vec3,
  },
}

impl BoundingBox {
  /// Creates a box already with some data in it. `min` and `max` must already
  /// be valid - this is unchecked.
  pub fn new_box(min: Vec3, max: Vec3) -> Self {
    Self::Box { min, max }
  }

  /// Returns whether the box is empty or not.
  pub fn is_empty(&self) -> bool {
    matches!(self, Self::Empty)
  }

  /// Returns the bounds of the box, assuming it is non-empty.
  pub fn as_box(&self) -> (Vec3, Vec3) {
    match self {
      Self::Empty => panic!("BoundingBox is not a box."),
      &Self::Box { min, max } => (min, max),
    }
  }

  pub fn center(&self) -> Option<Vec3> {
    match self {
      Self::Empty => None,
      &Self::Box { min, max } => Some((min + max) * 0.5),
    }
  }

  /// Computes the size of the bounding box. Returns 0 if the bounds are empty.
  pub fn size(&self) -> Vec3 {
    match self {
      Self::Empty => Vec3::ZERO,
      &Self::Box { min, max } => max - min,
    }
  }

  /// Determines if the bounding box is valid (min <= max).
  pub fn is_valid(&self) -> bool {
    let size = self.size();
    size.x >= 0.0 && size.y >= 0.0 && size.z >= 0.0
  }

  /// Expands the bounding box to contain the `other`.
  pub fn expand_to_bounds(&self, other: &Self) -> Self {
    match (self, other) {
      (Self::Empty, Self::Empty) => Self::Empty,
      (Self::Box { .. }, Self::Empty) => *self,
      (Self::Empty, Self::Box { .. }) => *other,
      (
        Self::Box { min, max },
        Self::Box { min: other_min, max: other_max },
      ) => Self::Box { min: min.min(*other_min), max: max.max(*other_max) },
    }
  }

  /// Expands the bounding box to contain `point`. If the box was empty, it will
  /// now hold only the `point`.
  pub fn expand_to_point(&self, point: Vec3) -> Self {
    match self {
      Self::Empty => Self::Box { min: point, max: point },
      &Self::Box { min, max } => {
        Self::Box { min: min.min(point), max: max.max(point) }
      }
    }
  }

  /// Expands the bounding box by `size`. An empty bounding box will still be
  /// empty after this.
  pub fn expand_by_size(&self, size: Vec3) -> BoundingBox {
    let expanded_box = match self {
      BoundingBox::Empty => BoundingBox::Empty,
      &BoundingBox::Box { min, max } => {
        BoundingBox::Box { min: min - size, max: max + size }
      }
    };

    if !expanded_box.is_valid() {
      return BoundingBox::Empty;
    }

    expanded_box
  }

  /// Determines if `point` is in `self`.
  pub fn contains_point(&self, point: Vec3) -> bool {
    match self {
      Self::Empty => false,
      Self::Box { min, max } => {
        min.x <= point.x
          && point.x <= max.x
          && min.y <= point.y
          && point.y <= max.y
          && min.z <= point.z
          && point.z <= max.z
      }
    }
  }

  /// Determines if `other` is fully contained by `self`.
  pub fn contains_bounds(&self, other: &Self) -> bool {
    let (other_min, other_max) = match other {
      Self::Empty => return false,
      Self::Box { min, max } => (min, max),
    };
    match self {
      Self::Empty => false,
      Self::Box { min, max } => {
        min.x <= other_min.x
          && other_max.x <= max.x
          && min.y <= other_min.y
          && other_max.y <= max.y
          && min.z <= other_min.z
          && other_max.z <= max.z
      }
    }
  }

  /// Detemrines if `other` intersects `self` at all.
  pub fn intersects_bounds(&self, other: &Self) -> bool {
    let (other_min, other_max) = match other {
      Self::Empty => return false,
      Self::Box { min, max } => (min, max),
    };
    match self {
      Self::Empty => false,
      Self::Box { min, max } => {
        min.x <= other_max.x
          && other_min.x <= max.x
          && min.y <= other_max.y
          && other_min.y <= max.y
          && min.z <= other_max.z
          && other_min.z <= max.z
      }
    }
  }

  /// Creates a conservative bounding box around `self` after transforming it by
  /// `transform`.
  pub fn transform<CS: CoordinateSystem>(
    &self,
    transform: &Transform<CS>,
  ) -> Self {
    let (min, max) = match self {
      BoundingBox::Empty => return BoundingBox::Empty,
      BoundingBox::Box { min, max } => (min, max),
    };
    let flat_max = Vec3::new(max.x, max.y, min.z);

    let points = [
      transform.apply(*min),
      transform.apply(flat_max),
      transform.apply(Vec3::new(min.x, max.y, min.z)),
      transform.apply(Vec3::new(max.x, min.y, min.z)),
    ];

    BoundingBox::Box {
      min: Vec3::new(
        points
          .iter()
          .map(|p| p.x)
          .min_by(|a, b| a.partial_cmp(b).unwrap())
          .unwrap(),
        points
          .iter()
          .map(|p| p.y)
          .min_by(|a, b| a.partial_cmp(b).unwrap())
          .unwrap(),
        points[0].z,
      ),
      max: Vec3::new(
        points
          .iter()
          .map(|p| p.x)
          .max_by(|a, b| a.partial_cmp(b).unwrap())
          .unwrap(),
        points
          .iter()
          .map(|p| p.y)
          .max_by(|a, b| a.partial_cmp(b).unwrap())
          .unwrap(),
        points[0].z + (max.z - min.z),
      ),
    }
  }
}

/// A transform that can be applied to Vec3's.
pub struct Transform<CS: CoordinateSystem> {
  /// The translation to apply.
  pub translation: CS::Coordinate,
  /// The rotation to apply around the "up" direction. Specifically, the up
  /// direction is perpendicular to the plane of movement.
  pub rotation: f32,
}

// Manual Clone impl to avoid `CS` having a Clone bound itself.
impl<CS: CoordinateSystem> Clone for Transform<CS> {
  fn clone(&self) -> Self {
    Self {
      translation: self.translation.clone(),
      rotation: self.rotation.clone(),
    }
  }
}

// Manual Debug impl to avoid `CS` having a Debug bound itself.
impl<CS: CoordinateSystem<Coordinate: std::fmt::Debug>> std::fmt::Debug
  for Transform<CS>
{
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    f.debug_struct("Transform")
      .field("translation", &self.translation)
      .field("rotation", &self.rotation)
      .finish()
  }
}

// Manual Default impl to avoid `CS` having a Default bound itself.
impl<CS: CoordinateSystem<Coordinate: Default>> Default for Transform<CS> {
  fn default() -> Self {
    Self { translation: Default::default(), rotation: Default::default() }
  }
}

// Manual PartialEq impl to avoid `CS` having a PartialEq bound itself.
impl<CS: CoordinateSystem<Coordinate: PartialEq>> PartialEq for Transform<CS> {
  fn eq(&self, other: &Self) -> bool {
    self.translation == other.translation && self.rotation == other.rotation
  }
}

impl<CS: CoordinateSystem> Transform<CS> {
  /// Applies the transformation.
  pub(crate) fn apply(&self, point: Vec3) -> Vec3 {
    Quat::from_rotation_z(self.rotation) * point
      + CS::to_landmass(&self.translation)
  }

  /// Inverses the transformation.
  pub(crate) fn apply_inverse(&self, point: Vec3) -> Vec3 {
    Quat::from_rotation_z(-self.rotation)
      * (point - CS::to_landmass(&self.translation))
  }
}

#[derive(Clone)]
pub enum BoundingBoxHierarchy<ValueType> {
  Leaf {
    bounds: BoundingBox,
    value: ValueType,
  },
  Branch {
    bounds: BoundingBox,
    children:
      Box<(BoundingBoxHierarchy<ValueType>, BoundingBoxHierarchy<ValueType>)>,
  },
}

impl<ValueType> BoundingBoxHierarchy<ValueType> {
  /// Creates a hierarchy from values and their bounding boxes. The values are
  /// all expected to be Some, and the values will be moved into the hierarchy
  /// (leaving behind None).
  pub fn new(values: &mut [(BoundingBox, Option<ValueType>)]) -> Self {
    assert!(!values.is_empty());
    if values.len() == 1 {
      let mut value = (BoundingBox::Empty, None);
      swap(&mut values[0], &mut value);
      return Self::Leaf { bounds: value.0, value: value.1.unwrap() };
    }

    let bounding_box = values
      .iter()
      .map(|v| &v.0)
      .fold(BoundingBox::Empty, |acc, b| acc.expand_to_bounds(b));
    let bounds_size = bounding_box.size();
    if bounds_size.x > bounds_size.y && bounds_size.x > bounds_size.z {
      values.sort_by_key(|v| OrdVar::new_unchecked(v.0.center().unwrap().x))
    } else if bounds_size.y > bounds_size.z {
      values.sort_by_key(|v| OrdVar::new_unchecked(v.0.center().unwrap().y))
    } else {
      values.sort_by_key(|v| OrdVar::new_unchecked(v.0.center().unwrap().z))
    }

    let split_index = values.len() / 2;

    Self::Branch {
      bounds: bounding_box,
      children: Box::new((
        Self::new(&mut values[..split_index]),
        Self::new(&mut values[split_index..]),
      )),
    }
  }

  #[cfg(test)]
  fn depth(&self) -> u32 {
    match self {
      BoundingBoxHierarchy::Leaf { .. } => 1,
      BoundingBoxHierarchy::Branch { children, .. } => {
        1 + children.0.depth().max(children.1.depth())
      }
    }
  }

  pub fn query_box(&self, query: BoundingBox) -> Vec<&ValueType> {
    let mut result = Vec::new();
    if let BoundingBox::Box { .. } = &query {
      self.query_box_recursive(&query, &mut result);
    }
    result
  }

  fn query_box_recursive<'a, 'b>(
    &'a self,
    query: &'b BoundingBox,
    result: &'b mut Vec<&'a ValueType>,
  ) {
    match self {
      Self::Leaf { bounds, value } => {
        if query.intersects_bounds(bounds) {
          result.push(value);
        }
      }
      Self::Branch { bounds, children } => {
        if query.intersects_bounds(bounds) {
          children.0.query_box_recursive(query, result);
          children.1.query_box_recursive(query, result);
        }
      }
    }
  }
}

#[cfg(test)]
#[path = "util_test.rs"]
mod test;
