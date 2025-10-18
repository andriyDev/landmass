use glam::Vec3;
use slotmap::new_key_type;

use crate::{CoordinateSystem, nav_data::NodeRef};

new_key_type! {
  /// The ID of an [`AnimationLink`].
  pub struct AnimationLinkId;
}

/// A link connecting two edges where an agent must perform some action (or
/// animation) to use the link.
///
/// This is often referred to as an off-mesh link in other navigation systems.
///
/// This is a "core" type meaning it has no generics. This type expresses
/// everything in the standard coordinate system.
#[derive(Debug)]
pub struct CoreAnimationLink {
  /// The edge that the agent must reach to use the animation link.
  ///
  /// The order of the edge is arbitrary.
  pub start_edge: (Vec3, Vec3),
  /// The edge that the agent will be sent to after using the animation link.
  ///
  /// The order of the edge must match the order of `start_edge`. So
  /// `start_edge.0` will take the agent to `end_edge.0` and the same for `.1`.
  pub end_edge: (Vec3, Vec3),
  /// The kind of the animation link.
  ///
  /// This is an arbitrary number that can be filtered on.
  pub kind: usize,
  /// The cost of taking this animation link.
  pub cost: f32,
  /// Whether the link can be traversed in either direction.
  ///
  /// This is a convenience to avoid needing to create two links to go in both
  /// directions.
  pub bidirectional: bool,
}

/// A link connecting two edges where an agent must perform some action (or
/// animation) to use the link.
///
/// This is often referred to as an off-mesh link in other navigation systems.
pub struct AnimationLink<CS: CoordinateSystem> {
  /// The edge that the agent must reach to use the animation link.
  ///
  /// The order of the edge is arbitrary.
  pub start_edge: (CS::Coordinate, CS::Coordinate),
  /// The edge that the agent will be sent to after using the animation link.
  ///
  /// The order of the edge must match the order of `start_edge`. So
  /// `start_edge.0` will take the agent to `end_edge.0` and the same for `.1`.
  pub end_edge: (CS::Coordinate, CS::Coordinate),
  /// The kind of the animation link.
  ///
  /// This is an arbitrary number that can be filtered on.
  pub kind: usize,
  /// The cost of taking this animation link.
  pub cost: f32,
  /// Whether the link can be traversed in either direction.
  ///
  /// This is a convenience to avoid needing to create two links to go in both
  /// directions.
  pub bidirectional: bool,
}

impl<CS: CoordinateSystem<Coordinate: std::fmt::Debug>> std::fmt::Debug
  for AnimationLink<CS>
{
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    f.debug_struct("AnimationLink")
      .field("start_edge", &self.start_edge)
      .field("end_edge", &self.end_edge)
      .field("kind", &self.kind)
      .field("cost", &self.cost)
      .finish()
  }
}

impl<CS: CoordinateSystem> AnimationLink<CS> {
  pub(crate) fn to_core(&self) -> CoreAnimationLink {
    CoreAnimationLink {
      start_edge: (
        CS::to_landmass(&self.start_edge.0),
        CS::to_landmass(&self.start_edge.1),
      ),
      end_edge: (
        CS::to_landmass(&self.end_edge.0),
        CS::to_landmass(&self.end_edge.1),
      ),
      kind: self.kind,
      cost: self.cost,
      bidirectional: self.bidirectional,
    }
  }

  pub(crate) fn from_core(core: &CoreAnimationLink) -> Self {
    Self {
      start_edge: (
        CS::from_landmass(&core.start_edge.0),
        CS::from_landmass(&core.start_edge.1),
      ),
      end_edge: (
        CS::from_landmass(&core.end_edge.0),
        CS::from_landmass(&core.end_edge.1),
      ),
      kind: core.kind,
      cost: core.cost,
      bidirectional: core.bidirectional,
    }
  }
}

/// The state of an animation link.
#[derive(Debug)]
pub(crate) struct CoreAnimationLinkState {
  /// The link given to us by the user.
  pub(crate) main_link: CoreAnimationLink,
  /// The portals that this animation link can be taken from.
  pub(crate) start_portals: Vec<NodePortal>,
  /// The portals that this animation link leads to.
  pub(crate) end_portals: Vec<NodePortal>,
}

impl CoreAnimationLinkState {
  pub(crate) fn new(link: CoreAnimationLink) -> Self {
    Self {
      main_link: link,
      start_portals: Default::default(),
      end_portals: Default::default(),
    }
  }
}

/// A node portal created from a world portal.
#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) struct NodePortal {
  /// The node that this portal belongs to.
  pub(crate) node: NodeRef,
  /// The interval along the original world portal that this node portal takes
  /// up. The values are always in ascending order, and both are in the range
  /// [0-1]. This is a fraction along the world portal.
  pub(crate) interval: (f32, f32),
}
