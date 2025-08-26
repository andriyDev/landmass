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
}

/// The state of an animation link.
pub(crate) struct AnimationLinkState<CS: CoordinateSystem> {
  /// The link given to us by the user.
  pub(crate) main_link: AnimationLink<CS>,
  /// The portals that this animation link can be taken from.
  pub(crate) start_portals: Vec<NodePortal>,
  /// The portals that this animation link leads to.
  pub(crate) end_portals: Vec<NodePortal>,
}

impl<CS: CoordinateSystem> AnimationLinkState<CS> {
  pub(crate) fn new(link: AnimationLink<CS>) -> Self {
    Self {
      main_link: link,
      start_portals: Default::default(),
      end_portals: Default::default(),
    }
  }
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

impl<CS: CoordinateSystem<Coordinate: std::fmt::Debug>> std::fmt::Debug
  for AnimationLinkState<CS>
{
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    f.debug_struct("AnimationLinkState")
      .field("main_link", &self.main_link)
      .field("start_portals", &self.start_portals)
      .field("end_portals", &self.end_portals)
      .finish()
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
