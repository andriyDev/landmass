use std::{
  collections::{HashMap, HashSet},
  marker::PhantomData,
  sync::Arc,
};

use glam::Vec3;
use slotmap::new_key_type;
use thiserror::Error;

use crate::{
  CoordinateSystem, IslandId, NavigationData,
  link::AnimationLinkId,
  nav_data::{NodeRef, OffMeshLinkId},
  path::{Path, PathIndex, StraightPathStep},
};

new_key_type! {
  /// The ID of an agent.
  pub struct AgentId;
}

/// The state of an agent.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AgentState {
  /// The agent is idle, due to not having a target. Note this does not mean
  /// that they are motionless. An agent will still avoid nearby agents.
  Idle,
  /// The agent has reached their target. The agent may resume moving if the
  /// target moves or otherwise changes.
  ReachedTarget,
  /// The agent has reached an animation link along its path to the target.
  ///
  /// See [`Agent::reached_animation_link`] for details about the link.
  ReachedAnimationLink,
  /// The agent is currently using an animation link.
  UsingAnimationLink,
  /// The agent has a path and is moving towards their target.
  Moving,
  /// The agent is not on a nav mesh.
  AgentNotOnNavMesh,
  /// The target is not on a nav mesh.
  TargetNotOnNavMesh,
  /// The agent has a target but cannot find a path to it.
  NoPath,
  /// The agent is paused.
  Paused,
}

/// An agent in an archipelago.
pub struct CoreAgent {
  /// The current position of the agent.
  pub position: Vec3,
  /// The current velocity of the agent.
  pub velocity: Vec3,
  /// The radius of the agent.
  pub radius: f32,
  /// The speed the agent prefers to move at. This should often be set lower
  /// than the [`Self::max_speed`] to allow the agent to "speed up" in order to
  /// get out of another agent's way.
  pub desired_speed: f32,
  /// The maximum speed that the agent can move at.
  pub max_speed: f32,
  /// The current target to move towards. Modifying this every update is fine.
  /// Paths will be reused for target points near each other if possible.
  /// However, swapping between two distant targets every update can be
  /// detrimental to be performance.
  pub current_target: Option<Vec3>,
  /// The condition to test for reaching the target.
  pub target_reached_condition: TargetReachedCondition,
  /// The distance at which an animation link can be used.
  ///
  /// If [`None`], this will use the same distance as
  /// [`Self::target_reached_condition`] but behaving like
  /// [`TargetReachedCondition::StraightPathDistance`].
  pub animation_link_reached_distance: Option<f32>,
  /// The animation links that the agent is allowed to use.
  ///
  /// Note, changing this at runtime may result in the agent continuing on a
  /// path that still contains a previously allowed animation link.
  pub permitted_animation_links: PermittedAnimationLinks,
  /// Whether this agent is "paused". Paused agents are not considered for
  /// avoidance, and will not recompute their paths. However, their paths are
  /// still kept "consistent" - meaning that once the agent becomes unpaused,
  /// it can reuse that path if it is still valid and relevant (the agent still
  /// wants to go to the same place).
  pub paused: bool,
  #[cfg(feature = "debug-avoidance")]
  /// If true, avoidance debug data will be stored during update iterations.
  /// This can later be used for visualization.
  pub keep_avoidance_data: bool,
  /// Overrides for the "default" costs of each type index.
  pub(crate) override_type_index_to_cost: HashMap<usize, f32>,
  /// The current path of the agent. None if a path is unavailable or a new
  /// path has not been computed yet (i.e., no path).
  pub(crate) current_path: Option<Path>,
  /// The desired velocity of the agent to move towards its goal.
  pub(crate) current_desired_move: Vec3,
  /// The state of the agent.
  pub(crate) state: AgentState,
  /// The animation link that the agent has reached. This includes the
  /// animation link, and the off mesh link being used.
  pub(crate) current_animation_link: Option<CoreReachedAnimationLink>,
  /// Whether this agent is currently using an animation link.
  pub(crate) using_animation_link: bool,
  #[cfg(feature = "debug-avoidance")]
  /// The avoidance data from the most recent update iteration. Only populated
  /// if [`Self::keep_avoidance_data`] is true.
  pub(crate) avoidance_data: Option<dodgy_2d::debug::DebugData>,
}

/// An animation link that an agent has reached (in order to use it).
#[derive(Debug)]
pub struct CoreReachedAnimationLink {
  /// The ID of the animation link.
  pub link_id: AnimationLinkId,
  /// The point that the animation link starts at.
  pub start_point: Vec3,
  /// The expected point that using the animation link will take the agent to.
  pub end_point: Vec3,
}

/// An animation link that an agent has reached (in order to use it).
pub struct ReachedAnimationLink<CS: CoordinateSystem> {
  /// The ID of the animation link.
  pub link_id: AnimationLinkId,
  /// The point that the animation link starts at.
  pub start_point: CS::Coordinate,
  /// The expected point that using the animation link will take the agent to.
  pub end_point: CS::Coordinate,
}

impl<CS: CoordinateSystem<Coordinate: std::fmt::Debug>> std::fmt::Debug
  for ReachedAnimationLink<CS>
{
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    f.debug_struct("ReachedAnimationLink")
      .field("link_id", &self.link_id)
      .field("start_point", &self.start_point)
      .field("end_point", &self.end_point)
      .finish()
  }
}

impl<CS: CoordinateSystem> ReachedAnimationLink<CS> {
  /// Converts the core representation into this coordinate system.
  fn from_core(link: &CoreReachedAnimationLink) -> Self {
    Self {
      link_id: link.link_id,
      start_point: CS::from_landmass(&link.start_point),
      end_point: CS::from_landmass(&link.end_point),
    }
  }
}

/// The condition to consider the agent as having reached its target. When this
/// condition is satisfied, the agent will stop moving.
#[derive(Clone, Copy, Debug)]
pub enum TargetReachedCondition {
  /// The target is reached if it is within the provided (Euclidean) distance
  /// of the agent. Useful if the target is surrounded by small obstacles
  /// which don't need to be navigated around (e.g. the agent just needs to
  /// be close enough to shoot at the target, which is surrounded by cover).
  /// Alternatively, if the distance is low, this can simply mean "when the
  /// agent is really close to the target". If None, the agent's radius is
  /// used.
  Distance(Option<f32>),
  /// The target is reached if it is "visible" (there is a straight line from
  /// the agent to the target), and the target is within the provided
  /// (Euclidean) distance of the agent. Useful if the agent should be able
  /// to see the target (e.g. a companion character should remain visible to
  /// the player, but should ideally not stand too close). If None, the agent's
  /// radius is used.
  VisibleAtDistance(Option<f32>),
  /// The target is reached if the "straight line" path from the agent to the
  /// target is less than the provided distance. "Straight line" path means if
  /// the agent's path goes around a corner, the distance will be computed
  /// going around the corner. This can be more computationally expensive, as
  /// the straight line path must be computed every update. Useful for agents
  /// that care about the actual walking distance to the target. If None, the
  /// agent's radius is used.
  StraightPathDistance(Option<f32>),
}

impl Default for TargetReachedCondition {
  fn default() -> Self {
    Self::Distance(None)
  }
}

/// Defines the list of animation links that an agent is allowed to use.
#[derive(Clone, Default, Debug)]
pub enum PermittedAnimationLinks {
  /// Every animation link is permitted.
  #[default]
  All,
  /// Only animation links whose kind is in this set are permitted.
  Kinds(Arc<HashSet<usize>>),
}

impl PermittedAnimationLinks {
  /// Returns whether the animation link is permitted.
  #[inline]
  pub(crate) fn is_permitted(&self, kind: usize) -> bool {
    match self {
      Self::All => true,
      Self::Kinds(kinds) => kinds.contains(&kind),
    }
  }
}

impl CoreAgent {
  /// Creates a new agent.
  pub(crate) fn new(
    position: Vec3,
    velocity: Vec3,
    radius: f32,
    desired_speed: f32,
    max_speed: f32,
  ) -> Self {
    Self {
      position,
      velocity,
      radius,
      desired_speed,
      max_speed,
      current_target: None,
      target_reached_condition: TargetReachedCondition::Distance(None),
      animation_link_reached_distance: None,
      permitted_animation_links: PermittedAnimationLinks::All,
      paused: false,
      #[cfg(feature = "debug-avoidance")]
      keep_avoidance_data: false,
      override_type_index_to_cost: HashMap::new(),
      current_path: None,
      current_desired_move: Vec3::ZERO,
      state: AgentState::Idle,
      current_animation_link: None,
      using_animation_link: false,
      #[cfg(feature = "debug-avoidance")]
      avoidance_data: None,
    }
  }

  /// Sets the type index cost for this agent to `cost`. Returns false if the
  /// cost is <= 0.0. Otherwise returns true.
  pub fn override_type_index_cost(
    &mut self,
    type_index: usize,
    cost: f32,
  ) -> bool {
    if cost <= 0.0 {
      return false;
    }
    self.override_type_index_to_cost.insert(type_index, cost);
    true
  }

  /// Removes the override cost for `type_index`. Returns true if `type_index`
  /// was overridden, false otherwise.
  pub fn remove_overridden_type_index_cost(
    &mut self,
    type_index: usize,
  ) -> bool {
    self.override_type_index_to_cost.remove(&type_index).is_some()
  }

  /// Returns the currently overriden type index costs.
  pub fn get_type_index_cost_overrides(
    &self,
  ) -> impl Iterator<Item = (usize, f32)> + '_ {
    self
      .override_type_index_to_cost
      .iter()
      .map(|(&type_index, &cost)| (type_index, cost))
  }

  /// Returns the desired velocity. This will only be updated if `update` was
  /// called on the associated [`crate::Archipelago`].
  pub fn get_desired_velocity(&self) -> Vec3 {
    self.current_desired_move
  }

  /// Returns the state of the agent. This will only be updated if `update` was
  /// called on the associated [`crate::Archipelago`].
  pub fn state(&self) -> AgentState {
    self.state
  }

  /// Returns the animation link that the agent reached last update.
  ///
  /// Returns None if the previous update the agent did not reach the animation
  /// link. This includes paused agents and agents using the link.
  pub fn reached_animation_link(&self) -> Option<&CoreReachedAnimationLink> {
    self.current_animation_link.as_ref()
  }

  /// Starts taking an animation link.
  ///
  /// This effectively pauses the agent. Use [`Self::end_animation_link`] to
  /// undo this. After starting the animation link, the agent's state will be
  /// [`AgentState::UsingAnimationLink`] and the
  /// [`Self::reached_animation_link`] will be cleared, after the next update.
  pub fn start_animation_link(
    &mut self,
  ) -> Result<(), NotReachedAnimationLinkError> {
    if self.current_animation_link.is_none() {
      return Err(NotReachedAnimationLinkError);
    }
    self.using_animation_link = true;
    Ok(())
  }

  /// Finishes taking an animation link.
  ///
  /// This effectively just unpauses the agent.
  pub fn end_animation_link(
    &mut self,
  ) -> Result<(), NotUsingAnimationLinkError> {
    if !self.using_animation_link {
      return Err(NotUsingAnimationLinkError);
    }
    self.using_animation_link = false;
    Ok(())
  }

  /// Returns whether the agent is currently using an animation link.
  /// Essentially this reports whether we have called
  /// [`Self::start_animation_link`] without ending the link yet.
  pub fn is_using_animation_link(&self) -> bool {
    self.using_animation_link
  }

  /// Gets the distance at which to consider to reach animation links.
  pub(crate) fn animation_link_reached_distance(&self) -> f32 {
    if let Some(distance) = self.animation_link_reached_distance {
      return distance;
    }
    match self.target_reached_condition {
      TargetReachedCondition::Distance(distance)
      | TargetReachedCondition::VisibleAtDistance(distance)
      | TargetReachedCondition::StraightPathDistance(distance) => {
        distance.unwrap_or(self.radius)
      }
    }
  }

  /// Determines if this agent has reached its target. `next_waypoint` and
  /// `target_waypoint` are formatted as an index into the `path` and the point
  /// of the waypoint. `next_waypoint` is the next waypoint on the way to the
  /// target. `target_waypoint` is the final waypoint that corresponds to the
  /// target.
  pub(crate) fn has_reached_target(
    &self,
    path: &Path,
    nav_data: &NavigationData,
    sampled_point: Vec3,
    next_waypoint: (PathIndex, StraightPathStep),
    target_waypoint: (PathIndex, Vec3),
  ) -> bool {
    match self.target_reached_condition {
      TargetReachedCondition::Distance(distance) => {
        let distance = distance.unwrap_or(self.radius);
        sampled_point.distance_squared(target_waypoint.1) < distance * distance
      }
      TargetReachedCondition::VisibleAtDistance(distance) => 'result: {
        let distance = distance.unwrap_or(self.radius);
        let StraightPathStep::Waypoint(next_point) = next_waypoint.1 else {
          // If the next waypoint isn't just a walk step, we assume we can't see
          // the target (since there's no straight line walkable path).
          break 'result false;
        };
        next_waypoint.0 == target_waypoint.0
          && sampled_point.distance_squared(next_point) < distance * distance
      }
      TargetReachedCondition::StraightPathDistance(distance) => 'result: {
        let distance = distance.unwrap_or(self.radius);
        // Check Euclidean distance first so we don't do the expensive path
        // following if the agent is not even close.
        if sampled_point.distance_squared(target_waypoint.1)
          > distance * distance
        {
          break 'result false;
        }

        // If the next waypoint is the target point, then we've already
        // computed the straight line distance and it is below the limit.
        if next_waypoint.0 == target_waypoint.0 {
          break 'result true;
        }

        let StraightPathStep::Waypoint(next_point) = next_waypoint.1 else {
          // If the next waypoint isn't just a walk step, we don't consider
          // ourselves within the straight path distance (since an animation
          // link does not count as a straight path).
          break 'result false;
        };

        let mut straight_line_distance = sampled_point.distance(next_point);
        let mut current_waypoint = (next_waypoint.0, next_point);

        while current_waypoint.0 != target_waypoint.0
          && straight_line_distance < distance
        {
          let next_waypoint = path.find_next_point_in_straight_path(
            nav_data,
            current_waypoint.0,
            current_waypoint.1,
            target_waypoint.0,
            target_waypoint.1,
          );

          let StraightPathStep::Waypoint(next_point) = next_waypoint.1 else {
            // If the next waypoint isn't just a walk step, we don't consider
            // ourselves within the straight path distance (since an animation
            // link does not count as a straight path).
            break 'result false;
          };

          straight_line_distance += current_waypoint.1.distance(next_point);
          current_waypoint = (next_waypoint.0, next_point);
        }

        straight_line_distance < distance
      }
    }
  }
}

/// An immutable borrow to an agent.
#[derive(Clone, Copy)]
pub struct AgentRef<'a, CS: CoordinateSystem> {
  /// The borrow to the core representation.
  pub(crate) agent: &'a CoreAgent,
  pub(crate) marker: PhantomData<CS>,
}

impl<'a, CS: CoordinateSystem> AgentRef<'a, CS> {
  /// Creates a ref from an agent borrow.
  pub(crate) fn new(agent: &'a CoreAgent) -> Self {
    Self { agent, marker: PhantomData }
  }

  /// The position of the agent.
  pub fn position(&self) -> CS::Coordinate {
    CS::from_landmass(&self.agent.position)
  }

  /// The velocity of the agent.
  pub fn velocity(&self) -> CS::Coordinate {
    CS::from_landmass(&self.agent.velocity)
  }

  /// The radius of the agent.
  pub fn radius(&self) -> f32 {
    self.agent.radius
  }

  /// The desired speed of the agent. See [`CoreAgent::desired_speed`] for more.
  pub fn desired_speed(&self) -> f32 {
    self.agent.desired_speed
  }

  /// The maximum speed that the agent can move at.
  pub fn max_speed(&self) -> f32 {
    self.agent.desired_speed
  }

  /// The current target to move towards. See [`CoreAgent::current_target`] for
  /// more.
  pub fn current_target(&self) -> Option<CS::Coordinate> {
    self.agent.current_target.as_ref().map(CS::from_landmass)
  }

  /// The condition to test for reaching the target.
  pub fn target_reached_condition(&self) -> TargetReachedCondition {
    self.agent.target_reached_condition
  }

  /// The distance at which an animation link can be used. See
  /// [`CoreAgent::animation_link_reached_distance`] for more.
  // TODO: FIX THIS DOC LINK.
  pub fn animation_link_reached_distance(&self) -> Option<f32> {
    self.agent.animation_link_reached_distance
  }

  /// The animation links that the agent is allowed to use. See
  /// [`CoreAgent::permitted_animation_links`].
  pub fn permitted_animation_links(&self) -> &PermittedAnimationLinks {
    &self.agent.permitted_animation_links
  }

  /// Whether this agent is paused. See [`CoreAgent::paused`] for more.
  pub fn paused(&self) -> bool {
    self.agent.paused
  }

  #[cfg(feature = "debug-avoidance")]
  /// Whether to store avoidance debug data. See
  /// [`CoreAgent::keep_avoidance_data`] for more.
  pub fn keep_avoidance_data(&self) -> bool {
    self.agent.keep_avoidance_data
  }

  /// Returns the currently overriden type index costs.
  pub fn get_type_index_cost_overrides(
    &self,
  ) -> impl Iterator<Item = (usize, f32)> + '_ {
    self.agent.get_type_index_cost_overrides()
  }

  /// Returns the desired velocity. This will only be updated if `update` was
  /// called on the associated [`crate::Archipelago`].
  pub fn get_desired_velocity(&self) -> CS::Coordinate {
    CS::from_landmass(&self.agent.get_desired_velocity())
  }

  /// Returns the state of the agent. This will only be updated if `update` was
  /// called on the associated [`crate::Archipelago`].
  pub fn state(&self) -> AgentState {
    self.agent.state()
  }

  /// Returns the animation link that the agent reached last update.
  ///
  /// Returns None if the previous update the agent did not reach the animation
  /// link. This includes paused agents and agents using the link.
  pub fn reached_animation_link(&self) -> Option<ReachedAnimationLink<CS>> {
    self.agent.reached_animation_link().map(ReachedAnimationLink::from_core)
  }

  /// Returns whether the agent is currently using an animation link.
  /// Essentially this reports whether we have called
  /// [`AgentMut::start_animation_link`] without ending the link yet.
  pub fn is_using_animation_link(&self) -> bool {
    self.agent.is_using_animation_link()
  }
}

/// A mutable borrow to an agent.
pub struct AgentMut<'a, CS> {
  /// The borrow to the core representation.
  pub(crate) agent: &'a mut CoreAgent,
  pub(crate) marker: PhantomData<CS>,
}

impl<'a, CS: CoordinateSystem> AgentMut<'a, CS> {
  /// Creates a ref from an agent borrow.
  pub(crate) fn new(agent: &'a mut CoreAgent) -> Self {
    Self { agent, marker: PhantomData }
  }

  /// The position of the agent.
  pub fn position(&self) -> CS::Coordinate {
    CS::from_landmass(&self.agent.position)
  }

  /// Set the position of the agent.
  pub fn set_position(&mut self, position: CS::Coordinate) {
    self.agent.position = CS::to_landmass(&position)
  }

  /// The velocity of the agent.
  pub fn velocity(&self) -> CS::Coordinate {
    CS::from_landmass(&self.agent.velocity)
  }

  /// Set the velocity of the agent.
  pub fn set_velocity(&mut self, velocity: CS::Coordinate) {
    self.agent.velocity = CS::to_landmass(&velocity);
  }

  /// The radius of the agent.
  pub fn radius(&self) -> f32 {
    self.agent.radius
  }

  /// Set the radius of the agent.
  pub fn set_radius(&mut self, radius: f32) {
    self.agent.radius = radius;
  }

  /// The desired speed of the agent. See [`CoreAgent::desired_speed`] for more.
  pub fn desired_speed(&self) -> f32 {
    self.agent.desired_speed
  }

  /// Set the desired speed of the agent. See [`CoreAgent::desired_speed`] for
  /// more.
  pub fn set_desired_speed(&mut self, desired_speed: f32) {
    self.agent.desired_speed = desired_speed;
  }

  /// The maximum speed that the agent can move at.
  pub fn max_speed(&self) -> f32 {
    self.agent.desired_speed
  }

  /// Set the maximum speed that the agent can move at.
  pub fn set_max_speed(&mut self, max_speed: f32) {
    self.agent.max_speed = max_speed;
  }

  /// The current target to move towards. See [`CoreAgent::current_target`] for
  /// more.
  pub fn current_target(&self) -> Option<CS::Coordinate> {
    self.agent.current_target.as_ref().map(CS::from_landmass)
  }

  /// Set the current target to move towards. See [`CoreAgent::current_target`]
  /// for more.
  pub fn set_current_target(&mut self, target: Option<CS::Coordinate>) {
    self.agent.current_target = target.as_ref().map(CS::to_landmass)
  }

  /// The condition to test for reaching the target.
  pub fn target_reached_condition(&self) -> TargetReachedCondition {
    self.agent.target_reached_condition
  }

  /// Set the condition to test for reaching the target.
  pub fn set_target_reached_condition(
    &mut self,
    target_reached_condition: TargetReachedCondition,
  ) {
    self.agent.target_reached_condition = target_reached_condition;
  }

  /// The distance at which an animation link can be used. See
  /// [`CoreAgent::animation_link_reached_distance`] for more.
  // TODO: FIX THIS DOC LINK.
  pub fn animation_link_reached_distance(&self) -> Option<f32> {
    self.agent.animation_link_reached_distance
  }

  /// Set the distance at which an animation link can be used. See
  /// [`CoreAgent::animation_link_reached_distance`] for more.
  // TODO: FIX THIS DOC LINK.
  pub fn set_animation_link_reached_distance(
    &mut self,
    animation_link_reached_distance: Option<f32>,
  ) {
    self.agent.animation_link_reached_distance =
      animation_link_reached_distance;
  }

  /// The animation links that the agent is allowed to use. See
  /// [`CoreAgent::permitted_animation_links`].
  pub fn permitted_animation_links(&self) -> &PermittedAnimationLinks {
    &self.agent.permitted_animation_links
  }

  /// Set the animation links that the agent is allowed to use. See
  /// [`CoreAgent::permitted_animation_links`].
  pub fn set_permitted_animation_links(
    &mut self,
    permitted_animation_links: PermittedAnimationLinks,
  ) {
    self.agent.permitted_animation_links = permitted_animation_links;
  }

  /// Whether this agent is paused. See [`CoreAgent::paused`] for more.
  pub fn paused(&self) -> bool {
    self.agent.paused
  }

  /// Set whether this agent is paused. See [`CoreAgent::paused`] for more.
  pub fn set_paused(&mut self, paused: bool) {
    self.agent.paused = paused;
  }

  #[cfg(feature = "debug-avoidance")]
  /// Whether to store avoidance debug data. See
  /// [`CoreAgent::keep_avoidance_data`] for more.
  pub fn keep_avoidance_data(&self) -> bool {
    self.agent.keep_avoidance_data
  }

  #[cfg(feature = "debug-avoidance")]
  /// Set whether to store avoidance debug data. See
  /// [`CoreAgent::keep_avoidance_data`] for more.
  pub fn set_keep_avoidance_data(&mut self, keep_avoidance_data: bool) {
    self.agent.keep_avoidance_data = keep_avoidance_data;
  }

  /// Sets the type index cost for this agent to `cost`. Returns false if the
  /// cost is <= 0.0. Otherwise returns true.
  pub fn override_type_index_cost(
    &mut self,
    type_index: usize,
    cost: f32,
  ) -> bool {
    self.agent.override_type_index_cost(type_index, cost)
  }

  /// Removes the override cost for `type_index`. Returns true if `type_index`
  /// was overridden, false otherwise.
  pub fn remove_overridden_type_index_cost(
    &mut self,
    type_index: usize,
  ) -> bool {
    self.agent.remove_overridden_type_index_cost(type_index)
  }

  /// Returns the currently overriden type index costs.
  pub fn get_type_index_cost_overrides(
    &self,
  ) -> impl Iterator<Item = (usize, f32)> + '_ {
    self.agent.get_type_index_cost_overrides()
  }

  /// Returns the desired velocity. This will only be updated if `update` was
  /// called on the associated [`crate::Archipelago`].
  pub fn get_desired_velocity(&self) -> CS::Coordinate {
    CS::from_landmass(&self.agent.get_desired_velocity())
  }

  /// Returns the state of the agent. This will only be updated if `update` was
  /// called on the associated [`crate::Archipelago`].
  pub fn state(&self) -> AgentState {
    self.agent.state()
  }

  /// Returns the animation link that the agent reached last update.
  ///
  /// Returns None if the previous update the agent did not reach the animation
  /// link. This includes paused agents and agents using the link.
  pub fn reached_animation_link(&self) -> Option<ReachedAnimationLink<CS>> {
    self.agent.reached_animation_link().map(ReachedAnimationLink::from_core)
  }

  /// Starts taking an animation link.
  ///
  /// This effectively pauses the agent. Use [`Self::end_animation_link`] to
  /// undo this. After starting the animation link, the agent's state will be
  /// [`AgentState::UsingAnimationLink`] and the
  /// [`Self::reached_animation_link`] will be cleared, after the next update.
  pub fn start_animation_link(
    &mut self,
  ) -> Result<(), NotReachedAnimationLinkError> {
    self.agent.start_animation_link()
  }

  /// Finishes taking an animation link.
  ///
  /// This effectively just unpauses the agent.
  pub fn end_animation_link(
    &mut self,
  ) -> Result<(), NotUsingAnimationLinkError> {
    self.agent.end_animation_link()
  }

  /// Returns whether the agent is currently using an animation link.
  /// Essentially this reports whether we have called
  /// [`Self::start_animation_link`] without ending the link yet.
  pub fn is_using_animation_link(&self) -> bool {
    self.agent.is_using_animation_link()
  }
}

/// The determination of what to do in regards to an agent's path.
#[derive(PartialEq, Eq, Debug)]
pub(crate) enum RepathResult {
  /// Do nothing.
  DoNothing,
  /// The existing path should be followed. Stores the index in a path for the
  /// first portal and the target point.
  FollowPath(PathIndex, PathIndex),
  /// Clear the path and don't repath, since there is no longer a target.
  ClearPathNoTarget,
  /// Clear the path and don't repath, since the agent is not on a valid node.
  ClearPathBadAgent,
  /// Clear the path and don't repath, since the target is not on a valid node.
  ClearPathBadTarget,
  /// Recompute the path.
  NeedsRepath,
}

pub(crate) fn does_agent_need_repath(
  agent: &CoreAgent,
  agent_node: Option<NodeRef>,
  target_node: Option<NodeRef>,
  invalidated_off_mesh_links: &HashSet<OffMeshLinkId>,
  invalidated_islands: &HashSet<IslandId>,
) -> RepathResult {
  if agent.current_target.is_none() {
    if agent.current_path.is_some() {
      return RepathResult::ClearPathNoTarget;
    } else {
      return RepathResult::DoNothing;
    }
  }

  let agent_node = match agent_node {
    None => return RepathResult::ClearPathBadAgent,
    Some(result) => result,
  };
  let target_node = match target_node {
    None => return RepathResult::ClearPathBadTarget,
    Some(result) => result,
  };

  let current_path = match &agent.current_path {
    None => return RepathResult::NeedsRepath,
    Some(current_path) => current_path,
  };

  if !current_path.is_valid(invalidated_off_mesh_links, invalidated_islands) {
    return RepathResult::NeedsRepath;
  }

  let Some(agent_node_index_in_path) =
    current_path.find_index_of_node(agent_node)
  else {
    return RepathResult::NeedsRepath;
  };

  let Some(target_node_index_in_path) =
    current_path.find_index_of_node_rev(target_node)
  else {
    return RepathResult::NeedsRepath;
  };

  if agent_node_index_in_path > target_node_index_in_path {
    return RepathResult::NeedsRepath;
  }

  RepathResult::FollowPath(agent_node_index_in_path, target_node_index_in_path)
}

#[derive(Error, Debug, Clone, Copy)]
#[error(
  "The agent hasn't reached an animation link, so it cannot start an animation link"
)]
pub struct NotReachedAnimationLinkError;

#[derive(Error, Debug, Clone, Copy)]
#[error(
  "The agent isn't already using an animation link, so it cannot end an animation link"
)]
pub struct NotUsingAnimationLinkError;

#[cfg(test)]
#[path = "agent_test.rs"]
mod test;
