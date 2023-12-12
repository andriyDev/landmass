use glam::Vec3;

use crate::{path::Path, NavigationData};

pub type AgentId = u32;

// The state of an agent.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AgentState {
  // The agent is idle, due to not having a target. Note this does not mean
  // that they are motionless. An agent will still avoid nearby agents.
  Idle,
  // The agent has reached their target. The agent may resume moving if the
  // target moves or otherwise changes.
  ReachedTarget,
  // The agent has a path and is moving towards their target.
  Moving,
  // The agent is not on a nav mesh.
  AgentNotOnNavMesh,
  // The target is not on a nav mesh.
  TargetNotOnNavMesh,
  // The agent has a target but cannot find a path to it.
  NoPath,
}

pub struct Agent {
  pub position: Vec3,
  pub velocity: Vec3,
  pub radius: f32,
  pub max_velocity: f32,
  pub current_target: Option<Vec3>,
  pub target_reached_condition: TargetReachedCondition,
  pub(crate) current_path: Option<Path>,
  pub(crate) current_desired_move: Vec3,
  pub(crate) state: AgentState,
}

// The condition to consider the agent as having reached its target. When this
// condition is satisfied, the agent will stop moving.
pub enum TargetReachedCondition {
  // The target is reached if it is within the provided (Euclidean) distance
  // of the agent. Useful if the target is surrounded by small obstacles
  // which don't need to be navigated around (e.g. the agent just needs to
  // be close enough to shoot at the target, which is surrounded by cover).
  // Alternatively, if the distance is low, this can simply mean "when the
  // agent is really close to the target".
  Distance(f32),
  // The target is reached if it is "visible" (there is a straight line from
  // the agent to the target), and the target is within the provided
  // (Euclidean) distance of the agent. Useful if the agent should be able
  // to see the target (e.g. a companion character should remain visible to
  // the player, but should ideally not stand too close).
  VisibleAtDistance(f32),
  // The target is reached if the "straight line" path from the agent to the
  // target is less than the provided distance. "Straight line" path means if
  // the agent's path goes around a corner, the distance will be computed
  // going around the corner. This can be more computationally expensive, as
  // the straight line path must be computed every update. Useful for agents
  // that care about the actual walking distance to the target.
  StraightPathDistance(f32),
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
      target_reached_condition: TargetReachedCondition::Distance(radius),
      current_path: None,
      current_desired_move: Vec3::ZERO,
      state: AgentState::Idle,
    }
  }

  pub fn get_desired_velocity(&self) -> Vec3 {
    self.current_desired_move
  }

  pub fn state(&self) -> AgentState {
    self.state
  }

  pub(crate) fn has_reached_target(
    &self,
    path: &Path,
    nav_data: &NavigationData,
    next_waypoint: (usize, Vec3),
    target_waypoint: (usize, Vec3),
  ) -> bool {
    match self.target_reached_condition {
      TargetReachedCondition::Distance(distance) => {
        self.position.distance_squared(target_waypoint.1) < distance * distance
      }
      TargetReachedCondition::VisibleAtDistance(distance) => {
        next_waypoint.0 == target_waypoint.0
          && self.position.distance_squared(next_waypoint.1)
            < distance * distance
      }
      TargetReachedCondition::StraightPathDistance(distance) => 'result: {
        // Check Euclidean distance first so we don't do the expensive path
        // following if the agent is not even close.
        if self.position.distance_squared(target_waypoint.1)
          > distance * distance
        {
          break 'result false;
        }

        // If the next waypoint is the target point, then we've already
        // computed the straight line distance and it is below the limit.
        if next_waypoint.0 == target_waypoint.0 {
          break 'result true;
        }

        let mut straight_line_distance =
          self.position.distance(next_waypoint.1);
        let mut current_waypoint = next_waypoint;

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

          straight_line_distance +=
            current_waypoint.1.distance(next_waypoint.1);
          current_waypoint = next_waypoint;
        }

        straight_line_distance < distance
      }
    }
  }
}

#[cfg(test)]
mod tests {
  use std::{f32::consts::PI, sync::Arc};

  use glam::Vec3;

  use crate::{
    nav_data::NodeRef, path::Path, Agent, Archipelago, NavigationMesh,
    TargetReachedCondition, Transform,
  };

  #[test]
  fn has_reached_target_at_end_node() {
    let nav_mesh =
      NavigationMesh { mesh_bounds: None, vertices: vec![], polygons: vec![] }
        .validate()
        .expect("nav mesh is valid");
    let transform =
      Transform { translation: Vec3::new(2.0, 3.0, 4.0), rotation: PI * 0.85 };
    let mut archipelago = Archipelago::new();
    let island_id = archipelago.add_island();
    archipelago
      .get_island_mut(island_id)
      .set_nav_mesh(transform, Arc::new(nav_mesh));
    let mut agent = Agent::create(
      /* position= */ transform.apply(Vec3::new(1.0, 0.0, 1.0)),
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.0,
      /* max_velocity= */ 0.0,
    );

    let path = Path {
      corridor: vec![NodeRef { island_id, polygon_index: 0 }],
      portal_edge_index: vec![],
    };

    for condition in [
      TargetReachedCondition::Distance(2.0),
      TargetReachedCondition::StraightPathDistance(2.0),
      TargetReachedCondition::VisibleAtDistance(2.0),
    ] {
      agent.target_reached_condition = condition;

      assert!(agent.has_reached_target(
        &path,
        &archipelago.nav_data,
        (0, transform.apply(Vec3::new(2.5, 0.0, 1.0))),
        (0, transform.apply(Vec3::new(2.5, 0.0, 1.0))),
      ));
      assert!(!agent.has_reached_target(
        &path,
        &archipelago.nav_data,
        (0, transform.apply(Vec3::new(3.5, 0.0, 1.0))),
        (0, transform.apply(Vec3::new(3.5, 0.0, 1.0))),
      ));
      assert!(agent.has_reached_target(
        &path,
        &archipelago.nav_data,
        (0, transform.apply(Vec3::new(2.0, 0.0, 2.0))),
        (0, transform.apply(Vec3::new(2.0, 0.0, 2.0))),
      ));
      assert!(!agent.has_reached_target(
        &path,
        &archipelago.nav_data,
        (0, transform.apply(Vec3::new(2.5, 0.0, 2.5))),
        (0, transform.apply(Vec3::new(2.5, 0.0, 2.5))),
      ));
    }
  }

  #[test]
  fn long_detour_reaches_target_in_different_ways() {
    let nav_mesh = NavigationMesh {
      mesh_bounds: None,
      vertices: vec![
        Vec3::new(1.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 11.0),
        Vec3::new(0.0, 0.0, 12.0),
        Vec3::new(2.0, 0.0, 11.0),
        Vec3::new(3.0, 0.0, 12.0),
        Vec3::new(2.0, 0.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2], vec![2, 1, 3, 4], vec![4, 3, 5]],
    }
    .validate()
    .expect("nav mesh is valid");

    let transform =
      Transform { translation: Vec3::new(2.0, 3.0, 4.0), rotation: PI * 0.85 };
    let mut archipelago = Archipelago::new();
    let island_id = archipelago.add_island();
    archipelago
      .get_island_mut(island_id)
      .set_nav_mesh(transform, Arc::new(nav_mesh));

    let mut agent = Agent::create(
      /* position= */ Vec3::ZERO,
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.0,
      /* max_velocity= */ 0.0,
    );

    let path = Path {
      corridor: vec![
        NodeRef { island_id, polygon_index: 0 },
        NodeRef { island_id, polygon_index: 1 },
        NodeRef { island_id, polygon_index: 2 },
      ],
      portal_edge_index: vec![1, 2],
    };

    {
      agent.position = transform.apply(Vec3::new(1.0, 0.0, 1.0));
      agent.target_reached_condition = TargetReachedCondition::Distance(1.1);

      // Agent started within 1.1 units of the destination, so they are close
      // enough.
      assert!(agent.has_reached_target(
        &path,
        &archipelago.nav_data,
        (1, transform.apply(Vec3::new(1.0, 0.0, 11.0))),
        (2, transform.apply(Vec3::new(2.0, 0.0, 1.0))),
      ));

      // Agent is just outside of 1.1 units, so they still have not reached the
      // end.
      agent.position = transform.apply(Vec3::new(1.0, 0.0, 2.0));
      assert!(!agent.has_reached_target(
        &path,
        &archipelago.nav_data,
        (1, transform.apply(Vec3::new(1.0, 0.0, 11.0))),
        (2, transform.apply(Vec3::new(2.0, 0.0, 1.0))),
      ));
    }

    {
      agent.target_reached_condition =
        TargetReachedCondition::VisibleAtDistance(15.0);

      // The agent cannot see the target and its path is still too long.
      agent.position = transform.apply(Vec3::new(1.0, 0.0, 1.0));
      assert!(!agent.has_reached_target(
        &path,
        &archipelago.nav_data,
        (1, transform.apply(Vec3::new(1.0, 0.0, 11.0))),
        (2, transform.apply(Vec3::new(2.0, 0.0, 1.0))),
      ));

      // The agent only has 12 units left to travel to the target, and yet the
      // agent still hasn't reached the target, since the target is not visible.
      agent.position = transform.apply(Vec3::new(1.0, 0.0, 10.0));
      assert!(!agent.has_reached_target(
        &path,
        &archipelago.nav_data,
        (1, transform.apply(Vec3::new(1.0, 0.0, 11.0))),
        (2, transform.apply(Vec3::new(2.0, 0.0, 1.0))),
      ));

      // The agent has now "rounded the corner", and so can see the target (and
      // is within the correct distance).
      agent.position = transform.apply(Vec3::new(2.0, 0.0, 11.0));
      assert!(agent.has_reached_target(
        &path,
        &archipelago.nav_data,
        (2, transform.apply(Vec3::new(2.0, 0.0, 1.0))),
        (2, transform.apply(Vec3::new(2.0, 0.0, 1.0))),
      ));

      // The agent can see the target but is still too far away.
      agent.position = transform.apply(Vec3::new(2.0, 0.0, 20.0));
      assert!(!agent.has_reached_target(
        &path,
        &archipelago.nav_data,
        (2, transform.apply(Vec3::new(2.0, 0.0, 1.0))),
        (2, transform.apply(Vec3::new(2.0, 0.0, 1.0))),
      ));
    }

    {
      agent.target_reached_condition =
        TargetReachedCondition::StraightPathDistance(15.0);

      // The agent's path is too long (21 units).
      agent.position = transform.apply(Vec3::new(1.0, 0.0, 1.0));
      assert!(!agent.has_reached_target(
        &path,
        &archipelago.nav_data,
        (1, transform.apply(Vec3::new(1.0, 0.0, 11.0))),
        (2, transform.apply(Vec3::new(2.0, 0.0, 1.0))),
      ));

      // The agent only has 12 units left to travel to the target, so they have
      // reached the target.
      agent.position = transform.apply(Vec3::new(1.0, 0.0, 10.0));
      assert!(agent.has_reached_target(
        &path,
        &archipelago.nav_data,
        (1, transform.apply(Vec3::new(1.0, 0.0, 11.0))),
        (2, transform.apply(Vec3::new(2.0, 0.0, 1.0))),
      ));

      // The agent can see the target but is still too far away.
      agent.position = transform.apply(Vec3::new(2.0, 0.0, 20.0));
      assert!(!agent.has_reached_target(
        &path,
        &archipelago.nav_data,
        (2, transform.apply(Vec3::new(2.0, 0.0, 1.0))),
        (2, transform.apply(Vec3::new(2.0, 0.0, 1.0))),
      ));
    }
  }
}
