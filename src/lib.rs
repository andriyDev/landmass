use std::collections::HashMap;

use bevy::{
  prelude::{
    Bundle, Component, Entity, GlobalTransform, IntoSystemConfig,
    IntoSystemSetConfig, Plugin, Query, Res, SystemSet, Vec3, With,
  },
  time::Time,
};
use landmass::AgentId;
use util::{bevy_vec3_to_glam_vec3, glam_vec3_to_bevy_vec3};

mod util;

pub mod nav_mesh;

pub struct LandmassPlugin;

pub mod prelude {
  pub use crate::Agent;
  pub use crate::AgentBundle;
  pub use crate::AgentDesiredVelocity;
  pub use crate::AgentTarget;
  pub use crate::AgentVelocity;
  pub use crate::Archipelago;
  pub use crate::ArchipelagoRef;
  pub use crate::LandmassPlugin;
  pub use crate::LandmassSystemSet;
}

// A bundle to create agents. This omits the GlobalTransform component, since
// this is commonly added in other bundles (which is redundant and can override
// previous bundles).
#[derive(Bundle)]
pub struct AgentBundle {
  pub agent: Agent,
  pub archipelago_ref: ArchipelagoRef,
  pub velocity: AgentVelocity,
  pub target: AgentTarget,
  pub desired_velocity: AgentDesiredVelocity,
}

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum LandmassSystemSet {
  SyncExistence,
  SyncValues,
  Update,
  Output,
}

impl Plugin for LandmassPlugin {
  fn build(&self, app: &mut bevy::prelude::App) {
    app.configure_sets((
      LandmassSystemSet::SyncExistence.before(LandmassSystemSet::SyncValues),
      LandmassSystemSet::SyncValues.before(LandmassSystemSet::Update),
      LandmassSystemSet::Update.before(LandmassSystemSet::Output),
    ));
    app.add_system(
      add_agents_to_archipelagos.in_set(LandmassSystemSet::SyncExistence),
    );
    app
      .add_system(sync_agent_input_state.in_set(LandmassSystemSet::SyncValues));
    app.add_system(update_archipelagos.in_set(LandmassSystemSet::Update));
    app.add_system(sync_desired_velocity.in_set(LandmassSystemSet::Output));
  }
}

#[derive(Component)]
pub struct Archipelago {
  archipelago: landmass::Archipelago,
  agents: HashMap<Entity, AgentId>,
}

impl Archipelago {
  pub fn new(mut landmass_archipelago: landmass::Archipelago) -> Self {
    for agent_id in landmass_archipelago.get_agent_ids().collect::<Vec<_>>() {
      landmass_archipelago.remove_agent(agent_id);
    }
    Self { archipelago: landmass_archipelago, agents: HashMap::new() }
  }

  fn get_agent(&self, entity: Entity) -> &landmass::Agent {
    self.archipelago.get_agent(*self.agents.get(&entity).unwrap())
  }

  fn get_agent_mut(&mut self, entity: Entity) -> &mut landmass::Agent {
    self.archipelago.get_agent_mut(*self.agents.get(&entity).unwrap())
  }
}

fn update_archipelagos(
  time: Res<Time>,
  mut archipelago_query: Query<&mut Archipelago>,
) {
  if time.delta_seconds() == 0.0 {
    return;
  }
  for mut archipelago in archipelago_query.iter_mut() {
    archipelago.archipelago.update(time.delta_seconds());
  }
}

#[derive(Component)]
pub struct Agent {
  pub radius: f32,
  pub max_velocity: f32,
}

#[derive(Component)]
pub struct ArchipelagoRef(pub Entity);

#[derive(Component, Default)]
pub struct AgentVelocity(pub Vec3);

#[derive(Component, Default)]
pub enum AgentTarget {
  #[default]
  None,
  Point(Vec3),
  Entity(Entity),
}

impl AgentTarget {
  fn to_point(
    &self,
    global_transform_query: &Query<&GlobalTransform>,
  ) -> Option<Vec3> {
    match self {
      &Self::Point(point) => Some(point),
      &Self::Entity(entity) => global_transform_query
        .get(entity)
        .ok()
        .map(|transform| transform.translation()),
      _ => None,
    }
  }
}

#[derive(Component, Default)]
pub struct AgentDesiredVelocity(Vec3);

impl AgentDesiredVelocity {
  pub fn velocity(&self) -> Vec3 {
    self.0
  }
}

fn add_agents_to_archipelagos(
  mut archipelago_query: Query<(Entity, &mut Archipelago)>,
  agent_query: Query<(Entity, &Agent, &ArchipelagoRef), With<GlobalTransform>>,
) {
  let mut archipelago_to_agents = HashMap::<_, HashMap<_, _>>::new();
  for (entity, agent, archipleago_ref) in agent_query.iter() {
    archipelago_to_agents
      .entry(archipleago_ref.0)
      .or_default()
      .insert(entity, agent);
  }

  for (archipelago_entity, mut archipelago) in archipelago_query.iter_mut() {
    let mut new_agent_map = archipelago_to_agents
      .remove(&archipelago_entity)
      .unwrap_or_else(|| HashMap::new());
    let archipelago = archipelago.as_mut();

    // Remove any agents that aren't in the `new_agent_map`. Also remove any
    // agents from the `new_agent_map` that are in the archipelago.
    archipelago.agents.retain(|agent_entity, agent_id| {
      match new_agent_map.remove(&agent_entity) {
        None => {
          archipelago.archipelago.remove_agent(*agent_id);
          false
        }
        Some(_) => true,
      }
    });

    for (new_agent_entity, new_agent) in new_agent_map.drain() {
      let agent_id =
        archipelago.archipelago.add_agent(landmass::Agent::create(
          /* position= */ glam::Vec3::ZERO,
          /* velocity= */ glam::Vec3::ZERO,
          new_agent.radius,
          new_agent.max_velocity,
        ));
      archipelago.agents.insert(new_agent_entity, agent_id);
    }
  }
}

fn sync_agent_input_state(
  agent_query: Query<
    (
      Entity,
      &ArchipelagoRef,
      &GlobalTransform,
      Option<&AgentVelocity>,
      Option<&AgentTarget>,
    ),
    With<Agent>,
  >,
  global_transform_query: Query<&GlobalTransform>,
  mut archipelago_query: Query<&mut Archipelago>,
) {
  for (
    agent_entity,
    &ArchipelagoRef(arch_entity),
    transform,
    velocity,
    target,
  ) in agent_query.iter()
  {
    let mut archipelago = match archipelago_query.get_mut(arch_entity) {
      Err(_) => continue,
      Ok(arch) => arch,
    };

    let agent = archipelago.get_agent_mut(agent_entity);
    agent.position = bevy_vec3_to_glam_vec3(transform.translation());
    if let Some(AgentVelocity(velocity)) = velocity {
      agent.velocity = bevy_vec3_to_glam_vec3(*velocity);
    }
    agent.current_target = target
      .and_then(|target| target.to_point(&global_transform_query))
      .map(bevy_vec3_to_glam_vec3);
  }
}

fn sync_desired_velocity(
  mut agent_query: Query<
    (Entity, &ArchipelagoRef, &mut AgentDesiredVelocity),
    With<Agent>,
  >,
  archipelago_query: Query<&Archipelago>,
) {
  for (agent_entity, &ArchipelagoRef(arch_entity), mut desired_velocity) in
    agent_query.iter_mut()
  {
    let archipelago = match archipelago_query.get(arch_entity).ok() {
      None => continue,
      Some(arch) => arch,
    };

    desired_velocity.0 = glam_vec3_to_bevy_vec3(
      archipelago.get_agent(agent_entity).get_desired_velocity(),
    );
  }
}

#[cfg(test)]
mod tests {
  use bevy::prelude::*;
  use landmass::NavigationMesh;

  use crate::{
    Agent, AgentBundle, AgentDesiredVelocity, AgentTarget, Archipelago,
    ArchipelagoRef, LandmassPlugin,
  };

  #[test]
  fn computes_path_for_agent_and_updates_desired_velocity() {
    let mut app = App::new();

    app
      .add_plugins(MinimalPlugins)
      .add_plugin(TransformPlugin)
      .add_plugin(LandmassPlugin);

    let archipelago_id = app
      .world
      .spawn(Archipelago::new({
        let mut archipelago =
          landmass::Archipelago::create_from_navigation_mesh(
            NavigationMesh {
              mesh_bounds: None,
              vertices: vec![
                glam::Vec3::new(1.0, 0.0, 1.0),
                glam::Vec3::new(4.0, 0.0, 1.0),
                glam::Vec3::new(4.0, 0.0, 4.0),
                glam::Vec3::new(3.0, 0.0, 4.0),
                glam::Vec3::new(3.0, 0.0, 2.0),
                glam::Vec3::new(1.0, 0.0, 2.0),
              ],
              polygons: vec![vec![0, 1, 4, 5], vec![1, 2, 3, 4]],
            }
            .validate()
            .expect("is valid"),
          );
        archipelago.agent_options.obstacle_avoidance_margin = 0.0;
        archipelago
      }))
      .id();

    let agent_id = app
      .world
      .spawn(TransformBundle {
        local: Transform::from_translation(Vec3::new(1.5, 0.0, 1.5)),
        ..Default::default()
      })
      .insert(AgentBundle {
        agent: Agent { radius: 0.5, max_velocity: 1.0 },
        archipelago_ref: ArchipelagoRef(archipelago_id),
        target: AgentTarget::Point(Vec3::new(3.5, 0.0, 3.5)),
        velocity: Default::default(),
        desired_velocity: Default::default(),
      })
      .id();

    // The first update propagates the global transform, and sets the start of
    // the delta time (in this update, delta time is 0).
    app.update();
    // The second update allows landmass to update properly.
    app.update();

    assert_eq!(
      app
        .world
        .get::<AgentDesiredVelocity>(agent_id)
        .expect("desired velocity was added")
        .0,
      Vec3::new(1.5, 0.0, 0.5).normalize(),
    );
  }

  #[test]
  fn adds_and_removes_agents() {
    let mut app = App::new();

    app.add_plugins(MinimalPlugins).add_plugin(LandmassPlugin);

    let archipelago_id = app
      .world
      .spawn(Archipelago::new(
        landmass::Archipelago::create_from_navigation_mesh(
          NavigationMesh {
            mesh_bounds: None,
            vertices: vec![],
            polygons: vec![],
          }
          .validate()
          .expect("is valid"),
        ),
      ))
      .id();

    let agent_id_1 = app
      .world
      .spawn(TransformBundle::default())
      .insert(AgentBundle {
        agent: Agent { radius: 0.5, max_velocity: 1.0 },
        archipelago_ref: ArchipelagoRef(archipelago_id),
        target: AgentTarget::None,
        velocity: Default::default(),
        desired_velocity: Default::default(),
      })
      .id();

    let agent_id_2 = app
      .world
      .spawn(TransformBundle::default())
      .insert(AgentBundle {
        agent: Agent { radius: 0.5, max_velocity: 1.0 },
        archipelago_ref: ArchipelagoRef(archipelago_id),
        target: AgentTarget::None,
        velocity: Default::default(),
        desired_velocity: Default::default(),
      })
      .id();

    app.update();

    let archipelago =
      app.world.get::<Archipelago>(archipelago_id).expect("archipelago exists");

    let mut agent_entities =
      archipelago.agents.keys().copied().collect::<Vec<_>>();
    agent_entities.sort();
    assert_eq!(agent_entities, {
      let mut expected = [agent_id_1, agent_id_2];
      expected.sort();
      expected
    });
    assert_eq!(archipelago.archipelago.get_agent_ids().len(), 2);

    let agent_id_3 = app
      .world
      .spawn(TransformBundle::default())
      .insert(AgentBundle {
        agent: Agent { radius: 0.5, max_velocity: 1.0 },
        archipelago_ref: ArchipelagoRef(archipelago_id),
        target: AgentTarget::None,
        velocity: Default::default(),
        desired_velocity: Default::default(),
      })
      .id();

    app.update();

    let archipelago =
      app.world.get::<Archipelago>(archipelago_id).expect("archipelago exists");

    let mut agent_entities =
      archipelago.agents.keys().copied().collect::<Vec<_>>();
    agent_entities.sort();
    assert_eq!(agent_entities, {
      let mut expected = [agent_id_1, agent_id_2, agent_id_3];
      expected.sort();
      expected
    });
    assert_eq!(archipelago.archipelago.get_agent_ids().len(), 3);

    app.world.despawn(agent_id_2);

    app.update();

    let archipelago =
      app.world.get::<Archipelago>(archipelago_id).expect("archipelago exists");

    let mut agent_entities =
      archipelago.agents.keys().copied().collect::<Vec<_>>();
    agent_entities.sort();
    assert_eq!(agent_entities, {
      let mut expected = [agent_id_1, agent_id_3];
      expected.sort();
      expected
    });
    assert_eq!(archipelago.archipelago.get_agent_ids().len(), 2);

    app.world.despawn(agent_id_1);
    app.world.despawn(agent_id_3);

    app.update();

    let archipelago =
      app.world.get::<Archipelago>(archipelago_id).expect("archipelago exists");

    assert_eq!(archipelago.agents.keys().copied().collect::<Vec<_>>(), []);
    assert_eq!(archipelago.archipelago.get_agent_ids().len(), 0);
  }
}
