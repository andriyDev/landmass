use super::{find_path, AStarProblem};

struct AdjacencyListProblemState {
  adjacency: Vec<(f32, i32, usize)>,
  heuristic: f32,
}

struct AdjacencyListProblem {
  states: Vec<AdjacencyListProblemState>,
  start: usize,
  end: usize,
}

impl AStarProblem for AdjacencyListProblem {
  type ActionType = i32;
  type StateType = usize;

  fn initial_state(&self) -> Self::StateType {
    self.start
  }

  fn successors(
    &self,
    state: &Self::StateType,
  ) -> Vec<(f32, Self::ActionType, Self::StateType)> {
    self.states[*state].adjacency.clone()
  }

  fn heuristic(&self, state: &Self::StateType) -> f32 {
    self.states[*state].heuristic
  }

  fn is_goal_state(&self, state: &Self::StateType) -> bool {
    *state == self.end
  }
}

#[test]
fn finds_simple_path_no_heuristic() {
  let problem = AdjacencyListProblem {
    start: 0,
    end: 1,
    states: vec![
      AdjacencyListProblemState {
        adjacency: vec![(10.0, 1, 1), (3.0, 2, 2)],
        heuristic: 0.0,
      },
      AdjacencyListProblemState {
        adjacency: vec![(10.0, 3, 0), (3.0, 4, 2)],
        heuristic: 0.0,
      },
      AdjacencyListProblemState {
        adjacency: vec![(4.0, 5, 1), (3.0, 6, 0)],
        heuristic: 0.0,
      },
    ],
  };

  let path = find_path(&problem).expect("Path should be found.");
  assert_eq!(path.path, [2, 5]);
  assert_eq!(path.stats.explored_nodes, 3);
}

#[test]
fn finds_no_path() {
  let problem = AdjacencyListProblem {
    start: 0,
    // This state doesn't exist, so a path can't be found!
    end: 3,
    states: vec![
      AdjacencyListProblemState {
        adjacency: vec![(10.0, 1, 1), (3.0, 2, 2)],
        heuristic: 0.0,
      },
      AdjacencyListProblemState {
        adjacency: vec![(10.0, 3, 0), (3.0, 4, 2)],
        heuristic: 0.0,
      },
      AdjacencyListProblemState {
        adjacency: vec![(4.0, 5, 1), (3.0, 6, 0)],
        heuristic: 0.0,
      },
    ],
  };

  let stats = find_path(&problem).expect_err("Path cannot be found.");
  assert_eq!(stats.explored_nodes, 3);
}

#[test]
fn grid_no_heuristic() {
  const WIDTH: usize = 5;

  let mut problem = AdjacencyListProblem {
    start: 0,
    end: WIDTH * WIDTH - 1,
    states: Vec::with_capacity(WIDTH * WIDTH),
  };

  for y in 0..WIDTH {
    for x in 0..WIDTH {
      let mut state = AdjacencyListProblemState {
        heuristic: 0.0,
        adjacency: Vec::with_capacity(4),
      };

      let state_index = problem.states.len();
      if x > 0 {
        state.adjacency.push((1.0, 1, state_index - 1));
      }
      if x < WIDTH - 1 {
        state.adjacency.push((1.0, 2, state_index + 1));
      }
      if y > 0 {
        state.adjacency.push((1.0, 3, state_index - WIDTH));
      }
      if y < WIDTH - 1 {
        state.adjacency.push((1.0, 4, state_index + WIDTH));
      }

      problem.states.push(state);
    }
  }

  let path = find_path(&problem).expect("Path should be found.");
  let direction_sums = path.path.iter().fold((0, 0, 0, 0), |mut acc, elem| {
    match elem {
      1 => acc.0 += 1,
      2 => acc.1 += 1,
      3 => acc.2 += 1,
      4 => acc.3 += 1,
      _ => panic!("Invalid direction: {}", elem),
    };
    acc
  });
  assert_eq!(direction_sums, (0, 4, 0, 4));
  // All paths look equally good, so all of them must be explored to ensure we
  // get the optimal path.
  assert_eq!(path.stats.explored_nodes, (WIDTH * WIDTH) as u32);
}

#[test]
fn grid_perfect_heuristic() {
  const WIDTH: usize = 5;

  let mut problem = AdjacencyListProblem {
    start: 0,
    end: WIDTH * WIDTH - 1,
    states: Vec::with_capacity(WIDTH * WIDTH),
  };

  for y in 0..WIDTH {
    for x in 0..WIDTH {
      let mut state = AdjacencyListProblemState {
        heuristic: (WIDTH * 2 - (x + y) - 2) as f32,
        adjacency: Vec::with_capacity(4),
      };

      let state_index = problem.states.len();
      if x > 0 {
        state.adjacency.push((1.0, 1, state_index - 1));
      }
      if x < WIDTH - 1 {
        state.adjacency.push((1.0, 2, state_index + 1));
      }
      if y > 0 {
        state.adjacency.push((1.0, 3, state_index - WIDTH));
      }
      if y < WIDTH - 1 {
        state.adjacency.push((1.0, 4, state_index + WIDTH));
      }

      problem.states.push(state);
    }
  }

  let path = find_path(&problem).expect("Path should be found.");
  assert_eq!(path.path, [4, 4, 4, 4, 2, 2, 2, 2]);
  // Every step towards the bottom right is the optimal path, so we should
  // only explore one path.
  assert_eq!(path.stats.explored_nodes, (WIDTH * 2 - 1) as u32);
}

#[test]
fn dead_end_wrong_heuristic() {
  let problem = AdjacencyListProblem {
    start: 0,
    end: 3,
    states: vec![
      AdjacencyListProblemState {
        adjacency: vec![(1.0, 1, 1), (2.0, 3, 2)],
        heuristic: 3.0,
      },
      AdjacencyListProblemState {
        adjacency: vec![(1.0, 2, 0)],
        heuristic: 1.0,
      },
      AdjacencyListProblemState {
        adjacency: vec![(1.0, 4, 0), (1.0, 1, 3)],
        heuristic: 1.0,
      },
      AdjacencyListProblemState {
        adjacency: vec![(1.0, 2, 2)],
        heuristic: 0.0,
      },
    ],
  };

  let path = find_path(&problem).expect("Path should be found.");
  assert_eq!(path.path, [3, 1]);
  // The dead end was explored, found to be a dead end, and then a new path
  // was found.
  assert_eq!(path.stats.explored_nodes, 4);
}
