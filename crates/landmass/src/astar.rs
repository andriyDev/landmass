use std::{
  cmp::Reverse,
  collections::{BinaryHeap, HashMap},
  hash::Hash,
};

/// A generic A* problem.
pub trait AStarProblem {
  /// The action that allows moving between states.
  type ActionType: Clone;
  /// The state that agents try to optimize.
  type StateType: Hash + Eq + Clone;

  /// Creates the initial state for the problem.
  fn initial_state(&self) -> Self::StateType;

  /// Creates all possible states that can be reached by a single action from
  /// `state`. The result stores the "cost" of the action, the action taken, and
  /// the resulting state.
  fn successors(
    &self,
    state: &Self::StateType,
  ) -> Vec<(f32, Self::ActionType, Self::StateType)>;

  /// Computes an estimate of the cost to reach a goal state from `state`. Must
  /// be non-negative and goal states must have an estimate of 0.
  fn heuristic(&self, state: &Self::StateType) -> f32;

  /// Determines whether `state` is a goal state.
  fn is_goal_state(&self, state: &Self::StateType) -> bool;
}

/// A node which represents a single path (by following the previous nodes).
struct Node<ProblemType: AStarProblem> {
  /// The cost of all actions taken by this path.
  cost: f32,
  /// The state that the path results in.
  state: ProblemType::StateType,
  /// The previous node in the path. This is stored as the index of the node
  /// and the action used to get to this state from the previous state. Only
  /// `None` for the initial state.
  previous_node: Option<(usize, ProblemType::ActionType)>,
}

/// A reference to a node.
struct NodeRef {
  /// The cost of the path of the node. This is a convenience for accessing the
  /// node's cost directly.
  cost: f32,
  /// The value of the [`AStarProblem::heuristic`] for this state.
  estimate: f32,
  /// The index of the node.
  index: usize,
}

impl PartialEq for NodeRef {
  fn eq(&self, other: &Self) -> bool {
    self.estimate == other.estimate
  }
}

impl Eq for NodeRef {}

// Since we are comparing floats which are not Ord, it is more meaningful to
// impl PartialOrd, then unwrap in Ord.
#[allow(clippy::non_canonical_partial_ord_impl)]
impl PartialOrd for NodeRef {
  fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
    match self.estimate.partial_cmp(&other.estimate) {
      Some(std::cmp::Ordering::Equal) => {
        Reverse(self.cost).partial_cmp(&Reverse(other.estimate))
      }
      Some(ord) => Some(ord),
      None => None,
    }
  }
}

impl Ord for NodeRef {
  fn cmp(&self, other: &Self) -> std::cmp::Ordering {
    self.partial_cmp(other).unwrap()
  }
}

/// Determines the list of actions taken by `node_ref`.
fn recover_path_from_node<ProblemType: AStarProblem>(
  node_ref: &NodeRef,
  nodes: Vec<Node<ProblemType>>,
) -> Vec<ProblemType::ActionType> {
  let mut path = Vec::new();
  let mut node_index = node_ref.index;
  loop {
    let node = &nodes[node_index];
    match &node.previous_node {
      None => break,
      Some((next_index, action)) => {
        path.push(action.clone());
        node_index = *next_index;
      }
    }
  }

  path.reverse();
  path
}

/// Stats about the pathfinding process.
#[derive(Debug)]
pub struct PathStats {
  /// The number of nodes that were explored. This can exceed the number of
  /// states if there are faster paths than the heuristic "predicts".
  pub explored_nodes: u32,
}

/// The result of pathfinding.
#[derive(Debug)]
pub struct PathResult<ActionType> {
  /// Stats about the pathfinding process.
  pub stats: PathStats,
  /// The found path.
  pub path: Vec<ActionType>,
}

/// Finds a path in `problem` to get from the initial state to a goal state.
/// Returns an `Err` if no path could be found.
pub fn find_path<ProblemType: AStarProblem>(
  problem: &ProblemType,
) -> Result<PathResult<ProblemType::ActionType>, PathStats> {
  let mut stats = PathStats { explored_nodes: 0 };

  let mut best_estimates = HashMap::new();

  let mut all_nodes = Vec::<Node<ProblemType>>::new();
  let mut open_nodes = BinaryHeap::new();

  fn try_add_node<ProblemType: AStarProblem>(
    problem: &ProblemType,
    node: Node<ProblemType>,
    all_nodes: &mut Vec<Node<ProblemType>>,
    open_nodes: &mut BinaryHeap<Reverse<NodeRef>>,
    best_estimates: &mut HashMap<ProblemType::StateType, f32>,
  ) {
    let estimate = node.cost + problem.heuristic(&node.state);
    let best_estimate =
      best_estimates.entry(node.state.clone()).or_insert(f32::INFINITY);
    if *best_estimate <= estimate {
      return;
    }
    *best_estimate = estimate;
    open_nodes.push(Reverse(NodeRef {
      cost: node.cost,
      estimate,
      index: all_nodes.len(),
    }));
    all_nodes.push(node);
  }

  let initial_node =
    Node { cost: 0.0, state: problem.initial_state(), previous_node: None };
  try_add_node(
    problem,
    initial_node,
    &mut all_nodes,
    &mut open_nodes,
    &mut best_estimates,
  );

  while let Some(Reverse(current_node_ref)) = open_nodes.pop() {
    let current_node = &all_nodes[current_node_ref.index];
    // If this node is not the best path to the state, skip it. This state must
    // have already been explored ahead of this node.
    if *best_estimates.get(&current_node.state).unwrap()
      < current_node_ref.estimate
    {
      continue;
    }
    stats.explored_nodes += 1;

    if problem.is_goal_state(&current_node.state) {
      return Ok(PathResult {
        stats,
        path: recover_path_from_node(&current_node_ref, all_nodes),
      });
    }

    let current_cost = current_node.cost;
    for (action_cost, action, state) in problem.successors(&current_node.state)
    {
      let new_node = Node {
        cost: current_cost + action_cost,
        state,
        previous_node: Some((current_node_ref.index, action)),
      };

      try_add_node(
        problem,
        new_node,
        &mut all_nodes,
        &mut open_nodes,
        &mut best_estimates,
      );
    }
  }

  Err(stats)
}

#[cfg(test)]
#[path = "astar_test.rs"]
mod test;
