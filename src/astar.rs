use crate::graph::RoadGraph;
use ordered_float::NotNan;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlannerStatus {
    Idle,
    Searching,
    FirstMatchFound,
    Done,
}

#[derive(Clone, PartialEq, Eq)]
pub(crate) struct HeapEntry {
    cost: Reverse<NotNan<f64>>,
    node_id: usize,
}

impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.cost
            .cmp(&other.cost)
            .then(self.node_id.cmp(&other.node_id))
    }
}

pub struct PlannerState {
    pub open_set: BinaryHeap<HeapEntry>,
    pub came_from: HashMap<usize, usize>,
    pub g_score: HashMap<usize, f64>,
    pub explored: HashSet<usize>,
    pub frontier: HashSet<usize>,
    pub best_path: Option<Vec<usize>>,
    pub locked_path: Option<Vec<usize>>,
    pub locked_path_dist: f64,
    pub status: PlannerStatus,
    pub expanded_count: usize,
    pub goal: Option<usize>,
}

impl PlannerState {
    pub fn new() -> Self {
        Self {
            open_set: BinaryHeap::new(),
            came_from: HashMap::new(),
            g_score: HashMap::new(),
            explored: HashSet::new(),
            frontier: HashSet::new(),
            best_path: None,
            locked_path: None,
            locked_path_dist: 0.0,
            status: PlannerStatus::Idle,
            expanded_count: 0,
            goal: None,
        }
    }

    pub fn reset(&mut self) {
        self.open_set.clear();
        self.came_from.clear();
        self.g_score.clear();
        self.explored.clear();
        self.frontier.clear();
        self.best_path = None;
        self.locked_path = None;
        self.locked_path_dist = 0.0;
        self.status = PlannerStatus::Idle;
        self.expanded_count = 0;
        self.goal = None;
    }

    pub fn start_search(&mut self, start: usize, goal: usize) {
        self.reset();
        self.goal = Some(goal);
        self.g_score.insert(start, 0.0);
        let f = NotNan::new(0.0).unwrap();
        self.open_set.push(HeapEntry {
            cost: Reverse(f),
            node_id: start,
        });
        self.frontier.insert(start);
        self.status = PlannerStatus::Searching;
    }

    /// Run up to `n_steps` A* iterations. Returns true if still searching.
    pub fn step(&mut self, graph: &RoadGraph, n_steps: usize) -> bool {
        let goal = match self.goal {
            Some(g) => g,
            None => return false,
        };

        for _ in 0..n_steps {
            if self.status == PlannerStatus::Done {
                return false;
            }

            let entry = match self.open_set.pop() {
                Some(e) => e,
                None => {
                    self.status = PlannerStatus::Done;
                    return false;
                }
            };

            let current = entry.node_id;

            if self.explored.contains(&current) {
                continue;
            }
            self.explored.insert(current);
            self.frontier.remove(&current);
            self.expanded_count += 1;

            if current == goal {
                let path = self.reconstruct_path(goal);
                let dist = *self.g_score.get(&goal).unwrap_or(&0.0);
                self.best_path = Some(path.clone());
                self.locked_path = Some(path);
                self.locked_path_dist = dist;
                if self.status == PlannerStatus::Searching {
                    self.status = PlannerStatus::FirstMatchFound;
                }
                self.status = PlannerStatus::Done;
                return false;
            }

            let g = *self.g_score.get(&current).unwrap_or(&f64::INFINITY);

            if current >= graph.adjacency.len() {
                continue;
            }

            for &edge_idx in &graph.adjacency[current] {
                let edge = &graph.edges[edge_idx];
                let neighbor = edge.to;

                if self.explored.contains(&neighbor) {
                    continue;
                }

                let new_g = g + edge.weight_meters;
                let old_g = *self.g_score.get(&neighbor).unwrap_or(&f64::INFINITY);

                if new_g < old_g {
                    self.came_from.insert(neighbor, current);
                    self.g_score.insert(neighbor, new_g);

                    let h = Self::heuristic(graph, neighbor, goal);
                    let f = new_g + h;

                    if let Ok(f_ord) = NotNan::new(f) {
                        self.open_set.push(HeapEntry {
                            cost: Reverse(f_ord),
                            node_id: neighbor,
                        });
                        self.frontier.insert(neighbor);
                    }
                }
            }
        }

        self.status == PlannerStatus::Searching
    }

    fn heuristic(graph: &RoadGraph, from: usize, to: usize) -> f64 {
        let n = graph.nodes.len();
        if from >= n || to >= n {
            return 0.0;
        }
        let p1 = graph.nodes[from].world_pos;
        let p2 = graph.nodes[to].world_pos;
        let dx = p1[0] - p2[0];
        let dy = p1[1] - p2[1];
        (dx * dx + dy * dy).sqrt()
    }

    fn reconstruct_path(&self, goal: usize) -> Vec<usize> {
        let mut path = vec![goal];
        let mut current = goal;
        let mut steps = 0;
        while let Some(&prev) = self.came_from.get(&current) {
            path.push(prev);
            current = prev;
            steps += 1;
            if steps > 1_000_000 {
                break;
            }
        }
        path.reverse();
        path
    }
}
