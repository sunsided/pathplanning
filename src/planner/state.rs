use crate::graph::RoadGraph;
use crate::planner::heuristic::Heuristic;
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
    pub(crate) cost: Reverse<NotNan<f64>>,
    pub(crate) node_id: usize,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Algorithm {
    AStar,
    Dijkstra,
    GreedyBestFirst,
}

impl Algorithm {
    pub fn name(&self) -> &'static str {
        match self {
            Algorithm::AStar => "A*",
            Algorithm::Dijkstra => "Dijkstra",
            Algorithm::GreedyBestFirst => "Greedy Best-First",
        }
    }

    pub fn short_label(&self) -> &'static str {
        match self {
            Algorithm::AStar => "A*",
            Algorithm::Dijkstra => "Dijkstra",
            Algorithm::GreedyBestFirst => "Greedy",
        }
    }

    pub fn all() -> &'static [Algorithm] {
        &[
            Algorithm::AStar,
            Algorithm::Dijkstra,
            Algorithm::GreedyBestFirst,
        ]
    }

    pub fn uses_heuristic(&self) -> bool {
        match self {
            Algorithm::AStar | Algorithm::GreedyBestFirst => true,
            Algorithm::Dijkstra => false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlannerConfig {
    pub algorithm: Algorithm,
    pub heuristic: Heuristic,
}

impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            algorithm: Algorithm::AStar,
            heuristic: Heuristic::Euclidean,
        }
    }
}

pub struct PlannerState {
    pub config: PlannerConfig,
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
            config: PlannerConfig::default(),
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

    #[allow(dead_code)]
    pub fn set_config(&mut self, cfg: PlannerConfig) -> bool {
        if self.config != cfg {
            self.config = cfg;
            true
        } else {
            false
        }
    }

    /// Run up to `n_steps` iterations. Returns true if still searching.
    pub fn step(&mut self, graph: &RoadGraph, n_steps: usize) -> bool {
        match self.config.algorithm {
            Algorithm::AStar => crate::planner::astar::step_astar(self, graph, n_steps),
            Algorithm::Dijkstra => crate::planner::dijkstra::step_dijkstra(self, graph, n_steps),
            Algorithm::GreedyBestFirst => crate::planner::greedy::step_greedy(self, graph, n_steps),
        }
    }

    #[allow(dead_code)]
    pub(crate) fn reconstruct_path(&self, goal: usize) -> Vec<usize> {
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
