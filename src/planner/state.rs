use crate::graph::{GraphEdge, RoadGraph};
use crate::planner::bidir_astar::BidirAStarState;
use crate::planner::cache::{Deadline, PrecomputeKind, PrecomputeQueue};
use crate::planner::ch::ChState;
use crate::planner::heuristic::Heuristic;
use crate::planner::mm::MmState;
use crate::planner::nbs::NbsState;
use crate::planner::rrt::RrtState;
use crate::planner::rrt_star::RrtStarState;
use ordered_float::NotNan;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};

/// Per-frame time budget for graph preprocessing.
const PRECOMPUTE_SLICE_MS: f64 = 10.0;

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
pub enum CostMode {
    ShortestPath,
    ShortestTime,
}

impl CostMode {
    pub fn name(&self) -> &'static str {
        match self {
            CostMode::ShortestPath => "Shortest Path",
            CostMode::ShortestTime => "Shortest Time",
        }
    }

    pub fn short_label(&self) -> &'static str {
        match self {
            CostMode::ShortestPath => "Dist",
            CostMode::ShortestTime => "Time",
        }
    }

    pub fn all() -> &'static [CostMode] {
        &[CostMode::ShortestPath, CostMode::ShortestTime]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Algorithm {
    AStar,
    BidirAStar,
    Mm,
    Nbs,
    ContractionHierarchy,
    Dijkstra,
    GreedyBestFirst,
    Rrt,
    RrtStar,
}

impl Algorithm {
    pub fn name(&self) -> &'static str {
        match self {
            Algorithm::AStar => "A*",
            Algorithm::BidirAStar => "Bidirectional A*",
            Algorithm::Mm => "MM (Meet in the Middle)",
            Algorithm::Nbs => "NBS (Near-Optimal Bidirectional)",
            Algorithm::ContractionHierarchy => "Contraction Hierarchies",
            Algorithm::Dijkstra => "Dijkstra",
            Algorithm::GreedyBestFirst => "Greedy Best-First",
            Algorithm::Rrt => "RRT",
            Algorithm::RrtStar => "RRT*",
        }
    }

    pub fn short_label(&self) -> &'static str {
        match self {
            Algorithm::AStar => "A*",
            Algorithm::BidirAStar => "Bi-A*",
            Algorithm::Mm => "MM",
            Algorithm::Nbs => "NBS",
            Algorithm::ContractionHierarchy => "CH",
            Algorithm::Dijkstra => "Dijkstra",
            Algorithm::GreedyBestFirst => "Greedy",
            Algorithm::Rrt => "RRT",
            Algorithm::RrtStar => "RRT*",
        }
    }

    pub fn all() -> &'static [Algorithm] {
        &[
            Algorithm::AStar,
            Algorithm::BidirAStar,
            Algorithm::Mm,
            Algorithm::Nbs,
            Algorithm::ContractionHierarchy,
            Algorithm::Dijkstra,
            Algorithm::GreedyBestFirst,
            Algorithm::Rrt,
            Algorithm::RrtStar,
        ]
    }

    pub fn uses_heuristic(&self) -> bool {
        match self {
            Algorithm::AStar
            | Algorithm::BidirAStar
            | Algorithm::Mm
            | Algorithm::Nbs
            | Algorithm::GreedyBestFirst => true,
            Algorithm::ContractionHierarchy
            | Algorithm::Dijkstra
            | Algorithm::Rrt
            | Algorithm::RrtStar => false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlannerConfig {
    pub algorithm: Algorithm,
    pub heuristic: Heuristic,
    pub cost_mode: CostMode,
}

impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            algorithm: Algorithm::AStar,
            heuristic: Heuristic::Euclidean,
            cost_mode: CostMode::ShortestTime,
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
    pub locked_path_dist_m: f64,
    pub locked_path_time_s: f64,
    pub status: PlannerStatus,
    pub expanded_count: usize,
    pub start: Option<usize>,
    pub goal: Option<usize>,
    pub rrt: Option<RrtState>,
    pub rrt_star: Option<RrtStarState>,
    pub bidir_astar: Option<BidirAStarState>,
    pub mm: Option<MmState>,
    pub nbs: Option<NbsState>,
    pub ch: Option<ChState>,
    /// Background graph preprocessing; survives `reset` so restarting a
    /// search does not throw the work away.
    pub precompute: PrecomputeQueue,
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
            locked_path_dist_m: 0.0,
            locked_path_time_s: 0.0,
            status: PlannerStatus::Idle,
            expanded_count: 0,
            start: None,
            goal: None,
            rrt: None,
            rrt_star: None,
            bidir_astar: None,
            mm: None,
            nbs: None,
            ch: None,
            precompute: PrecomputeQueue::default(),
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
        self.locked_path_dist_m = 0.0;
        self.locked_path_time_s = 0.0;
        self.status = PlannerStatus::Idle;
        self.expanded_count = 0;
        self.start = None;
        self.goal = None;
        self.rrt = None;
        self.rrt_star = None;
        self.bidir_astar = None;
        self.mm = None;
        self.nbs = None;
        self.ch = None;
    }

    pub fn start_search(&mut self, start: usize, goal: usize) {
        self.reset();
        self.start = Some(start);
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
        if !self.prepare(graph) {
            return true;
        }
        match self.config.algorithm {
            Algorithm::AStar => crate::planner::astar::step_astar(self, graph, n_steps),
            Algorithm::BidirAStar => {
                crate::planner::bidir_astar::step_bidir_astar(self, graph, n_steps)
            }
            Algorithm::Mm => crate::planner::mm::step_mm(self, graph, n_steps),
            Algorithm::Nbs => crate::planner::nbs::step_nbs(self, graph, n_steps),
            Algorithm::ContractionHierarchy => crate::planner::ch::step_ch(self, graph, n_steps),
            Algorithm::Dijkstra => crate::planner::dijkstra::step_dijkstra(self, graph, n_steps),
            Algorithm::GreedyBestFirst => crate::planner::greedy::step_greedy(self, graph, n_steps),
            Algorithm::Rrt => crate::planner::rrt::step_rrt(self, graph, n_steps),
            Algorithm::RrtStar => crate::planner::rrt_star::step_rrt_star(self, graph, n_steps),
        }
    }

    /// Graph preprocessing the current config needs before it can search.
    fn required_precompute(&self, graph: &RoadGraph) -> Option<(PrecomputeKind, CostMode)> {
        let kind = if self.config.algorithm == Algorithm::ContractionHierarchy {
            PrecomputeKind::Hierarchy
        } else if self.config.algorithm.uses_heuristic() && self.config.heuristic == Heuristic::Alt
        {
            PrecomputeKind::Landmarks
        } else {
            return None;
        };
        let mode = self.config.cost_mode;
        (!graph.has_precomputed(kind, mode)).then_some((kind, mode))
    }

    /// Spends one frame's time slice on what the search waits for. Returns
    /// true once the search may run.
    fn prepare(&mut self, graph: &RoadGraph) -> bool {
        let Some(required) = self.required_precompute(graph) else {
            return true;
        };
        self.precompute_tick(graph, Some(required));
        self.required_precompute(graph).is_none()
    }

    /// Spends one frame's time slice on background preprocessing (landmarks
    /// and contraction hierarchies for both cost modes).
    pub fn precompute_background(&mut self, graph: &RoadGraph) {
        self.precompute_tick(graph, None);
    }

    fn precompute_tick(&mut self, graph: &RoadGraph, urgent: Option<(PrecomputeKind, CostMode)>) {
        let deadline = Deadline::in_ms(PRECOMPUTE_SLICE_MS);
        self.precompute.tick(graph, urgent, &deadline);
    }

    /// Builds still missing, with their completed fraction.
    pub fn precompute_pending<'a>(
        &'a self,
        graph: &'a RoadGraph,
    ) -> impl Iterator<Item = (PrecomputeKind, CostMode, f32)> + 'a {
        self.precompute.pending(graph)
    }

    /// Label and completed fraction of preprocessing the search is waiting on.
    pub fn precompute_progress(&self, graph: &RoadGraph) -> Option<(&'static str, f32)> {
        let (kind, mode) = self.required_precompute(graph)?;
        self.precompute_pending(graph)
            .find(|&(k, m, _)| k == kind && m == mode)
            .map(|(k, _, done)| (k.label(), done))
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

    /// Compute the cost of traversing a single edge under the given cost mode.
    pub(crate) fn edge_cost(graph: &RoadGraph, edge: &GraphEdge, mode: CostMode) -> f64 {
        match mode {
            CostMode::ShortestPath => edge.weight_meters,
            CostMode::ShortestTime => {
                let deg = graph.node_out_degree.get(edge.to).copied().unwrap_or(0);
                let stop = if deg >= 3 {
                    edge.road_class.intersection_stop_s()
                } else {
                    0.0
                };
                edge.travel_time_s + stop
            }
        }
    }

    /// Walk the path once at match time to fill both distance and time metrics.
    pub(crate) fn fill_path_metrics(&mut self, graph: &RoadGraph, path: &[usize]) {
        let mut dist_m = 0.0;
        let mut time_s = 0.0;
        for i in 0..path.len().saturating_sub(1) {
            let from = path[i];
            let to = path[i + 1];
            for &edge_idx in &graph.adjacency[from] {
                let edge = &graph.edges[edge_idx];
                if edge.to == to {
                    dist_m += edge.weight_meters;
                    time_s += Self::edge_cost(graph, edge, CostMode::ShortestTime);
                    break;
                }
            }
        }
        self.locked_path_dist_m = dist_m;
        self.locked_path_time_s = time_s;
    }
}
