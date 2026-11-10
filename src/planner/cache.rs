use crate::graph::RoadGraph;
use crate::planner::alt::{Landmarks, LandmarksBuilder};
use crate::planner::ch::{ChBuilder, ContractionHierarchy};
use crate::planner::state::{CostMode, HeapEntry, PlannerState};
use ordered_float::NotNan;
use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::sync::OnceLock;

/// Graph-derived planner data, built lazily on first use and dropped by
/// [`RoadGraph::rebuild_derived`] whenever the topology changes.
#[derive(Default)]
pub struct Precomputed {
    rev_adjacency: OnceLock<Vec<Vec<usize>>>,
    landmarks: [OnceLock<Landmarks>; 2],
    ch: [OnceLock<ContractionHierarchy>; 2],
}

impl std::fmt::Debug for Precomputed {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Precomputed")
            .field("rev_adjacency", &self.rev_adjacency.get().is_some())
            .field(
                "landmarks",
                &self.landmarks.each_ref().map(|l| l.get().is_some()),
            )
            .field("ch", &self.ch.each_ref().map(|c| c.get().is_some()))
            .finish()
    }
}

fn mode_index(mode: CostMode) -> usize {
    match mode {
        CostMode::ShortestPath => 0,
        CostMode::ShortestTime => 1,
    }
}

impl RoadGraph {
    /// rev_adjacency()[node] = edge indices with `edge.to == node`.
    pub(crate) fn rev_adjacency(&self) -> &[Vec<usize>] {
        self.precomputed.rev_adjacency.get_or_init(|| {
            let mut rev = vec![Vec::new(); self.nodes.len()];
            for (idx, edge) in self.edges.iter().enumerate() {
                rev[edge.to].push(idx);
            }
            rev
        })
    }

    pub(crate) fn landmarks(&self, mode: CostMode) -> &Landmarks {
        self.precomputed.landmarks[mode_index(mode)].get_or_init(|| Landmarks::build(self, mode))
    }

    pub(crate) fn contraction_hierarchy(&self, mode: CostMode) -> &ContractionHierarchy {
        self.precomputed.ch[mode_index(mode)]
            .get_or_init(|| ContractionHierarchy::build(self, mode))
    }

    /// Outgoing `(neighbor, cost)` pairs of `node`, or incoming ones (as seen
    /// when walking edges backwards) if `backward` is set.
    pub(crate) fn neighbors(
        &self,
        node: usize,
        backward: bool,
        mode: CostMode,
    ) -> impl Iterator<Item = (usize, f64)> + '_ {
        let edges = if backward {
            self.rev_adjacency().get(node)
        } else {
            self.adjacency.get(node)
        };
        edges.into_iter().flatten().map(move |&idx| {
            let edge = &self.edges[idx];
            let neighbor = if backward { edge.from } else { edge.to };
            (neighbor, PlannerState::edge_cost(self, edge, mode))
        })
    }
}

/// Wall-clock budget for resumable preprocessing.
pub(crate) struct Deadline(f64);

impl Deadline {
    pub fn in_ms(ms: f64) -> Self {
        Self(now_ms() + ms)
    }

    pub fn never() -> Self {
        Self(f64::INFINITY)
    }

    pub fn passed(&self) -> bool {
        self.0.is_finite() && now_ms() >= self.0
    }
}

// std::time::Instant panics on wasm32-unknown-unknown.
#[cfg(target_arch = "wasm32")]
fn now_ms() -> f64 {
    js_sys::Date::now()
}

#[cfg(not(target_arch = "wasm32"))]
fn now_ms() -> f64 {
    static EPOCH: OnceLock<std::time::Instant> = OnceLock::new();
    EPOCH
        .get_or_init(std::time::Instant::now)
        .elapsed()
        .as_secs_f64()
        * 1000.0
}

/// Resumable single-source Dijkstra; `dist[v]` is the cost from `source` to
/// `v` (or from `v` to `source` if `backward`), `INFINITY` if unreachable.
pub(crate) struct DijkstraRun {
    pub dist: Vec<f64>,
    heap: BinaryHeap<HeapEntry>,
    backward: bool,
}

impl DijkstraRun {
    pub fn new(graph: &RoadGraph, source: usize, backward: bool) -> Self {
        let mut dist = vec![f64::INFINITY; graph.nodes.len()];
        dist[source] = 0.0;
        let heap = BinaryHeap::from([HeapEntry {
            cost: Reverse(NotNan::new(0.0).unwrap()),
            node_id: source,
        }]);
        Self {
            dist,
            heap,
            backward,
        }
    }

    /// Settles nodes until done (returns true) or `deadline` passes.
    pub fn advance(&mut self, graph: &RoadGraph, mode: CostMode, deadline: &Deadline) -> bool {
        let mut popped = 0usize;
        while let Some(HeapEntry { cost, node_id }) = self.heap.pop() {
            let d = cost.0.into_inner();
            if d > self.dist[node_id] {
                continue;
            }
            for (neighbor, c) in graph.neighbors(node_id, self.backward, mode) {
                let nd = d + c;
                if nd < self.dist[neighbor]
                    && let Ok(key) = NotNan::new(nd)
                {
                    self.dist[neighbor] = nd;
                    self.heap.push(HeapEntry {
                        cost: Reverse(key),
                        node_id: neighbor,
                    });
                }
            }
            popped += 1;
            if popped.is_multiple_of(1024) && deadline.passed() {
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
pub(crate) fn dijkstra_all(
    graph: &RoadGraph,
    source: usize,
    backward: bool,
    mode: CostMode,
) -> Vec<f64> {
    let mut run = DijkstraRun::new(graph, source, backward);
    run.advance(graph, mode, &Deadline::never());
    run.dist
}

/// Graph preprocessing that the current planner config is waiting for.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum PrecomputeKind {
    Landmarks,
    Hierarchy,
}

enum Builder {
    Landmarks(LandmarksBuilder),
    Hierarchy(ChBuilder),
}

/// An in-progress preprocessing job, advanced a time slice per frame.
struct PrecomputeJob {
    kind: PrecomputeKind,
    mode: CostMode,
    builder: Builder,
}

impl PrecomputeJob {
    fn new(graph: &RoadGraph, kind: PrecomputeKind, mode: CostMode) -> Self {
        let builder = match kind {
            PrecomputeKind::Landmarks => Builder::Landmarks(LandmarksBuilder::new(graph, mode)),
            PrecomputeKind::Hierarchy => Builder::Hierarchy(ChBuilder::new(graph, mode)),
        };
        Self {
            kind,
            mode,
            builder,
        }
    }

    /// Works until `deadline`; on completion stores the result on the graph
    /// and returns true.
    fn advance(&mut self, graph: &RoadGraph, deadline: &Deadline) -> bool {
        let i = mode_index(self.mode);
        match &mut self.builder {
            Builder::Landmarks(b) => b.advance(graph, deadline).map(|l| {
                let _ = graph.precomputed.landmarks[i].set(l);
            }),
            Builder::Hierarchy(b) => b.advance(graph, deadline).map(|ch| {
                let _ = graph.precomputed.ch[i].set(ch);
            }),
        }
        .is_some()
    }

    fn progress(&self) -> f32 {
        match &self.builder {
            Builder::Landmarks(b) => b.progress(),
            Builder::Hierarchy(b) => b.progress(),
        }
    }
}

impl PrecomputeKind {
    pub fn label(self) -> &'static str {
        match self {
            PrecomputeKind::Landmarks => "LANDMARKS",
            PrecomputeKind::Hierarchy => "HIERARCHY",
        }
    }
}

/// Every build, in background order: landmarks first (cheaper, used by more
/// planners), then hierarchies.
const ALL_BUILDS: [(PrecomputeKind, CostMode); 4] = [
    (PrecomputeKind::Landmarks, CostMode::ShortestTime),
    (PrecomputeKind::Landmarks, CostMode::ShortestPath),
    (PrecomputeKind::Hierarchy, CostMode::ShortestTime),
    (PrecomputeKind::Hierarchy, CostMode::ShortestPath),
];

/// Builds all graph preprocessing in the background, one time slice per
/// frame. Partially built jobs are kept when priorities change.
#[derive(Default)]
pub struct PrecomputeQueue {
    jobs: Vec<PrecomputeJob>,
}

impl PrecomputeQueue {
    /// Builds not yet stored on the graph, with the completed fraction.
    pub fn pending<'a>(
        &'a self,
        graph: &'a RoadGraph,
    ) -> impl Iterator<Item = (PrecomputeKind, CostMode, f32)> + 'a {
        ALL_BUILDS
            .into_iter()
            .filter(|&(kind, mode)| !graph.has_precomputed(kind, mode))
            .map(|(kind, mode)| {
                let job = self.jobs.iter().find(|j| j.kind == kind && j.mode == mode);
                (kind, mode, job.map_or(0.0, PrecomputeJob::progress))
            })
    }

    /// Spends one time slice on `urgent` if given and missing, otherwise on
    /// the next missing build. Does nothing once everything is built.
    pub(crate) fn tick(
        &mut self,
        graph: &RoadGraph,
        urgent: Option<(PrecomputeKind, CostMode)>,
        deadline: &Deadline,
    ) {
        let Some((kind, mode)) = urgent
            .into_iter()
            .chain(ALL_BUILDS)
            .find(|&(kind, mode)| !graph.has_precomputed(kind, mode))
        else {
            return;
        };
        let idx = match self
            .jobs
            .iter()
            .position(|j| j.kind == kind && j.mode == mode)
        {
            Some(idx) => idx,
            None => {
                self.jobs.push(PrecomputeJob::new(graph, kind, mode));
                self.jobs.len() - 1
            }
        };
        if self.jobs[idx].advance(graph, deadline) {
            self.jobs.swap_remove(idx);
        }
    }
}

impl RoadGraph {
    pub(crate) fn has_precomputed(&self, kind: PrecomputeKind, mode: CostMode) -> bool {
        let i = mode_index(mode);
        match kind {
            PrecomputeKind::Landmarks => self.precomputed.landmarks[i].get().is_some(),
            PrecomputeKind::Hierarchy => self.precomputed.ch[i].get().is_some(),
        }
    }
}
