# 06 — Bellman-Ford

Bellman-Ford finds shortest paths by relaxing **all edges, V-1 times**.
It is slower than Dijkstra but handles **negative-weight edges** and can
detect **negative-weight cycles**.

---

## Core Algorithm

```
dist[src] = 0;   dist[all others] = ∞
for i in 1 .. V-1:
    for each edge (u, v, weight):
        if dist[u] + weight < dist[v]:
            dist[v] = dist[u] + weight
            prev[v] = u

# Negative cycle check (optional V-th iteration)
for each edge (u, v, weight):
    if dist[u] + weight < dist[v]:
        has_negative_cycle = True
```

After V-1 iterations any node reachable via a *simple* path (at most V-1
edges) has its correct shortest distance.  A further relaxation means a
shorter cycle exists — i.e., a negative cycle.

---

## Why V-1 Iterations?

A simple path in a graph with V nodes can visit at most V nodes, hence at
most **V-1 edges**.  Iteration i guarantees all paths of ≤ i edges are
correctly relaxed.  After V-1 iterations all paths are covered.

```
Iteration 1: all single-hop paths are optimal
Iteration 2: all two-hop paths are optimal
...
Iteration V-1: all paths are optimal
```

---

## Step-by-Step Example

```
s1 --2-- s2 --3-- s3
  \              /
   \----10------/
```

Edges: (s1,s2,2), (s2,s3,3), (s1,s3,10)   — finding shortest paths from s1.

**After iteration 1** (relax all edges once):

| Node | dist | via  |
|------|------|------|
| s1   | 0    | —    |
| s2   | 2    | s1   |
| s3   | 10   | s1   |  ← direct edge used; s2 not yet settled

**After iteration 2:**

| Node | dist | via  |
|------|------|------|
| s1   | 0    | —    |
| s2   | 2    | s1   |
| s3   | 5    | s2   |  ← 2+3=5 < 10, relaxed

No more changes — algorithm done.

---

## Negative-Weight Edges in SDN

In a plain packet network link weights are always non-negative (latency,
hop count, 1/bandwidth).  Bellman-Ford is included here to:

1. Teach the correctness proof (V-1 iterations)
2. Demonstrate negative-cycle detection in controller code
3. Model future use-cases where weights might be derived from
   measurements that produce negative residuals

---

## Complexity Comparison

| Property          | Bellman-Ford     | Dijkstra                 |
|-------------------|------------------|--------------------------|
| Time complexity   | O(V × E)         | O((V+E) log V)           |
| Negative weights  | YES              | NO (undefined behaviour) |
| Negative cycles   | Detects          | N/A                      |
| Per-source        | Yes              | Yes                      |
| All-pairs         | Run V times      | Run V times              |
| Practical speed   | Slower           | Faster for typical nets  |

For a typical SDN network with 50 switches and 100 links:
- Bellman-Ford: 50 × 100 = 5 000 relax operations
- Dijkstra:     (50+100) × log(50) ≈ 900 operations

---

## Controller Behaviour

The `BellmanFordSwitch` controller loads weights from `link_weights.json`
and builds a `{(u,v): weight}` dict from the current adjacency:

```python
def _build_weight_dict(self):
    d = {}
    for u, neighbours in self.adjacency.items():
        for v, _ in neighbours:
            key = (min(u,v), max(u,v))
            bw = self.loaded_weights.get(key, {}).get("bandwidth_mbps", 1)
            d[(u, v)] = bw
            d[(v, u)] = bw
    return d
```

If no weight file exists the weight defaults to 1 (equal to Dijkstra
hop-count on the same topology).

The negative-cycle check is logged but does not crash the controller —
a fallback path or drop is used.

---

## Running the Lab

```bash
# Terminal 1 — weighted topology shows real effect of asymmetric links
python3 SPF/topo-weighted_lab.py

# Terminal 2
python3 SPF/bellman_ford_osken_controller.py --verbose

# In Mininet
mininet> h1 ping h6
mininet> dpctl dump-flows -O OpenFlow13
```

Compare the installed paths against `dijkstra_osken_controller.py` on the
same topology — the cheapest path should agree when all weights are positive.

---

## See Also

- `algorithms/bellman_ford.py` — pure Python implementation
- `bellman_ford_osken_controller.py` — controller (136 lines)
- `link_weights.json` — per-link bandwidth configuration
- `docs/04-dijkstra.md` — Dijkstra comparison
