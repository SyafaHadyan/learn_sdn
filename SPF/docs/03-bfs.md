# BFS — Breadth-First Search Routing

## Algorithm Overview

Breadth-First Search finds the **minimum hop count** path in an unweighted graph.
It explores nodes **level by level** — all nodes at distance 1 first, then
distance 2, and so on.  The first time a node is visited, it is guaranteed to
be on a shortest path.

## Visualisation on Lab Topology

```
Topology:
    h1   h2       h3   h4       h5   h6
     \   /         \   /         \   /
      s1 --(L1)-- s2 --(L2)-- s3
      \________________________/
               (L3)

BFS from s1, exploring hop by hop:

  Level 0:  {s1}
            distance[s1] = 0

  Level 1:  {s2, s3}   (immediate neighbours)
            distance[s2] = 1
            distance[s3] = 1

(All nodes reached — done)

Shortest paths from s1:
  s1 -> s2:  cost = 1  (direct)
  s1 -> s3:  cost = 1  (direct via L3)
  s1 -> s1:  cost = 0
```

## Step-by-Step Walkthrough

For path `h1 -> h5` (s1 → s3):

```
Step 1: h1 arrives at s1 (port 1). Controller receives packet_in.
        src_mac = h1:mac, src_sw = s1, src_port = 1
        dst_mac = h5:mac, dst_sw = s3, dst_port = 1

Step 2: Run BFS from s1:
        Queue: [s1]        visited: {s1}   distance: {s1:0}
        Dequeue s1 -> neighbours: s2 (port 3), s3 (port 4)
        Queue: [s2, s3]    distance: {s1:0, s2:1, s3:1}

        Dequeue s2 -> neighbour s3 already visited
        Dequeue s3 -> all neighbours visited
        Done.

Step 3: Reconstruct path s1 -> s3:
        previous[s3] = None? No — s3 is a direct neighbour of s1.
        Actually previous[s3] = s1 (first seen from s1).
        Path: [s1, s3]

Step 4: Annotate with ports:
        s1 out_port to s3 = port 4    (from port_map[(s1, s3)])
        s3 in_port from s1 = port 3   (from port_map[(s3, s1)])
        Result: [(s1, in_port=1, out_port=4), (s3, in_port=3, out_port=1)]

Step 5: Install flows on each switch (in both directions):
        s1: match(in_port=1, src=h1:mac, dst=h5:mac) -> output(4)
        s3: match(in_port=3, src=h1:mac, dst=h5:mac) -> output(1)
        (reverse direction installed automatically)
```

## Why BFS Works Here

All inter-switch links have equal cost (hop count = 1).  BFS visits nodes in
exactly non-decreasing hop order, so the first time it reaches any node, it
has found the shortest path.

**Key property:** `dist[v] = k` when v is first dequeued, and `k` is the
minimum number of hops from src to v.

## Complexity Analysis

```
V = number of switches   E = number of inter-switch links

Time:  O(V + E)   — each node and edge visited once
Space: O(V)       — queue and distance/previous arrays

Comparison:
  BFS:      O(V + E)          — optimal for unweighted graphs
  Dijkstra: O((V+E) log V)    — heap overhead, needed for weights
```

## Code Walk-Through

```python
# algorithms/bfs.py

def bfs(adjacency, src, weights=None):
    # Phase 1: Initialise — O(V+E) to collect all vertices
    distance = {v: float("inf") for v in all_vertices}
    previous = {v: None for v in all_vertices}
    distance[src] = 0

    # Phase 2: BFS loop — O(V+E)
    queue = deque([src])
    while queue:
        u = queue.popleft()           # O(1) — FIFO, not priority queue
        for v, _ in adjacency[u]:
            if distance[v] == float("inf"):   # not yet visited
                distance[v] = distance[u] + 1
                previous[v] = u
                queue.append(v)

    return distance, previous
```

The controller calls this and passes the result to `_reconstruct_path()`:

```python
# bfs_osken_controller.py

def compute_path(self, src, dst, first_port, final_port):
    distance, previous = bfs(self.adjacency, src)
    return self._reconstruct_path(src, dst, first_port, final_port,
                                   distance, previous)
```

## Running This Controller

```bash
# Terminal 1
python3 SPF/topo-spf_lab.py

# Terminal 2
python3 SPF/bfs_osken_controller.py

# Mininet CLI
mininet> pingall
mininet> h1 ping h6
mininet> dpctl dump-flows -O OpenFlow13
```
