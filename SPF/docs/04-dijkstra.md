# Dijkstra — Weighted Shortest Path

## Algorithm Overview

Dijkstra's algorithm finds the shortest path from a source to all other nodes
in a graph with **non-negative edge weights**.  It is the most widely used
shortest path algorithm and runs in O((V+E) log V) with a binary heap.

## Key Insight: Greedy Expansion

Dijkstra works by always expanding the **nearest unvisited node** first.
This greedy choice is safe because all edge weights are non-negative — a
later path through more intermediaries can never be shorter than the direct
path to the current nearest node.

```
Invariant: when a node u is dequeued (pop from min-heap):
  distance[u] is FINAL and CORRECT.
```

## Visualisation on Diamond Topology (ECMP)

```
Topology (all edges weight 1):

    s1 --(1)--> s2 --(1)--> s4
     \                      /
      \--(1)--> s3 --(1)--/

BFS from s1:                    Dijkstra from s1:
  Same result when weights=1!   (heap just replaces queue)
  distance: {s1:0, s2:1, s3:1, s4:2}
  previous: {s2:s1, s3:s1, s4: s2 or s3}  (one parent, not both)
```

## Step-by-Step: Weighted Topology

```
Links:  s1-(10)--s2-(10)--s4   s1-(1)--s3-(1)--s4   s1-(100)--s4(direct)

Dijkstra from s1:
  Heap: [(0, s1)]
  dist: {s1:0, s2:inf, s3:inf, s4:inf}

  Step 1: Pop (0, s1). Relax neighbours:
    s2: min(inf, 0+10) = 10 -> push (10, s2)
    s3: min(inf, 0+1)  = 1  -> push (1, s3)
    s4: min(inf, 0+100)= 100 -> push (100, s4)
  dist: {s1:0, s2:10, s3:1, s4:100}

  Step 2: Pop (1, s3). Relax:
    s4: min(100, 1+1) = 2   -> push (2, s4)   IMPROVEMENT!
  dist: {s1:0, s2:10, s3:1, s4:2}

  Step 3: Pop (2, s4). Best cost to s4 found (2 via s3).
  Step 4: Pop (10, s2). Relax s4: min(2, 10+10)=2, no improvement.
  Step 5: Pop (100, s4) — stale entry, skip (100 > dist[s4]=2).

  Result: shortest path s1->s4 = cost 2, via s3.
```

## Dijkstra vs BFS

| Property       | BFS             | Dijkstra          |
|----------------|-----------------|-------------------|
| Data structure | FIFO queue      | Min-heap          |
| Edge weights   | Must be equal   | Any non-negative  |
| Complexity     | O(V + E)        | O((V+E) log V)    |
| Stop early?    | When dst popped | When dst popped   |
| Negative wgts? | N/A (ignores)   | NOT SUPPORTED     |

**Rule of thumb:** Use BFS for hop-count routing. Use Dijkstra when links
have different costs (bandwidth-inverse, delay, etc.).

## Multipath Variant: `dijkstra_multi_parent`

Standard Dijkstra keeps only one predecessor per node.  The multipath
variant keeps **all equal-cost predecessors**:

```python
# Standard:
  previous[v] = best_predecessor   (single node)

# Multi-parent:
  parents[v] = {all predecessors with equal cost}   (set)
```

For the diamond topology:
```
  parents[s4] = {s2, s3}   <- two equal-cost path predecessors
```

This enables ECMP by enumerating all paths through the parent sets.

## Code Walk-Through

```python
# algorithms/dijkstra.py

def dijkstra(adjacency, src, weights=None):
    # O(V+E) initialisation
    distance = {v: float("inf") for v in all_vertices}
    previous = {v: None for v in all_vertices}
    distance[src] = 0

    heap = [(0, src)]   # (cost, node)
    while heap:
        d, u = heapq.heappop(heap)        # O(log V) pop
        if d > distance[u]:
            continue                       # stale entry

        for v, _ in adjacency.get(u, []):
            w = weights.get((u, v), 1) if weights else 1
            alt = d + w
            if alt < distance[v]:          # relaxation
                distance[v] = alt
                previous[v] = u
                heapq.heappush(heap, (alt, v))   # O(log V) push

    return distance, previous
```

## Running This Controller

```bash
# Basic hop-count routing:
python3 SPF/topo-spf_lab.py       # Terminal 1
python3 SPF/dijkstra_osken_controller.py   # Terminal 2

# Multipath ECMP:
python3 SPF/topo-mesh_lab.py      # Terminal 1
python3 SPF/dijkstra_multipath_osken_controller.py  # Terminal 2
```
