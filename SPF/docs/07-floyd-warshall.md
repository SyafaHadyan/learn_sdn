# Floyd-Warshall — All-Pairs Shortest Paths

## Algorithm Overview

Floyd-Warshall computes shortest paths between **all pairs** of nodes in a
single O(V^3) pass.  It uses dynamic programming and the key insight that
the shortest path from u to v either:
1. Does not go through node k, or
2. Goes through node k (and we already know the best paths to/from k)

## The Recurrence

```
Let D[u][v][k] = shortest path from u to v using only intermediaries {1..k}

Base case:   D[u][v][0] = edge weight(u,v)  if edge exists
                        = inf               otherwise
             D[u][u][0] = 0

Recurrence:  D[u][v][k] = min(D[u][v][k-1],        <- don't use k
                               D[u][k][k-1] + D[k][v][k-1])  <- use k
```

In practice, we use a 2D array and update in-place (the values are consistent):

```
for k in all_nodes:               <- try each intermediate node
    for u in all_nodes:
        for v in all_nodes:
            if dist[u][k] + dist[k][v] < dist[u][v]:
                dist[u][v] = dist[u][k] + dist[k][v]
                next_hop[u][v] = next_hop[u][k]   <- path goes via k first
```

## Visualisation on 3-Node Topology

```
Topology:           s1 -- s2 -- s3
(all edges cost 1)

Initial dist matrix:
       s1  s2  s3
  s1 [  0,  1, inf ]
  s2 [  1,  0,   1 ]
  s3 [inf,  1,   0 ]

After k=s1:    dist[s2][s3] via s1? = dist[s2][s1] + dist[s1][s3] = 1+inf = inf
               No improvement.

After k=s2:    dist[s1][s3] via s2? = dist[s1][s2] + dist[s2][s3] = 1+1 = 2
               Update! dist[s1][s3] = 2, next_hop[s1][s3] = s2

Final dist matrix:
       s1  s2  s3
  s1 [  0,  1,  2 ]
  s2 [  1,  0,  1 ]
  s3 [  2,  1,  0 ]

Path reconstruction: s1 -> s3
  next_hop[s1][s3] = s2    (go via s2 first)
  next_hop[s2][s3] = s3    (s3 is directly adjacent)
  Path: [s1, s2, s3]  ✓
```

## SDN Advantage: Global Pre-computation

In traditional routing, Floyd-Warshall is **impractical** — it would need to
run on every router with full topology knowledge (which no single router has).

In SDN:
```
Controller runs Floyd-Warshall once on startup:
  O(V^3) = O(10^3) = 1,000 operations for 10 switches
           O(50^3) = 125,000 for 50 switches
           O(100^3) = 1,000,000 for 100 switches

Then each path query is O(V) table lookup (no recomputation).
```

The controller pre-computes all paths in `_on_topology_changed()`:

```python
class FloydWarshallSwitch(SPFBaseController):

    def _on_topology_changed(self):
        # Called by base class after topology update
        self._fw_dist, self._fw_next = floyd_warshall(
            self.adjacency, self.switches
        )
        # Now dist[u][v] and next[u][v] are ready for all pairs

    def compute_path(self, src, dst, first_port, final_port):
        # O(V) path reconstruction — no algorithm needed at query time
        path = [src]
        current = src
        while current != dst:
            current = self._fw_next[current][dst]
            path.append(current)
        # Annotate with port numbers and return
```

## Comparison with Dijkstra

| Aspect           | Floyd-Warshall         | Dijkstra               |
|------------------|------------------------|------------------------|
| Query type       | All-pairs              | Single-source          |
| Time complexity  | O(V^3) — pre-compute   | O((V+E) log V) per src |
| Space            | O(V^2)                 | O(V)                   |
| Negative weights | Yes (no neg cycles)    | No                     |
| Best for         | Small networks, FW cache | Large sparse networks |
| SDN advantage    | Pre-compute once       | Run per topology change|

For 10 switches, V^3 = 1000. Dijkstra^10 = 10 * V log V = 10 * 10 * 4 = 400.
Floyd-Warshall is competitive for small networks and beats many-source Dijkstra.

## Running This Controller

```bash
# Terminal 1
python3 SPF/topo-spf_lab.py

# Terminal 2
python3 SPF/floyd_warshall_osken_controller.py

# With verbose logging (see when FW pre-computes):
python3 SPF/floyd_warshall_osken_controller.py --verbose
```

Add `link s1 s2 down` in Mininet to trigger a topology change and watch
the controller re-run Floyd-Warshall automatically.
