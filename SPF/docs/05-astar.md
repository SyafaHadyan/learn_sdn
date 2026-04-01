# 05 — A* Heuristic Search

A* is Dijkstra with a *heuristic* — a function that estimates the remaining
cost to the goal.  The estimate lets the algorithm skip unpromising nodes
and reach the destination faster in practice.

---

## Core Idea

Define the priority of node **v** as:

```
f(v) = g(v) + h(v)
```

Where:
- **g(v)** — actual cost from source to v (same as Dijkstra's `dist[v]`)
- **h(v)** — estimated cost from v to destination

A* expands nodes in order of f(v) instead of g(v).

If h(v) is *admissible* — never **over-estimates** the true cost — then A*
is guaranteed to find the optimal path.

---

## Admissible Heuristic for an SDN Network

In a hop-count network a good heuristic is:
> "How many hops would it take to reach the destination ignoring all edge
>  weights (i.e., every edge costs 1)?"

This is computed once by running reverse-BFS from the destination:

```python
def build_reverse_hop_heuristic(adjacency, dst):
    dist = {dst: 0}
    queue = deque([dst])
    while queue:
        node = queue.popleft()
        for neighbour, _ in adjacency.get(node, []):
            if neighbour not in dist:
                dist[neighbour] = dist[node] + 1
                queue.append(neighbour)
    return dist
```

Because hop-count ≤ actual weighted cost, this is admissible.

---

## Step-by-Step Example

Topology (same diamond as the Dijkstra doc):

```
     s1
    /  \
   s2   s3        edge weights below
    \  /
     s4
```

Edges:  s1-s2 = 1,  s1-s3 = 4,  s2-s4 = 2,  s3-s4 = 1
Goal: shortest path s1 → s4

**Step 1 — Build heuristic `h` via reverse-BFS from s4:**

```
h(s4) = 0
h(s2) = 1  (s2 is 1 hop from s4)
h(s3) = 1  (s3 is 1 hop from s4)
h(s1) = 2  (s1 is 2 hops from s4)
```

**Step 2 — A* expansion:**

| Step | Node | g  | h  | f   | Notes               |
|------|------|----|----|-----|---------------------|
| 0    | s1   | 0  | 2  | 2   | start               |
| 1    | s2   | 1  | 1  | 2   | pop s1, push s2, s3 |
| 2    | s4   | 3  | 0  | 3   | pop s2, push s4     |

s4 is reached after 2 pops.  Dijkstra would also pop s3 (f=4+1=5) which A*
skips here because it already found s4.

---

## When Does A* Win?

A* shines when the destination is in a *specific direction* in the graph and
the heuristic can rule out most detours.  In a random or dense graph the
savings are smaller.  In sparse topologies with clear geometry A* may expand
half as many nodes as Dijkstra.

| Property         | Dijkstra                | A*                        |
|------------------|-------------------------|---------------------------|
| Heuristic needed | No                      | Yes (reverse-BFS here)    |
| Optimal?         | Yes (non-neg weights)   | Yes (admissible h)        |
| Nodes expanded   | O(V) worst case         | Fewer (direction-biased)  |
| Pre-computation  | None                    | O(V+E) per destination    |
| Best use         | Any topology            | Large sparse topologies   |

---

## Controller Design

`AStarSwitch` caches the heuristic per destination:

```python
def compute_path(self, src, dst, first_port, final_port):
    if dst not in self.heuristic_hop_cache:
        self.heuristic_hop_cache[dst] = \
            build_reverse_hop_heuristic(self.adjacency, dst)
    h = self.heuristic_hop_cache[dst]
    distance, previous = astar(self.adjacency, src, dst, h)
    return self._reconstruct_path(src, dst, first_port, final_port,
                                  distance, previous)
```

The cache is invalidated in `_on_topology_changed()` because link changes
alter hop distances.

---

## Running the Lab

```bash
# Terminal 1
python3 SPF/topo-spf_lab.py

# Terminal 2
python3 SPF/astar_osken_controller.py --verbose

# In Mininet
mininet> h1 ping h6
```

Compare the flow tables with the Dijkstra controller — for the default
unweighted topology the paths should be identical.

---

## See Also

- `algorithms/astar.py` — pure Python implementation
- `astar_osken_controller.py` — controller (102 lines)
- `astar_multipath_osken_controller.py` — A* + ECMP multipath
- `docs/04-dijkstra.md` — side-by-side comparison
