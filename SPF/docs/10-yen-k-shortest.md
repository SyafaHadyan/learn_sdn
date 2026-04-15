# 10 — Yen\'s K-Shortest Paths

Yen\'s algorithm finds the **K best simple paths** between two nodes in
non-decreasing order of cost.  Unlike multipath ECMP (which uses parallel
equal-cost paths from a single tree), Yen\'s algorithm finds *diverse*
paths even when they have different costs.

---

## Motivation

Suppose a network has:
- Path A: s1-s2-s4  cost 2   (primary)
- Path B: s1-s3-s4  cost 3   (slightly longer but fully disjoint)
- Path C: s1-s2-s3-s4  cost 5 (shares partial overlap with A)

Standard ECMP installs paths A and C (same cost if balanced), or just A
if costs differ.  Yen\'s K-Shortest installs A and B — the two cheapest
*distinct* paths regardless of cost equality.

---

## Algorithm Overview

```
A = [shortest path from src to dst]   # run Dijkstra first
B = candidate heap

for k = 1..K-1:
    # Each node along the k-1-th path can be a "spur node"
    for each spur_node in A[k-1][:-1]:
        root_path = A[k-1][: index(spur_node)+1 ]

        # Block edges that would duplicate a previously found path
        for p in A:
            if p shares root_path:
                remove edge  spur_node -> next_on_p  temporarily

        # Block nodes already on root_path (enforce simple path)
        remove root_path nodes (except spur_node) temporarily

        spur_path = Dijkstra(spur_node -> dst)
        if spur_path exists:
            total = root_path + spur_path
            if total not in B:
                push total onto B

        restore all removed edges and nodes

    if B is empty: break
    A[k] = pop cheapest from B

return A   # list of K paths in order
```

---

## Step-by-Step Example

Diamond topology:

```
   s1
  /  \
s2    s3
  \  /
   s4
```

Edges (all cost 1).  Find K=2 paths from s1 to s4.

**K=0** — Dijkstra finds: s1-s2-s4 (cost 2)

**K=1 — spur at s1:**
- Block edge s1→s2 (to avoid duplicating path 0)
- Dijkstra finds: s1-s3-s4 (cost 2)
- Push candidate: s1-s3-s4

**Pop from B:** s1-s3-s4 (cost 2)  → second path found.

Result: [ [s1,s2,s4], [s1,s3,s4] ] — both disjoint, both cost 2.

---

## Key Properties

| Property              | ECMP (Dijkstra multipath)  | Yen\'s K-Shortest          |
|-----------------------|----------------------------|---------------------------|
| Path diversity        | Equal-cost only            | Any cost order             |
| Number of paths       | All equal-cost             | Exactly K (or fewer)       |
| Guarantee disjoint?   | Not required               | Not required (simple only) |
| Complexity            | O((V+E) log V)             | O(K V (E + V log V))       |
| Typical K             | Unbounded per-topology     | Small K (2–5)              |

---

## Controller Behaviour

`KShortestSwitch` calls `compute_k_shortest_paths` per host-pair:

```python
def compute_k_shortest_paths(self, src, dst, first_port, final_port):
    node_paths = yen_k_shortest(self.adjacency, src, dst, K_PATHS)
    if not node_paths:
        return [], []
    paths = [self._decorate_path(p, first_port, final_port)
             for p in node_paths]
    return paths
```

Each `node_path` is a list of DPIDs e.g. `[1, 2, 4]`.  The helper
`_decorate_path` converts it to `[(dpid, in_port, out_port), ...]` triples
used by the flow installer.

The K-shortest paths are then installed as an OpenFlow **SELECT GROUP**,
so traffic is hashed across all K paths simultaneously.

---

## Tuning K

`K_PATHS = 2` by default (defined at module top in `kshortest_osken_controller.py`).

Increasing K:
- More path diversity and fault tolerance
- More GROUP buckets per flow entry
- Higher computation cost (varies as O(K))
- OVS has practical limits (typically K ≤ 8 recommended)

---

## Running the Lab

```bash
# Terminal 1 — mesh topology has good path diversity
python3 SPF/topo-mesh_lab.py

# Terminal 2
python3 SPF/kshortest_osken_controller.py --verbose

# Show installed groups (one SELECT group per host pair)
mininet> dpctl dump-groups -O OpenFlow13
```

---

## See Also

- `algorithms/yen_k_shortest.py` — pure Python implementation
- `kshortest_osken_controller.py` — controller (382 lines)
- `docs/09-ecmp.md` — OpenFlow GROUP table detail
- `topo-mesh_lab.py` — topology with multiple disjoint paths
