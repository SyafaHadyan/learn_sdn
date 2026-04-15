# SPF Lab — Teaching Overview

## What This Lab Is About

This lab implements Software-Defined Networking (SDN) path-computation controllers
using [OSKen](https://github.com/openstack/os-ken) (a Ryu fork) and
[Mininet](https://mininet.org/).  Each controller demonstrates a different
routing algorithm running as a centralised SDN application.

## Why SDN Is Interesting for Routing

In traditional IP networking, every router runs its own copy of a routing
protocol (OSPF, IS-IS) and only knows the local topology.  In SDN:

```
Traditional IP:                     SDN:

 [R1]---[R2]---[R3]           [Controller]
  |               |                 |
 runs             runs        knows EVERYTHING
 OSPF             OSPF             |
 locally          locally    [S1]---[S2]---[S3]
                               (dumb switches)
```

The controller has a **global view** of the whole network.  This makes
expensive algorithms (like Floyd-Warshall O(V^3)) practical because they
run once on one machine — not distributed across every router.

## Lab Structure

```
SPF/
├── algorithms/          Pure-Python algorithms (no SDN deps)
│   ├── bfs.py
│   ├── dijkstra.py
│   ├── astar.py
│   ├── bellman_ford.py
│   ├── floyd_warshall.py
│   ├── widest_path.py
│   └── yen_k_shortest.py
│
├── base_controller.py   Shared SDN infrastructure (all controllers inherit)
│
├── *_osken_controller.py  Thin subclasses — override only compute_path()
│
├── topo-*.py            Mininet topology definitions
├── link_weights.json    Per-link bandwidth for weighted routing
│
├── tests/               pytest test suite for algorithms/
└── docs/                Teaching guides (this directory)
```

## Algorithm Comparison Table

| Algorithm       | Complexity       | Weights | Multipath | Teaching Focus                     |
|-----------------|-----------------|---------|-----------|-------------------------------------|
| BFS             | O(V+E)          | No      | No        | Simplest baseline, hop-count        |
| Dijkstra        | O((V+E) log V)  | Yes     | No        | Standard shortest path              |
| A*              | O((V+E) log V)  | Yes     | No        | Heuristic guidance, fewer expansions|
| Bellman-Ford    | O(V*E)          | Yes     | No        | Negative weights, relaxation proof  |
| Floyd-Warshall  | O(V^3)          | Yes     | No        | All-pairs, global SDN advantage     |
| Widest Path     | O((V+E) log V)  | Yes*    | No        | QoS bandwidth routing               |
| Dijkstra MP     | O((V+E) log V)  | No      | Yes       | ECMP equal-cost multipath          |
| A* Multipath    | O((V+E) log V)  | No      | Yes       | ECMP with heuristic search          |
| K-Shortest (Yen)| O(KV(E+V logV)) | No      | Yes       | Path diversity, Yen's algorithm     |

*Widest Path requires weights; degrades to unit-weight routing without them.

## Quick Start

### Terminal 1 — Start Mininet topology

```bash
# Basic 3-switch ring (works with all controllers):
python3 SPF/topo-spf_lab.py

# Weighted 6-switch topology (for widest-path / bellman-ford):
python3 SPF/topo-weighted_lab.py

# Mesh topology (for ECMP / multipath):
python3 SPF/topo-mesh_lab.py
```

### Terminal 2 — Start controller

```bash
# Pick any controller:
python3 SPF/dijkstra_osken_controller.py
python3 SPF/bfs_osken_controller.py
python3 SPF/astar_osken_controller.py
python3 SPF/bellman_ford_osken_controller.py
python3 SPF/floyd_warshall_osken_controller.py
python3 SPF/widest_path_osken_controller.py
python3 SPF/dijkstra_multipath_osken_controller.py
python3 SPF/astar_multipath_osken_controller.py
python3 SPF/kshortest_osken_controller.py
```

### Verify in Mininet CLI

```
mininet> pingall          # Test full connectivity
mininet> h1 ping h6       # Test specific pair
mininet> dpctl dump-flows -O OpenFlow13   # Inspect installed flows
```

## Learning Path

Recommended reading order:

1. [01-sdn-concepts.md](01-sdn-concepts.md) — SDN architecture background
2. [02-openflow-primer.md](02-openflow-primer.md) — OpenFlow 1.3 fundamentals
3. [03-bfs.md](03-bfs.md) — BFS: the simplest algorithm
4. [04-dijkstra.md](04-dijkstra.md) — Dijkstra: the standard approach
5. [05-astar.md](05-astar.md) — A*: heuristic search
6. [06-bellman-ford.md](06-bellman-ford.md) — Bellman-Ford: negative weights
7. [07-floyd-warshall.md](07-floyd-warshall.md) — Floyd-Warshall: all pairs
8. [08-widest-path.md](08-widest-path.md) — Widest path: QoS routing
9. [09-ecmp.md](09-ecmp.md) — ECMP and multipath
10. [10-yen-k-shortest.md](10-yen-k-shortest.md) — Yen's K-shortest paths
