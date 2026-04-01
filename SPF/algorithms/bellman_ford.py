"""Bellman-Ford shortest path algorithm.

Bellman-Ford relaxes ALL edges repeatedly for |V|-1 rounds, guaranteeing
that the shortest path is found even when edge weights are negative.
It also detects negative-weight cycles (where no finite shortest path exists).

Complexity:  O(V * E)  time,  O(V)  space
             With early termination O(l * E) where l = shortest path length
Use when:    Edge weights may be negative (e.g., delay improvements, credits).
Limitation:  Slower than Dijkstra for non-negative weights.

Routing protocol analogy:
    Bellman-Ford is the foundation of distance-vector protocols (RIP, BGP).
    Each router iteratively exchanges distance estimates with neighbours until
    convergence — this is precisely Bellman-Ford distributed over a network.
    The "count-to-infinity" problem in RIP occurs when a node becomes
    unreachable and routers keep incrementing estimates indefinitely.
"""


def bellman_ford(adjacency, src, weights=None):
    """Bellman-Ford algorithm from a single source.

    Args:
        adjacency: dict  {node: [(neighbor, out_port), ...]}
        src:       source node
        weights:   optional dict  {(u, v): cost}.  Default cost = 1.
                   Negative costs are supported.

    Returns:
        (distance, previous, has_negative_cycle)
        distance[v]          = shortest cost from src to v  (inf if unreachable)
        previous[v]          = predecessor on shortest path from src
        has_negative_cycle   = True if a negative cycle reachable from src exists
    """
    # --- Phase 1: Collect all vertices and build edge list ---
    vertices = set()
    edges = []                              # list of (u, v, cost)
    for u in adjacency:
        vertices.add(u)
        for v, _ in adjacency[u]:
            vertices.add(v)
            cost = weights[(u, v)] if weights and (u, v) in weights else 1
            edges.append((u, v, cost))

    if src not in vertices:
        return {}, {}, False

    # --- Phase 2: Initialise distances ---
    distance = {v: float("inf") for v in vertices}
    previous = {v: None for v in vertices}
    distance[src] = 0

    n = len(vertices)

    # --- Phase 3: Relax all edges |V|-1 times ---
    # After round i, shortest paths using at most i edges are correct.
    # After |V|-1 rounds, all simple-path shortest distances are correct.
    for _iteration in range(n - 1):
        updated = False                     # early-termination flag

        for u, v, cost in edges:
            if distance[u] == float("inf"):
                continue                   # u unreachable, skip
            if distance[u] + cost < distance[v]:
                distance[v] = distance[u] + cost
                previous[v] = u
                updated = True

        if not updated:
            # No change this round — optimal distances already found
            break

    # --- Phase 4: Detect negative-weight cycles ---
    # If a distance can still be improved after |V|-1 rounds, a negative
    # cycle exists on some path from src.
    has_negative_cycle = False
    for u, v, cost in edges:
        if distance[u] != float("inf") and distance[u] + cost < distance[v]:
            has_negative_cycle = True
            break

    return distance, previous, has_negative_cycle
