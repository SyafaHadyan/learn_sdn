"""Dijkstra's shortest path algorithm.

Dijkstra finds shortest paths from a single source to all other nodes
in a graph with *non-negative* edge weights.  It uses a min-heap (priority
queue) to always expand the closest unvisited node (greedy strategy).

Complexity:  O((V + E) log V)  time with a binary heap,  O(V)  space
Use when:    Edge weights are non-negative (hop count, delay, cost, etc.).
Limitation:  Incorrect on graphs with negative-weight edges.

Two functions are provided:
    dijkstra()              – returns the single best predecessor per node.
    dijkstra_multi_parent() – returns ALL equal-cost predecessors, needed
                              for ECMP (equal-cost multipath) enumeration.
"""

import heapq
from collections import defaultdict


def dijkstra(adjacency, src, weights=None):
    """Dijkstra's algorithm — single shortest path.

    Args:
        adjacency: dict  {node: [(neighbor, out_port), ...]}
        weights:   optional dict  {(u, v): cost}.  Default cost = 1.

    Returns:
        (distance, previous)
        distance[v] = shortest cost from src to v  (inf if unreachable)
        previous[v] = single best predecessor on shortest path from src
    """
    # --- Phase 1: Initialise ---
    distance = {}
    previous = {}
    for node in adjacency:
        distance[node] = float("inf")
        previous[node] = None
        for neighbor, _ in adjacency[node]:
            distance.setdefault(neighbor, float("inf"))
            previous.setdefault(neighbor, None)

    if src not in distance:
        return distance, previous

    distance[src] = 0
    # Min-heap entries: (current_dist, node)
    heap = [(0, src)]
    visited = set()

    # --- Phase 2: Greedy expansion ---
    while heap:
        d, u = heapq.heappop(heap)          # extract minimum-distance node

        if u in visited:                    # skip stale heap entries
            continue
        visited.add(u)

        # --- Phase 3: Relaxation ---
        # For each neighbour v of u, check if the path through u is shorter.
        for v, _ in adjacency.get(u, []):
            cost = weights[(u, v)] if weights and (u, v) in weights else 1
            alt = d + cost
            if alt < distance.get(v, float("inf")):
                distance[v] = alt
                previous[v] = u
                heapq.heappush(heap, (alt, v))

    return distance, previous


def dijkstra_multi_parent(adjacency, src, weights=None):
    """Dijkstra variant that keeps ALL equal-cost predecessors per node.

    Used by multipath controllers to enumerate all equal-cost paths for
    ECMP load balancing.

    Args:
        adjacency: dict  {node: [(neighbor, out_port), ...]}
        weights:   optional dict  {(u, v): cost}.  Default cost = 1.

    Returns:
        (distance, parents)
        distance[v] = shortest cost from src to v
        parents[v]  = set of nodes u where dist[u] + cost(u,v) == dist[v]
    """
    # --- Phase 1: Initialise ---
    distance = {}
    parents = defaultdict(set)

    for node in adjacency:
        distance[node] = float("inf")
        for neighbor, _ in adjacency[node]:
            distance.setdefault(neighbor, float("inf"))

    if src not in distance:
        return distance, parents

    distance[src] = 0
    heap = [(0, src)]

    # --- Phase 2: Greedy expansion, collecting ALL equal-cost parents ---
    while heap:
        d, u = heapq.heappop(heap)

        if d > distance.get(u, float("inf")):   # stale entry
            continue

        for v, _ in adjacency.get(u, []):
            cost = weights[(u, v)] if weights and (u, v) in weights else 1
            alt = d + cost

            if alt < distance.get(v, float("inf")):
                # Strictly better path found — replace parent set
                distance[v] = alt
                parents[v] = {u}
                heapq.heappush(heap, (alt, v))
            elif alt == distance.get(v, float("inf")) and u not in parents[v]:
                # Equal-cost path found — add to parent set (no re-push needed)
                parents[v].add(u)

    return distance, parents
