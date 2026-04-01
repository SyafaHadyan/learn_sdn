"""Yen's K-shortest simple paths algorithm.

Yen's algorithm finds the K shortest loopless paths between src and dst
in a directed (or undirected) weighted graph.

Algorithm overview:
    1. Find the shortest path A[0] using Dijkstra.
    2. For k = 1 … K-1:
       a. For each spur node i along path A[k-1]:
          - Remove edges that would immediately duplicate an existing path.
          - Remove all nodes in root_path[0..i-1] to avoid cycles.
          - Run Dijkstra from spur_node to dst on the restricted graph.
          - If a spur path is found, candidate = root_path + spur_path.
       b. Add the cheapest candidate to A.

Complexity:  O(K * V * (E + V log V))  time
Use when:    Multiple diverse paths are needed for ECMP load balancing,
             failover, or traffic engineering.

ECMP relevance:
    OpenFlow SELECT groups distribute traffic across multiple next hops.
    K-shortest paths (K=2 or 4) provides the bucket list for the group.
    Even when all paths have equal hop count Yen's algorithm correctly
    enumerates them if we treat them as having equal cost.
"""

import heapq


def _dijkstra_restricted(adjacency, src, dst, banned_edges, banned_nodes, weights):
    """Dijkstra on a graph with specific edges and nodes removed.

    Args:
        adjacency:   full adjacency dict
        src:         source node
        dst:         destination node  (used only to detect unreachability)
        banned_edges: set of (u, v) edge tuples to skip
        banned_nodes: set of nodes to skip entirely
        weights:     optional {(u,v): cost} dict

    Returns:
        (distance, previous) restricted to the modified graph
    """
    distance = {}
    previous = {}

    for node in adjacency:
        if node in banned_nodes:
            continue
        distance[node] = float("inf")
        previous[node] = None
        for neighbor, _ in adjacency[node]:
            if neighbor in banned_nodes:
                continue
            distance.setdefault(neighbor, float("inf"))
            previous.setdefault(neighbor, None)

    if src not in distance:
        return distance, previous

    distance[src] = 0
    heap = [(0, src)]
    visited = set()

    while heap:
        d, u = heapq.heappop(heap)
        if u in visited:
            continue
        visited.add(u)

        for v, _ in adjacency.get(u, []):
            if v in banned_nodes:
                continue
            if (u, v) in banned_edges:
                continue
            cost = weights[(u, v)] if weights and (u, v) in weights else 1
            alt = d + cost
            if alt < distance.get(v, float("inf")):
                distance[v] = alt
                previous[v] = u
                heapq.heappush(heap, (alt, v))

    return distance, previous


def _reconstruct(previous, src, dst):
    """Reconstruct path from previous pointers.  Returns node list or None."""
    if dst not in previous or previous.get(dst) is None and dst != src:
        return None
    path = [dst]
    current = previous.get(dst)
    while current is not None:
        if current in path:
            return None             # cycle — malformed graph
        path.append(current)
        if current == src:
            break
        current = previous.get(current)
    if not path or path[-1] != src:
        return None
    return list(reversed(path))


def _path_cost(path, weights):
    """Total cost of a path (list of nodes)."""
    total = 0
    for u, v in zip(path[:-1], path[1:]):
        total += weights[(u, v)] if weights and (u, v) in weights else 1
    return total


def yen_k_shortest(adjacency, src, dst, k, weights=None):
    """Yen's K-shortest simple paths.

    Args:
        adjacency: dict  {node: [(neighbor, out_port), ...]}
        src:       source node
        dst:       destination node
        k:         number of paths to find (at most)
        weights:   optional dict  {(u, v): cost}.  Default cost = 1.

    Returns:
        List of at most k paths, each path is a list of node IDs.
        Paths are sorted by increasing cost.
        Empty list if no path exists.
    """
    if src == dst:
        return [[src]]

    # --- Step 1: Find first shortest path ---
    dist0, prev0 = _dijkstra_restricted(adjacency, src, dst, set(), set(), weights)
    first = _reconstruct(prev0, src, dst)
    if first is None:
        return []

    A = [first]                             # confirmed K shortest paths
    B = []                                  # candidate heap: (cost, path)

    # --- Step 2: Iteratively find paths A[1] … A[k-1] ---
    for i in range(1, k):
        prev_path = A[i - 1]

        # For each spur node along the previous best path:
        for j in range(len(prev_path) - 1):
            spur_node = prev_path[j]
            root_path = prev_path[:j + 1]   # fixed prefix up to spur_node

            # Remove edges used by any existing path with the same root
            banned_edges = set()
            for existing in A:
                if len(existing) > j and existing[:j + 1] == root_path:
                    u = existing[j]
                    v = existing[j + 1]
                    banned_edges.add((u, v))
                    banned_edges.add((v, u))    # undirected graph

            # Remove all nodes in root (except spur_node) to avoid cycles
            banned_nodes = set(root_path[:-1])

            dist_s, prev_s = _dijkstra_restricted(
                adjacency, spur_node, dst, banned_edges, banned_nodes, weights
            )
            spur_path = _reconstruct(prev_s, spur_node, dst)

            if spur_path:
                # Combine root and spur (root ends at spur_node, spur starts there)
                candidate = root_path[:-1] + spur_path
                cost = _path_cost(candidate, weights)

                # Avoid duplicates in candidate list
                if candidate not in [p for _, p in B] and candidate not in A:
                    heapq.heappush(B, (cost, candidate))

        if not B:
            break                           # no more candidates available

        _cost, best = heapq.heappop(B)
        A.append(best)

    return A
