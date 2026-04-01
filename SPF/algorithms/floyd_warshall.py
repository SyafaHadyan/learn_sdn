"""Floyd-Warshall all-pairs shortest paths algorithm.

Floyd-Warshall uses dynamic programming to compute shortest paths between
every pair of nodes in a single O(V^3) pass.  For each intermediate node k,
it checks whether routing through k improves any pairwise distance.

Complexity:  O(V^3)  time,  O(V^2)  space
Use when:    You need ALL pairs of shortest paths upfront (e.g., in a
             centralised SDN controller with a small number of switches).
Limitation:  Impractical for large networks (> ~100 switches due to O(V^3)).
             Cannot handle negative-weight cycles.

SDN advantage:
    A centralised controller has global topology visibility, making all-pairs
    precomputation natural.  After a topology change, the controller can
    recompute all-pairs paths and proactively push flows for all known hosts.
    This avoids per-packet path computation latency.

Comparison with Dijkstra:
    Dijkstra (single-source): O((V+E) log V) — run once per source.
    Floyd-Warshall (all-pairs): O(V^3) — single pass for all pairs.
    Break-even: Floyd-Warshall wins when V^3 < V * (V+E) log V,
                approximately when E >> V (dense graphs).
"""


def floyd_warshall(adjacency, switches, weights=None):
    """Floyd-Warshall all-pairs shortest paths.

    Args:
        adjacency: dict  {node: [(neighbor, out_port), ...]}
        switches:  list of all switch IDs (must include all nodes)
        weights:   optional dict  {(u, v): cost}.  Default cost = 1.

    Returns:
        (dist, next_hop)
        dist[u][v]     = shortest cost from u to v  (inf if unreachable)
        next_hop[u][v] = first switch to visit on the shortest path from u to v
                         (None if unreachable, u if u == v)
    """
    # --- Phase 1: Initialise distance and next-hop matrices ---
    INF = float("inf")

    # Start with all distances as infinity
    dist = {u: {v: INF for v in switches} for u in switches}
    next_hop = {u: {v: None for v in switches} for u in switches}

    # Distance from any node to itself is 0
    for u in switches:
        dist[u][u] = 0
        next_hop[u][u] = u

    # Populate direct edges
    for u in adjacency:
        if u not in dist:
            continue
        for v, _ in adjacency[u]:
            if v not in dist[u]:
                continue
            cost = weights[(u, v)] if weights and (u, v) in weights else 1
            if cost < dist[u][v]:
                dist[u][v] = cost
                next_hop[u][v] = v          # first hop from u to v is v itself

    # --- Phase 2: Dynamic programming over intermediate nodes ---
    # Recurrence:
    #   dist[u][v] = min( dist[u][v],  dist[u][k] + dist[k][v] )
    # After considering all k, dist[u][v] holds the true shortest path.
    for k in switches:
        for u in switches:
            if dist[u][k] == INF:
                continue                    # no path from u to k, skip row
            for v in switches:
                through_k = dist[u][k] + dist[k][v]
                if through_k < dist[u][v]:
                    dist[u][v] = through_k
                    # The first hop from u to v (via k) is the same as the
                    # first hop from u to k.
                    next_hop[u][v] = next_hop[u][k]

    return dist, next_hop
