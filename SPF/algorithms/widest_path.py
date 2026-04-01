"""Widest-path (maximum-bottleneck) shortest path algorithm.

The widest path maximises the *minimum edge bandwidth* along the path —
the path whose bottleneck link has the highest bandwidth.  This is useful
for Quality-of-Service (QoS) routing where available bandwidth matters more
than hop count.

Also known as: maximum-capacity path, max-min bandwidth path.

Algorithm:
    Modified Dijkstra where:
    - "distance" is replaced by *maximum achievable bottleneck bandwidth*
    - We use a max-heap (negate bandwidth for Python's min-heap)
    - Relaxation: bottleneck(u→v) = min(bottleneck_so_far[u], bandwidth(u,v))
    - We keep the path with the highest bottleneck

Complexity:  O((V + E) log V)  time,  O(V)  space
Use when:    Link bandwidths are known and you want maximum throughput paths.
Limitation:  Requires accurate bandwidth information per link.

Example (why widest path ≠ shortest path):
    s1 ─(10 Mbps)─ s2 ─(10 Mbps)─ s3   (2 hops, bottleneck = 10 Mbps)
    s1 ─(100 Mbps)─────────────── s3    (1 hop,  bottleneck = 100 Mbps)
    Shortest path: s1→s2→s3 (2 hops)
    Widest path:   s1→s3    (100 Mbps bottleneck — 10× better bandwidth)
"""

import heapq


def widest_path(adjacency, src, weights):
    """Maximum-bottleneck path from src to all reachable nodes.

    Args:
        adjacency: dict  {node: [(neighbor, out_port), ...]}
        src:       source node
        weights:   dict  {(u, v): bandwidth_mbps}  (required, no default)
                   Missing edges are treated as bandwidth = 0.

    Returns:
        (max_bw, previous)
        max_bw[v]   = bottleneck bandwidth of widest path from src to v
                      (0 if unreachable)
        previous[v] = predecessor on widest path from src
    """
    # --- Phase 1: Initialise ---
    max_bw = {}
    previous = {}
    for node in adjacency:
        max_bw[node] = 0
        previous[node] = None
        for neighbor, _ in adjacency[node]:
            max_bw.setdefault(neighbor, 0)
            previous.setdefault(neighbor, None)

    if src not in max_bw:
        return max_bw, previous

    # Source can send at unlimited rate to itself
    max_bw[src] = float("inf")

    # Max-heap via negated bandwidth: (-bottleneck, node)
    heap = [(-float("inf"), src)]
    visited = set()

    # --- Phase 2: Greedy max-bottleneck expansion ---
    while heap:
        neg_bw, u = heapq.heappop(heap)
        current_bw = -neg_bw

        if u in visited:                        # skip stale entries
            continue
        visited.add(u)

        # --- Phase 3: Widest-path relaxation ---
        # The bottleneck of reaching v via u is limited by:
        #   min(widest-path bandwidth to u,  link bandwidth u→v)
        for v, _ in adjacency.get(u, []):
            link_bw = weights.get((u, v), 0)
            bottleneck = min(current_bw, link_bw)

            if bottleneck > max_bw.get(v, 0):  # strictly better path found
                max_bw[v] = bottleneck
                previous[v] = u
                heapq.heappush(heap, (-bottleneck, v))

    return max_bw, previous
