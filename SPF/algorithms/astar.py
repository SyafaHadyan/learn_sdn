"""A* (A-star) heuristic shortest path algorithm.

A* is an informed search algorithm that extends Dijkstra by incorporating
a heuristic estimate h(n) of the remaining cost from node n to the goal.
It expands nodes in order of f(n) = g(n) + h(n), where g(n) is the known
cost from source to n.

Complexity:  O((V + E) log V)  with a consistent (monotone) heuristic
Use when:    A good heuristic is available that reduces search space.
Limitation:  Heuristic must be admissible (never overestimates true cost).

Key advantage over Dijkstra:
    When h(n) is exact (no overestimation), A* expands only nodes on the
    optimal path and avoids expanding nodes that cannot improve the solution.
    In hop-count routing, reverse BFS gives an exact lower bound.

A* for SDN:
    The network graph is small (tens of switches), so the heuristic savings
    are modest.  A* is included here to teach the concept and demonstrate
    how informed search differs from blind Dijkstra.
"""

import heapq
from collections import deque


def build_reverse_hop_heuristic(adjacency, dst):
    """Build a hop-distance heuristic table via reverse BFS from dst.

    For hop-count routing this is an *exact* lower bound: h(n) equals the
    minimum number of hops from n to dst.  An exact lower bound is both
    admissible and consistent, guaranteeing A* finds the optimal path.

    Args:
        adjacency: dict  {node: [(neighbor, out_port), ...]}
        dst:       destination node

    Returns:
        dict  {node: hop_distance_to_dst}
        Nodes unreachable from dst have no entry (callers should return inf).
    """
    hops = {dst: 0}
    queue = deque([dst])

    while queue:
        u = queue.popleft()
        for v, _ in adjacency.get(u, []):       # treat graph as undirected
            if v not in hops:
                hops[v] = hops[u] + 1
                queue.append(v)

    return hops


def astar(adjacency, src, dst, heuristic, weights=None):
    """A* shortest path from src to dst.

    Args:
        adjacency: dict  {node: [(neighbor, out_port), ...]}
        src:       source node
        dst:       destination node
        heuristic: dict  {node: estimated_cost_to_dst}
                   or callable  fn(node) -> estimated_cost_to_dst
        weights:   optional dict  {(u, v): cost}.  Default cost = 1.

    Returns:
        (distance, previous)
        distance[v] = known cost of shortest path from src to v
        previous[v] = predecessor on shortest path from src

    Note:
        Only nodes on or near the optimal path are fully explored.
        Nodes with h(n) == inf are never expanded (unreachable from dst).
    """
    # Helper to look up heuristic value
    def h(node):
        if callable(heuristic):
            return heuristic(node)
        return heuristic.get(node, float("inf"))

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

    # Heap entries: (f_score, g_score, node)
    # g_score = cost from src; f_score = g + h (estimated total cost)
    h_src = h(src)
    heap = [(h_src, 0, src)]
    closed = set()                          # nodes whose optimal cost is final

    # --- Phase 2: Guided expansion ---
    while heap:
        f, g, u = heapq.heappop(heap)

        if u in closed:                     # already settled
            continue
        if g > distance.get(u, float("inf")):   # stale heap entry
            continue

        closed.add(u)

        # Early exit: reached destination with optimal cost
        if u == dst:
            break

        # --- Phase 3: Relaxation with heuristic guidance ---
        for v, _ in adjacency.get(u, []):
            cost = weights[(u, v)] if weights and (u, v) in weights else 1
            tentative_g = g + cost

            if tentative_g < distance.get(v, float("inf")):
                distance[v] = tentative_g
                previous[v] = u
                h_v = h(v)
                if h_v < float("inf"):      # skip nodes unreachable from dst
                    heapq.heappush(heap, (tentative_g + h_v, tentative_g, v))

    return distance, previous
