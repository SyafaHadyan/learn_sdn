"""Bellman-Ford SPF OpenFlow controller.

Single-source shortest path forwarding using the Bellman-Ford algorithm.
Bellman-Ford is O(V*E) — slower than Dijkstra — but supports negative-weight
edges and detects negative-weight cycles, making it essential for teaching
the general-case relaxation-based approach to shortest paths.

Complexity:   O(V * E) — relaxes all edges V-1 times
Metric:       configurable weights from link_weights.json; defaults to hop count
Negative wts: supported (unlike Dijkstra)
Multipath:    no - single best path per (src, dst)
ECMP:         no

When to use Bellman-Ford vs Dijkstra:
    Bellman-Ford: negative weights, negative-cycle detection, simpler proof
    Dijkstra:     positive weights only, faster O((V+E) log V)
    For SDN hop-count routing: Dijkstra is preferred; Bellman-Ford is for teaching.

Run:
    python3 bellman_ford_osken_controller.py
    python3 bellman_ford_osken_controller.py --verbose

See Also:
    dijkstra_osken_controller.py      - faster for non-negative weights
    floyd_warshall_osken_controller.py - all-pairs, same relaxation idea
"""

import json
import os
import sys

from base_controller import SPFBaseController
from algorithms.bellman_ford import bellman_ford

BF_FLOW_COOKIE = 0x42464F520000000001  # "BFOR" in hex (truncated to 64-bit)
BF_FLOW_COOKIE = 0x42464F5200000001
BF_FLOW_COOKIE_MASK = 0xFFFFFFFFFFFFFFFF
BF_FLOW_PRIORITY = 100

# Optional static weight file — relative to this controller's directory.
# Format: {"links": {""dpid1:dpid2"": {"bandwidth_mbps": 100}, ...}}
WEIGHTS_FILE = os.path.join(os.path.dirname(__file__), "link_weights.json")


def _load_weights(path):
    """Load link weights from JSON; return {} if file is absent or malformed."""
    try:
        with open(path) as f:
            data = json.load(f)
        # Convert "dpid1:dpid2" string keys to (int, int) tuples.
        raw = data.get("links", {})
        return {
            tuple(int(x) for x in k.split(":")): v.get("bandwidth_mbps", 1)
            for k, v in raw.items()
        }
    except (FileNotFoundError, json.JSONDecodeError, ValueError):
        return {}


class BellmanFordSwitch(SPFBaseController):
    """Shortest path forwarding using the Bellman-Ford relaxation algorithm.

    Supports optional link weights from link_weights.json.  If no weight file
    is found, all edges get weight 1 (identical to BFS/Dijkstra hop count).

    Teaching note — Bellman-Ford termination proof:
        After k iterations the algorithm has found all shortest paths that
        use at most k edges.  With V nodes, the longest simple path has V-1
        edges, so V-1 iterations are always sufficient (for no negative cycle).
        A Vth iteration that still improves a distance proves a negative cycle.
    """

    FLOW_COOKIE = BF_FLOW_COOKIE

    def __init__(self, *args, **kwargs):
        super(BellmanFordSwitch, self).__init__(*args, **kwargs)
        # Load once at startup; topology changes do not affect link weights.
        raw_weights = _load_weights(WEIGHTS_FILE)
        # Convert to adjacency-indexed form: (u, v) -> weight
        self._link_weights = raw_weights
        if raw_weights:
            self.logger.info("[BF-WEIGHTS] loaded %d link weights from %s",
                             len(raw_weights), WEIGHTS_FILE)
        else:
            self.logger.info("[BF-WEIGHTS] no weight file; using hop-count metric")

    def _build_weight_dict(self):
        """Build weights dict keyed by (u, v) from current adjacency."""
        if not self._link_weights:
            return None          # algorithms/bellman_ford uses hop-count by default
        weights = {}
        for u in self.adjacency:
            for v, _ in self.adjacency[u]:
                key = (min(u, v), max(u, v))
                weights[(u, v)] = self._link_weights.get(key, 1)
                weights[(v, u)] = self._link_weights.get(key, 1)
        return weights

    def compute_path(self, src, dst, first_port, final_port):
        """Compute shortest path using Bellman-Ford edge-relaxation.

        Algorithm steps:
            1. Initialise distance[src]=0, all others=inf
            2. Relax all edges V-1 times (guarantees shortest simple paths)
            3. Vth pass detects negative cycles (logged as warning if found)
            4. Reconstruct path via predecessor pointers
        """
        self.logger.debug("[PATH-QUERY] Bellman-Ford: s%d -> s%d", src, dst)

        weights = self._build_weight_dict()

        # --- Phase 1+2: Relax all edges V-1 times ---
        distance, previous, has_neg_cycle = bellman_ford(
            self.adjacency, src, weights=weights
        )

        # --- Phase 3: Report negative cycle (informational) ---
        if has_neg_cycle:
            self.logger.warning("[BF-NEGCYCLE] negative-weight cycle detected in topology!")

        reachable = sum(1 for d in distance.values() if d != float("inf"))
        self.logger.info("[SPF-DONE] BF s%d->s%d reachable=%d/%d",
                         src, dst, reachable, len(distance))

        # --- Phase 4: Reconstruct path ---
        return self._reconstruct_path(src, dst, first_port, final_port, distance, previous)


if __name__ == '__main__':
    current_file = os.path.abspath(__file__)
    passthrough_args = sys.argv[1:]
    if '--observe-links' not in passthrough_args:
        passthrough_args = ['--observe-links'] + passthrough_args
    sys.argv = ['bellman_ford_osken_controller', *passthrough_args, current_file]
    from os_ken.cmd.manager import main
    sys.exit(main())
