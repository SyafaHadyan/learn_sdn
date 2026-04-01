"""Widest-Path (Max-Bandwidth) OpenFlow controller.

Routes each flow along the path whose bottleneck bandwidth is maximised.
Instead of minimising hop count, this algorithm maximises the minimum
link bandwidth on the path - useful for QoS-aware routing in networks
with heterogeneous link capacities.

Complexity:   O((V+E) log V) - modified Dijkstra with max-heap
Metric:       bandwidth from link_weights.json (falls back to hop count)
Multipath:    no - single best path per (src, dst)
ECMP:         no

Algorithm summary:
    Like Dijkstra, but "distance" is bottleneck bandwidth (maximise, not minimise).
    Relax: bw[v] = max(bw[v], min(bw[u], edge_bw(u,v)))
    Max-heap: negate bandwidth to use Python's min-heap.

When to use widest-path:
    Video streaming, bulk data transfer, or any flow sensitive to
    bandwidth rather than latency.  Requires per-link bandwidth in
    link_weights.json.

Run:
    python3 widest_path_osken_controller.py

See Also:
    dijkstra_osken_controller.py      - minimises hop count / cost
    bellman_ford_osken_controller.py  - supports negative-weight metrics
"""

import json
import os
import sys

from base_controller import SPFBaseController
from algorithms.widest_path import widest_path

WP_FLOW_COOKIE = 0x5749445000000001     # "WIDP" in hex
WP_FLOW_COOKIE_MASK = 0xFFFFFFFFFFFFFFFF
WP_FLOW_PRIORITY = 100

WEIGHTS_FILE = os.path.join(os.path.dirname(__file__), "link_weights.json")


def _load_weights(path):
    """Load bandwidth-per-link from JSON; return {} if absent or malformed."""
    try:
        with open(path) as f:
            data = json.load(f)
        raw = data.get("links", {})
        return {
            tuple(int(x) for x in k.split(":")): v.get("bandwidth_mbps", 1)
            for k, v in raw.items()
        }
    except (FileNotFoundError, json.JSONDecodeError, ValueError):
        return {}


class WidestPathSwitch(SPFBaseController):
    """QoS-aware routing: maximise bottleneck bandwidth on each path.

    Requires link_weights.json for per-link bandwidth values.
    Falls back to hop-count routing (all weights = 1) if the file is absent.

    Teaching note - widest path vs shortest path:
        Shortest path: minimise sum of edge costs.
        Widest path:   maximise the minimum edge bandwidth on the path.
        Also called "Maximum Bottleneck Path".

        Example:  s1 -(10Mb)- s2 -(10Mb)- s3   bottleneck = 10 Mb (2 hops)
                  s1 -(100Mb)------------- s3   bottleneck = 100 Mb (1 hop)
        Shortest path chooses s1->s2->s3 (fewer hops).
        Widest path chooses s1->s3 (10x more bandwidth).
    """

    FLOW_COOKIE = WP_FLOW_COOKIE

    def __init__(self, *args, **kwargs):
        super(WidestPathSwitch, self).__init__(*args, **kwargs)
        raw_weights = _load_weights(WEIGHTS_FILE)
        self._link_weights = raw_weights
        if raw_weights:
            self.logger.info("[WP-WEIGHTS] loaded %d bandwidth entries from %s",
                             len(raw_weights), WEIGHTS_FILE)
        else:
            self.logger.warning("[WP-WEIGHTS] no weight file; "
                                "widest-path degrades to unit-weight routing")

    def _build_weight_dict(self):
        """Build (u, v) -> bandwidth dict indexed by directed edges."""
        if not self._link_weights:
            # No weights: treat every link as 1 Mbps (equal, uninformative)
            weights = {}
            for u in self.adjacency:
                for v, _ in self.adjacency[u]:
                    weights[(u, v)] = 1
            return weights
        weights = {}
        for u in self.adjacency:
            for v, _ in self.adjacency[u]:
                key = (min(u, v), max(u, v))
                bw = self._link_weights.get(key, 1)
                weights[(u, v)] = bw
                weights[(v, u)] = bw
        return weights

    def compute_path(self, src, dst, first_port, final_port):
        """Compute maximum-bandwidth-bottleneck path.

        Algorithm steps:
            1. Build per-link bandwidth weights from loaded config
            2. Run widest_path (modified Dijkstra: maximise bottleneck BW)
            3. Convert max_bw result to distance convention for _reconstruct_path
               (unreachable = max_bw[v] == 0 -> map to float("inf"))
        """
        self.logger.debug("[PATH-QUERY] WidestPath: s%d -> s%d", src, dst)

        weights = self._build_weight_dict()

        # --- Phase 1+2: Modified Dijkstra maximising bottleneck bandwidth ---
        # max_bw[v] = bottleneck bandwidth of widest path src -> v (0=unreachable)
        # previous[v] = predecessor on widest path
        max_bw, previous = widest_path(self.adjacency, src, weights)

        dst_bw = max_bw.get(dst, 0.0)
        if dst_bw > 0:
            self.logger.info("[WP-DONE] s%d->s%d bottleneck=%.1f Mbps", src, dst, dst_bw)
        else:
            self.logger.warning("[WP-UNREACHABLE] s%d->s%d no path with non-zero bandwidth",
                                src, dst)

        # --- Phase 3: Convert to distance convention (_reconstruct_path uses inf=unreachable) ---
        # max_bw is maximised (large = good); _reconstruct_path expects dist where
        # small = good and inf = unreachable.  Map: unreachable (bw=0) -> inf, else 1.
        all_nodes = set(self.adjacency.keys()) | set(self.switches)
        distance = {
            v: float("inf") if max_bw.get(v, 0.0) == 0.0 else 1.0
            for v in all_nodes
        }
        distance[src] = 0.0  # source is always reachable

        return self._reconstruct_path(src, dst, first_port, final_port, distance, previous)


if __name__ == '__main__':
    current_file = os.path.abspath(__file__)
    passthrough_args = sys.argv[1:]
    if '--observe-links' not in passthrough_args:
        passthrough_args = ['--observe-links'] + passthrough_args
    sys.argv = ['widest_path_osken_controller', *passthrough_args, current_file]
    from os_ken.cmd.manager import main
    sys.exit(main())
