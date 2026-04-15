"""Floyd-Warshall All-Pairs SPF OpenFlow controller.

Computes shortest paths between ALL pairs of switches in one O(V^3)
pre-computation pass, then answers individual path queries in O(V) time
using the stored next-hop table.  This demonstrates the global-view
advantage of the SDN controller: it can afford expensive pre-computation
because it runs off the data plane.

Complexity:   O(V^3) pre-computation; O(V) per path query
Metric:       configurable weights from link_weights.json; defaults to hop count
Multipath:    no - single best path per (src, dst)
ECMP:         no

SDN teaching note:
    Traditional distributed routing (OSPF, IS-IS) runs shortest-path locally
    on each router with only local network-view.  An SDN controller runs on a
    single machine with the full global topology, making O(V^3) algorithms
    like Floyd-Warshall practical even for moderately large networks.

Run:
    python3 floyd_warshall_osken_controller.py
    python3 floyd_warshall_osken_controller.py --verbose

See Also:
    dijkstra_osken_controller.py   - single-source, O((V+E) log V) per query
    bellman_ford_osken_controller.py - all-source, supports negative weights
"""

import json
import os
import sys

from base_controller import SPFBaseController
from algorithms.floyd_warshall import floyd_warshall

FW_FLOW_COOKIE = 0x464C4F5700000001     # "FLOW" in hex
FW_FLOW_COOKIE_MASK = 0xFFFFFFFFFFFFFFFF
FW_FLOW_PRIORITY = 100

WEIGHTS_FILE = os.path.join(os.path.dirname(__file__), "link_weights.json")


def _load_weights(path):
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


class FloydWarshallSwitch(SPFBaseController):
    """All-pairs shortest-path forwarding using Floyd-Warshall.

    Overrides _on_topology_changed() to pre-compute the all-pairs
    distance matrix and next-hop table immediately after any topology update.
    Individual path queries then just do table lookups.

    Internal tables (set after each topo change):
        _fw_dist[u][v]     = shortest path cost from u to v
        _fw_next[u][v]     = first switch on the shortest path from u to v
    """

    FLOW_COOKIE = FW_FLOW_COOKIE

    def __init__(self, *args, **kwargs):
        super(FloydWarshallSwitch, self).__init__(*args, **kwargs)
        self._fw_dist = {}           # dist[u][v] = shortest cost u->v
        self._fw_next = {}           # next[u][v] = first hop on path u->v
        raw_weights = _load_weights(WEIGHTS_FILE)
        self._link_weights = raw_weights
        if raw_weights:
            self.logger.info("[FW-WEIGHTS] loaded %d link weights from %s",
                             len(raw_weights), WEIGHTS_FILE)
        else:
            self.logger.info("[FW-WEIGHTS] no weight file; using hop-count metric")

    def _build_weight_dict(self):
        if not self._link_weights:
            return None
        weights = {}
        for u in self.adjacency:
            for v, _ in self.adjacency[u]:
                key = (min(u, v), max(u, v))
                weights[(u, v)] = self._link_weights.get(key, 1)
                weights[(v, u)] = self._link_weights.get(key, 1)
        return weights

    def _on_topology_changed(self):
        """Pre-compute Floyd-Warshall immediately after topology changes.

        Floyd-Warshall runs once here; compute_path() uses the cached tables.
        Algorithm steps:
            1. Initialise: dist[u][v] = edge_weight(u,v); dist[u][u] = 0
            2. For each intermediate node k: relax dist[u][v] via k
            3. Store next-hop table for O(V) path reconstruction
        """
        if not self.switches:
            self._fw_dist = {}
            self._fw_next = {}
            return

        self.logger.info("[FW-COMPUTE] running Floyd-Warshall on %d switches",
                         len(self.switches))
        weights = self._build_weight_dict()

        # --- Floyd-Warshall all-pairs pre-computation ---
        self._fw_dist, self._fw_next = floyd_warshall(
            self.adjacency, self.switches, weights=weights
        )

        # Count reachable pairs
        reachable = sum(
            1 for u in self.switches for v in self.switches
            if u != v and self._fw_dist.get(u, {}).get(v, float("inf")) != float("inf")
        )
        total_pairs = len(self.switches) * (len(self.switches) - 1)
        self.logger.info("[FW-DONE] reachable_pairs=%d/%d", reachable, total_pairs)

    def compute_path(self, src, dst, first_port, final_port):
        """Reconstruct path from Floyd-Warshall next-hop table.

        Because _on_topology_changed() pre-computed all pairs, this method
        only needs to walk the next-hop chain: O(V).

        Reconstruction from next-hop:
            path = [src]
            while path[-1] != dst:
                path.append(next_hop[path[-1]][dst])
        """
        self.logger.debug("[PATH-QUERY] FW: s%d -> s%d", src, dst)

        if not self._fw_dist or src not in self._fw_dist:
            self.logger.warning("[FW-NOTREADY] tables not yet built, falling back to empty path")
            return []

        if src not in self.switches or dst not in self.switches:
            return []

        if src == dst:
            return [(src, first_port, final_port)]

        if self._fw_dist.get(src, {}).get(dst, float("inf")) == float("inf"):
            self.logger.warning("[PATH-UNREACHABLE] FW: s%d->s%d unreachable", src, dst)
            return []

        # --- Walk next-hop table to reconstruct node list ---
        node_path = [src]
        current = src
        visited = {src}
        while current != dst:
            nxt = self._fw_next.get(current, {}).get(dst)
            if nxt is None or nxt in visited:
                self.logger.error("[PATH-CORRUPT] FW: s%d->s%d reconstruction failed at s%d",
                                  src, dst, current)
                return []
            visited.add(nxt)
            node_path.append(nxt)
            current = nxt

        # --- Annotate with port numbers ---
        result = []
        in_port = first_port
        for s1, s2 in zip(node_path[:-1], node_path[1:]):
            out_port = self._get_port(s1, s2)
            if out_port is None:
                self.logger.error("[PATH-PORTMAP] FW: s%d->s%d port missing", s1, s2)
                return []
            result.append((s1, in_port, out_port))
            in_port = self._get_port(s2, s1)
        result.append((dst, in_port, final_port))

        self.logger.info("[PATH-COMPUTED] FW s%d->s%d cost=%.1f hops=%d",
                         src, dst,
                         self._fw_dist.get(src, {}).get(dst, float("inf")),
                         len(result) - 1)
        return result


if __name__ == '__main__':
    current_file = os.path.abspath(__file__)
    passthrough_args = sys.argv[1:]
    if '--observe-links' not in passthrough_args:
        passthrough_args = ['--observe-links'] + passthrough_args
    sys.argv = ['floyd_warshall_osken_controller', *passthrough_args, current_file]
    from os_ken.cmd.manager import main
    sys.exit(main())
