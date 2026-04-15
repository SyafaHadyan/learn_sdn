"""Dijkstra SPF OpenFlow controller.

Single-source shortest path forwarding using Dijkstra's algorithm.
Inherits all SDN infrastructure from SPFBaseController; adds only the
path-computation algorithm.

Complexity:   O((V+E) log V) per path query
Metric:       hop count (all edges weight 1)
Multipath:    no - single best path per (src, dst)
ECMP:         no

Run:
    python3 dijkstra_osken_controller.py
    python3 dijkstra_osken_controller.py --verbose

See Also:
    astar_osken_controller.py              - same metric, guided search
    dijkstra_multipath_osken_controller.py - adds ECMP over equal-cost paths
"""

import os
import sys

from base_controller import SPFBaseController
from algorithms.dijkstra import dijkstra

# Kept as module-level constants for backward compatibility:
# dijkstra_multipath_osken_controller.py imports these by name.
SPF_FLOW_COOKIE = 0x5346500000000001
SPF_FLOW_COOKIE_MASK = 0xFFFFFFFFFFFFFFFF
SPF_FLOW_PRIORITY = 100


class DijkstraSwitch(SPFBaseController):
    """Single shortest-path forwarding using Dijkstra's algorithm.

    Overrides compute_path() to call algorithms/dijkstra.py.
    All SDN infrastructure (topology, host learning, flows, flooding)
    is provided by SPFBaseController.
    """

    FLOW_COOKIE = SPF_FLOW_COOKIE

    def compute_path(self, src, dst, first_port, final_port):
        """Compute shortest path using Dijkstra's algorithm.

        Algorithm steps:
            1. Run Dijkstra from src - O((V+E) log V)
            2. Reconstruct path by following predecessor pointers
        """
        self.logger.debug("[PATH-QUERY] Dijkstra: s%d -> s%d", src, dst)

        # --- Phase 1: Run Dijkstra from source switch ---
        # Returns distance[v] = min hop count from src to v
        #         previous[v] = predecessor of v on the shortest path
        distance, previous = dijkstra(self.adjacency, src)

        reachable = sum(1 for d in distance.values() if d != float("inf"))
        self.logger.info("[SPF-DONE] s%d->s%d reachable=%d/%d",
                         src, dst, reachable, len(distance))

        # --- Phase 2: Reconstruct path from predecessor pointers ---
        return self._reconstruct_path(src, dst, first_port, final_port, distance, previous)


if __name__ == '__main__':
    current_file = os.path.abspath(__file__)
    passthrough_args = sys.argv[1:]
    if '--observe-links' not in passthrough_args:
        passthrough_args = ['--observe-links'] + passthrough_args
    sys.argv = ['dijkstra_osken_controller', *passthrough_args, current_file]
    from os_ken.cmd.manager import main
    import sys as _sys
    _sys.exit(main())
