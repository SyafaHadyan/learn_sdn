"""Tests for algorithms/astar.py"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from algorithms.astar import astar, build_reverse_hop_heuristic
from algorithms.dijkstra import dijkstra


class TestHeuristic:
    def test_heuristic_zero_at_dst(self, linear3):
        h = build_reverse_hop_heuristic(linear3.adj, 3)
        assert h[3] == 0

    def test_heuristic_correct_distances(self, linear3):
        h = build_reverse_hop_heuristic(linear3.adj, 3)
        assert h[2] == 1
        assert h[1] == 2

    def test_heuristic_ring4(self, ring4):
        h = build_reverse_hop_heuristic(ring4.adj, 3)
        # s1 to s3: 2 hops
        assert h[1] == 2

    def test_heuristic_is_admissible(self, diamond):
        # Admissible: h(v) <= actual shortest path cost from v to dst
        dst = 4
        h = build_reverse_hop_heuristic(diamond.adj, dst)
        d, _ = dijkstra(diamond.adj, dst)   # use actual costs as reference
        for v in diamond.adj:
            assert h.get(v, 0) <= d.get(v, float("inf")) + 0.01


class TestAStar:
    def test_linear3_same_as_dijkstra(self, linear3):
        h = build_reverse_hop_heuristic(linear3.adj, 3)
        da, _ = astar(linear3.adj, 1, 3, h)
        dd, _ = dijkstra(linear3.adj, 1)
        # A* distance to dst must match Dijkstra
        assert da[3] == dd[3]

    def test_ring4_same_as_dijkstra(self, ring4):
        h = build_reverse_hop_heuristic(ring4.adj, 3)
        da, _ = astar(ring4.adj, 1, 3, h)
        dd, _ = dijkstra(ring4.adj, 1)
        assert da[3] == dd[3]

    def test_disconnected_unreachable(self, disconnected):
        h = build_reverse_hop_heuristic(disconnected.adj, 3)
        d, prev = astar(disconnected.adj, 1, 3, h)
        assert d.get(3, float("inf")) == float("inf")
