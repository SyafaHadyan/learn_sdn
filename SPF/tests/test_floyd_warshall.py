"""Tests for algorithms/floyd_warshall.py"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from algorithms.floyd_warshall import floyd_warshall
from algorithms.dijkstra import dijkstra


class TestFloydWarshallBasic:
    def test_linear3_all_pairs(self, linear3):
        dist, next_hop = floyd_warshall(linear3.adj, linear3.switches)
        assert dist[1][3] == 2
        assert dist[3][1] == 2
        assert dist[1][2] == 1

    def test_ring4_diameter(self, ring4):
        dist, _ = floyd_warshall(ring4.adj, ring4.switches)
        # Diameter = 2 (opposite nodes)
        assert dist[1][3] == 2
        assert dist[2][4] == 2

    def test_matches_dijkstra_for_all_pairs(self, diamond):
        dist_fw, _ = floyd_warshall(diamond.adj, diamond.switches)
        for src in diamond.switches:
            dist_dj, _ = dijkstra(diamond.adj, src)
            for dst in diamond.switches:
                assert abs(dist_fw[src][dst] - dist_dj.get(dst, float("inf"))) < 1e-9

    def test_disconnected_unreachable(self, disconnected):
        dist, _ = floyd_warshall(disconnected.adj, disconnected.switches)
        assert dist[1][3] == float("inf")
        assert dist[3][1] == float("inf")

    def test_next_hop_consistency(self, linear3):
        _, next_hop = floyd_warshall(linear3.adj, linear3.switches)
        # Next hop from 1 to 3 must be 2
        assert next_hop[1][3] == 2
        # Next hop from 3 to 1 must be 2
        assert next_hop[3][1] == 2
