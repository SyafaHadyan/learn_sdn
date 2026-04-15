"""Tests for algorithms/bellman_ford.py"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from algorithms.bellman_ford import bellman_ford
from algorithms.dijkstra import dijkstra


class TestBellmanFordBasic:
    def test_linear3_matches_dijkstra(self, linear3):
        d_bf, _, nc = bellman_ford(linear3.adj, 1)
        d_dj, _ = dijkstra(linear3.adj, 1)
        assert not nc
        for v in d_dj:
            assert abs(d_bf.get(v, float("inf")) - d_dj[v]) < 1e-9

    def test_ring4_matches_dijkstra(self, ring4):
        d_bf, _, nc = bellman_ford(ring4.adj, 1)
        d_dj, _ = dijkstra(ring4.adj, 1)
        assert not nc
        for v in d_dj:
            assert abs(d_bf.get(v, float("inf")) - d_dj[v]) < 1e-9

    def test_disconnected_unreachable(self, disconnected):
        d, _, nc = bellman_ford(disconnected.adj, 1)
        assert not nc
        assert d.get(3, float("inf")) == float("inf")

    def test_no_negative_cycle_in_normal_graph(self, diamond):
        _, _, nc = bellman_ford(diamond.adj, 1)
        assert not nc


class TestBellmanFordNegativeWeights:
    def test_negative_weight_still_finds_correct_path(self):
        # DAG: 1->2 (cost 3), 2->3 (cost -2), no back edges -> no cycle
        # Shortest path 1->3: cost 1
        adj = {1: [(2, 12)], 2: [(3, 23)], 3: []}
        weights = {(1, 2): 3, (2, 3): -2}
        d, prev, nc = bellman_ford(adj, 1, weights=weights)
        assert not nc
        assert abs(d[3] - 1) < 1e-9

    def test_returns_has_negative_cycle_flag(self):
        # Negative cycle: 1->2->3->1 with total cost < 0
        adj = {1: [(2, 12)], 2: [(3, 23)], 3: [(1, 31)]}
        weights = {(1, 2): 1, (2, 3): 1, (3, 1): -10}
        _, _, nc = bellman_ford(adj, 1, weights=weights)
        assert nc
