"""Tests for algorithms/dijkstra.py"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from algorithms.dijkstra import dijkstra, dijkstra_multi_parent


class TestDijkstraBasic:
    def test_linear3_distances(self, linear3):
        d, _ = dijkstra(linear3.adj, 1)
        assert d[1] == 0
        assert d[2] == 1
        assert d[3] == 2

    def test_linear3_predecessors(self, linear3):
        _, prev = dijkstra(linear3.adj, 1)
        assert prev[2] == 1
        assert prev[3] == 2

    def test_ring4_distances(self, ring4):
        d, _ = dijkstra(ring4.adj, 1)
        assert d[1] == 0
        assert d[2] == 1
        assert d[3] == 2
        assert d[4] == 1

    def test_disconnected_unreachable(self, disconnected):
        d, _ = dijkstra(disconnected.adj, 1)
        assert d.get(3, float("inf")) == float("inf")

    def test_symmetric_distances(self, diamond):
        d1, _ = dijkstra(diamond.adj, 1)
        d4, _ = dijkstra(diamond.adj, 4)
        assert d1[4] == d4[1]   # symmetric unweighted graph


class TestDijkstraWeights:
    def test_prefers_fewer_hops_with_equal_weights(self, linear3):
        w = {(1, 2): 1, (2, 1): 1, (2, 3): 1, (3, 2): 1}
        d, _ = dijkstra(linear3.adj, 1, weights=w)
        assert d[3] == 2

    def test_weighted_shortest_path_cost(self, weighted):
        d, _ = dijkstra(weighted.adj, 1, weights=weighted.weights)
        # 1->4 via s3: 1+1=2; via s2: 10+10=20; direct: 100 — cheapest is 2
        assert d[4] == 2

    def test_weighted_path_reconstruction(self, weighted):
        d, prev = dijkstra(weighted.adj, 1, weights=weighted.weights)
        # Reconstruct path 1->4
        path = []
        node = 4
        while node is not None:
            path.append(node)
            node = prev.get(node)
        path.reverse()
        assert path[0] == 1
        assert path[-1] == 4
        # Cheapest path goes via 3 (cost 1+1=2); not direct or via 2
        assert 3 in path


class TestDijkstraMultiParent:
    def test_diamond_has_two_parents(self, diamond):
        _, parents = dijkstra_multi_parent(diamond.adj, 1)
        # Node 4 is reachable from 2 and from 3 with equal cost
        assert parents[4] == {2, 3}

    def test_linear_single_parent(self, linear3):
        _, parents = dijkstra_multi_parent(linear3.adj, 1)
        assert parents[2] == {1}
        assert parents[3] == {2}

    def test_returns_sets(self, ring4):
        _, parents = dijkstra_multi_parent(ring4.adj, 1)
        for v, p_set in parents.items():
            assert isinstance(p_set, set)
