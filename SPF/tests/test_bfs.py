"""Tests for algorithms/bfs.py"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from algorithms.bfs import bfs


class TestBFSBasic:
    def test_single_node_src_is_reachable(self, linear3):
        d, _ = bfs(linear3.adj, 1)
        assert d[1] == 0

    def test_linear3_distances(self, linear3):
        d, _ = bfs(linear3.adj, 1)
        assert d[1] == 0
        assert d[2] == 1
        assert d[3] == 2

    def test_linear3_predecessors(self, linear3):
        _, prev = bfs(linear3.adj, 1)
        assert prev[2] == 1
        assert prev[3] == 2
        assert prev[1] is None

    def test_ring4_shortest_path_distance(self, ring4):
        # s1 to s3: 2 hops (s1->s2->s3 or s1->s4->s3)
        d, _ = bfs(ring4.adj, 1)
        assert d[3] == 2

    def test_ring4_adjacent_hops(self, ring4):
        d, _ = bfs(ring4.adj, 1)
        assert d[2] == 1
        assert d[4] == 1

    def test_disconnected_unreachable(self, disconnected):
        d, _ = bfs(disconnected.adj, 1)
        assert d[3] == float("inf")
        assert d[4] == float("inf")

    def test_disconnected_reachable(self, disconnected):
        d, _ = bfs(disconnected.adj, 1)
        assert d[2] == 1

    def test_unknown_src(self):
        adj = {1: [(2, 12)], 2: [(1, 21)]}
        d, prev = bfs(adj, 99)
        # 99 not in adj; should return without crash
        assert isinstance(d, dict)

    def test_weights_ignored(self, weighted):
        # BFS ignores the weights parameter
        d_bfs, _ = bfs(weighted.adj, 1)
        # All neighbors at 1 hop regardless of weight
        assert d_bfs[2] == 1
        assert d_bfs[3] == 1
        assert d_bfs[4] == 1
