"""Tests for algorithms/widest_path.py"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from algorithms.widest_path import widest_path


class TestWidestPathBasic:
    def test_direct_link_is_widest(self, weighted):
        # Direct link 1->4 is 100 Mbps; via 1->2->4 is min(10,10)=10 Mbps
        max_bw, prev = widest_path(weighted.adj, 1, weighted.weights)
        assert max_bw[4] == 100.0

    def test_chooses_high_bw_path(self, weighted):
        max_bw, prev = widest_path(weighted.adj, 1, weighted.weights)
        # Predecessor of 4 should be 1 (direct 100 Mbps) not 2 (10 Mbps path)
        assert prev[4] == 1

    def test_source_has_infinite_bw(self, weighted):
        max_bw, _ = widest_path(weighted.adj, 1, weighted.weights)
        assert max_bw[1] == float("inf")

    def test_unreachable_has_zero_bw(self, disconnected):
        w = {(1, 2): 10, (2, 1): 10, (3, 4): 10, (4, 3): 10}
        max_bw, _ = widest_path(disconnected.adj, 1, w)
        assert max_bw.get(3, 0) == 0
        assert max_bw.get(4, 0) == 0

    def test_linear_bottleneck(self, linear3):
        # Chain: 1--(5 Mbps)--2--(2 Mbps)--3  bottleneck 1->3 = 2 Mbps
        w = {(1, 2): 5, (2, 1): 5, (2, 3): 2, (3, 2): 2}
        max_bw, _ = widest_path(linear3.adj, 1, w)
        assert max_bw[3] == 2.0
