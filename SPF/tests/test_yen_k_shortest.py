"""Tests for algorithms/yen_k_shortest.py"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from algorithms.yen_k_shortest import yen_k_shortest


def path_nodes_equal(p, expected):
    return list(p) == list(expected)


class TestYenBasic:
    def test_linear_first_path(self, linear3):
        paths = yen_k_shortest(linear3.adj, 1, 3, k=1)
        assert len(paths) == 1
        assert paths[0] == [1, 2, 3]

    def test_only_one_path_in_linear(self, linear3):
        # Linear chain has only 1 simple path 1->3
        paths = yen_k_shortest(linear3.adj, 1, 3, k=3)
        assert len(paths) == 1

    def test_diamond_has_two_equal_paths(self, diamond):
        paths = yen_k_shortest(diamond.adj, 1, 4, k=2)
        assert len(paths) == 2
        path_switch_seqs = [tuple(p) for p in paths]
        assert (1, 2, 4) in path_switch_seqs or (1, 3, 4) in path_switch_seqs

    def test_diamond_no_duplicates(self, diamond):
        paths = yen_k_shortest(diamond.adj, 1, 4, k=2)
        path_tuples = [tuple(p) for p in paths]
        assert len(path_tuples) == len(set(path_tuples))

    def test_same_source_destination(self, linear3):
        # src == dst should return empty or single trivial path
        paths = yen_k_shortest(linear3.adj, 1, 1, k=2)
        assert paths == [] or paths == [[1]]

    def test_unreachable_returns_empty(self, disconnected):
        paths = yen_k_shortest(disconnected.adj, 1, 3, k=2)
        assert paths == []

    def test_paths_are_simple(self, diamond):
        # Each returned path must not repeat a node
        paths = yen_k_shortest(diamond.adj, 1, 4, k=3)
        for path in paths:
            assert len(path) == len(set(path)), f"path has repeated nodes: {path}"
