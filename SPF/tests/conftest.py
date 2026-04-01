"""Pytest configuration and shared test fixtures for SPF algorithm tests.

Fixtures:
    linear3   - s1-s2-s3 linear chain (3 nodes, 2 edges)
    ring4     - s1-s2-s3-s4-s1 ring (4 nodes, 4 edges)
    diamond   - s1->{s2,s3}->s4 diamond (4 nodes, 4 edges, ECMP scenario)
    weighted  - 4-node graph with unequal link bandwidths
    disconnected - two separate components

Adjacency format (same as controllers):
    {node: [(neighbor, out_port), ...]}

All fixtures return named tuples with:
    .adj      - adjacency dict
    .switches - list of all DPIDs
    .weights  - (optional) per-link bandwidth dict {(u,v): bw}
"""

import pytest
from collections import namedtuple

Graph = namedtuple("Graph", ["adj", "switches", "weights"])


@pytest.fixture
def linear3():
    """Simple chain: s1 -- s2 -- s3."""
    adj = {
        1: [(2, 11)],
        2: [(1, 21), (3, 22)],
        3: [(2, 32)],
    }
    return Graph(adj=adj, switches=[1, 2, 3], weights={})


@pytest.fixture
def ring4():
    """4-node ring: s1-s2-s3-s4-s1.

    ASCII:  s1 -- s2
            |      |
            s4 -- s3
    """
    adj = {
        1: [(2, 11), (4, 14)],
        2: [(1, 21), (3, 23)],
        3: [(2, 32), (4, 34)],
        4: [(3, 43), (1, 41)],
    }
    return Graph(adj=adj, switches=[1, 2, 3, 4], weights={})


@pytest.fixture
def diamond():
    """Diamond ECMP graph: two equal-cost paths s1->s4.

    ASCII:  s1 -- s2 -- s4
             \-- s3 --/
    All edges weight 1 -> two shortest paths of cost 2.
    """
    adj = {
        1: [(2, 12), (3, 13)],
        2: [(1, 21), (4, 24)],
        3: [(1, 31), (4, 34)],
        4: [(2, 42), (3, 43)],
    }
    return Graph(adj=adj, switches=[1, 2, 3, 4], weights={})


@pytest.fixture
def weighted():
    """4-node graph with different link bandwidths.

    ASCII:  s1 --(10)-- s2 --(10)-- s4
             \--(100 Mbps)----------/
            s1 --(1 Mbps)-- s3 --(1 Mbps)-- s4

    Widest path s1->s4: direct s1->s4 (100 Mbps)
    Shortest path (hops): s1->s2->s4 or s1->s4 (both 1 hop from direct)
    """
    adj = {
        1: [(2, 12), (3, 13), (4, 14)],
        2: [(1, 21), (4, 24)],
        3: [(1, 31), (4, 34)],
        4: [(1, 41), (2, 42), (3, 43)],
    }
    weights = {
        (1, 2): 10, (2, 1): 10,
        (1, 3): 1,  (3, 1): 1,
        (1, 4): 100, (4, 1): 100,
        (2, 4): 10, (4, 2): 10,
        (3, 4): 1,  (4, 3): 1,
    }
    return Graph(adj=adj, switches=[1, 2, 3, 4], weights=weights)


@pytest.fixture
def disconnected():
    """Two separate components: {1,2} and {3,4}."""
    adj = {
        1: [(2, 12)],
        2: [(1, 21)],
        3: [(4, 34)],
        4: [(3, 43)],
    }
    return Graph(adj=adj, switches=[1, 2, 3, 4], weights={})
