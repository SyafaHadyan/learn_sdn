#!/usr/bin/env python3
"""Mesh topology for multipath and K-shortest path labs.

Topology design: 6 switches in a partial mesh with 3 disjoint paths between
corner nodes.  This creates ECMP scenarios where Dijkstra multipath,
A* multipath, and K-shortest path controllers can demonstrate load balancing.

ASCII art (switch layout):

          h1    h2
           |    |
          (1)  (2)
           s1
          / | \
        (3)(4)(5)
        /   |   \
    (1)s2  (1)s3  (1)s4
      \      |      /
      (2)   (2)  (2)
        \    |   /
         s5--s6
        (1)(1)(2)
             |
             h3
             
Simplified 4-switch diamond + extras:

           h1--(1)s1(2)--+--------(1)s2(2)--h4
                  |                   |
                 (3)                 (3)
                  |                   |
              (1)s3(2)------------(1)s4(1)--h2
                  |
                 (3)
                  |
                  h3

Use this topology for:
  - Dijkstra multipath: shows 2 equal-cost paths s1->s4
  - K-shortest paths:   shows K>2 diverse paths through mesh
  - ECMP:               SELECT groups on s1 forward to both s2 and s3
"""

from functools import partial

from mininet.topo import Topo
from mininet.net import Mininet
from mininet.node import RemoteController
from mininet.util import dumpNodeConnections
from mininet.log import setLogLevel, info
from mininet.cli import CLI


class MeshTopo(Topo):
    """Diamond + lateral mesh for multipath/ECMP demonstrations.

    Topology:

        h1         h2
        |           |
       (1)s1(3)--(3)s4(2)--h3
       (2)           (1)
        |             |
       (2)s2(1)---(2)s3(1)
       (3)           (3)
        |             |
       (1)s5(2)---(1)s6(2)--h4

    Inter-switch links (equal weight for ECMP demo):
        s1 - s4  (via port 3 / port 3)   upper path
        s1 - s2  (via port 2 / port 2)   lower-left
        s4 - s3  (via port 1 / port 1)   lower-right
        s2 - s3  (via port 1 / port 2)   middle cross
        s2 - s5  (via port 3 / port 1)   lower-left
        s3 - s6  (via port 3 / port 1)   lower-right
        s5 - s6  (via port 2 / port 2)   bottom
    """

    def addSwitch(self, name, **opts):
        kwargs = {"protocols": "OpenFlow13"}
        kwargs.update(opts)
        return super(MeshTopo, self).addSwitch(name, **kwargs)

    def __init__(self):
        Topo.__init__(self)

        info("*** Adding hosts\n")
        h1 = self.addHost("h1", ip="10.0.0.1/24")
        h2 = self.addHost("h2", ip="10.0.0.2/24")
        h3 = self.addHost("h3", ip="10.0.0.3/24")
        h4 = self.addHost("h4", ip="10.0.0.4/24")

        info("*** Adding switches\n")
        s1 = self.addSwitch("s1")
        s2 = self.addSwitch("s2")
        s3 = self.addSwitch("s3")
        s4 = self.addSwitch("s4")
        s5 = self.addSwitch("s5")
        s6 = self.addSwitch("s6")

        info("*** Adding host links\n")
        self.addLink(s1, h1, port1=1, port2=1)
        self.addLink(s4, h2, port1=2, port2=1)
        self.addLink(s4, h3, port1=4, port2=1)
        self.addLink(s6, h4, port1=2, port2=1)

        info("*** Adding inter-switch links (mesh)\n")
        # Upper ring: s1 <-> s4
        self.addLink(s1, s4, port1=3, port2=3)
        # Left: s1 <-> s2
        self.addLink(s1, s2, port1=2, port2=2)
        # Right: s4 <-> s3
        self.addLink(s4, s3, port1=1, port2=1)
        # Middle cross: s2 <-> s3
        self.addLink(s2, s3, port1=1, port2=2)
        # Lower left: s2 <-> s5
        self.addLink(s2, s5, port1=3, port2=1)
        # Lower right: s3 <-> s6
        self.addLink(s3, s6, port1=3, port2=1)
        # Bottom: s5 <-> s6
        self.addLink(s5, s6, port1=2, port2=2)


def run():
    topo = MeshTopo()
    net = Mininet(
        topo=topo,
        controller=partial(RemoteController, ip="127.0.0.1", port=6633),
    )
    net.start()
    info("*** Dumping host connections\n")
    dumpNodeConnections(net.hosts)
    info("\n*** Network is running. Type 'pingall' or 'h1 ping h2'.\n")
    info("    For ECMP: use dijkstra_multipath or kshortest controller.\n")
    CLI(net)
    net.stop()


if __name__ == "__main__":
    setLogLevel("info")
    run()
