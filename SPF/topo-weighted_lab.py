#!/usr/bin/env python3
"""Weighted topology for bandwidth-aware routing labs.

Topology design: 6 switches, 6 hosts, asymmetric link bandwidths.
The "high-bandwidth spine" path (s1->s2->s4->s6) has 1000 Mbps links.
The "low-bandwidth edge" paths have 100 Mbps.  Widest-path routing should
prefer the high-bandwidth spine; hop-count routing may choose a shorter path.

ASCII art:

    h1          h2
    |           |
  (1)s1(2)---(2)s2(3)--h3
      |   \       \
     (3)  (4)     (4)
      |     \       \
    (1)s3  (1)s4(2)--h4
      |         \
     (2)         (3)
      |           \
    (1)s5(2)---(2)s6(1)--h5
                    \
                    (3)--h6

Link bandwidths (defined in link_weights.json):
    s1-s2: 1000 Mbps (spine)    s1-s3: 100 Mbps
    s2-s4: 1000 Mbps (spine)    s3-s4: 100 Mbps
    s4-s6: 1000 Mbps (spine)    s2-s5: 100 Mbps
    s5-s6:  100 Mbps            s1-s4:  50  Mbps (bottleneck shortcut)
    s3-s5:  200 Mbps

Route analysis (h1 -> h5, widest-path vs hop-count):
    Hop min: s1->s4->s6 (2 hops, spine - bottleneck 50 Mbps)
    Widest:  s1->s2->s4->s6 (3 hops, spine - bottleneck 1000 Mbps)

Use with:
    python3 SPF/widest_path_osken_controller.py      (maximises bottleneck BW)
    python3 SPF/dijkstra_osken_controller.py          (minimises hops)
    python3 SPF/bellman_ford_osken_controller.py      (alternative with weights)
"""

from mininet.topo import Topo
from mininet.net import Mininet
from mininet.node import RemoteController
from mininet.util import dumpNodeConnections
from mininet.log import setLogLevel, info
from mininet.cli import CLI


class WeightedTopo(Topo):
    """6-switch, 6-host topology with asymmetric link bandwidths."""

    def addSwitch(self, name, **opts):
        kwargs = {"protocols": "OpenFlow13"}
        kwargs.update(opts)
        return super(WeightedTopo, self).addSwitch(name, **kwargs)

    def __init__(self):
        Topo.__init__(self)

        info("*** Adding hosts\n")
        h1 = self.addHost("h1", ip="10.0.0.1/24")
        h2 = self.addHost("h2", ip="10.0.0.2/24")
        h3 = self.addHost("h3", ip="10.0.0.3/24")
        h4 = self.addHost("h4", ip="10.0.0.4/24")
        h5 = self.addHost("h5", ip="10.0.0.5/24")
        h6 = self.addHost("h6", ip="10.0.0.6/24")

        info("*** Adding switches\n")
        s1 = self.addSwitch("s1")
        s2 = self.addSwitch("s2")
        s3 = self.addSwitch("s3")
        s4 = self.addSwitch("s4")
        s5 = self.addSwitch("s5")
        s6 = self.addSwitch("s6")

        info("*** Adding host links\n")
        self.addLink(s1, h1, port1=1, port2=1)
        self.addLink(s2, h2, port1=1, port2=1)
        self.addLink(s2, h3, port1=3, port2=1)
        self.addLink(s4, h4, port1=2, port2=1)
        self.addLink(s6, h5, port1=1, port2=1)
        self.addLink(s6, h6, port1=3, port2=1)

        # Spine (high-bandwidth 1000 Mbps)
        info("*** Adding inter-switch links\n")
        self.addLink(s1, s2, port1=2, port2=2)         # 1000 Mbps
        self.addLink(s2, s4, port1=4, port2=1)         # 1000 Mbps
        self.addLink(s4, s6, port1=3, port2=2)         # 1000 Mbps

        # Edges (lower bandwidth)
        self.addLink(s1, s3, port1=3, port2=1)         # 100 Mbps
        self.addLink(s3, s4, port1=2, port2=4)         # 100 Mbps
        self.addLink(s2, s5, port1=5, port2=1)         # 100 Mbps
        self.addLink(s5, s6, port1=2, port2=4)         # 100 Mbps

        # Bottleneck shortcut (low BW, few hops)
        self.addLink(s1, s4, port1=4, port2=5)         # 50 Mbps


def run():
    topo = WeightedTopo()
    net = Mininet(
        topo=topo,
        controller=partial(RemoteController, ip="127.0.0.1", port=6633),
    )
    net.start()
    info("*** Dumping host connections\n")
    dumpNodeConnections(net.hosts)
    info("\n*** Network is running. Use Mininet CLI to test.\n")
    info("    Suggested tests:\n")
    info("      h1 ping h5            # observe path selection\n")
    info("      pingall               # check full connectivity\n")
    info("      dpctl dump-flows -O OpenFlow13\n")
    CLI(net)
    net.stop()


if __name__ == "__main__":
    from functools import partial
    setLogLevel("info")
    run()
