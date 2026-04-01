#!/usr/bin/env python

"""ECMP Topology - 4 switches with 2 equal-cost paths for load balancing demonstration.

This topology creates symmetric equal-cost paths between hosts using a diamond topology,
enabling ECMP (Equal-Cost Multi-Path) routing demonstration.

Diamond Topology Structure:
                h1        h2
                 |        |
                 +-- s1 --+
                    /  \
                   /    \
                 s2      s3
                   \    /
                    \  /
                 +-- s4 --+
                 |        |
                h3        h4

Paths from h1 (at s1) to h4 (at s4):
- Path 1: h1 → s1 → s2 → s4 → h4 (3 hops) ✅
- Path 2: h1 → s1 → s3 → s4 → h4 (3 hops) ✅
Both paths have equal cost → ECMP load-balances!
"""

from mininet.topo import Topo
from mininet.net import Mininet
from mininet.node import RemoteController
from mininet.util import dumpNodeConnections
from mininet.log import setLogLevel, info
from mininet.cli import CLI


class ECMP_Topology(Topo):
    """ECMP-capable topology with symmetric equal-cost paths (diamond topology).
    
    Creates 2 equal-cost paths between h1 and h4:
    - Path 1: h1 → s1 → s2 → s4 → h4 (3 hops)
    - Path 2: h1 → s1 → s3 → s4 → h4 (3 hops)
    Both paths have identical cost for true ECMP.
    """

    def __init__(self):
        # Initialize Topology
        Topo.__init__(self)

        # Add hosts
        info('*** Add Hosts\n')
        h1 = self.addHost('h1', ip='10.1.0.1/8')
        h2 = self.addHost('h2', ip='10.1.0.2/8')
        h3 = self.addHost('h3', ip='10.2.0.3/8')
        h4 = self.addHost('h4', ip='10.2.0.4/8')

        # Add switches (diamond topology)
        info('*** Add Switches\n')
        s1 = self.addSwitch('s1')  # Source switch
        s2 = self.addSwitch('s2')  # Top middle switch
        s3 = self.addSwitch('s3')  # Bottom middle switch
        s4 = self.addSwitch('s4')  # Destination switch

        # Add links: host to switch
        info('*** Add Host Links\n')
        self.addLink(s1, h1, port1=1, port2=1)
        self.addLink(s1, h2, port1=2, port2=1)
        self.addLink(s4, h3, port1=1, port2=1)
        self.addLink(s4, h4, port1=2, port2=1)

        # Add links: switch to switch (diamond pattern with equal costs)
        info('*** Add Switch Links\n')
        # Create 2 equal-cost paths from s1 to s4 (both 3 hops)
        self.addLink(s1, s2, port1=3, port2=1)  # Path 1 top edge
        self.addLink(s1, s3, port1=4, port2=1)  # Path 2 bottom edge
        self.addLink(s2, s4, port1=2, port2=3)  # Path 1 convergence
        self.addLink(s3, s4, port1=2, port2=4)  # Path 2 convergence


def run():
    """Run the ECMP topology with Mininet."""
    topo = ECMP_Topology()
    net = Mininet(
        topo=topo,
        controller=RemoteController,
        autoSetMacs=True,
        autoStaticArp=True,
        waitConnected=True
    )

    info('\n*** Disabling IPv6\n')
    for host in net.hosts:
        info(f'disable ipv6 in {host}\n')
        host.cmd('sysctl -w net.ipv6.conf.all.disable_ipv6=1')

    for sw in net.switches:
        info(f'disable ipv6 in {sw}\n')
        sw.cmd('sysctl -w net.ipv6.conf.all.disable_ipv6=1')

    info('\n\n*** *********************\n')
    info('Starting ECMP topology...\n')
    net.start()

    # Print topology info
    info('\n*** Topology Information\n')
    info('Switches: {}\n'.format([sw.name for sw in net.switches]))
    info('Hosts: {}\n'.format([h.name for h in net.hosts]))
    info('Links: {}\n'.format(net.links))

    # Dump connections
    info('*** Node Connections\n')
    dumpNodeConnections(net.hosts)

    info('\n*** Starting CLI\n')
    CLI(net)
    net.stop()


if __name__ == '__main__':
    setLogLevel('info')
    run()
