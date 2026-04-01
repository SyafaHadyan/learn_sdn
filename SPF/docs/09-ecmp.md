# ECMP — Equal-Cost Multipath Forwarding

## What is ECMP?

Equal-Cost Multi-Path (ECMP) routing distributes traffic across **multiple
equal-cost paths** between the same source and destination.

```
Without ECMP:    h1 --> s1 --> s2 --> s4 --> h2
                            (only one path used)

With ECMP:       h1 --> s1 --> s2 --> s4 --> h2
                           \-> s3 ->/
                            (both paths used, load balanced)
```

Benefits:
- Higher aggregate throughput (both links utilised)
- Path redundancy (if s2 fails, traffic continues via s3)

## OpenFlow SELECT Group

In OpenFlow 1.3, ECMP is implemented using **group tables** with type `SELECT`:

```
Group Table Entry:
  Group ID: 7
  Type: SELECT
  Buckets:
    - weight=1, action: output(port 3)   -> next-hop via s2
    - weight=1, action: output(port 4)   -> next-hop via s3
```

The switch hardware selects one bucket per flow using a hash function
(typically on 5-tuple: src IP, dst IP, protocol, src port, dst port).
This ensures all packets in the same flow take the same path (avoiding
out-of-order delivery) while distributing different flows across paths.

## How the Controller Installs ECMP

For path `h1 -> h4` through diamond topology:

```
Step 1: Compute equal-cost paths
        - dijkstra_multi_parent returns: parents[s4] = {s2, s3}
        - enumerate paths: [s1,s2,s4] and [s1,s3,s4]

Step 2: Install unicast flows on transit/egress switches
        s2: match(src=h1, dst=h4) -> output(to s4 port)
        s3: match(src=h1, dst=h4) -> output(to s4 port)
        s4: match(src=h1, dst=h4) -> output(to h4 port)

Step 3: Install SELECT group on ingress switch
        s1: SELECT group, buckets=[output(to s2 port), output(to s3 port)]

Step 4: Install group-action flow on ingress
        s1: match(src=h1, dst=h4) -> group(7)
```

ASCII flow diagram:

```
h1 --port1/--> s1 ---group(7)---> [s2 or s3] ---> s4 ---> h4
                |                     ^
                |   SELECT group 7    |
                +---(50% to s2, 50% to s3)---+
```

## Controller Implementation

The multipath controllers (`dijkstra_multipath_osken_controller.py`,
`astar_multipath_osken_controller.py`) follow this pattern:

```python
class DijkstraMultipathSwitch(DijkstraSwitch):

    def install_multipath(self, paths, src_mac, dst_mac):
        ingress_switch = paths[0][0][0]      # first switch of first path
        ingress_in_port = paths[0][0][1]
        
        # Collect all possible first-hop out-ports:
        out_ports = [path[0][2] for path in paths]
        
        # Install unicast flows on non-ingress hops:
        for path in paths:
            for sw, in_p, out_p in path[1:]:
                self._install_unicast_flow(dp, in_p, out_p, src_mac, dst_mac)
        
        # Install SELECT group on ingress:
        group_id = self._alloc_group_id(src_mac, dst_mac)
        self._install_select_group(ingress_dp, group_id, out_ports)
        self._install_group_flow(ingress_dp, ingress_in_port, 
                                  src_mac, dst_mac, group_id)
```

## K-Shortest Paths vs Equal-Cost Multipath

| Approach              | Controller                          | Paths Found           |
|-----------------------|-------------------------------------|-----------------------|
| Equal-cost only       | dijkstra_multipath / astar_multipath| Only paths with same hop count |
| K distinct paths      | kshortest_osken_controller          | K paths, may differ in length  |

```
Diamond topology example:

Equal-cost:   [s1,s2,s4]  and  [s1,s3,s4]  -- both cost 2 hops

K=3 paths:    [s1,s2,s4]     cost 2
              [s1,s3,s4]     cost 2
              [s1,s2,s3,s4]  cost 3   <- longer path, more diversity
```

Yen's algorithm (K-shortest) finds path **diversity**, not just equal cost.
This is useful when you need backup paths or want traffic engineering control.

## Lab: Observing ECMP in Action

```bash
# Start mesh topology (more disjoint paths than ring)
python3 SPF/topo-mesh_lab.py

# Start multipath controller
python3 SPF/dijkstra_multipath_osken_controller.py

# In Mininet CLI:
mininet> pingall
mininet> dpctl dump-flows -O OpenFlow13    # see unicast flows
mininet> dpctl dump-groups -O OpenFlow13  # see SELECT groups
```

Look for group entries with multiple buckets — that is ECMP.
