# ECMP (Equal-Cost Multi-Path) Lab

## Overview

This lab demonstrates **ECMP (Equal-Cost Multi-Path)** routing using **Yen's K-shortest paths algorithm**. ECMP enables load balancing across multiple paths with equal cost, improving network utilization and providing path diversity.

## Learning Objectives

- Understand ECMP principles and benefits
- Implement Yen's K-shortest paths algorithm
- Learn consistent hashing for flow-based load balancing
- Measure load distribution across multiple paths
- Compare single-path vs multi-path forwarding

## Topology

The ECMP topology uses a **diamond topology** to create **2 truly equal-cost paths**:

```
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
```

**Equal-cost paths from h1 (at s1, port 1) to h4 (at s4, port 2):**
- **Path 1**: h1 → s1 (port 3) → s2 (port 2) → s4 (port 3) → h4 (3 hops) ✅
- **Path 2**: h1 → s1 (port 4) → s3 (port 2) → s4 (port 4) → h4 (3 hops) ✅

**Both paths have identical cost (3 hops = 1 hop-count each)** for true ECMP load balancing!

## Files

- `topo-ecmp_lab.py`: ECMP-capable Mininet topology
- `kshortest_osken_controller.py`: K-shortest paths controller with ECMP

## Running the Lab

### Terminal 1: Start Topology

```bash
cd /workspaces/learn_sdn/SPF
python3 topo-ecmp_lab.py
```

### Terminal 2: Start Controller

```bash
cd /workspaces/learn_sdn/SPF
python3 kshortest_osken_controller.py
```

**Optional: Verbose logging**
```bash
python3 kshortest_osken_controller.py --verbose
```

## Verification Steps

### 1. Check Installed Flows and Groups

In the Mininet CLI, check the flow tables:

```bash
# Dump flows for all switches (recommended in Mininet CLI)
dpctl dump-flows -O OpenFlow13 | grep -i cookie

# Dump one specific switch (also valid in Mininet CLI)
sh ovs-ofctl -O OpenFlow13 dump-flows s1 | grep -i cookie

# Look for actions=group:<id> on ingress rules (for ECMP)
# and output rules on downstream switches.
```

Check group table entries:

```bash
# Check ECMP group definitions across all switches
dpctl dump-groups -O OpenFlow13

# Or per-switch
sh ovs-ofctl -O OpenFlow13 dump-groups s1
sh ovs-ofctl -O OpenFlow13 dump-groups s4
```

### 2. Test Basic Connectivity

Generate baseline traffic between h1 and h4:

```bash
# In Mininet CLI
h1 ping h4
```

Notes:
- A single ping flow is usually pinned to one ECMP bucket.
- This is expected for flow-based ECMP (not packet-by-packet balancing).

### 3. Measure Bucket Utilization (Recommended)

Generate parallel traffic so multiple flows are hashed across buckets:

```bash
# In Mininet CLI
h4 iperf -s -D
h1 iperf -c 10.2.0.4 -P 8 -t 8
```

Inspect per-bucket counters:

```bash
# Bucket-level packet/byte counters (best ECMP evidence)
dpctl dump-group-stats -O OpenFlow13

# Or per-switch
sh ovs-ofctl -O OpenFlow13 dump-group-stats s1
sh ovs-ofctl -O OpenFlow13 dump-group-stats s4
```

Expected result:
- Both bucket counters increase (not necessarily 50:50).
- Uneven split is normal because hash distribution depends on active flows.

### 4. Correlate with Link Counters

Use interface counters as secondary confirmation:

```bash
# In another shell (outside Mininet)
bwm-ng -t 1000 -d
```

Interpretation tips:
- Single flow can make one ECMP link look dominant.
- For ECMP verification, trust group bucket counters first, bwm-ng second.

### 5. Verify Path Diversity

Check that multiple paths are installed:

```bash
# Path branches on s1/s4 should show both inter-switch links being used
sh ovs-ofctl -O OpenFlow13 dump-flows s1
sh ovs-ofctl -O OpenFlow13 dump-flows s2
sh ovs-ofctl -O OpenFlow13 dump-flows s3
sh ovs-ofctl -O OpenFlow13 dump-flows s4
```

## Algorithm Details

### Yen's K-Shortest Paths Algorithm

**Complexity**: O(K·V·(E + V log V))

**Steps:**
1. Find first shortest path using Dijkstra
2. For each subsequent path:
   - Generate spur paths by deviating from previous paths
   - Remove edges/nodes from root paths to avoid duplicates
   - Find shortest spur path
   - Combine root + spur path
   - Add to candidate set
3. Select next shortest path from candidates
4. Repeat until K paths found

### ECMP Load Balancing

**Consistent Hashing**:
- Flow hash computed from: `src_mac:dst_mac:src_port:dst_port:eth_type`
- Hash value: `CRC32(flow_tuple) % HASH_ROUNDS`
- Path selection: `flow_hash % num_paths`

**Benefits:**
- Same flow always uses same path (flow consistency)
- Different flows distributed across paths (load balancing)
- Minimal remapping when paths change

## Troubleshooting

### Issue: No flows installed

**Check:**
1. Controller is running: `ps aux | grep kshortest`
2. Switches connected: `dpctl dump-flows -O OpenFlow13`
3. Logs: Check controller output for errors

### Issue: Only one path installed

**Check:**
1. Topology has equal-cost paths: Verify `topo-ecmp_lab.py`
2. K value: Check `K_PATHS = 2` in controller
3. Logs: Look for `[ECMP-INSTALL]` messages

### Issue: Load not balanced

**Check:**
1. Generate enough flows: use `iperf -P 8` or higher
2. Verify group buckets: `dpctl dump-groups -O OpenFlow13`
3. Verify bucket counters: `dpctl dump-group-stats -O OpenFlow13`
4. Single-flow ping or single TCP stream can legitimately stay on one path

## Advanced Exercises

### Exercise 1: Modify K Value

Change `K_PATHS` in `kshortest_osken_controller.py`:

```python
K_PATHS = 3  # Try 3 paths
```

Observe how load distribution changes.

### Exercise 2: Add Link Failure

Simulate link failure and observe recovery:

```bash
# In Mininet CLI
link s1 s2 down
# Observe traffic rerouting to remaining path
link s1 s2 up
# Observe path restoration
```

### Exercise 3: Measure Performance

Measure path computation time:

```python
# Add timing to compute_k_shortest_paths()
import time
start = time.time()
k_paths = self.yen_k_shortest_paths(src, dst, K)
elapsed = time.time() - start
self.logger.info("[KSP-TIME] computed %d paths in %.3f seconds", len(k_paths), elapsed)
```

## Comparison with Dijkstra

| Metric | Dijkstra (Single Path) | K-Shortest (ECMP) |
|--------|----------------------|-------------------|
| Paths computed | 1 | K (typically 2-5) |
| Load balancing | ❌ No | ✅ Yes |
| Path diversity | ❌ No | ✅ Yes |
| Computation time | Fast | K× slower |
| Flow consistency | ✅ Yes | ✅ Yes |
| Fault tolerance | ❌ Single point of failure | ✅ Multiple paths |

## References

- Yen, J. Y. (1971). "Finding the K Shortest Loopless Paths in a Network"
- RFC 5683: "Multipath TCP Considerations"
- OpenFlow ECMP implementations in production SDN controllers

## Next Steps

After completing this lab, you can explore:
- **A* Pathfinding**: Faster path computation with heuristics
- **Bellman-Ford**: Distributed distance-vector routing
- **Constraint-Based Routing**: QoS-aware path selection
