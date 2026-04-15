# Widest Path — Maximum Bottleneck Bandwidth Routing

## Motivation

Traditional hop-count routing ignores link capacities.  Consider:

```
h1 --(10 Mbps)-- s1 --(10 Mbps)-- s2 --(10 Mbps)-- s3 -- h2
                   \                                /
                    \--------(100 Mbps)-----------/

Hop-count shortest:  h1 -> s1 -> s2 -> s3 -> h2   (2 hops via s2)
                     bottleneck = 10 Mbps

Widest path:         h1 -> s1 ------------> s3 -> h2   (1 hop shortcut)
                     bottleneck = 100 Mbps
```

If you are streaming 4K video or doing bulk data transfer, the widest path
gives **10x more bandwidth** even though it looks "worse" by hop count.

## Algorithm

Widest path is a **modified Dijkstra** where:
- "Distance" is replaced by **bottleneck bandwidth** (we maximise, not minimise)
- Relaxation: `bw[v] = max(bw[v], min(bw[u], edge_bandwidth(u,v)))`
- Max-heap (negate bandwidth to use Python's min-heap)

```
Standard Dijkstra:   dist[v] = min(dist[v], dist[u] + edge_cost(u,v))
Widest path:         bw[v]   = max(bw[v],   min(bw[u], edge_bw(u,v)))
```

The `min()` represents the **bottleneck**: the path's bandwidth is limited
by its weakest link.  We want to maximise this minimum.

## Visualisation

```
Topology (link bandwidths):
  s1 --(10)-- s2 --(10)-- s4
  s1 --(1)--- s3 --(1)--- s4
  s1 --(100)------------- s4   (direct link)

Widest path from s1:

  Init: bw = {s1:inf, s2:0, s3:0, s4:0}
  Heap: [(-inf, s1)]   (negated for max-heap simulation)

  Pop (-inf, s1): explore neighbours
    s2: max(0, min(inf, 10)) = 10   -> push (-10, s2)
    s3: max(0, min(inf, 1))  = 1    -> push (-1, s3)
    s4: max(0, min(inf, 100))= 100  -> push (-100, s4)
  bw = {s1:inf, s2:10, s3:1, s4:100}

  Pop (-100, s4): best bottleneck to s4 = 100 Mbps via direct link!
  (The direct s1->s4 link wins.)

  Result: widest path s1->s4 = 100 Mbps  (direct)
  Compare: via s2: min(10, 10) = 10 Mbps; via s3: min(1, 1) = 1 Mbps
```

## Link Weights Configuration

Widest path requires bandwidth values.  Create `link_weights.json` in the
SPF directory (it exists already in this lab):

```json
{
  "links": {
    "1:2": {"bandwidth_mbps": 1000},
    "1:3": {"bandwidth_mbps": 100},
    "2:4": {"bandwidth_mbps": 1000},
    "3:4": {"bandwidth_mbps": 100}
  }
}
```

Key format: `"min_dpid:max_dpid"` (always lower DPID first).
The controller loads this at startup.

## Lab: Comparing Routing Strategies

Use `topo-weighted_lab.py` which has the spine/edge topology:

```
Spine (1000 Mbps):  s1 -> s2 -> s4 -> s6   (3 hops)
Shortcut (50 Mbps): s1 -> s4              (1 hop)
h1 attached to s1, h5 attached to s6.

Hop-count routing (dijkstra): h1 -> s1 -> s4 -> s6 -> h5 (bottleneck 50 Mbps)
Widest-path routing:          h1 -> s1 -> s2 -> s4 -> s6 -> h5 (bottleneck 1000 Mbps)
```

```bash
# Test with Dijkstra (hop-count):
python3 SPF/topo-weighted_lab.py     # Terminal 1
python3 SPF/dijkstra_osken_controller.py  # Terminal 2
# Check flows: should see path via s4 (shortcut, fewer hops)

# Test with Widest Path:
python3 SPF/topo-weighted_lab.py     # Terminal 1
python3 SPF/widest_path_osken_controller.py  # Terminal 2
# Check flows: should see path via s2->s4 (spine, more bandwidth)
```

## Without a Weight File

If `link_weights.json` is absent, the controller falls back to treating all
link bandwidths as equal (unit weight), behaving like Dijkstra:

```
[WP-WEIGHTS] no weight file; widest-path degrades to unit-weight routing
```
