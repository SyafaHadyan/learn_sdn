# SDN Concepts — Background

## 1. The Problem with Traditional Networking

In a traditional IP network, every router independently runs a distributed
routing protocol:

```
            OSPF flooding
    [R1] <-----------> [R2] <-----------> [R3]
     |                   |                  |
    ospf                ospf              ospf
    local               local             local
```

Each router:
- Floods link-state advertisements (LSAs) to all other routers
- Builds its own complete view of the topology (LSPF database)
- Runs Dijkstra locally to compute its own forwarding table

**Problems:**
- Configuration complexity: each device configured independently
- Innovation barrier: can't add new forwarding behaviour without firmware update
- Limited visibility: no single point knows what the whole network is doing

## 2. Software-Defined Networking (SDN) Architecture

SDN separates the **control plane** (routing decisions) from the **data plane**
(packet forwarding):

```
+-----------------------------------------------+
|            SDN CONTROLLER                      |  <-- control plane
|  (Python, OSKen, knows global topology)        |     runs on server
+----+-------+-------+-------+--------+----------+
     |       |       |       |        |
  OpenFlow  OpenFlow OpenFlow  ...   OpenFlow
     |       |       |       |        |
  +----+  +----+  +----+             +----+
  | S1 |  | S2 |  | S3 |   ...      | Sn |  <-- data plane
  +----+  +----+  +----+             +----+
    dumb switches, forward by flow table rules only
```

**Key insight:** The controller has a **global view**. It runs any algorithm
it wants — Dijkstra, Floyd-Warshall, A* — on a central server with full
topology knowledge.

## 3. OpenFlow: The Southbound API

The controller communicates with switches via **OpenFlow**.  The switch has a
**flow table**: a list of (match, action) rules.

```
Flow Table Entry:
  Match:  in_port=1, eth_src=aa:bb:cc:dd:ee:ff, eth_dst=11:22:33:44:55:66
  Action: output(3)

  Translation: "If a packet arrives on port 1, from MAC aa:bb... to MAC 11:22...,
               forward it out port 3."
```

When a packet arrives at a switch:
1. Switch checks the flow table for a matching rule
2. If match found → execute action (forward/drop/modify)
3. If **no match** → send packet to controller (`packet_in` event)

The controller receives `packet_in`, computes the path, installs flow rules
across all switches on the path, then sends the packet on its way.

## 4. Topology Discovery with LLDP

The controller uses **Link Layer Discovery Protocol (LLDP)** to learn which
switches are connected to which:

```
Controller
   |
   +--> "S1, send LLDP packet out all ports"
   +--> "S2, send LLDP packet out all ports"
   ...
   
   S1--(port 3)----(port 2)--S2
   
   S2 receives LLDP from S1 on port 2 -> reports to controller:
   "Link exists: S1:3 <-> S2:2"
```

OSKen's `--observe-links` flag enables this automatically.  The controller
maintains the adjacency graph and updates it when links go up/down.

## 5. How This Lab Works

```
Mininet (network emulator)        OSKen Controller (this lab)
         |                                    |
  virtual switches (OVS) <---OpenFlow TCP---> controller
  virtual hosts (network namespaces)
         |
  connected via virtual ethernet pairs (veth)
```

When you run `python3 topo-spf_lab.py`, Mininet creates:
- 3 virtual switches (Open vSwitch)
- 6 virtual hosts
- Virtual ethernet links between them

When `python3 dijkstra_osken_controller.py` starts:
- It connects to all switches via OpenFlow
- It learns the topology via LLDP
- It responds to `packet_in` events by computing paths and installing flows

## 6. Controller Class Hierarchy

All controllers in this lab share a common base class:

```
SPFBaseController (base_controller.py)
   Handles: topology, host learning, flows, flooding, lifecycle
   
   Must override: compute_path(src, dst, first_port, final_port)
   |
   +-- BFSSwitch
   +-- DijkstraSwitch
   |   +-- DijkstraMultipathSwitch
   +-- AStarSwitch
   |   +-- AStarMultipathSwitch
   +-- BellmanFordSwitch
   +-- FloydWarshallSwitch
   +-- WidestPathSwitch
   +-- KShortestPathsController
```

Each subclass overrides **only** `compute_path()` to plug in its algorithm.
See [02-openflow-primer.md](02-openflow-primer.md) for the OpenFlow details.
