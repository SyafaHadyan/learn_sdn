# 02 — OpenFlow Primer

This guide explains the OpenFlow 1.3 messages that the controllers use.
You do not need to understand every field — focus on the highlighted patterns.

---

## What is OpenFlow?

OpenFlow is a protocol that lets a remote controller program the **forwarding table**
inside each switch.  Without OpenFlow the switch decides locally; with OpenFlow the
controller decides centrally and pushes rules down.

```
  ┌──────────────────────┐
  │     Controller       │  (Python OSKen application)
  │   (your algorithm)   │
  └──────────┬───────────┘
             │  OpenFlow 1.3 (TCP port 6653)
   ┌─────────┴──────────┐
   │    Virtual Switch  │  (Open vSwitch in Mininet)
   │  [ flow table ]    │
   └────────────────────┘
```

---

## Flow Table Entry Structure

Every entry has three parts:

```
┌──────────────────────────────────────────────────────────┐
│  MATCH      │  INSTRUCTIONS / ACTIONS  │  METADATA       │
│  - in_port  │  - output to port        │  - priority      │
│  - eth_dst  │  - drop                  │  - hard_timeout  │
│  - eth_src  │  - goto meter            │  - idle_timeout  │
│  (etc.)     │                          │  - cookie        │
└──────────────────────────────────────────────────────────┘
```

**Priority** decides which rule wins when multiple entries match.
Higher number = higher priority.  The table-miss rule has priority 0.

**Cookie** is a 64-bit tag the controller writes.  It lets the controller
delete only *its own* flows without touching flows added by other apps.

---

## Key Messages

### OFPFeaturesRequest / OFPSwitchFeatures

Sent by the controller on connect to get the switch's datapath ID (DPID).
OSKen fires the `switch_features_handler` event automatically.

```python
@set_ev_cls(ofp_event.EventOFPSwitchFeatures, CONFIG_DISPATCHER)
def switch_features_handler(self, ev):
    datapath = ev.msg.datapath
    dpid = datapath.id          # 64-bit integer identifier
```

### OFPFlowMod — ADD

Inserts a forwarding rule:

```python
match = parser.OFPMatch(in_port=in_port, eth_dst=dst_mac)
actions = [parser.OFPActionOutput(out_port)]
inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]

mod = parser.OFPFlowMod(
    datapath=dp,
    cookie=FLOW_COOKIE,
    priority=FLOW_PRIORITY,   # 100 in all controllers
    match=match,
    instructions=inst,
    command=ofproto.OFPFC_ADD,
)
dp.send_msg(mod)
```

### OFPFlowMod — DELETE_STRICT

Removes **exactly one** matching rule (same match + priority):

```python
mod = parser.OFPFlowMod(
    datapath=dp,
    cookie=FLOW_COOKIE,
    cookie_mask=0xFFFFFFFFFFFFFFFF,
    match=match,
    command=ofproto.OFPFC_DELETE_STRICT,
    out_port=ofproto.OFPP_ANY,
    out_group=ofproto.OFPG_ANY,
)
dp.send_msg(mod)
```

### OFPFlowMod — DELETE (wildcard with cookie)

Removes *all* flows that share a cookie — used during controller shutdown or
full topology invalidation:

```python
mod = parser.OFPFlowMod(
    datapath=dp,
    cookie=FLOW_COOKIE,
    cookie_mask=0xFFFFFFFFFFFFFFFF,
    command=ofproto.OFPFC_DELETE,
    out_port=ofproto.OFPP_ANY,
    out_group=ofproto.OFPG_ANY,
)
dp.send_msg(mod)
```

### OFPPacketIn

When a packet hits no rule (table-miss) the switch sends it to the controller:

```python
@set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
def _packet_in_handler(self, ev):
    msg = ev.msg
    datapath = msg.datapath
    in_port = msg.match['in_port']
    pkt = packet.Packet(msg.data)
    eth = pkt.get_protocol(ethernet.ethernet)
```

### OFPPacketOut

Controller pushes a specific packet out of a port:

```python
out = parser.OFPPacketOut(
    datapath=dp,
    buffer_id=msg.buffer_id,
    in_port=in_port,
    actions=[parser.OFPActionOutput(out_port)],
    data=msg.data,
)
dp.send_msg(out)
```

---

## Table-Miss Rule

Every controller installs a priority=0 catch-all rule on connect that sends
unmatched packets to the controller:

```
priority=0   match=<anything>   action=OUTPUT:CONTROLLER
```

Without this rule unmatched packets are silently dropped.

---

## GROUP Table (ECMP)

A group entry contains a list of *buckets*; the SELECT type picks one bucket
per flow using an internal hash (typically on src+dst MAC/IP).

```
OFPGroupMod  command=ADD  type=SELECT  group_id=42
  bucket[0]: actions=[OUTPUT:port2]
  bucket[1]: actions=[OUTPUT:port5]
```

A flow rule that reaches this group:

```
match=(eth_dst=00:00:00:00:00:06)  instructions=[GROUP:42]
```

See `docs/09-ecmp.md` for the full ECMP installation pattern.

---

## Further Reading

- OpenFlow Specification 1.3: https://opennetworking.org/
- Open vSwitch: https://www.openvswitch.org/
- OSKen API reference: https://docs.openstack.org/os-ken/latest/
