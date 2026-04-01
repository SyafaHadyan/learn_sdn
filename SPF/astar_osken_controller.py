"""A* Shortest Path Forwarding (SPF) OpenFlow controller.

Computes and installs optimal paths on switches using A* search,
with broadcast tree flooding for unknown destinations.
"""

try:
    import warnings
    warnings.simplefilter('ignore', DeprecationWarning)
    import eventlet
    eventlet.monkey_patch()
except ImportError:
    pass

import heapq
import signal
import sys
import time

from collections import defaultdict, deque

from os_ken.base import app_manager
from os_ken.controller import ofp_event
from os_ken.controller.handler import CONFIG_DISPATCHER, MAIN_DISPATCHER
from os_ken.controller.handler import set_ev_cls
from os_ken.lib.packet import ethernet
from os_ken.lib.packet import ether_types
from os_ken.lib.packet import packet
from os_ken.ofproto import ofproto_v1_3
from os_ken.topology import event
from os_ken.topology.api import get_link, get_switch

ASTAR_FLOW_COOKIE = 0x4153545200000001
ASTAR_FLOW_COOKIE_MASK = 0xFFFFFFFFFFFFFFFF
ASTAR_FLOW_PRIORITY = 100
TABLE_MISS_PRIORITY = 0
HOST_STALE_SECONDS = 300


def _handle_sigint(signum, frame):
    """Handle SIGINT (Ctrl+C) by gracefully stopping OSKen."""
    print("\n[SIGNAL] SIGINT received, initiating graceful shutdown...", file=sys.stderr, flush=True)
    app_manager.AppManager.stop()


signal.signal(signal.SIGINT, _handle_sigint)


class AStarSwitch(app_manager.OSKenApp):
    OFP_VERSIONS = [ofproto_v1_3.OFP_VERSION]
    TOPOLOGY_EVENTS = [
        event.EventSwitchEnter,
        event.EventSwitchLeave,
        event.EventPortAdd,
        event.EventPortDelete,
        event.EventPortModify,
        event.EventLinkAdd,
        event.EventLinkDelete,
    ]

    def __init__(self, *args, **kwargs):
        super(AStarSwitch, self).__init__(*args, **kwargs)
        self.datapaths = {}
        self.switches = []
        self.mymacs = {}
        self.host_last_seen = {}
        self.adjacency = defaultdict(list)  # Maps dpid -> [(neighbor_dpid, out_port), ...]
        self.port_map = {}  # Maps (dpid, neighbor_dpid) -> out_port for quick lookup
        self.topology_signature = None
        self.installed_paths = {}
        self.broadcast_tree = {}
        self.access_ports = defaultdict(set)
        self.heuristic_hop_cache = {}

    def _switch_name(self, dpid):
        return f"s{dpid}"

    def _format_path(self, path_tuples):
        if not path_tuples:
            return "(empty path)"
        return " -> ".join(self._switch_name(sw) for sw, _, _ in path_tuples)

    def _learn_host(self, mac_addr, dpid, port_no):
        self.mymacs[mac_addr] = (dpid, port_no)
        self.host_last_seen[mac_addr] = time.time()

    def _is_access_port(self, dpid, port_no):
        return port_no in self.access_ports.get(dpid, set())

    def _get_port(self, src_dpid, dst_dpid):
        return self.port_map.get((src_dpid, dst_dpid))

    def _install_unicast_flow(self, datapath, in_port, out_port, src_mac, dst_mac):
        parser = datapath.ofproto_parser
        ofproto = datapath.ofproto
        match = parser.OFPMatch(in_port=in_port, eth_src=src_mac, eth_dst=dst_mac)
        actions = [parser.OFPActionOutput(out_port)]
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]

        delete_mod = parser.OFPFlowMod(
            datapath=datapath,
            cookie=ASTAR_FLOW_COOKIE,
            cookie_mask=ASTAR_FLOW_COOKIE_MASK,
            command=ofproto.OFPFC_DELETE_STRICT,
            out_port=ofproto.OFPP_ANY,
            out_group=ofproto.OFPG_ANY,
            priority=ASTAR_FLOW_PRIORITY,
            match=match,
        )
        datapath.send_msg(delete_mod)

        add_mod = parser.OFPFlowMod(
            datapath=datapath,
            cookie=ASTAR_FLOW_COOKIE,
            command=ofproto.OFPFC_ADD,
            idle_timeout=0,
            hard_timeout=0,
            priority=ASTAR_FLOW_PRIORITY,
            match=match,
            instructions=inst,
        )
        datapath.send_msg(add_mod)

    def _purge_stale_hosts(self):
        now = time.time()
        stale = {mac for mac, last_seen in self.host_last_seen.items()
                 if now - last_seen > HOST_STALE_SECONDS}
        if not stale:
            return
        self.mymacs = {mac: loc for mac, loc in self.mymacs.items() if mac not in stale}
        self.host_last_seen = {mac: t for mac, t in self.host_last_seen.items() if mac not in stale}
        self.installed_paths = {k: v for k, v in self.installed_paths.items()
                                if k[0] not in stale and k[1] not in stale}

    def _purge_hosts_on_departed_switches(self):
        """Remove hosts on switches no longer in topology."""
        current_switches = set(self.switches)
        departed_macs = {mac for mac, (dpid, _) in self.mymacs.items() if dpid not in current_switches}
        if not departed_macs:
            return
        self.logger.debug("[HOST-PURGE] removing %d host(s) from departed switches", len(departed_macs))
        self.mymacs = {mac: loc for mac, loc in self.mymacs.items() if mac not in departed_macs}
        self.host_last_seen = {mac: t for mac, t in self.host_last_seen.items() if mac not in departed_macs}
        self.installed_paths = {k: v for k, v in self.installed_paths.items()
                                if k[0] not in departed_macs and k[1] not in departed_macs}

    def _active_hosts(self):
        self._purge_stale_hosts()
        return sorted(self.mymacs.keys())

    def stop(self):
        """Cleanup when controller stops."""
        self.logger.info("[CONTROLLER-STOP] shutting down, clearing %d hosts and %d cached paths",
                         len(self.mymacs), len(self.installed_paths))
        self._flush_all_astar_flows()
        self.mymacs.clear()
        self.host_last_seen.clear()
        self.installed_paths.clear()
        self.broadcast_tree.clear()
        self.heuristic_hop_cache.clear()
        self.datapaths.clear()
        super(AStarSwitch, self).stop()
        self.logger.info("[CONTROLLER-EXIT] cleanup complete, exiting gracefully")
        sys.exit(0)

    def _update_host_location(self, mac, dpid, port):
        current_loc = self.mymacs.get(mac)
        new_loc = (dpid, port)
        if current_loc != new_loc:
            is_new = mac not in self.mymacs
            self._learn_host(mac, dpid, port)
            event_type = "discovered" if is_new else "moved"
            self.logger.info("[HOST-LEARN] MAC %s %s at switch %d port %d", mac, event_type, dpid, port)
        else:
            self.host_last_seen[mac] = time.time()

    def _delete_astar_flows(self, datapath):
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        mod = parser.OFPFlowMod(
            datapath=datapath,
            cookie=ASTAR_FLOW_COOKIE,
            cookie_mask=ASTAR_FLOW_COOKIE_MASK,
            command=ofproto.OFPFC_DELETE,
            out_port=ofproto.OFPP_ANY,
            out_group=ofproto.OFPG_ANY,
            match=parser.OFPMatch(),
        )
        datapath.send_msg(mod)

    def _flush_all_astar_flows(self):
        for datapath in self.datapaths.values():
            self._delete_astar_flows(datapath)
        self.installed_paths.clear()

    def _reinstall_all_known_routes(self):
        known_hosts = self._active_hosts()
        installed = 0
        skipped = 0
        unreachable = 0

        self.logger.debug("[ROUTE-INSTALL] starting batch route installation for %d hosts", len(known_hosts))

        for src_mac in known_hosts:
            for dst_mac in known_hosts:
                if src_mac == dst_mac:
                    continue
                src_loc = self.mymacs.get(src_mac)
                dst_loc = self.mymacs.get(dst_mac)
                if src_loc is None or dst_loc is None:
                    skipped += 1
                    continue

                path = self.compute_path(src_loc[0], dst_loc[0], src_loc[1], dst_loc[1])
                if path:
                    self.install_path(path, src_mac, dst_mac)
                    installed += 1
                else:
                    unreachable += 1
                    self.logger.warning("[ROUTE-INSTALL] %s->%s: path not found (s%d->s%d)",
                                        src_mac, dst_mac, src_loc[0], dst_loc[0])

        self.logger.info("[TOPO] proactive route refresh: installed=%d, skipped=%d, unreachable=%d, active_hosts=%d",
                         installed, skipped, unreachable, len(known_hosts))

    def _build_reverse_hop_heuristic(self, dst):
        """Build exact hop-distance-to-destination map by reverse BFS from dst."""
        hops = {dst: 0}
        queue = deque([dst])

        while queue:
            u = queue.popleft()
            for v, _ in self.adjacency.get(u, []):
                if v not in hops:
                    hops[v] = hops[u] + 1
                    queue.append(v)

        return hops

    def _heuristic_hops(self, node, dst):
        table = self.heuristic_hop_cache.get(dst)
        if table is None:
            table = self._build_reverse_hop_heuristic(dst)
            self.heuristic_hop_cache[dst] = table
        return table.get(node, float('inf'))

    def _a_star_spf_heap(self, src, dst):
        """A* SPF. Returns (distance_dict, previous_dict)."""
        if src not in self.adjacency and src not in self.switches:
            return {}, {}

        distance = {dpid: float('inf') for dpid in self.switches}
        previous = {dpid: None for dpid in self.switches}

        for u in self.adjacency:
            distance.setdefault(u, float('inf'))
            previous.setdefault(u, None)
            for v, _ in self.adjacency[u]:
                distance.setdefault(v, float('inf'))
                previous.setdefault(v, None)

        distance[src] = 0
        src_h = self._heuristic_hops(src, dst)

        if src != dst and src_h == float('inf'):
            self.logger.warning("[A*-UNREACHABLE] s%d->s%d: destination not reachable in current graph", src, dst)
            return distance, previous

        open_heap = [(src_h, 0, src)]
        closed = set()
        edge_exams = 0
        relaxations = 0
        expanded = 0

        self.logger.debug("[A*-INIT] computing shortest path: src=s%d, dst=s%d, vertices=%d, edges=%d",
                          src, dst, len(distance), sum(len(neighbors) for neighbors in self.adjacency.values()))

        while open_heap:
            f_score, g_score, u = heapq.heappop(open_heap)

            if u in closed:
                continue

            if g_score > distance[u]:
                continue

            closed.add(u)
            expanded += 1

            if u == dst:
                self.logger.debug("[A*-GOAL] reached destination s%d with g=%d f=%d after %d expansion(s)",
                                  dst, g_score, f_score, expanded)
                break

            debug_enabled = self.logger.isEnabledFor(10)
            for v, out_port in self.adjacency.get(u, []):
                edge_exams += 1
                tentative_g = distance[u] + 1

                if tentative_g < distance[v]:
                    relaxations += 1
                    distance[v] = tentative_g
                    previous[v] = u
                    h = self._heuristic_hops(v, dst)
                    if h == float('inf'):
                        continue
                    heapq.heappush(open_heap, (tentative_g + h, tentative_g, v))
                    if debug_enabled:
                        self.logger.debug("[A*-RELAX] edge s%d->s%d: g=%d, h=%d, f=%d (port=%d)",
                                          u, v, tentative_g, h, tentative_g + h, out_port)
                elif tentative_g == distance[v] and (previous[v] is None or u < previous[v]):
                    relaxations += 1
                    previous[v] = u

        reachable = sum(1 for d in distance.values() if d != float('inf'))
        self.logger.info("[A*-DONE] s%d->s%d: reachable=%d/%d, expanded=%d, edge_exams=%d, relaxations=%d",
                         src, dst, reachable, len(distance), expanded, edge_exams, relaxations)

        return distance, previous

    def _single_source_shortest_paths(self, src):
        """Compute shortest-path tree rooted at src using unit-weight Dijkstra."""
        distance = {dpid: float('inf') for dpid in self.switches}
        previous = {dpid: None for dpid in self.switches}
        distance[src] = 0

        pq = [(0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > distance[u]:
                continue

            for v, _ in self.adjacency.get(u, []):
                alt = d + 1
                if alt < distance.get(v, float('inf')):
                    distance[v] = alt
                    previous[v] = u
                    heapq.heappush(pq, (alt, v))
                elif alt == distance.get(v, float('inf')) and (previous[v] is None or u < previous[v]):
                    previous[v] = u

        return distance, previous

    def _build_broadcast_tree(self):
        """Build spanning tree rooted at minimum DPID switch."""
        if not self.switches:
            self.broadcast_tree = {}
            return

        root = min(self.switches)
        self.logger.debug("[TREE-BUILD] constructing spanning tree rooted at s%d from %d switches",
                          root, len(self.switches))

        distance, previous = self._single_source_shortest_paths(root)

        tree = defaultdict(set)
        tree_edges = 0
        for node in self.switches:
            parent = previous[node]
            if parent is not None:
                tree[parent].add(node)
                tree[node].add(parent)
                tree_edges += 1
                self.logger.debug("[TREE-EDGE] s%d->s%d: tree parent-child link", parent, node)

        self.broadcast_tree = {node: sorted(list(neighbors)) for node, neighbors in tree.items()}

        finite_distances = [d for d in distance.values() if d != float('inf')]
        tree_diameter = max(finite_distances) if finite_distances else 0
        self.logger.info("[TREE-DONE] root=s%d, tree_edges=%d, tree_diameter(hops)=%s",
                         root, tree_edges, tree_diameter)

    def _flood_over_tree(self, datapath, in_port, data, buffer_id):
        parser = datapath.ofproto_parser
        out_ports_set = set()
        if self.broadcast_tree:
            for neighbor in self.broadcast_tree.get(datapath.id, []):
                out_port = self._get_port(datapath.id, neighbor)
                if out_port is not None and out_port != in_port:
                    out_ports_set.add(out_port)

        access_set = self.access_ports.get(datapath.id, set())
        for access_port in access_set:
            if access_port != in_port:
                out_ports_set.add(access_port)

        if not out_ports_set:
            self.logger.debug("[PKT-DROP] s%d: no controlled flood ports available", datapath.id)
            return

        actions = [parser.OFPActionOutput(port_no) for port_no in sorted(out_ports_set)]
        data_arg = None if buffer_id != datapath.ofproto.OFP_NO_BUFFER else data
        out = parser.OFPPacketOut(
            datapath=datapath,
            buffer_id=buffer_id,
            in_port=in_port,
            actions=actions,
            data=data_arg,
        )
        datapath.send_msg(out)

    def compute_path(self, src, dst, first_port, final_port):
        """Compute shortest path. Returns list of (switch, in_port, out_port) or []."""
        if src not in self.switches or dst not in self.switches:
            self.logger.warning("[PATH-QUERY] invalid endpoints: src=s%d, dst=s%d", src, dst)
            return []

        self.logger.debug("[PATH-QUERY] computing path: s%d -> s%d", src, dst)
        distance, previous = self._a_star_spf_heap(src, dst)

        if src != dst and distance.get(dst, float('inf')) == float('inf'):
            self.logger.warning("[PATH-UNREACHABLE] s%d: no path to s%d from s%d", dst, src, dst)
            return []

        if src == dst:
            self.logger.debug("[PATH-LOCAL] s%d: same-switch path, no forwarding needed", src)
            return [(src, first_port, final_port)]

        path_nodes = [dst]
        current = previous.get(dst)
        while current is not None:
            path_nodes.append(current)
            if current == src:
                break
            current = previous.get(current)

        if not path_nodes or path_nodes[-1] != src:
            self.logger.error("[PATH-CORRUPT] s%d->s%d: path reconstruction failed", src, dst)
            return []

        path_nodes.reverse()
        self.logger.debug("[PATH-RECONSTRUCT] s%d->s%d: reconstructed %d-hop path: %s",
                          src, dst, len(path_nodes) - 1, self._format_path([(sw, 0, 0) for sw in path_nodes]))

        result = []
        in_port = first_port
        for s1, s2 in zip(path_nodes[:-1], path_nodes[1:]):
            out_port = self._get_port(s1, s2)
            if out_port is None:
                self.logger.error("[PATH-PORTMAP] s%d->s%d: port mapping lost during path build", s1, s2)
                return []
            result.append((s1, in_port, out_port))
            in_port = self._get_port(s2, s1)
            if in_port is None and s2 != dst:
                self.logger.error("[PATH-PORTMAP] s%d->s%d: reverse port mapping lost", s2, s1)
                return []

        result.append((dst, in_port, final_port))
        self.logger.info("[PATH-COMPUTED] s%d->s%d: %d-hop path with %d flow entries",
                         src, dst, len(result) - 1, len(result) * 2)
        return result

    def install_path(self, p, src_mac, dst_mac):
        key = (src_mac, dst_mac)
        last = self.installed_paths.get(key)
        if last == p:
            return

        self.installed_paths[key] = p
        self.logger.debug("[FLOW-INSTALL] %s -> %s: %s", src_mac, dst_mac, self._format_path(p))
        self._install_flows_both_directions(p, src_mac, dst_mac)

    def _install_flows_both_directions(self, path, src_mac, dst_mac):
        for sw, in_port, out_port in path:
            datapath = self.datapaths.get(int(sw))
            if datapath:
                self._install_unicast_flow(datapath, in_port, out_port, src_mac, dst_mac)

        for sw, in_port, out_port in reversed(path):
            datapath = self.datapaths.get(int(sw))
            if datapath:
                self._install_unicast_flow(datapath, out_port, in_port, dst_mac, src_mac)

    @set_ev_cls(ofp_event.EventOFPSwitchFeatures, CONFIG_DISPATCHER)
    def switch_features_handler(self, ev):
        datapath = ev.msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        match = parser.OFPMatch()
        actions = [parser.OFPActionOutput(ofproto.OFPP_CONTROLLER, ofproto.OFPCML_NO_BUFFER)]
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]
        mod = datapath.ofproto_parser.OFPFlowMod(
            datapath=datapath,
            match=match,
            cookie=0,
            command=ofproto.OFPFC_ADD,
            idle_timeout=0,
            hard_timeout=0,
            priority=TABLE_MISS_PRIORITY,
            instructions=inst,
        )
        datapath.send_msg(mod)

    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def _packet_in_handler(self, ev):
        msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        in_port = msg.match['in_port']
        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocol(ethernet.ethernet)

        if eth.ethertype == ether_types.ETH_TYPE_LLDP:
            return

        dst = eth.dst
        src = eth.src
        dpid = datapath.id

        if self._is_access_port(dpid, in_port):
            self._update_host_location(src, dpid, in_port)

        if src not in self.mymacs:
            self.logger.debug("[PKT-DROP] %s -> %s: source host not learned on an access port", src, dst)
            return

        if dst in self.mymacs:
            if (src, dst) in self.installed_paths:
                p = self.installed_paths[(src, dst)]
                out_port = None
                for sw, _, port in p:
                    if sw == dpid:
                        out_port = port
                        break
                if out_port is None:
                    self.logger.warning("[PKT-DROP] %s -> %s: current switch s%d not in cached path", src, dst, dpid)
                    return
            else:
                src_sw, src_port = self.mymacs[src]
                dst_sw, dst_port = self.mymacs[dst]
                p = self.compute_path(src_sw, dst_sw, src_port, dst_port)
                if p:
                    self.logger.debug("[PKT-FWD] %s -> %s: path computed with %d hop(s)", src, dst, len(p))
                    self.install_path(p, src, dst)
                    out_port = p[0][2]
                else:
                    self.logger.warning("[PKT-DROP] %s -> %s: no path available in current topology", src, dst)
                    return
        else:
            self.logger.debug(
                "[PKT-FLOOD] %s -> %s: destination unknown, controlled flood over broadcast tree",
                src,
                dst,
            )
            self._flood_over_tree(datapath, in_port, msg.data, msg.buffer_id)
            return

        actions = [parser.OFPActionOutput(out_port)]
        data = None
        if msg.buffer_id == ofproto.OFP_NO_BUFFER:
            data = msg.data
        out = parser.OFPPacketOut(datapath=datapath, buffer_id=msg.buffer_id, in_port=in_port,
                                  actions=actions, data=data)
        datapath.send_msg(out)

    @set_ev_cls(TOPOLOGY_EVENTS)
    def get_topology_data(self, ev):
        switch_list = get_switch(self, None)
        switch_ids = sorted(switch.dp.id for switch in switch_list)
        links_list = get_link(self, None)
        new_mylinks = sorted(
            (link.src.dpid, link.dst.dpid, link.src.port_no, link.dst.port_no)
            for link in links_list
        )

        if not new_mylinks:
            if self.topology_signature is not None:
                self.logger.debug("[TOPO] ignoring transient empty topology snapshot")
            return

        self.switches = switch_ids
        self.datapaths = {switch.dp.id: switch.dp for switch in switch_list}

        inter_switch_ports = defaultdict(set)
        for s1, s2, port1, port2 in new_mylinks:
            inter_switch_ports[s1].add(port1)
            inter_switch_ports[s2].add(port2)

        self.access_ports = defaultdict(set)
        for switch in switch_list:
            all_ports = {port.port_no for port in getattr(switch, 'ports', [])}
            self.access_ports[switch.dp.id] = all_ports - inter_switch_ports[switch.dp.id]

        new_adjacency = defaultdict(list)
        new_port_map = {}
        for s1, s2, port1, port2 in new_mylinks:
            new_adjacency[s1].append((s2, port1))
            new_adjacency[s2].append((s1, port2))
            new_port_map[(s1, s2)] = port1
            new_port_map[(s2, s1)] = port2

        old_signature = self.topology_signature
        new_signature = (tuple(sorted(self.switches)), tuple(sorted(new_mylinks)))
        self.adjacency = new_adjacency
        self.port_map = new_port_map

        if old_signature != new_signature:
            self.heuristic_hop_cache.clear()

            if old_signature is not None:
                _, old_links = old_signature
                old_link_set = set(old_links)
                new_link_set = set(new_mylinks)
                added = sorted(new_link_set - old_link_set)
                removed = sorted(old_link_set - new_link_set)

                self.logger.info("[TOPO-SNAPSHOT] old: %d switches, %d links | new: %d switches, %d links | delta: +%d -%d",
                                 len(old_signature[0]), len(old_links),
                                 len(self.switches), len(new_mylinks),
                                 len(added), len(removed))

                if added:
                    self.logger.info("[TOPO-UP] %d link(s) up: %s",
                                     len(added), [(s1, s2) for s1, s2, _, _ in added])
                if removed:
                    self.logger.warning("[TOPO-DOWN] %d link(s) down: %s",
                                        len(removed), [(s1, s2) for s1, s2, _, _ in removed])

                self._purge_hosts_on_departed_switches()

                self.logger.info("[TOPO-REFRESH] topology changed, flushing %d installed flows and reinstalling routes",
                                 len(self.installed_paths))
                self._flush_all_astar_flows()
                self.installed_paths.clear()
                self._reinstall_all_known_routes()
            else:
                self.logger.info("[TOPO-INITIAL] initial topology snapshot: %d switch(es), %d link(s)",
                                 len(self.switches), len(new_mylinks))

            self.topology_signature = new_signature
            self._build_broadcast_tree()


if __name__ == '__main__':
    import os
    import sys

    current_file = os.path.abspath(__file__)
    passthrough_args = sys.argv[1:]
    if '--observe-links' not in passthrough_args:
        passthrough_args = ['--observe-links'] + passthrough_args
    sys.argv = ['astar_osken_controller', *passthrough_args, current_file]

    from os_ken.cmd.manager import main
    sys.exit(main())