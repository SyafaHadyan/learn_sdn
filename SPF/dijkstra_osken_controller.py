"""Dijkstra SPF controller for OSKen."""

try:
    import warnings
    warnings.simplefilter('ignore', DeprecationWarning)
    import eventlet
    eventlet.monkey_patch()
except ImportError:
    pass

import time

from os_ken.base import app_manager
from os_ken.controller import ofp_event
from os_ken.controller.handler import CONFIG_DISPATCHER, MAIN_DISPATCHER
from os_ken.controller.handler import set_ev_cls
from os_ken.ofproto import ofproto_v1_3
from os_ken.lib.packet import packet
from os_ken.lib.packet import ethernet
from os_ken.lib.packet import ether_types
from os_ken.topology.api import get_switch, get_link
from os_ken.topology import event
from collections import defaultdict

SPF_FLOW_COOKIE = 0x5346500000000001
SPF_FLOW_COOKIE_MASK = 0xFFFFFFFFFFFFFFFF
SPF_FLOW_PRIORITY = 100
TABLE_MISS_PRIORITY = 0
HOST_STALE_SECONDS = 300

class DijkstraSwitch(app_manager.OSKenApp):
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
        super(DijkstraSwitch, self).__init__(*args, **kwargs)
        self.topology_api_app = self
        self.datapath_list = []
        self.datapaths = {}
        self.switches = []
        self.mymacs = {}
        self.host_last_seen = {}
        self.adjacency = defaultdict(lambda: defaultdict(lambda: None))
        self.mylinks = []
        self.topology_signature = None
        self.installed_paths = {}
        self.broadcast_tree = {}
        self.access_ports = defaultdict(set)

    def _switch_name(self, dpid):
        return f"s{dpid}"

    def _format_path(self, path_tuples):
        if not path_tuples:
            return "(empty path)"
        return " -> ".join(self._switch_name(sw) for sw, _, _ in path_tuples)

    def _format_switches(self, switch_list):
        if not switch_list:
            return "(no switches)"
        names = ", ".join(self._switch_name(s) for s in switch_list)
        dpids = ", ".join(str(s) for s in switch_list)
        return f"{names} (DPIDs: {dpids})"

    def _topology_signature_for(self, switches_list, links_list):
        return tuple(sorted(switches_list)), tuple(sorted(links_list))

    def _learn_host(self, mac_addr, dpid, port_no):
        self.mymacs[mac_addr] = (dpid, port_no)
        self.host_last_seen[mac_addr] = time.time()

    def _is_access_port(self, dpid, port_no):
        return port_no in self.access_ports.get(dpid, set())

    def _install_unicast_flow(self, datapath, in_port, out_port, src_mac, dst_mac):
        parser = datapath.ofproto_parser
        ofproto = datapath.ofproto
        match = parser.OFPMatch(in_port=in_port, eth_src=src_mac, eth_dst=dst_mac)
        actions = [parser.OFPActionOutput(out_port)]
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]

        delete_mod = parser.OFPFlowMod(
            datapath=datapath,
            cookie=SPF_FLOW_COOKIE,
            cookie_mask=SPF_FLOW_COOKIE_MASK,
            command=ofproto.OFPFC_DELETE_STRICT,
            out_port=ofproto.OFPP_ANY,
            out_group=ofproto.OFPG_ANY,
            priority=SPF_FLOW_PRIORITY,
            match=match,
        )
        datapath.send_msg(delete_mod)

        add_mod = parser.OFPFlowMod(
            datapath=datapath,
            cookie=SPF_FLOW_COOKIE,
            command=ofproto.OFPFC_ADD,
            idle_timeout=0,
            hard_timeout=0,
            priority=SPF_FLOW_PRIORITY,
            match=match,
            instructions=inst,
        )
        datapath.send_msg(add_mod)

    def _purge_stale_hosts(self):
        now = time.time()
        stale = [mac_addr for mac_addr, last_seen in self.host_last_seen.items()
                 if now - last_seen > HOST_STALE_SECONDS]
        for mac_addr in stale:
            self.mymacs.pop(mac_addr, None)
            self.host_last_seen.pop(mac_addr, None)
            for route_key in list(self.installed_paths.keys()):
                if mac_addr in route_key:
                    self.installed_paths.pop(route_key, None)

    def _active_hosts(self):
        self._purge_stale_hosts()
        return sorted(self.mymacs.keys())

    def _delete_spf_flows(self, datapath):
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        mod = parser.OFPFlowMod(
            datapath=datapath,
            cookie=SPF_FLOW_COOKIE,
            cookie_mask=SPF_FLOW_COOKIE_MASK,
            command=ofproto.OFPFC_DELETE,
            out_port=ofproto.OFPP_ANY,
            out_group=ofproto.OFPG_ANY,
            match=parser.OFPMatch(),
        )
        datapath.send_msg(mod)

    def _flush_all_spf_flows(self):
        for datapath in self.datapaths.values():
            self._delete_spf_flows(datapath)
        self.installed_paths.clear()

    def _reinstall_all_known_routes(self):
        known_hosts = self._active_hosts()
        installed = 0
        for src_mac in known_hosts:
            for dst_mac in known_hosts:
                if src_mac == dst_mac:
                    continue
                src_loc = self.mymacs.get(src_mac)
                dst_loc = self.mymacs.get(dst_mac)
                if src_loc is None or dst_loc is None:
                    continue
                path = self.compute_path(src_loc[0], dst_loc[0], src_loc[1], dst_loc[1])
                if path:
                    self.install_path(path, src_mac, dst_mac)
                    installed += 1
        self.logger.info("[TOPO] proactive route refresh completed: %d host-pair route(s) installed", installed)

    def _build_broadcast_tree(self):
        if not self.switches:
            self.broadcast_tree = {}
            return

        root = min(self.switches)
        distance = {dpid: float('Inf') for dpid in self.switches}
        previous = {dpid: None for dpid in self.switches}
        distance[root] = 0
        remaining = set(self.switches)

        while remaining:
            current = self.minimum_distance(distance, remaining)
            if current is None:
                break
            remaining.remove(current)
            if distance[current] == float('Inf'):
                break

            for neighbor in sorted(self.switches):
                if self.adjacency[current][neighbor] is None:
                    continue
                alt = distance[current] + 1
                if alt < distance[neighbor] or (alt == distance[neighbor] and (previous[neighbor] is None or current < previous[neighbor])):
                    distance[neighbor] = alt
                    previous[neighbor] = current

        tree = defaultdict(set)
        for node in self.switches:
            parent = previous[node]
            if parent is not None:
                tree[parent].add(node)
                tree[node].add(parent)

        self.broadcast_tree = {node: sorted(neighbors) for node, neighbors in tree.items()}

    def _flood_over_tree(self, datapath, in_port, data, buffer_id):
        parser = datapath.ofproto_parser
        out_ports = []

        if self.broadcast_tree:
            for neighbor in self.broadcast_tree.get(datapath.id, []):
                out_port = self.adjacency[datapath.id][neighbor]
                if out_port is not None and out_port != in_port:
                    out_ports.append(out_port)

        for access_port in sorted(self.access_ports.get(datapath.id, set())):
            if access_port != in_port:
                out_ports.append(access_port)

        if not out_ports:
            self.logger.debug("[PKT-DROP] s%d: no controlled flood ports available", datapath.id)
            return

        actions = [parser.OFPActionOutput(port_no) for port_no in sorted(set(out_ports))]
        data_arg = None if buffer_id != datapath.ofproto.OFP_NO_BUFFER else data
        out = parser.OFPPacketOut(
            datapath=datapath,
            buffer_id=buffer_id,
            in_port=in_port,
            actions=actions,
            data=data_arg,
        )
        datapath.send_msg(out)

    def minimum_distance(self, distance, Q):
        node = None
        min_val = float('Inf')
        for v in sorted(Q):
            d = distance.get(v, float('Inf'))
            if d < min_val:
                min_val = d
                node = v
        return node

    def compute_path(self, src, dst, first_port, final_port):
        if src not in self.switches or dst not in self.switches:
            return []

        distance = {}
        previous = {}

        for dpid in self.switches:
            distance[dpid] = float('Inf')
            previous[dpid] = None

        distance[src] = 0
        Q = set(self.switches)

        while len(Q) > 0:
            u = self.minimum_distance(distance, Q)
            if u is None:
                break
            Q.remove(u)
            if distance[u] == float('Inf'):
                break
            for p in self.switches:
                if self.adjacency[u][p] is not None:
                    w = 1
                    alt = distance[u] + w
                    if alt < distance[p] or (alt == distance[p] and (previous[p] is None or u < previous[p])):
                        distance[p] = alt
                        previous[p] = u

        if src != dst and distance.get(dst, float('Inf')) == float('Inf'):
            return []

        if src == dst:
            return [(src, first_port, final_port)]

        path_nodes = [dst]
        current = previous.get(dst)
        while current is not None:
            path_nodes.append(current)
            if current == src:
                break
            current = previous.get(current)

        if not path_nodes or path_nodes[-1] != src:
            return []

        path_nodes.reverse()

        result = []
        in_port = first_port
        for s1, s2 in zip(path_nodes[:-1], path_nodes[1:]):
            out_port = self.adjacency[s1][s2]
            if out_port is None:
                return []
            result.append((s1, in_port, out_port))
            in_port = self.adjacency[s2][s1]
            if in_port is None and s2 != dst:
                return []

        result.append((dst, in_port, final_port))
        return result

    def install_path(self, p, src_mac, dst_mac):
        key = (src_mac, dst_mac)
        last = self.installed_paths.get(key)
        if last == p:
            return

        self.installed_paths[key] = p
        if len(p) > 1:
            self.logger.debug("[FLOW-INSTALL] %s -> %s: %s", src_mac, dst_mac, self._format_path(p))

        for sw, in_port, out_port in p:
            datapath = self.datapaths.get(int(sw))
            if datapath is None:
                self.logger.warning("datapath with id %s not found in datapath list", sw)
                continue
            self._install_unicast_flow(datapath, in_port, out_port, src_mac, dst_mac)

        for sw, in_port, out_port in reversed(p):
            rev_datapath = self.datapaths.get(int(sw))
            if rev_datapath is None:
                self.logger.warning("datapath with id %s not found when installing reverse flow", sw)
                continue
            rev_in = out_port
            rev_out = in_port
            self._install_unicast_flow(rev_datapath, rev_in, rev_out, dst_mac, src_mac)

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
            if src not in self.mymacs:
                self._learn_host(src, dpid, in_port)
                self.logger.info("[HOST-LEARN] MAC %s discovered at switch %d port %d", src, dpid, in_port)
            else:
                learned_dpid, learned_port = self.mymacs[src]
                if learned_dpid != dpid or learned_port != in_port:
                    self._learn_host(src, dpid, in_port)
                    self.logger.info("[HOST-LEARN] MAC %s moved to switch %d port %d", src, dpid, in_port)
                else:
                    self.host_last_seen[src] = time.time()

        if src not in self.mymacs:
            self.logger.debug("[PKT-DROP] %s -> %s: source host not learned on an access port", src, dst)
            return

        if dst in self.mymacs:
            src_sw, src_port = self.mymacs[src]
            dst_sw, dst_port = self.mymacs[dst]
            p = self.compute_path(src_sw, dst_sw, src_port, dst_port)
            if p:
                self.logger.debug(
                    "[PATH] %s at s%d:p%d -> %s at s%d:p%d: %s",
                    src,
                    src_sw,
                    src_port,
                    dst,
                    dst_sw,
                    dst_port,
                    p,
                )
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
        switch_list = get_switch(self.topology_api_app, None)
        switch_ids = sorted(switch.dp.id for switch in switch_list)
        links_list = get_link(self.topology_api_app, None)
        new_mylinks = sorted(
            (link.src.dpid, link.dst.dpid, link.src.port_no, link.dst.port_no)
            for link in links_list
        )

        if not new_mylinks:
            if self.topology_signature is not None:
                self.logger.debug("[TOPO] ignoring transient empty topology snapshot")
            return

        self.switches = switch_ids
        self.datapath_list = [switch.dp for switch in switch_list]
        self.datapath_list.sort(key=lambda dp: dp.id)
        self.datapaths = {dp.id: dp for dp in self.datapath_list}

        self.access_ports = defaultdict(set)
        for switch in switch_list:
            all_ports = {port.port_no for port in getattr(switch, 'ports', [])}
            inter_switch_ports = set()
            for link in links_list:
                if link.src.dpid == switch.dp.id:
                    inter_switch_ports.add(link.src.port_no)
                if link.dst.dpid == switch.dp.id:
                    inter_switch_ports.add(link.dst.port_no)
            self.access_ports[switch.dp.id] = all_ports - inter_switch_ports

        new_adjacency = defaultdict(lambda: defaultdict(lambda: None))
        for s1, s2, port1, port2 in new_mylinks:
            new_adjacency[s1][s2] = port1
            new_adjacency[s2][s1] = port2

        old_signature = self.topology_signature
        new_signature = self._topology_signature_for(self.switches, new_mylinks)
        self.adjacency = new_adjacency
        self.mylinks = new_mylinks

        if old_signature != new_signature:
            if old_signature is not None:
                _, old_links = old_signature
                old_link_set = set(old_links)
                new_link_set = set(new_mylinks)
                added = sorted(new_link_set - old_link_set)
                removed = sorted(old_link_set - new_link_set)
                if added:
                    self.logger.info("[TOPO-CHANGE] link up: %s", [(s1, s2) for s1, s2, _, _ in added])
                if removed:
                    self.logger.info("[TOPO-CHANGE] link down: %s", [(s1, s2) for s1, s2, _, _ in removed])
                self.logger.info(
                    "[TOPO] refreshed topology: %d switch(es), %d link(s)",
                    len(self.switches),
                    len(new_mylinks),
                )
                self._flush_all_spf_flows()
                self._reinstall_all_known_routes()

            self.topology_signature = new_signature
            self._build_broadcast_tree()

if __name__ == '__main__':
    import os
    import sys

    current_file = os.path.abspath(__file__)
    passthrough_args = sys.argv[1:]
    if '--observe-links' not in passthrough_args:
        passthrough_args = ['--observe-links'] + passthrough_args
    sys.argv = ['dijkstra_osken_controller', *passthrough_args, current_file]

    from os_ken.cmd.manager import main
    sys.exit(main())
