"""Dijkstra multipath OpenFlow controller with ECMP group forwarding.

Builds equal-cost shortest paths using Dijkstra parent sets, then installs an
OpenFlow SELECT group on ingress switches to load-balance across next hops.

Complexity:   O((V+E) log V) per (src, dst) pair
Metric:       hop count (all edges weight 1)
Multipath:    yes - up to K_PATHS equal-cost paths per (src, dst)
ECMP:         yes - OpenFlow SELECT group on ingress switch

Run:
    python3 dijkstra_multipath_osken_controller.py

See Also:
    dijkstra_osken_controller.py     - single-path variant
    astar_multipath_osken_controller.py - A* variant with multipath
"""

import hashlib

from os_ken.controller import ofp_event
from os_ken.controller.handler import MAIN_DISPATCHER
from os_ken.controller.handler import set_ev_cls
from os_ken.lib.packet import ethernet, ether_types, packet

from dijkstra_osken_controller import (
    DijkstraSwitch,
    SPF_FLOW_COOKIE,
    SPF_FLOW_COOKIE_MASK,
    SPF_FLOW_PRIORITY,
)
from algorithms.dijkstra import dijkstra_multi_parent

import os
import sys

K_PATHS = 4
GROUP_ID_SPACE = 0x7FFFFFFF


class DijkstraMultipathSwitch(DijkstraSwitch):
    """Dijkstra SPF with equal-cost multipath (ECMP) forwarding.

    Inherits all single-path infrastructure from DijkstraSwitch.
    Overrides path installation to use OpenFlow SELECT groups for ECMP.

    How ECMP works:
        1. Run dijkstra_multi_parent -> all equal-cost predecessors per node
        2. Enumerate up to K_PATHS distinct node-paths from src to dst
        3. On ingress switch: install SELECT group (weight=1 per bucket)
        4. On transit/egress switches: install standard unicast flows
    """

    def __init__(self, *args, **kwargs):
        super(DijkstraMultipathSwitch, self).__init__(*args, **kwargs)
        # Caches for multipath state (in addition to base class state)
        self.path_cache = {}           # (src, dst, k) -> [node_path, ...]
        self.flow_groups = {}          # (src_mac, dst_mac) -> (ingress_dpid, group_id)

    # ─────────────────────────────────────────────────────────────────────────
    # ECMP group management
    # ─────────────────────────────────────────────────────────────────────────

    def _alloc_group_id(self, src_mac, dst_mac):
        """Allocate a deterministic group-id for (src_mac, dst_mac)."""
        key = (src_mac, dst_mac)
        existing = self.flow_groups.get(key)
        if existing is not None:
            return existing[1]

        seed = f"{src_mac}->{dst_mac}".encode()
        candidate = int(hashlib.md5(seed).hexdigest()[:8], 16) & GROUP_ID_SPACE
        if candidate == 0:
            candidate = 1

        used_ids = {gid for _, gid in self.flow_groups.values()}
        while candidate in used_ids:
            candidate = (candidate % GROUP_ID_SPACE) + 1
        return candidate

    def _install_group_flow(self, datapath, in_port, src_mac, dst_mac, group_id):
        """Install ingress flow that forwards to SELECT group."""
        parser = datapath.ofproto_parser
        ofproto = datapath.ofproto
        match = parser.OFPMatch(in_port=in_port, eth_src=src_mac, eth_dst=dst_mac)
        actions = [parser.OFPActionGroup(group_id)]
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]

        datapath.send_msg(parser.OFPFlowMod(
            datapath=datapath,
            cookie=SPF_FLOW_COOKIE,
            cookie_mask=SPF_FLOW_COOKIE_MASK,
            command=ofproto.OFPFC_DELETE_STRICT,
            out_port=ofproto.OFPP_ANY,
            out_group=ofproto.OFPG_ANY,
            priority=SPF_FLOW_PRIORITY,
            match=match,
        ))
        datapath.send_msg(parser.OFPFlowMod(
            datapath=datapath,
            cookie=SPF_FLOW_COOKIE,
            command=ofproto.OFPFC_ADD,
            idle_timeout=0,
            hard_timeout=0,
            priority=SPF_FLOW_PRIORITY,
            match=match,
            instructions=inst,
        ))

    def _install_select_group(self, datapath, group_id, out_ports):
        """Install or replace OpenFlow SELECT group (one bucket per next-hop port)."""
        parser = datapath.ofproto_parser
        ofproto = datapath.ofproto

        unique_ports = sorted(set(out_ports))
        if not unique_ports:
            return False

        buckets = [
            parser.OFPBucket(
                weight=1,
                watch_port=ofproto.OFPP_ANY,
                watch_group=ofproto.OFPG_ANY,
                actions=[parser.OFPActionOutput(p)],
            )
            for p in unique_ports
        ]

        datapath.send_msg(parser.OFPGroupMod(
            datapath=datapath,
            command=ofproto.OFPGC_DELETE,
            type_=ofproto.OFPGT_SELECT,
            group_id=group_id,
            buckets=[],
        ))
        datapath.send_msg(parser.OFPGroupMod(
            datapath=datapath,
            command=ofproto.OFPGC_ADD,
            type_=ofproto.OFPGT_SELECT,
            group_id=group_id,
            buckets=buckets,
        ))
        self.logger.debug("[ECMP-GROUP] s%d group=%d ports=%s",
                          datapath.id, group_id, unique_ports)
        return True

    def _delete_multipath_groups(self, datapath):
        """Delete all ECMP groups for a given datapath."""
        parser = datapath.ofproto_parser
        ofproto = datapath.ofproto
        group_ids = {gid for dpid, gid in self.flow_groups.values()
                     if dpid == datapath.id}
        for gid in group_ids:
            datapath.send_msg(parser.OFPGroupMod(
                datapath=datapath,
                command=ofproto.OFPGC_DELETE,
                type_=ofproto.OFPGT_SELECT,
                group_id=gid,
                buckets=[],
            ))

    # ─────────────────────────────────────────────────────────────────────────
    # Multipath computation
    # ─────────────────────────────────────────────────────────────────────────

    def _enumerate_node_paths(self, src, dst, parents, limit):
        """Enumerate up to `limit` equal-cost node-paths via DFS on parent sets."""
        if src == dst:
            return [[src]]
        if dst not in parents:
            return []

        results = []
        stack = [(dst, [dst])]
        while stack and len(results) < limit:
            node, rev = stack.pop()
            if node == src:
                results.append(list(reversed(rev)))
                continue
            for p in sorted(parents.get(node, set()), reverse=True):
                if p not in rev:
                    stack.append((p, rev + [p]))
        return results

    def _decorate_path(self, node_path, first_port, final_port):
        """Annotate a node-path with (dpid, in_port, out_port) tuples."""
        if not node_path:
            return []
        if len(node_path) == 1:
            return [(node_path[0], first_port, final_port)]

        result = []
        in_port = first_port
        for s1, s2 in zip(node_path[:-1], node_path[1:]):
            out_port = self._get_port(s1, s2)
            if out_port is None:
                self.logger.error("[PATH-PORTMAP] s%d->s%d missing port", s1, s2)
                return []
            result.append((s1, in_port, out_port))
            in_port = self._get_port(s2, s1)
            if in_port is None and s2 != node_path[-1]:
                self.logger.error("[PATH-PORTMAP] s%d->s%d reverse port missing", s2, s1)
                return []
        result.append((node_path[-1], in_port, final_port))
        return result

    def compute_multipath(self, src, dst, first_port, final_port, k=K_PATHS):
        """Return up to k decorated equal-cost paths from src to dst."""
        if src not in self.switches or dst not in self.switches:
            return []
        if src == dst:
            return [[(src, first_port, final_port)]]

        cache_key = (src, dst, k)
        if cache_key in self.path_cache:
            node_paths = self.path_cache[cache_key]
        else:
            distance, parents = dijkstra_multi_parent(self.adjacency, src)
            if distance.get(dst, float("inf")) == float("inf"):
                return []
            node_paths = self._enumerate_node_paths(src, dst, parents, k)
            self.path_cache[cache_key] = node_paths
            self.logger.info("[PATH-MP] s%d->s%d cost=%d paths=%d",
                             src, dst, int(distance[dst]), len(node_paths))

        return [d for path in node_paths[:k]
                for d in [self._decorate_path(path, first_port, final_port)] if d]

    def install_multipath(self, paths, src_mac, dst_mac):
        """Install ECMP SELECT group on ingress + unicast flows on transit."""
        if not paths:
            return

        key = (src_mac, dst_mac)
        if self.installed_paths.get(key) == paths:
            return

        ingress_dpid = paths[0][0][0]
        ingress_in_port = paths[0][0][1]
        ingress_out_ports = [p[0][2] for p in paths if p and p[0][0] == ingress_dpid]

        ingress_dp = self.datapaths.get(ingress_dpid)
        if ingress_dp is None:
            self.logger.warning("[ECMP-INSTALL] s%d datapath unavailable", ingress_dpid)
            return

        # Transit and egress hops: deterministic unicast flows
        for path in paths:
            for sw, in_p, out_p in path[1:]:
                dp = self.datapaths.get(sw)
                if dp:
                    self._install_unicast_flow(dp, in_p, out_p, src_mac, dst_mac)

        group_id = self._alloc_group_id(src_mac, dst_mac)
        if self._install_select_group(ingress_dp, group_id, ingress_out_ports):
            self.flow_groups[key] = (ingress_dpid, group_id)
            self._install_group_flow(ingress_dp, ingress_in_port, src_mac, dst_mac, group_id)
            self.logger.info("[ECMP-INSTALL] %s->%s paths=%d ingress=s%d group=%d",
                             src_mac, dst_mac, len(paths), ingress_dpid, group_id)
            self.installed_paths[key] = paths

    # ─────────────────────────────────────────────────────────────────────────
    # Overrides
    # ─────────────────────────────────────────────────────────────────────────

    def _flush_all_flows(self):
        """Flush flows AND delete all ECMP groups."""
        for dp in self.datapaths.values():
            self._delete_all_flows(dp)
            self._delete_multipath_groups(dp)
        self.installed_paths.clear()
        self.path_cache.clear()
        self.flow_groups.clear()

    def _reinstall_all_known_routes(self):
        """Reinstall multipath ECMP routes for all known host pairs."""
        hosts = self._active_hosts()
        installed = skipped = unreachable = 0
        for src_mac in hosts:
            for dst_mac in hosts:
                if src_mac == dst_mac:
                    continue
                src_loc = self.mymacs.get(src_mac)
                dst_loc = self.mymacs.get(dst_mac)
                if not src_loc or not dst_loc:
                    skipped += 1
                    continue
                paths = self.compute_multipath(
                    src_loc[0], dst_loc[0], src_loc[1], dst_loc[1]
                )
                if paths:
                    self.install_multipath(paths, src_mac, dst_mac)
                    installed += 1
                else:
                    unreachable += 1
        self.logger.info("[TOPO] multipath refresh: installed=%d skipped=%d unreachable=%d hosts=%d",
                         installed, skipped, unreachable, len(hosts))

    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def _packet_in_handler(self, ev):
        """Handle packet-in; forward via group if ECMP is installed."""
        msg = ev.msg
        dp = msg.datapath
        ofproto = dp.ofproto
        parser = dp.ofproto_parser
        in_port = msg.match["in_port"]
        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocol(ethernet.ethernet)

        if eth.ethertype == ether_types.ETH_TYPE_LLDP:
            return

        src, dst, dpid = eth.src, eth.dst, dp.id

        if self._is_access_port(dpid, in_port):
            self._update_host_location(src, dpid, in_port)

        if src not in self.mymacs:
            return

        if dst in self.mymacs:
            key = (src, dst)
            if key in self.installed_paths:
                paths = self.installed_paths[key]
            else:
                src_sw, src_port = self.mymacs[src]
                dst_sw, dst_port = self.mymacs[dst]
                paths = self.compute_multipath(src_sw, dst_sw, src_port, dst_port)
                if paths:
                    self.install_multipath(paths, src, dst)
                else:
                    self.logger.warning("[PKT-DROP] %s->%s: no multipath available", src, dst)
                    return

            # Try unicast output port for current switch in first path
            out_port = next((p for sw, _, p in paths[0] if sw == dpid), None)
            if out_port is not None:
                data = msg.data if msg.buffer_id == ofproto.OFP_NO_BUFFER else None
                dp.send_msg(parser.OFPPacketOut(
                    datapath=dp, buffer_id=msg.buffer_id,
                    in_port=in_port,
                    actions=[parser.OFPActionOutput(out_port)],
                    data=data,
                ))
                return

            # Fall back to group action if this switch is the ingress
            group_info = self.flow_groups.get((src, dst))
            if group_info and group_info[0] == dpid:
                data = msg.data if msg.buffer_id == ofproto.OFP_NO_BUFFER else None
                dp.send_msg(parser.OFPPacketOut(
                    datapath=dp, buffer_id=msg.buffer_id,
                    in_port=in_port,
                    actions=[parser.OFPActionGroup(group_info[1])],
                    data=data,
                ))
                return

            self.logger.warning("[PKT-DROP] %s->%s: s%d not in any multipath", src, dst, dpid)
        else:
            self._flood_over_tree(dp, in_port, msg.data, msg.buffer_id)

    def stop(self):
        self.path_cache.clear()
        self.flow_groups.clear()
        super(DijkstraMultipathSwitch, self).stop()


if __name__ == '__main__':
    current_file = os.path.abspath(__file__)
    passthrough_args = sys.argv[1:]
    if '--observe-links' not in passthrough_args:
        passthrough_args = ['--observe-links'] + passthrough_args
    sys.argv = ['dijkstra_multipath_osken_controller', *passthrough_args, current_file]
    from os_ken.cmd.manager import main
    sys.exit(main())
