"""A* multipath OpenFlow controller with ECMP group forwarding.

Computes equal-cost shortest paths using A* with parent tracking, then installs
an OpenFlow SELECT group on ingress switches for multipath load sharing.
"""

import hashlib
import heapq

from collections import defaultdict

from os_ken.controller import ofp_event
from os_ken.controller.handler import CONFIG_DISPATCHER, MAIN_DISPATCHER
from os_ken.controller.handler import set_ev_cls
from os_ken.lib.packet import ethernet
from os_ken.lib.packet import ether_types
from os_ken.lib.packet import packet

from astar_osken_controller import ASTAR_FLOW_COOKIE
from astar_osken_controller import ASTAR_FLOW_COOKIE_MASK
from astar_osken_controller import ASTAR_FLOW_PRIORITY
from astar_osken_controller import AStarSwitch

K_PATHS = 4
GROUP_ID_SPACE = 0x7FFFFFFF


class AStarMultipathSwitch(AStarSwitch):
    """A* SPF with equal-cost multipath forwarding."""

    def __init__(self, *args, **kwargs):
        super(AStarMultipathSwitch, self).__init__(*args, **kwargs)
        self.path_cache = {}
        self.flow_groups = {}  # (src_mac, dst_mac) -> (ingress_dpid, group_id)

    def _alloc_group_id(self, src_mac, dst_mac):
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
            candidate += 1
            if candidate > GROUP_ID_SPACE:
                candidate = 1

        return candidate

    def _install_group_flow(self, datapath, in_port, src_mac, dst_mac, group_id):
        parser = datapath.ofproto_parser
        ofproto = datapath.ofproto

        match = parser.OFPMatch(in_port=in_port, eth_src=src_mac, eth_dst=dst_mac)
        actions = [parser.OFPActionGroup(group_id)]
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

    def _install_select_group(self, datapath, group_id, out_ports):
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
                actions=[parser.OFPActionOutput(port_no)],
            )
            for port_no in unique_ports
        ]

        delete_group = parser.OFPGroupMod(
            datapath=datapath,
            command=ofproto.OFPGC_DELETE,
            type_=ofproto.OFPGT_SELECT,
            group_id=group_id,
            buckets=[],
        )
        datapath.send_msg(delete_group)

        add_group = parser.OFPGroupMod(
            datapath=datapath,
            command=ofproto.OFPGC_ADD,
            type_=ofproto.OFPGT_SELECT,
            group_id=group_id,
            buckets=buckets,
        )
        datapath.send_msg(add_group)

        self.logger.debug("[ECMP-GROUP] s%d group=%d buckets=%s", datapath.id, group_id, unique_ports)
        return True

    def _delete_multipath_groups(self, datapath):
        parser = datapath.ofproto_parser
        ofproto = datapath.ofproto

        group_ids = {group_id for dpid, group_id in self.flow_groups.values() if dpid == datapath.id}
        for group_id in group_ids:
            mod = parser.OFPGroupMod(
                datapath=datapath,
                command=ofproto.OFPGC_DELETE,
                type_=ofproto.OFPGT_SELECT,
                group_id=group_id,
                buckets=[],
            )
            datapath.send_msg(mod)

    def _flush_all_astar_flows(self):
        for datapath in self.datapaths.values():
            self._delete_astar_flows(datapath)
            self._delete_multipath_groups(datapath)
        self.installed_paths.clear()
        self.path_cache.clear()
        self.flow_groups.clear()

    def _a_star_multi_parent(self, src, dst):
        """Run A* while tracking all equal-cost predecessors."""
        if src not in self.adjacency and src not in self.switches:
            return {}, defaultdict(set)

        distance = {dpid: float('inf') for dpid in self.switches}
        parents = defaultdict(set)

        for u in self.adjacency:
            distance.setdefault(u, float('inf'))
            for v, _ in self.adjacency[u]:
                distance.setdefault(v, float('inf'))

        distance[src] = 0
        src_h = self._heuristic_hops(src, dst)

        if src != dst and src_h == float('inf'):
            return distance, parents

        open_heap = [(src_h, 0, src)]
        best_goal_cost = float('inf')
        edge_exams = 0
        relaxations = 0
        expanded = 0

        self.logger.debug("[A*-MP-INIT] src=s%d dst=s%d vertices=%d edges=%d",
                          src, dst, len(distance), sum(len(neighbors) for neighbors in self.adjacency.values()))

        while open_heap:
            f_score, g_score, u = heapq.heappop(open_heap)
            if g_score > distance[u]:
                continue
            if f_score > best_goal_cost:
                break

            expanded += 1
            if u == dst:
                best_goal_cost = min(best_goal_cost, g_score)
                continue

            for v, _ in self.adjacency.get(u, []):
                edge_exams += 1
                tentative_g = g_score + 1
                if tentative_g > best_goal_cost:
                    continue

                if tentative_g < distance[v]:
                    relaxations += 1
                    distance[v] = tentative_g
                    parents[v] = {u}

                    h = self._heuristic_hops(v, dst)
                    if h != float('inf'):
                        heapq.heappush(open_heap, (tentative_g + h, tentative_g, v))
                elif tentative_g == distance[v]:
                    if u not in parents[v]:
                        relaxations += 1
                        parents[v].add(u)

                    h = self._heuristic_hops(v, dst)
                    if h != float('inf'):
                        heapq.heappush(open_heap, (tentative_g + h, tentative_g, v))

        self.logger.info("[A*-MP-DONE] s%d->s%d expanded=%d edge_exams=%d relaxations=%d shortest_cost=%s",
                         src, dst, expanded, edge_exams, relaxations,
                         "inf" if distance.get(dst, float('inf')) == float('inf') else int(distance[dst]))

        return distance, parents

    def _enumerate_shortest_node_paths(self, src, dst, parents, limit):
        if src == dst:
            return [[src]]

        if dst not in parents:
            return []

        results = []
        stack = [(dst, [dst])]

        while stack and len(results) < limit:
            node, reverse_path = stack.pop()
            if node == src:
                results.append(list(reversed(reverse_path)))
                continue

            for parent in sorted(parents.get(node, set()), reverse=True):
                if parent in reverse_path:
                    continue
                stack.append((parent, reverse_path + [parent]))

        return results

    def _decorate_node_path(self, node_path, first_port, final_port):
        if not node_path:
            return []

        if len(node_path) == 1:
            return [(node_path[0], first_port, final_port)]

        result = []
        in_port = first_port

        for s1, s2 in zip(node_path[:-1], node_path[1:]):
            out_port = self._get_port(s1, s2)
            if out_port is None:
                self.logger.error("[PATH-PORTMAP] s%d->s%d missing forward port", s1, s2)
                return []

            result.append((s1, in_port, out_port))
            in_port = self._get_port(s2, s1)
            if in_port is None and s2 != node_path[-1]:
                self.logger.error("[PATH-PORTMAP] s%d->s%d missing reverse port", s2, s1)
                return []

        result.append((node_path[-1], in_port, final_port))
        return result

    def compute_multipath(self, src, dst, first_port, final_port, k_paths=K_PATHS):
        if src not in self.switches or dst not in self.switches:
            self.logger.warning("[PATH-QUERY] invalid endpoints: src=s%d, dst=s%d", src, dst)
            return []

        if src == dst:
            return [[(src, first_port, final_port)]]

        cache_key = (src, dst, k_paths)
        if cache_key in self.path_cache:
            base_node_paths = self.path_cache[cache_key]
            self.logger.debug("[PATH-CACHE] hit for s%d->s%d K=%d", src, dst, k_paths)
        else:
            distance, parents = self._a_star_multi_parent(src, dst)
            if distance.get(dst, float('inf')) == float('inf'):
                return []

            base_node_paths = self._enumerate_shortest_node_paths(src, dst, parents, k_paths)
            self.path_cache[cache_key] = base_node_paths
            self.logger.info("[PATH-COMPUTED] s%d->s%d equal-cost paths=%d shortest_cost=%d",
                             src, dst, len(base_node_paths), int(distance[dst]))

        decorated = []
        for node_path in base_node_paths[:k_paths]:
            path = self._decorate_node_path(node_path, first_port, final_port)
            if path:
                decorated.append(path)

        return decorated

    def install_multipath(self, paths, src_mac, dst_mac):
        if not paths:
            return

        cache_key = (src_mac, dst_mac)
        if self.installed_paths.get(cache_key) == paths:
            return

        ingress_dpid = paths[0][0][0]
        ingress_in_port = paths[0][0][1]
        ingress_out_ports = [path[0][2] for path in paths if path and path[0][0] == ingress_dpid]

        ingress_dp = self.datapaths.get(ingress_dpid)
        if ingress_dp is None:
            self.logger.warning("[ECMP-INSTALL] ingress switch s%d datapath unavailable", ingress_dpid)
            return

        for path in paths:
            for switch_dpid, in_port, out_port in path[1:]:
                datapath = self.datapaths.get(switch_dpid)
                if datapath is not None:
                    self._install_unicast_flow(datapath, in_port, out_port, src_mac, dst_mac)

        group_id = self._alloc_group_id(src_mac, dst_mac)
        if self._install_select_group(ingress_dp, group_id, ingress_out_ports):
            self.flow_groups[(src_mac, dst_mac)] = (ingress_dpid, group_id)
            self._install_group_flow(ingress_dp, ingress_in_port, src_mac, dst_mac, group_id)
            self.logger.info("[ECMP-INSTALL] %s->%s paths=%d ingress=s%d group=%d",
                             src_mac, dst_mac, len(paths), ingress_dpid, group_id)
            self.installed_paths[cache_key] = paths

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

                paths = self.compute_multipath(src_loc[0], dst_loc[0], src_loc[1], dst_loc[1], K_PATHS)
                if paths:
                    self.install_multipath(paths, src_mac, dst_mac)
                    installed += 1
                else:
                    unreachable += 1
                    self.logger.warning("[ROUTE-INSTALL] %s->%s: no path found (s%d->s%d)",
                                        src_mac, dst_mac, src_loc[0], dst_loc[0])

        self.logger.info("[TOPO] proactive route refresh: installed=%d, skipped=%d, unreachable=%d, active_hosts=%d",
                         installed, skipped, unreachable, len(known_hosts))

    @set_ev_cls(ofp_event.EventOFPErrorMsg, [CONFIG_DISPATCHER, MAIN_DISPATCHER])
    def _ofp_error_handler(self, ev):
        msg = ev.msg
        dp = getattr(msg, 'datapath', None)
        dpid = dp.id if dp else -1
        data_hex = msg.data.hex() if isinstance(msg.data, (bytes, bytearray)) else str(msg.data)
        if len(data_hex) > 120:
            data_hex = data_hex[:120] + "..."
        self.logger.error("[OFP-ERROR] s%d type=0x%02x code=0x%02x data=%s",
                          dpid, msg.type, msg.code, data_hex)

    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def _packet_in_handler(self, ev):
        msg = ev.msg
        datapath = msg.datapath
        parser = datapath.ofproto_parser
        ofproto = datapath.ofproto
        in_port = msg.match['in_port']

        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocol(ethernet.ethernet)
        if eth.ethertype == ether_types.ETH_TYPE_LLDP:
            return

        src_mac = eth.src
        dst_mac = eth.dst
        dpid = datapath.id

        if self._is_access_port(dpid, in_port):
            self._update_host_location(src_mac, dpid, in_port)

        if src_mac not in self.mymacs:
            self.logger.debug("[PKT-DROP] %s -> %s: source host not learned on an access port", src_mac, dst_mac)
            return

        if dst_mac in self.mymacs:
            if (src_mac, dst_mac) in self.installed_paths:
                paths = self.installed_paths[(src_mac, dst_mac)]
            else:
                src_sw, src_port = self.mymacs[src_mac]
                dst_sw, dst_port = self.mymacs[dst_mac]
                paths = self.compute_multipath(src_sw, dst_sw, src_port, dst_port, K_PATHS)
                if paths:
                    self.install_multipath(paths, src_mac, dst_mac)
                else:
                    self.logger.warning("[PKT-DROP] %s -> %s: no path available in current topology", src_mac, dst_mac)
                    return

            out_port = None
            selected_path = paths[0]
            for switch_dpid, _, port_out in selected_path:
                if switch_dpid == dpid:
                    out_port = port_out
                    break

            if out_port is not None:
                data = None if msg.buffer_id != ofproto.OFP_NO_BUFFER else msg.data
                out = parser.OFPPacketOut(
                    datapath=datapath,
                    buffer_id=msg.buffer_id,
                    in_port=in_port,
                    actions=[parser.OFPActionOutput(out_port)],
                    data=data,
                )
                datapath.send_msg(out)
                return

            group_info = self.flow_groups.get((src_mac, dst_mac))
            if group_info is not None and group_info[0] == dpid:
                data = None if msg.buffer_id != ofproto.OFP_NO_BUFFER else msg.data
                out = parser.OFPPacketOut(
                    datapath=datapath,
                    buffer_id=msg.buffer_id,
                    in_port=in_port,
                    actions=[parser.OFPActionGroup(group_info[1])],
                    data=data,
                )
                datapath.send_msg(out)
                return

            self.logger.warning("[PKT-DROP] %s -> %s: current switch s%d not present in selected multipath",
                                src_mac, dst_mac, dpid)
            return

        self._flood_over_tree(datapath, in_port, msg.data, msg.buffer_id)

    def stop(self):
        self.path_cache.clear()
        self.flow_groups.clear()
        super(AStarMultipathSwitch, self).stop()


if __name__ == '__main__':
    import os
    import sys

    current_file = os.path.abspath(__file__)
    passthrough_args = sys.argv[1:]
    if '--observe-links' not in passthrough_args:
        passthrough_args = ['--observe-links'] + passthrough_args
    sys.argv = ['astar_multipath_osken_controller', *passthrough_args, current_file]

    from os_ken.cmd.manager import main
    sys.exit(main())
