"""K-Shortest Paths Forwarding Controller using Yen's Algorithm.

Implements Yen's K-shortest paths algorithm for ECMP (Equal-Cost Multi-Path)
and path diversity. Provides load balancing across multiple paths.

Complexity: O(K·V·(E + V log V)) per destination pair
"""

try:
    import warnings
    warnings.simplefilter('ignore', DeprecationWarning)
    import eventlet
    eventlet.monkey_patch()
except ImportError:
    pass

import time
import heapq
import signal
import sys
import hashlib

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

# Configuration
K_SHORTEST_COOKIE = 0x4B53500000000001  # "KSP" in hex
K_SHORTEST_COOKIE_MASK = 0xFFFFFFFFFFFFFFFF
K_SHORTEST_PRIORITY = 100
TABLE_MISS_PRIORITY = 0
HOST_STALE_SECONDS = 300
K_PATHS = 2  # Number of paths to compute (ECMP)
HASH_ROUNDS = 1000000  # Hash space size
GROUP_ID_SPACE = 0x7FFFFFFF


def _handle_sigint(signum, frame):
    """Handle SIGINT (Ctrl+C) by gracefully stopping OSKen."""
    print("\n[SIGNAL] SIGINT received, initiating graceful shutdown...", file=sys.stderr, flush=True)
    app_manager.AppManager.stop()


signal.signal(signal.SIGINT, _handle_sigint)


class KShortestPathsController(app_manager.OSKenApp):
    """K-shortest paths controller with ECMP load balancing using Yen's algorithm."""

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
        super(KShortestPathsController, self).__init__(*args, **kwargs)
        self.datapaths = {}
        self.switches = []
        self.mymacs = {}
        self.host_last_seen = {}
        self.adjacency = defaultdict(list)
        self.port_map = {}
        self.topology_signature = None
        self.installed_paths = {}
        self.broadcast_tree = {}
        self.access_ports = defaultdict(set)
        self.path_cache = {}  # Cache for computed paths
        self.flow_groups = {}  # Maps (src_mac, dst_mac) -> (ingress_dpid, group_id)

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

    def _compute_flow_hash(self, src_mac, dst_mac, src_port, dst_port, eth_type=0x0800):
        """Compute consistent hash for flow classification.
        
        Uses CRC32-like hash for good distribution.
        Returns hash value in range [0, HASH_ROUNDS).
        """
        # Create hash input from flow tuple
        flow_tuple = f"{src_mac}:{dst_mac}:{src_port}:{dst_port}:{eth_type}"
        hash_value = int(hashlib.md5(flow_tuple.encode()).hexdigest(), 16)
        return hash_value % HASH_ROUNDS

    def _alloc_group_id(self, src_mac, dst_mac):
        """Allocate deterministic group-id with collision handling."""
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

    def _select_ecmp_path(self, src_mac, dst_mac, k_paths):
        """Select one path from K paths using consistent hashing.
        
        Args:
            src_mac: Source MAC address
            dst_mac: Destination MAC address
            k_paths: List of K paths (each path is list of (switch, in_port, out_port))
        
        Returns:
            Selected path index (0 to K-1)
        """
        if not k_paths:
            return 0
        
        if len(k_paths) == 1:
            return 0
        
        # Compute hash and select path
        src_port = int(src_mac.split(':')[-1], 16) if src_mac else 0
        dst_port = int(dst_mac.split(':')[-1], 16) if dst_mac else 0
        
        flow_hash = self._compute_flow_hash(src_mac, dst_mac, src_port, dst_port)
        path_index = flow_hash % len(k_paths)
        
        self.logger.debug("[ECMP-SELECT] hash=%d, paths=%d, selected_path=%d",
                         flow_hash, len(k_paths), path_index)
        
        return path_index

    def _install_unicast_flow(self, datapath, in_port, out_port, src_mac, dst_mac):
        """Install unicast flow for a single path segment."""
        parser = datapath.ofproto_parser
        ofproto = datapath.ofproto

        match = parser.OFPMatch(in_port=in_port, eth_src=src_mac, eth_dst=dst_mac)
        actions = [parser.OFPActionOutput(out_port)]
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]

        # Delete existing flow
        delete_mod = parser.OFPFlowMod(
            datapath=datapath,
            cookie=K_SHORTEST_COOKIE,
            cookie_mask=K_SHORTEST_COOKIE_MASK,
            command=ofproto.OFPFC_DELETE_STRICT,
            out_port=ofproto.OFPP_ANY,
            out_group=ofproto.OFPG_ANY,
            priority=K_SHORTEST_PRIORITY,
            match=match,
        )
        datapath.send_msg(delete_mod)

        # Add new flow
        add_mod = parser.OFPFlowMod(
            datapath=datapath,
            cookie=K_SHORTEST_COOKIE,
            command=ofproto.OFPFC_ADD,
            idle_timeout=0,
            hard_timeout=0,
            priority=K_SHORTEST_PRIORITY,
            match=match,
            instructions=inst,
        )
        datapath.send_msg(add_mod)

    def _install_group_flow(self, datapath, in_port, src_mac, dst_mac, group_id):
        """Install ingress flow that forwards matching traffic to SELECT group."""
        parser = datapath.ofproto_parser
        ofproto = datapath.ofproto

        match = parser.OFPMatch(in_port=in_port, eth_src=src_mac, eth_dst=dst_mac)
        actions = [parser.OFPActionGroup(group_id)]
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]

        delete_mod = parser.OFPFlowMod(
            datapath=datapath,
            cookie=K_SHORTEST_COOKIE,
            cookie_mask=K_SHORTEST_COOKIE_MASK,
            command=ofproto.OFPFC_DELETE_STRICT,
            out_port=ofproto.OFPP_ANY,
            out_group=ofproto.OFPG_ANY,
            priority=K_SHORTEST_PRIORITY,
            match=match,
        )
        datapath.send_msg(delete_mod)

        add_mod = parser.OFPFlowMod(
            datapath=datapath,
            cookie=K_SHORTEST_COOKIE,
            command=ofproto.OFPFC_ADD,
            idle_timeout=0,
            hard_timeout=0,
            priority=K_SHORTEST_PRIORITY,
            match=match,
            instructions=inst,
        )
        datapath.send_msg(add_mod)

    def _install_select_group(self, datapath, group_id, out_ports):
        """Install or replace OpenFlow SELECT group with one bucket per next-hop port."""
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

    def _delete_ksp_groups(self, datapath):
        """Delete all ECMP groups installed by this controller on a datapath."""
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

    def _purge_stale_hosts(self):
        """Remove hosts that haven't been seen recently."""
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
        self._flush_all_ksp_flows()
        self.mymacs.clear()
        self.host_last_seen.clear()
        self.installed_paths.clear()
        self.broadcast_tree.clear()
        self.datapaths.clear()
        self.path_cache.clear()
        self.flow_groups.clear()
        super(KShortestPathsController, self).stop()
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

    def _delete_ksp_flows(self, datapath):
        """Delete all K-shortest flows from a switch."""
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        mod = parser.OFPFlowMod(
            datapath=datapath,
            cookie=K_SHORTEST_COOKIE,
            cookie_mask=K_SHORTEST_COOKIE_MASK,
            command=ofproto.OFPFC_DELETE,
            out_port=ofproto.OFPP_ANY,
            out_group=ofproto.OFPG_ANY,
            match=parser.OFPMatch(),
        )
        datapath.send_msg(mod)

    def _flush_all_ksp_flows(self):
        """Delete all K-shortest flows from all switches."""
        for datapath in self.datapaths.values():
            self._delete_ksp_flows(datapath)
            self._delete_ksp_groups(datapath)
        self.installed_paths.clear()
        self.flow_groups.clear()

    def _reinstall_all_known_routes(self):
        """Reinstall all known routes after topology change."""
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
                    
                k_paths = self.compute_k_shortest_paths(src_loc[0], dst_loc[0], src_loc[1], dst_loc[1], K=K_PATHS)
                if k_paths:
                    self.install_k_paths(k_paths, src_mac, dst_mac)
                    installed += 1
                else:
                    unreachable += 1
                    self.logger.warning("[ROUTE-INSTALL] %s->%s: no paths found (s%d->s%d)",
                                        src_mac, dst_mac, src_loc[0], dst_loc[0])
        
        self.logger.info("[TOPO] proactive route refresh: installed=%d, skipped=%d, unreachable=%d, active_hosts=%d",
                         installed, skipped, unreachable, len(known_hosts))

    def _dijkstra_spf_heap(self, src, dst):
        """Dijkstra SPF with O((V+E) log V) complexity. Returns (distance_dict, previous_dict)."""
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
        pq = [(0, src)]
        visited = set()
        
        self.logger.debug("[SPF-INIT] computing shortest path: src=s%d, dst=s%d", src, dst)
        
        while pq:
            d, u = heapq.heappop(pq)
            
            if u in visited:
                continue
            visited.add(u)
            
            if d > distance[u]:
                continue
            
            for v, out_port in self.adjacency[u]:
                alt = distance[u] + 1
                
                if alt < distance[v]:
                    distance[v] = alt
                    previous[v] = u
                    heapq.heappush(pq, (alt, v))
        
        return distance, previous

    def yen_k_shortest_paths(self, src, dst, k):
        """Yen's K-shortest paths algorithm (simplified for equal-cost paths).
        
        For ECMP, we find K shortest paths by finding disjoint or equal-cost paths.
        This simplified version finds the first shortest path, then uses modified Dijkstra
        to find K-1 additional equal-cost or nearly equal-cost paths.
        
        Args:
            src: Source switch DPID
            dst: Destination switch DPID
            k: Number of paths to find
        
        Returns:
            List of K paths, each path is list of (switch, in_port, out_port)
        """
        if src == dst:
            return []
        
        # Find first shortest path using Dijkstra
        distance, previous = self._dijkstra_spf_heap(src, dst)
        
        if distance.get(dst, float('inf')) == float('inf'):
            return []
        
        # Reconstruct first path
        A = []  # List of K shortest paths
        first_cost = distance.get(dst, float('inf'))
        
        # Reconstruct first path
        path_nodes = [dst]
        current = previous.get(dst)
        while current is not None:
            path_nodes.append(current)
            if current == src:
                break
            current = previous.get(current)
        path_nodes.reverse()
        
        if path_nodes and len(path_nodes) >= 2:
            first_path = self._build_path_with_ports(path_nodes, src, dst)
            if first_path:
                A.append((len(first_path), first_path))
        
        if len(A) >= k:
            return [path for _, path in A[:k]]
        
        # For ECMP, find additional equal-cost or least-cost disjoint paths
        # Use a simpler approach: find all alternative paths by removing each edge from first path
        all_candidate_paths = []
        
        for i in range(len(path_nodes) - 1):
            # Try removing each edge from first path
            removed_edge = (path_nodes[i], path_nodes[i + 1])
            
            # Save original adjacency
            saved_adjacency = {}
            saved_adjacency[removed_edge[0]] = self.adjacency[removed_edge[0]][:]
            saved_adjacency[removed_edge[1]] = self.adjacency[removed_edge[1]][:]
            
            # Remove edge
            self.adjacency[removed_edge[0]] = [
                (n, p) for n, p in self.adjacency[removed_edge[0]] if n != removed_edge[1]
            ]
            self.adjacency[removed_edge[1]] = [
                (n, p) for n, p in self.adjacency[removed_edge[1]] if n != removed_edge[0]
            ]
            
            # Compute alternative path
            alt_distance, alt_previous = self._dijkstra_spf_heap(src, dst)
            
            if alt_distance.get(dst, float('inf')) != float('inf'):
                # Reconstruct path
                alt_path_nodes = [dst]
                current = alt_previous.get(dst)
                while current is not None:
                    alt_path_nodes.append(current)
                    if current == src:
                        break
                    current = alt_previous.get(current)
                alt_path_nodes.reverse()
                
                if alt_path_nodes and len(alt_path_nodes) >= 2:
                    alt_path = self._build_path_with_ports(alt_path_nodes, src, dst)
                    if alt_path and not self._path_in_list(alt_path, all_candidate_paths):
                        all_candidate_paths.append((alt_distance.get(dst), alt_path))
            
            # Restore adjacency
            self.adjacency[removed_edge[0]] = saved_adjacency[removed_edge[0]]
            self.adjacency[removed_edge[1]] = saved_adjacency[removed_edge[1]]
        
        # Sort by cost and add to result
        all_candidate_paths.sort(key=lambda x: x[0])
        for cost, path in all_candidate_paths:
            if len(A) < k:
                A.append((cost, path))
        
        return [path for _, path in A[:k]]
    
    def _path_in_list(self, path, path_list):
        """Check if a path is already in the list."""
        for _, existing_path in path_list:
            if self._paths_are_equal(path, existing_path):
                return True
        return False

    def _build_path_with_ports(self, path_nodes, src, dst):
        """Build path with port numbers from list of switch nodes.
        
        Returns list of (switch_dpid, in_port, out_port) tuples.
        Note: in_port and out_port will be set by caller for first/last ports.
        """
        if len(path_nodes) < 2:
            return []
        
        result = []
        
        # Build path through all nodes
        for i in range(len(path_nodes)):
            current_node = path_nodes[i]
            
            if i == 0:
                # First node: out_port goes to next switch
                next_node = path_nodes[i + 1]
                out_port = self._get_port(current_node, next_node)
                if out_port is None:
                    self.logger.error("[PATH-BUILD] s%d->s%d: port mapping missing", current_node, next_node)
                    return None
                # in_port will be set by caller
                result.append((current_node, 0, out_port))
            
            elif i == len(path_nodes) - 1:
                # Last node: in_port comes from previous switch
                prev_node = path_nodes[i - 1]
                in_port = self._get_port(current_node, prev_node)
                if in_port is None:
                    self.logger.error("[PATH-BUILD] s%d->s%d: reverse port mapping missing", prev_node, current_node)
                    return None
                # out_port will be set by caller
                result.append((current_node, in_port, 0))
            
            else:
                # Middle nodes: both in_port and out_port
                prev_node = path_nodes[i - 1]
                next_node = path_nodes[i + 1]
                
                in_port = self._get_port(current_node, prev_node)
                out_port = self._get_port(current_node, next_node)
                
                if in_port is None or out_port is None:
                    self.logger.error("[PATH-BUILD] s%d: port mappings missing", current_node)
                    return None
                
                result.append((current_node, in_port, out_port))
        
        return result

    def _paths_are_equal(self, path1, path2):
        """Check if two paths are identical (same sequence of switches)."""
        if len(path1) != len(path2):
            return False
        for p1, p2 in zip(path1, path2):
            if p1[0] != p2[0]:
                return False
        return True

    def compute_k_shortest_paths(self, src, dst, first_port, final_port, K=K_PATHS):
        """Compute K shortest paths from src to dst.
        
        Args:
            src: Source switch DPID
            dst: Destination switch DPID
            first_port: Input port at source switch
            final_port: Output port at destination switch
            K: Number of paths to compute
        
        Returns:
            List of K paths, each path is list of (switch, in_port, out_port)
        """
        if src not in self.switches or dst not in self.switches:
            self.logger.warning("[PATH-QUERY] invalid endpoints: src=s%d, dst=s%d", src, dst)
            return []

        # Same-switch forwarding does not need inter-switch path computation.
        if src == dst:
            self.logger.debug("[PATH-QUERY] local path on s%d (in_port=%d, out_port=%d)",
                              src, first_port, final_port)
            return [[(src, first_port, final_port)]]
        
        # Check cache
        cache_key = (src, dst, K)
        if cache_key in self.path_cache:
            self.logger.debug("[PATH-CACHE] hit for s%d->s%d, K=%d", src, dst, K)
            base_paths = self.path_cache[cache_key]
        else:
            self.logger.debug("[PATH-QUERY] computing K=%d shortest paths: s%d -> s%d", K, src, dst)
            base_paths = self.yen_k_shortest_paths(src, dst, K)
            self.path_cache[cache_key] = base_paths
            self.logger.info("[PATH-COMPUTED] s%d->s%d: computed %d base paths", src, dst, len(base_paths))
        
        # Build host-specific paths from immutable switch-level base paths.
        decorated_paths = []
        for base_path in base_paths[:K]:
            if not base_path:
                continue

            path = list(base_path)
            path[0] = (path[0][0], first_port, path[0][2])
            path[-1] = (path[-1][0], path[-1][1], final_port)
            decorated_paths.append(path)

        return decorated_paths

    def install_k_paths(self, k_paths, src_mac, dst_mac):
        """Install K paths using OpenFlow SELECT group on ingress switch."""
        if not k_paths:
            return

        cache_key = (src_mac, dst_mac)
        if self.installed_paths.get(cache_key) == k_paths:
            return
        
        self.logger.info("[ECMP-INSTALL] installing %d paths for %s->%s", len(k_paths), src_mac, dst_mac)

        ingress_dpid = k_paths[0][0][0]
        ingress_in_port = k_paths[0][0][1]
        ingress_out_ports = [path[0][2] for path in k_paths if path and path[0][0] == ingress_dpid]

        ingress_dp = self.datapaths.get(ingress_dpid)
        if ingress_dp is None:
            self.logger.warning("[ECMP-INSTALL] ingress switch s%d datapath unavailable", ingress_dpid)
            return
        
        for path in k_paths:
            if not path:
                continue
            
            self.logger.debug("[ECMP-PATH] %s", self._format_path(path))
            
            # Ingress hop is handled by SELECT group; install fixed forwarding on remaining hops.
            for switch_dpid, in_port, out_port in path[1:]:
                if switch_dpid in self.datapaths:
                    self._install_unicast_flow(
                        self.datapaths[switch_dpid],
                        in_port,
                        out_port,
                        src_mac,
                        dst_mac
                    )

        group_id = self._alloc_group_id(src_mac, dst_mac)
        if self._install_select_group(ingress_dp, group_id, ingress_out_ports):
            self.flow_groups[(src_mac, dst_mac)] = (ingress_dpid, group_id)
            self._install_group_flow(ingress_dp, ingress_in_port, src_mac, dst_mac, group_id)
        
        self.installed_paths[cache_key] = k_paths

    def _build_broadcast_tree(self):
        """Build spanning tree rooted at minimum DPID switch."""
        if not self.switches:
            self.broadcast_tree = {}
            return

        root = min(self.switches)
        distance, previous = self._dijkstra_spf_heap(root, root)

        tree = defaultdict(set)
        for node in self.switches:
            parent = previous[node]
            if parent is not None:
                tree[parent].add(node)
                tree[node].add(parent)

        self.broadcast_tree = {node: sorted(list(neighbors)) for node, neighbors in tree.items()}

    def _flood_over_tree(self, datapath, in_port, data, buffer_id):
        """Flood packet over broadcast tree."""
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

    @set_ev_cls(ofp_event.EventOFPSwitchFeatures, CONFIG_DISPATCHER)
    def switch_features_handler(self, ev):
        """Install table-miss flow entry to send all packets to controller."""
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

    @set_ev_cls(ofp_event.EventOFPErrorMsg, [CONFIG_DISPATCHER, MAIN_DISPATCHER])
    def _ofp_error_handler(self, ev):
        """Log OpenFlow error replies from switches (e.g. failed group mods)."""
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
        """Handle packet-in events with K-shortest paths forwarding."""
        msg = ev.msg
        datapath = msg.datapath
        in_port = msg.match['in_port']
        data = msg.data
        
        parser = datapath.ofproto_parser
        ofproto = datapath.ofproto
        
        # Parse packet
        try:
            pkt = packet.Packet(data)
            eth = pkt.get_protocols(ethernet.ethernet)[0]
            
            # Ignore LLDP packets
            if eth.ethertype == ether_types.ETH_TYPE_LLDP:
                return
            
            src_mac = eth.src
            dst_mac = eth.dst
            
            # Only learn hosts on access ports (skip inter-switch ports)
            if self._is_access_port(datapath.id, in_port):
                # Learn source MAC from ANY packet type received on access port
                self._update_host_location(src_mac, datapath.id, in_port)
            
            # Drop packet if source is unknown (never learned on any access port)
            if src_mac not in self.mymacs:
                self.logger.debug("[PKT-DROP] s%d: source %s not learned on any access port",
                                 datapath.id, src_mac)
                return
            
            if eth.ethertype == ether_types.ETH_TYPE_IP:
                # IPv4 packet - attempt to forward
                
                # Check if destination is known
                dst_loc = self.mymacs.get(dst_mac)
                if dst_loc:
                    # Destination known: compute K paths and forward
                    src_loc = self.mymacs.get(src_mac)
                    if src_loc:
                        k_paths = self.compute_k_shortest_paths(
                            src_loc[0], dst_loc[0], in_port, dst_loc[1]
                        )
                        
                        if k_paths:
                            self.install_k_paths(k_paths, src_mac, dst_mac)

                            # Send the current packet over a concrete path to avoid
                            # first-packet races while multi-switch rules are being applied.
                            selected_path = k_paths[0]
                            for switch_dpid, _, port_out in selected_path:
                                if switch_dpid == datapath.id:
                                    data_arg = None if msg.buffer_id != ofproto.OFP_NO_BUFFER else data
                                    out = parser.OFPPacketOut(
                                        datapath=datapath,
                                        buffer_id=msg.buffer_id,
                                        in_port=in_port,
                                        actions=[parser.OFPActionOutput(port_out)],
                                        data=data_arg,
                                    )
                                    datapath.send_msg(out)
                                    self.logger.debug("[PKT-FWD] s%d: immediate forwarding %s->%s on port %d",
                                                      datapath.id, src_mac, dst_mac, port_out)
                                    return

                            group_info = self.flow_groups.get((src_mac, dst_mac))
                            if group_info is not None and group_info[0] == datapath.id:
                                data_arg = None if msg.buffer_id != ofproto.OFP_NO_BUFFER else data
                                out = parser.OFPPacketOut(
                                    datapath=datapath,
                                    buffer_id=msg.buffer_id,
                                    in_port=in_port,
                                    actions=[parser.OFPActionGroup(group_info[1])],
                                    data=data_arg,
                                )
                                datapath.send_msg(out)
                                self.logger.debug("[PKT-FWD] s%d: fallback group forwarding %s->%s via group %d",
                                                  datapath.id, src_mac, dst_mac, group_info[1])
                                return
            
            # Destination unknown: flood over broadcast tree
            self._flood_over_tree(datapath, in_port, data, msg.buffer_id)
            self.logger.debug("[PKT-FLOOD] s%d: flooding unknown destination %s", datapath.id, dst_mac)
            
        except Exception as e:
            self.logger.error("[PKT-ERROR] failed to process packet: %s", str(e))

    @set_ev_cls(TOPOLOGY_EVENTS)
    def get_topology_data(self, ev):
        """Unified topology event handler that gets complete topology snapshots.
        
        Instead of handling individual events, we query the complete topology
        and compare old vs new signatures. This provides atomic topology updates.
        """
        switch_list = get_switch(self, None)
        switch_ids = sorted(switch.dp.id for switch in switch_list)
        links_list = get_link(self, None)
        new_mylinks = sorted(
            (link.src.dpid, link.dst.dpid, link.src.port_no, link.dst.port_no)
            for link in links_list
        )

        # Ignore transient empty topology snapshots
        if not new_mylinks:
            if self.topology_signature is not None:
                self.logger.debug("[TOPO] ignoring transient empty topology snapshot")
            return

        self.switches = switch_ids
        self.datapaths = {switch.dp.id: switch.dp for switch in switch_list}

        # Build inter-switch ports in single pass: O(E) not O(N*E)
        inter_switch_ports = defaultdict(set)
        for s1, s2, port1, port2 in new_mylinks:
            inter_switch_ports[s1].add(port1)
            inter_switch_ports[s2].add(port2)
        
        # Compute access ports: all ports minus inter-switch ports
        self.access_ports = defaultdict(set)
        for switch in switch_list:
            all_ports = {port.port_no for port in getattr(switch, 'ports', [])}
            self.access_ports[switch.dp.id] = all_ports - inter_switch_ports[switch.dp.id]

        # Build new adjacency and port maps
        new_adjacency = defaultdict(list)
        new_port_map = {}
        for s1, s2, port1, port2 in new_mylinks:
            new_adjacency[s1].append((s2, port1))
            new_adjacency[s2].append((s1, port2))
            new_port_map[(s1, s2)] = port1
            new_port_map[(s2, s1)] = port2

        # Create topology signature for comparison
        old_signature = self.topology_signature
        new_signature = (tuple(sorted(self.switches)), tuple(sorted(new_mylinks)))
        
        self.adjacency = new_adjacency
        self.port_map = new_port_map

        # Handle topology changes
        if old_signature != new_signature:
            if old_signature is not None:
                # Topology changed: compute delta
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
                
                # Purge hosts on departed switches
                self._purge_hosts_on_departed_switches()
                
                # Clear path cache and reinstall routes
                self.logger.info("[TOPO-REFRESH] topology changed, flushing %d installed flows and reinstalling routes",
                                 len(self.installed_paths))
                self._flush_all_ksp_flows()
                self.path_cache.clear()
                self.installed_paths.clear()
                self.flow_groups.clear()
                self._reinstall_all_known_routes()
            else:
                # Initial topology discovery
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
    sys.argv = ['kshortest_osken_controller', *passthrough_args, current_file]

    from os_ken.cmd.manager import main
    sys.exit(main())
