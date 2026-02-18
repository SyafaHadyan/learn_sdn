"""Dijkstra SPF controller (OS-Ken).
This controller implements Dijkstra's algorithm to find the shortest path between two hosts in a network. It listens for packet-in events and topology changes, and updates the flow tables of the switches accordingly.
The controller maintains a list of known switches, a mapping of host MAC addresses to their corresponding switch and port, and an adjacency map of the switches. When a packet-in event occurs, it checks
"""

from os_ken.base import app_manager
from os_ken.controller import ofp_event
from os_ken.controller.handler import CONFIG_DISPATCHER, MAIN_DISPATCHER
from os_ken.controller.handler import set_ev_cls
from os_ken.ofproto import ofproto_v1_3
from os_ken.lib.mac import haddr_to_bin
from os_ken.lib.packet import packet
from os_ken.lib.packet import ethernet
from os_ken.lib.packet import ether_types
from os_ken.lib import mac
from os_ken.topology.api import get_switch, get_link
from os_ken.topology import event, switches
from collections import defaultdict

# switches
switches = []

# mymacs[srcmac]->(switch, port)
mymacs = {}

# adjacency map [sw1][sw2]->port from sw1 to sw2
adjacency = defaultdict(lambda:defaultdict(lambda:None))


# getting the node with lowest distance in Q
def minimum_distance(distance, Q):
    node = None
    min_val = float('Inf')
    for v in Q:
        d = distance.get(v, float('Inf'))
        if d < min_val:
            min_val = d
            node = v
    return node

def get_path (src, dst, first_port, final_port):
    # executing Dijkstra's algorithm (silent)
    
    # defining dictionaries for saving each node's distance and its previous node in the path from first node to that node
    distance = {}
    previous = {}

    # setting initial distance of every node to infinity
    for dpid in switches:
        distance[dpid] = float('Inf')
        previous[dpid] = None

    # setting distance of the source to 0
    distance[src] = 0

    # creating a set of all nodes
    Q = set(switches)

    # checking for all undiscovered nodes whether there is a path that goes through them to their adjacent nodes which will make its adjacent nodes closer to src
    while len(Q) > 0:
        # getting the closest node to src among undiscovered nodes
        u = minimum_distance(distance, Q)
        if u is None:
            break
        # removing the node from Q
        Q.remove(u)
        # calculate minimum distance for all adjacent nodes to u
        for p in switches:
            # if u and other switches are adjacent
            if adjacency[u][p] != None:
                # setting the weight to 1 so that we count the number of routers in the path
                w = 1
                # if the path via u to p has lower cost then make the cost equal to this new path's cost
                if distance[u] + w < distance[p]:
                    distance[p] = distance[u] + w
                    previous[p] = u

    # creating a list of switches between src and dst which are in the shortest path obtained by Dijkstra's algorithm reversely
    r = []
    p = dst
    r.append(p)
    # set q to the last node before dst 
    q = previous[p]
    while q is not None:
        if q == src:
            r.append(q)
            break
        p = q
        r.append(p)
        q = previous[p]

    # reversing r as it was from dst to src
    r.reverse()

    # setting path 
    if src == dst:
        path=[src]
    else:
        path=r

    # Now adding in_port and out_port to the path
    r = []
    in_port = first_port
    for s1, s2 in zip(path[:-1], path[1:]):
        out_port = adjacency[s1][s2]
        r.append((s1, in_port, out_port))
        in_port = adjacency[s2][s1]
    r.append((dst, in_port, final_port))
    return r

class DijkstraSwitch(app_manager.OSKenApp):
    OFP_VERSIONS = [ofproto_v1_3.OFP_VERSION]

    def __init__(self, *args, **kwargs):
        super(DijkstraSwitch, self).__init__(*args, **kwargs)
        self.topology_api_app = self
        self.datapath_list = []
        self.switches = []
        self.mymacs = {}
        # reference to module-level adjacency structure
        self.adjacency = adjacency
        self.mylinks = []
        # keep last installed paths to avoid noisy re-install logging
        self.installed_paths = {}

    def minimum_distance(self, distance, Q):
        node = None
        min_val = float('Inf')
        for v in Q:
            d = distance.get(v, float('Inf'))
            if d < min_val:
                min_val = d
                node = v
        return node

    def compute_path(self, src, dst, first_port, final_port):
        # Dijkstra using current topology stored in self.switches and self.adjacency (silent)
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
            for p in self.switches:
                if self.adjacency[u][p] is not None:
                    w = 1
                    if distance[u] + w < distance[p]:
                        distance[p] = distance[u] + w
                        previous[p] = u

        # build path from src to dst
        r = []
        p = dst
        r.append(p)
        q = previous.get(p)
        while q is not None:
            if q == src:
                r.append(q)
                break
            p = q
            r.append(p)
            q = previous.get(p)

        r.reverse()

        if src == dst:
            path = [src]
        else:
            path = r

        # attach in_port/out_port info
        result = []
        in_port = first_port
        for s1, s2 in zip(path[:-1], path[1:]):
            out_port = self.adjacency[s1][s2]
            result.append((s1, in_port, out_port))
            in_port = self.adjacency[s2][s1]
        result.append((dst, in_port, final_port))
        return result

    def install_path(self, p, src_mac, dst_mac):
        key = (src_mac, dst_mac)
        # suppress logging if same path already installed
        last = self.installed_paths.get(key)
        if last == p:
            return
        self.installed_paths[key] = p
        # only log when path spans multiple switches (multi-hop paths)
        if len(p) > 1:
            path_str = ' -> '.join(str(sw) for sw, _, _ in p)
            print(f"[FLOW-INSTALL] {src_mac} -> {dst_mac}: installed path {path_str}")
        # adding path to flow table of each switch inside the shortest path
        for sw, in_port, out_port in p:
            # get the datapath for this switch
            datapath = next((dp for dp in self.datapath_list if dp.id == int(sw)), None)
            if datapath is None:
                print(f"Warning: datapath with id {sw} not found in datapath_list")
                continue
            parser = datapath.ofproto_parser
            ofproto = datapath.ofproto
            # setting match part of the flow table
            match = parser.OFPMatch(in_port=in_port, eth_src=src_mac, eth_dst=dst_mac)
            # setting actions part of the flow table
            actions = [parser.OFPActionOutput(out_port)]
            # getting instructions based on the actions
            inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS , actions)]
            # remove any existing flow matching this 5-tuple (in_port, eth_src, eth_dst)
            delete_mod = datapath.ofproto_parser.OFPFlowMod(datapath=datapath, match=match,
                                                           command=ofproto.OFPFC_DELETE,
                                                           out_port=ofproto.OFPP_ANY, out_group=ofproto.OFPG_ANY,
                                                           priority=1)
            datapath.send_msg(delete_mod)
            # add new flow
            mod = datapath.ofproto_parser.OFPFlowMod(datapath=datapath, match=match, idle_timeout=10, hard_timeout=30,
                                                     priority=1, instructions=inst)
            datapath.send_msg(mod)

        # Install reverse flows (dst -> src) immediately to avoid early packet loss
        for sw, in_port, out_port in reversed(p):
            rev_datapath = next((dp for dp in self.datapath_list if dp.id == int(sw)), None)
            if rev_datapath is None:
                print(f"Warning: datapath with id {sw} not found when installing reverse flow")
                continue
            rev_parser = rev_datapath.ofproto_parser
            rev_ofproto = rev_datapath.ofproto
            # reverse match: packet arrives from the forward out_port and should be sent to forward in_port
            rev_in = out_port
            rev_out = in_port
            # match reversed src/dst
            rev_match = rev_parser.OFPMatch(in_port=rev_in, eth_src=dst_mac, eth_dst=src_mac)
            rev_actions = [rev_parser.OFPActionOutput(rev_out)]
            rev_inst = [rev_parser.OFPInstructionActions(rev_ofproto.OFPIT_APPLY_ACTIONS, rev_actions)]
            # remove existing reverse flow then add
            rev_delete = rev_datapath.ofproto_parser.OFPFlowMod(datapath=rev_datapath, match=rev_match,
                                                                command=rev_ofproto.OFPFC_DELETE,
                                                                out_port=rev_ofproto.OFPP_ANY, out_group=rev_ofproto.OFPG_ANY,
                                                                priority=1)
            rev_datapath.send_msg(rev_delete)
            rev_mod = rev_datapath.ofproto_parser.OFPFlowMod(datapath=rev_datapath, match=rev_match,
                                                            idle_timeout=10, hard_timeout=30,
                                                            priority=1, instructions=rev_inst)
            rev_datapath.send_msg(rev_mod)
        # done

    # defining event handler for setup and configuring of switches
    @set_ev_cls(ofp_event.EventOFPSwitchFeatures , CONFIG_DISPATCHER)
    def switch_features_handler(self , ev):
        # print("switch_features_handler function is called")
        # getting the datapath, ofproto and parser objects of the event
        datapath = ev.msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        # setting match condition to nothing so that it will match to anything
        match = parser.OFPMatch()
        # setting action to send packets to OpenFlow Controller without buffering
        actions = [parser.OFPActionOutput(ofproto.OFPP_CONTROLLER, ofproto.OFPCML_NO_BUFFER)]
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS , actions)]
        # setting the priority to 0 so that it will be that last entry to match any packet inside any flow table
        mod = datapath.ofproto_parser.OFPFlowMod(
                            datapath=datapath, match=match, cookie=0,
                            command=ofproto.OFPFC_ADD, idle_timeout=0, hard_timeout=0,
                            priority=0, instructions=inst)
        # finalizing the mod 
        datapath.send_msg(mod)

    # defining an event handler for packets coming to switches event
    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def _packet_in_handler(self, ev):
        # getting msg, datapath, ofproto and parser objects
        msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        # getting the port switch received the packet with
        in_port = msg.match['in_port']
        # creating a packet encoder/decoder class with the raw data obtained by msg
        pkt = packet.Packet(msg.data)
        # getting the protocl that matches the received packet
        eth = pkt.get_protocol(ethernet.ethernet)

        # avoid broadcasts from LLDP 
        if eth.ethertype == 35020 or eth.ethertype == 34525:
            return

        # getting source and destination of the link
        dst = eth.dst
        src = eth.src
        dpid = datapath.id
        # print(f"Packet-In detected for SRC: {src} to DST: {dst} on SWITCH DPID: {dpid}")

        # add the host to the mymacs of the first switch that gets the packet
        if src not in self.mymacs.keys():
            self.mymacs[src] = (dpid, in_port)
            print(f"[HOST-LEARN] MAC {src} discovered at switch {dpid} port {in_port}")

        # finding shortest path if destination exists in mymacs
        if dst in self.mymacs.keys():
            # Destination known: compute shortest path and install flows
            p = self.compute_path(self.mymacs[src][0], self.mymacs[dst][0], self.mymacs[src][1], self.mymacs[dst][1])
            print(f"[PKT-FWD] {src} -> {dst}: path computed with {len(p)} hop(s)")
            self.install_path(p, src, dst)
            out_port = p[0][2]
        else:
            # Destination unknown: fall back to flooding
            print(f"[PKT-FLOOD] {src} -> {dst}: destination unknown, flooding")
            out_port = ofproto.OFPP_FLOOD

        # getting actions part of the flow table
        actions = [parser.OFPActionOutput(out_port)]

        data = None
        if msg.buffer_id == ofproto.OFP_NO_BUFFER:
            data = msg.data
        out = parser.OFPPacketOut(datapath=datapath, buffer_id=msg.buffer_id, in_port=in_port,
                                  actions=actions, data=data)
        datapath.send_msg(out)

    # defining an event handler for adding/deleting of switches, hosts, ports and links event
    events = [event.EventSwitchEnter,
              event.EventSwitchLeave, event.EventPortAdd,
              event.EventPortDelete, event.EventPortModify,
              event.EventLinkAdd, event.EventLinkDelete]
    @set_ev_cls(events)
    def get_topology_data(self, ev):
        global switches
        global mylinks
        global adjacency
        # print("get_topology_data is called.")
        # getting the list of known switches 
        switch_list = get_switch(self.topology_api_app, None)  
        switches = [switch.dp.id for switch in switch_list]
        # keep instance switch list in sync with topology
        self.switches = switches
        # print("current known switches=", switches)
        # getting the list of datapaths from the list of switches
        self.datapath_list = [switch.dp for switch in switch_list]
        # sorting the datapath list based on their id so that indexing them in install_function will be correct
        self.datapath_list.sort(key=lambda dp: dp.id)

        # getting the list of links between switches
        links_list = get_link(self.topology_api_app, None)
        new_mylinks = [(link.src.dpid, link.dst.dpid, link.src.port_no, link.dst.port_no) for link in links_list]

        # reset adjacency cleanly and repopulate
        adjacency = defaultdict(lambda:defaultdict(lambda:None))
        self.adjacency = adjacency
        for s1, s2, port1, port2 in new_mylinks:
            adjacency[s1][s2] = port1
            adjacency[s2][s1] = port2

        # If links changed, recompute paths for known hosts
        if new_mylinks != getattr(self, 'mylinks', []):
            # compute added/removed links for logging
            old_set = set(getattr(self, 'mylinks', []))
            new_set = set(new_mylinks)
            added = new_set - old_set
            removed = old_set - new_set
            if added:
                print(f"[TOPO-CHANGE] Link up: {[(s1, s2) for s1, s2, _, _ in added]}")
            if removed:
                print(f"[TOPO-CHANGE] Link down: {[(s1, s2) for s1, s2, _, _ in removed]}")

            self.mylinks = new_mylinks
            macs = list(self.mymacs.keys())
            for src_mac in macs:
                for dst_mac in macs:
                    if src_mac == dst_mac:
                        continue
                    try:
                        src_sw, src_port = self.mymacs[src_mac]
                        dst_sw, dst_port = self.mymacs[dst_mac]
                    except Exception:
                        continue
                    # compute and install new path
                    p = self.compute_path(src_sw, dst_sw, src_port, dst_port)
                    if p:
                        print(f"[PKT-FWD] {src_mac} -> {dst_mac}: path computed with {len(p)} hop(s)")
                        self.install_path(p, src_mac, dst_mac)
