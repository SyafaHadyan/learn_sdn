#!/usr/bin/env python3
"""ECMP bandwidth-distribution verifier for the diamond topology.

Why a single iperf stream doesn't show ECMP distribution
---------------------------------------------------------
OVS implements OpenFlow SELECT groups using a **hash of the packet's
5-tuple** (src IP, dst IP, proto, src port, dst port).  Within one TCP
connection all packets share the same 5-tuple, so every packet maps to
the SAME bucket — i.e. the same physical path.  ECMP distributes FLOWS,
not individual packets within a flow.

What this script does instead
------------------------------
1. Generates concurrent UDP flows between FOUR distinct (src,dst) pairs:
     h1 → h4   (MAC pair A)
     h2 → h3   (MAC pair B)
     h3 → h2   (MAC pair C — reverse)
     h4 → h1   (MAC pair D — reverse)
   Each pair gets its own SELECT group keyed on (src_mac, dst_mac).
   The bucket chosen for A may differ from B, spreading traffic across
   both equal-cost paths (s1→s2→s4 and s1→s3→s4).

2. Reads s1's per-port byte counters (port 3 → s2, port 4 → s3) before
   and after the traffic burst.

3. Shows an ASCII distribution bar and reports pass / partial / fail.

4. Dumps the actual OVS SELECT-group bucket stats as ground truth.

Usage
-----
  Terminal 1:  python3 SPF/dijkstra_multipath_osken_controller.py
               # or astar_multipath_osken_controller.py
               # or kshortest_osken_controller.py
  Terminal 2:  python3 SPF/verify_ecmp.py

Notes
-----
- 'balance ratio' = min(path_bytes) / max(path_bytes).
  Perfect balance = 1.0, acceptable = > 0.3 (hash skew with few flows).
- OVS SELECT-group hash quality depends on how many distinct 5-tuples
  hit the group; 4 UDP sender pairs with IPERF_PARALLEL streams each
  gives ~4×IPERF_PARALLEL distinct (src_port, dst_port) combinations.
"""

import re
import subprocess
import sys
import time

from mininet.link import TCLink
from mininet.log import setLogLevel
from mininet.net import Mininet
from mininet.node import RemoteController
from mininet.topo import Topo

# ─── Traffic parameters ──────────────────────────────────────────────────────
IPERF_DURATION = 12    # seconds of sustained traffic
IPERF_PARALLEL = 6     # UDP streams per sender (different src ports → better hash)
IPERF_BW_MBPS  = 10    # Mbps per stream (well below 100 Mbps link capacity)
IPERF_PORT_AB  = 5301  # iperf3 server port for h1→h4 and h2→h3
IPERF_PORT_CD  = 5302  # iperf3 server port for h3→h2 and h4→h1


# ─── Topology ────────────────────────────────────────────────────────────────

class DiamondTopo(Topo):
    """Identical diamond topology to topo-ecmp_lab.py.

    Diamond structure (4 switches, 4 hosts):

                    h1        h2
                     |        |
                    (1)      (2)
                     +── s1 ──+
                        /  \\
                      (3)  (4)
                      /      \\
                   s2          s3
                    (2)      (2)
                      \\      /
                      (3)  (4)
                     +── s4 ──+
                    (1)      (2)
                     |        |
                    h3        h4

    Equal-cost paths s1 → s4:
      Path A: s1 port3 → s2 port2 → s4 port3
      Path B: s1 port4 → s3 port2 → s4 port4
    """

    def __init__(self):
        Topo.__init__(self)
        h1 = self.addHost('h1', ip='10.1.0.1/8')
        h2 = self.addHost('h2', ip='10.1.0.2/8')
        h3 = self.addHost('h3', ip='10.2.0.3/8')
        h4 = self.addHost('h4', ip='10.2.0.4/8')

        s1 = self.addSwitch('s1')
        s2 = self.addSwitch('s2')
        s3 = self.addSwitch('s3')
        s4 = self.addSwitch('s4')

        # Host links (no shaping)
        self.addLink(s1, h1, port1=1, port2=1)
        self.addLink(s1, h2, port1=2, port2=1)
        self.addLink(s4, h3, port1=1, port2=1)
        self.addLink(s4, h4, port1=2, port2=1)

        # Switch-to-switch links: 100 Mbps, 2 ms, HFSC qdisc (no HTB warnings)
        self.addLink(s1, s2, port1=3, port2=1, bw=100, delay='2ms', use_hfsc=True)
        self.addLink(s1, s3, port1=4, port2=1, bw=100, delay='2ms', use_hfsc=True)
        self.addLink(s2, s4, port1=2, port2=3, bw=100, delay='2ms', use_hfsc=True)
        self.addLink(s3, s4, port1=2, port2=4, bw=100, delay='2ms', use_hfsc=True)


# ─── OVS helpers ─────────────────────────────────────────────────────────────

def _ovs(cmd_list):
    try:
        return subprocess.check_output(
            cmd_list, stderr=subprocess.DEVNULL, timeout=6
        ).decode()
    except Exception:
        return ""


def dump_port_bytes(bridge):
    """Return {port_no: {'rx': bytes, 'tx': bytes}} parsed from dump-ports."""
    raw = _ovs(['ovs-ofctl', 'dump-ports', bridge, '-O', 'OpenFlow13'])
    result = {}
    lines = raw.splitlines()
    i = 0
    while i < len(lines):
        line = lines[i]
        pm = re.match(r'\s+port\s+(\d+):', line)
        if pm:
            port = int(pm.group(1))
            # Stats may span this line and the next
            block = line
            if i + 1 < len(lines) and not re.match(r'\s+port\s+', lines[i + 1]):
                block += ' ' + lines[i + 1]
            rx = re.search(r'rx\s+pkts=\d+,\s+bytes=(\d+)', block)
            tx = re.search(r'tx\s+pkts=\d+,\s+bytes=(\d+)', block)
            result[port] = {
                'rx': int(rx.group(1)) if rx else 0,
                'tx': int(tx.group(1)) if tx else 0,
            }
        i += 1
    return result


def dump_groups(bridge):
    return _ovs(['ovs-ofctl', 'dump-groups', bridge, '-O', 'OpenFlow13'])


def dump_group_stats(bridge):
    return _ovs(['ovs-ofctl', 'dump-group-stats', bridge, '-O', 'OpenFlow13'])


# ─── Pretty-print helpers ─────────────────────────────────────────────────────

def bar(label, value, total, width=40):
    frac = value / total if total > 0 else 0
    filled = int(frac * width)
    b = '█' * filled + '░' * (width - filled)
    print(f'  {label:<22s} [{b}] {frac * 100:5.1f}%  ({value:>12,} B)')


def section(title):
    print(f'\n{"─" * 58}')
    print(f'  {title}')
    print(f'{"─" * 58}')


# ─── Main verification ────────────────────────────────────────────────────────

def main():
    setLogLevel('warning')

    print('╔══════════════════════════════════════════════════════╗')
    print('║       ECMP Bandwidth Distribution Verifier          ║')
    print('║   diamond topology — 2 equal-cost paths s1 → s4    ║')
    print('╚══════════════════════════════════════════════════════╝')
    print()
    print('  IMPORTANT: A multipath controller must be running in')
    print('  another terminal before this script starts.')
    print('  Supported controllers:')
    print('    python3 SPF/dijkstra_multipath_osken_controller.py')
    print('    python3 SPF/astar_multipath_osken_controller.py')
    print('    python3 SPF/kshortest_osken_controller.py')
    print()

    # ── Start topology ────────────────────────────────────────────────────
    topo = DiamondTopo()
    net = Mininet(
        topo=topo,
        controller=RemoteController,
        link=TCLink,
        autoSetMacs=True,
        autoStaticArp=True,
        waitConnected=True,
    )

    for host in net.hosts:
        host.cmd('sysctl -w net.ipv6.conf.all.disable_ipv6=1 >/dev/null 2>&1')
    for sw in net.switches:
        sw.cmd('sysctl -w net.ipv6.conf.all.disable_ipv6=1 >/dev/null 2>&1')

    net.start()
    h1, h2, h3, h4 = net.get('h1', 'h2', 'h3', 'h4')

    # ── Prime MAC learning ────────────────────────────────────────────────
    section('Step 1 — Prime MAC learning (8 s)')
    print('  Sending initial pings so the controller learns all MAC/port mappings...')
    for src, dst in [(h1, h4), (h4, h1), (h2, h3), (h3, h2)]:
        src.cmd(f'ping -c 2 -W 2 {dst.IP()} >/dev/null 2>&1')
    time.sleep(6)
    print('  Done.')

    # ── Show installed groups ─────────────────────────────────────────────
    section('Step 2 — OVS SELECT groups installed on s1')
    grp_table = dump_groups('s1')
    if 'select' in grp_table.lower():
        for line in grp_table.splitlines():
            if line.strip() and 'OFPGT_SELECT' not in line:
                # Show compact group entries
                print(f'  {line}')
    else:
        print('  *** No SELECT groups found on s1!')
        print('  *** Is the multipath controller running?')
        print('  *** (Hint: run pings first so routes install, then check again.)')

    # ── Snapshot bytes BEFORE ─────────────────────────────────────────────
    before = dump_port_bytes('s1')

    # ── Generate traffic ──────────────────────────────────────────────────
    section(f'Step 3 — Generate {IPERF_DURATION}s of concurrent UDP traffic')
    print(f'  4 sender pairs × {IPERF_PARALLEL} parallel streams × {IPERF_BW_MBPS} Mbps = '
          f'{4 * IPERF_PARALLEL * IPERF_BW_MBPS} Mbps total aim')
    print()
    print('  Why 4 pairs?  Each (src_mac, dst_mac) gets its own SELECT group;')
    print('  OVS hashes each group\'s flows independently, so different pairs')
    print('  likely land on different buckets (= different physical paths).')
    print()
    print('  Senders:')
    print(f'    h1 → h4  (port {IPERF_PORT_AB}) — MAC pair A')
    print(f'    h2 → h3  (port {IPERF_PORT_AB}) — MAC pair B')
    print(f'    h3 → h2  (port {IPERF_PORT_CD}) — MAC pair C (reverse)')
    print(f'    h4 → h1  (port {IPERF_PORT_CD}) — MAC pair D (reverse)')

    # Start iperf3 servers
    h4.cmd(f'iperf3 -s -D -p {IPERF_PORT_AB} --one-off 2>/dev/null')
    h3.cmd(f'iperf3 -s -D -p {IPERF_PORT_AB} --one-off 2>/dev/null')
    h2.cmd(f'iperf3 -s -D -p {IPERF_PORT_CD} --one-off 2>/dev/null')
    h1.cmd(f'iperf3 -s -D -p {IPERF_PORT_CD} --one-off 2>/dev/null')
    time.sleep(0.5)

    bw_arg = f'{IPERF_BW_MBPS}M'
    iperf_cmd = (
        f'iperf3 -u -b {bw_arg} -t {IPERF_DURATION} -P {IPERF_PARALLEL}'
        f' 2>/dev/null'
    )

    # Launch all 4 senders concurrently (sendCmd = non-blocking)
    h1.sendCmd(f'{iperf_cmd} -c {h4.IP()} -p {IPERF_PORT_AB}')
    h2.sendCmd(f'{iperf_cmd} -c {h3.IP()} -p {IPERF_PORT_AB}')
    h3.sendCmd(f'{iperf_cmd} -c {h2.IP()} -p {IPERF_PORT_CD}')
    h4.sendCmd(f'{iperf_cmd} -c {h1.IP()} -p {IPERF_PORT_CD}')

    # Progress indicator
    for i in range(IPERF_DURATION + 2):
        pct = int(100 * i / (IPERF_DURATION + 2))
        bar_len = int(40 * i / (IPERF_DURATION + 2))
        sys.stdout.write(
            f'\r  Progress: [{"#" * bar_len}{" " * (40 - bar_len)}] {pct}%'
        )
        sys.stdout.flush()
        time.sleep(1)
    print()

    # Collect output (discard, we only need stats)
    h1.waitOutput(verbose=False)
    h2.waitOutput(verbose=False)
    h3.waitOutput(verbose=False)
    h4.waitOutput(verbose=False)

    # ── Snapshot bytes AFTER ──────────────────────────────────────────────
    after = dump_port_bytes('s1')

    # ── Compute deltas ────────────────────────────────────────────────────
    section('Step 4 — Path distribution on s1 (tx bytes delta)')

    p3_tx = after.get(3, {}).get('tx', 0) - before.get(3, {}).get('tx', 0)
    p4_tx = after.get(4, {}).get('tx', 0) - before.get(4, {}).get('tx', 0)
    total  = p3_tx + p4_tx

    print('  Port 3 → s2 → s4  (Path A)')
    print('  Port 4 → s3 → s4  (Path B)')
    print()

    if total == 0:
        print('  *** Zero bytes measured on ports 3 and 4!')
        print('      Possible causes:')
        print('        • Multipath controller not running')
        print('        • Routes not yet installed (increase wait time)')
        print('        • parse error — run: ovs-ofctl dump-ports s1 -O OpenFlow13')
    else:
        bar('Port 3 → s2 (Path A)', p3_tx, total)
        bar('Port 4 → s3 (Path B)', p4_tx, total)
        mn = min(p3_tx, p4_tx)
        mx = max(p3_tx, p4_tx)
        ratio = mn / mx if mx > 0 else 0.0
        print()
        print(f'  Total forwarded  : {total:,} bytes')
        print(f'  Balance ratio    : {ratio:.2f}  (1.0 = perfect, ≥0.25 = good)')
        print()
        if ratio >= 0.25:
            print('  ✅  ECMP is distributing traffic across BOTH equal-cost paths.')
        elif p3_tx > 0 and p4_tx > 0:
            print('  ℹ   Both paths active but distribution is skewed.')
            print('      Hash skew is normal with a small number of flows.')
            print('      Try increasing IPERF_PARALLEL at the top of this file.')
        elif p3_tx == 0 or p4_tx == 0:
            print('  ⚠   All traffic on one path — the other path was unused.')
            print('      Hash collision: all flows mapped to the same bucket.')
            print('      This is a known OVS hash limitation with identical')
            print('      src/dst IPs. Try adding more diverse host pairs.')

    # ── Group stats (ground truth) ────────────────────────────────────────
    section('Step 5 — OVS SELECT group bucket stats (ground truth)')
    gstats = dump_group_stats('s1')
    if gstats.strip():
        for line in gstats.splitlines():
            if line.strip():
                print(f'  {line}')
    else:
        print('  (no group stats returned)')

    # ── Flow table overview ───────────────────────────────────────────────
    section('Step 6 — s1 flow table (installed forwarding rules)')
    raw_flows = _ovs(['ovs-ofctl', 'dump-flows', 's1', '-O', 'OpenFlow13'])
    nflows = sum(1 for l in raw_flows.splitlines() if 'cookie' in l)
    print(f'  {nflows} flows installed on s1.')
    for line in raw_flows.splitlines():
        if 'group:' in line or 'output:' in line:
            # Show only the interesting action lines
            m = re.search(r'(eth_src=[\w:]+,eth_dst=[\w:]+).*?(group:\d+|output:\d+)', line)
            if m:
                print(f'  {m.group(1):50s}  →  {m.group(2)}')

    print()
    net.stop()
    print('Done — network stopped.')


if __name__ == '__main__':
    main()
