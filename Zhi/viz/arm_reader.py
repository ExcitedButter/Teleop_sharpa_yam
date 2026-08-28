#!/home/zhicao/Desktop/Zhi/Zhi/.venv/bin/python
"""Follower arm joint state -> UDP JSON for viz_panel.py.

Runs the portal pollers in their OWN process because portal os._exit(1)s the
whole process (and kills its child processes) whenever its internal client
thread hits an unhandled error - e.g. 'No route to host' while the yambox
reboots (killed the embedded viz live, 2026-08-25). viz_panel watches this
reader and respawns it if it dies.

Publishes {"t": .., "L": {"q": [6], "t": .., "mov": ..}, "R": {...}} on
udp/59262 at ~20 Hz. Keys are PHYSICAL sides (L=11333, R=11334).
"""
import json, os, socket, sys, threading, time

import numpy as np

YAMBOX_IP = os.environ.get("YAMBOX_IP", "192.168.1.9")
ARM_PORTS = {"L": 11333, "R": 11334}   # physical left / right followers
OUT_PORT = 59262


class Poller(threading.Thread):
    """Gentle persistent-client poller (never churns connections - client
    churn leaks fds in minimum_gello's portal server until it jams)."""

    def __init__(self, side):
        super().__init__(daemon=True)
        self.side, self.q, self.t = side, None, 0.0
        self.hist = []

    def mov(self):
        if len(self.hist) < 2:
            return 0.0
        qs = np.array([q for _, q in self.hist])
        return float((qs.max(0) - qs.min(0)).max())

    def run(self):
        import portal
        client, misses = None, 0
        while True:
            if client is None:
                try:
                    s = socket.create_connection((YAMBOX_IP, ARM_PORTS[self.side]), timeout=2)
                    s.close()
                    client = portal.Client("%s:%d" % (YAMBOX_IP, ARM_PORTS[self.side]))
                    misses = 0
                except OSError:
                    time.sleep(5)
                    continue
            try:
                self.q = np.asarray(client.get_joint_pos().result(timeout=1.5), float)[:6]
                self.t = time.time()
                self.hist.append((self.t, self.q))
                self.hist = [(t, q) for t, q in self.hist if self.t - t < 2.0]
                misses = 0
                time.sleep(1 / 20)
            except Exception:
                misses += 1
                time.sleep(min(5.0, 0.5 * misses))


def main():
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = ("127.0.0.1", OUT_PORT)
    pollers = {s: Poller(s) for s in ("L", "R")}
    for p in pollers.values():
        p.start()
    while True:
        msg = {"t": time.time()}
        for s, p in pollers.items():
            if p.q is not None:
                msg[s] = {"q": [float(v) for v in p.q], "t": p.t, "mov": p.mov()}
        out.sendto(json.dumps(msg).encode(), dst)
        time.sleep(1 / 20)


if __name__ == "__main__":
    sys.exit(main())
