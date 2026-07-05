#!/usr/bin/env python3
"""
Geführtes IMU-Capture für die XREAL One — zum Reverse-Engineering der
unbekannten Felder (Offsets 58/62/66) und zum Magnetometer-Test.

Du wirst durch fünf Phasen geführt. Halte die Brille in die Hand und
folge den Anweisungen. Alles wird als CSV geloggt: capture.csv

    python3 capture_guided.py

Wichtig für den Magnetometer-Test (Phase SPIN): die Brille FLACH halten
(Bügel waagerecht) und langsam einmal ganz um die Hochachse drehen.
"""
import socket, struct, time, csv, sys

IP, PORT = "169.254.2.1", 52998
HEADER = bytes.fromhex("273600000080")
TAG = bytes.fromhex("00401f000040")
FRAME_LEN = 134
OUT = "capture.csv"

# Kandidaten-Felder (aligned float32 offsets)
FIELDS = {
    "gyroX": 34, "gyroY": 38, "gyroZ": 42,
    "accX": 46, "accY": 50, "accZ": 54,
    "f58": 58, "f62": 62, "f66": 66, "temp": 70, "f74": 74,
}

# (phase, sekunden, anweisung)
PHASES = [
    ("still",  3, "STILL halten — nicht bewegen"),
    ("yaw",    4, "YAW: Brille langsam nach LINKS und RECHTS drehen"),
    ("pitch",  4, "PITCH: Brille langsam nach OBEN und UNTEN kippen"),
    ("roll",   4, "ROLL: Brille langsam nach LINKS/RECHTS neigen (kippen)"),
    ("spin",   8, "SPIN: Brille FLACH halten, langsam einmal 360 Grad um die Hochachse drehen"),
    ("still2", 2, "STILL halten — Ende"),
]


def parse(frame):
    if len(frame) < FRAME_LEN:
        return None
    m = frame.find(TAG)
    if m != 78:
        return None
    try:
        return {k: struct.unpack("<f", frame[o:o+4])[0] for k, o in FIELDS.items()}
    except struct.error:
        return None


def frames(sock):
    buf = b""
    while True:
        chunk = sock.recv(8192)
        if not chunk:
            return
        buf += chunk
        while True:
            h = buf.find(HEADER)
            if h == -1:
                buf = buf[-4:] if len(buf) > 4 else buf
                break
            nxt = buf.find(HEADER, h + 1)
            if nxt == -1:
                buf = buf[h:]
                break
            yield buf[h:nxt]
            buf = buf[nxt:]


def main():
    print(f"Connecting {IP}:{PORT} ...")
    s = socket.socket(); s.settimeout(5)
    try:
        s.connect((IP, PORT))
    except OSError as e:
        print("NO-GO:", e); return
    print("GO. Warmup ...")

    gen = frames(s)
    # Warmup: 300 Frames wegwerfen (Nullframes beim Connect)
    warm = 0
    for fr in gen:
        if parse(fr):
            warm += 1
            if warm >= 300:
                break

    rows = []
    t0 = time.time()
    print("\n=== LOS ===\n")
    for phase, secs, instr in PHASES:
        print(f"\n>>> {phase.upper()}  ({secs}s):  {instr}")
        for c in range(3, 0, -1):
            print(f"    start in {c} ...", end="\r", flush=True); _spin_wait(gen, 0.4)
        print("    ● AUFNAHME LÄUFT   ")
        p_end = time.time() + secs
        for fr in gen:
            d = parse(fr)
            if not d:
                continue
            t = time.time()
            d2 = {"t": round(t - t0, 4), "phase": phase}
            d2.update({k: round(v, 5) for k, v in d.items()})
            rows.append(d2)
            if time.time() >= p_end:
                break
    s.close()

    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t", "phase"] + list(FIELDS))
        w.writeheader(); w.writerows(rows)
    print(f"\n=== FERTIG === {len(rows)} Samples -> {OUT}")


def _spin_wait(gen, secs):
    """Verwerfe Frames für ~secs, damit der Stream nicht verstopft."""
    end = time.time() + secs
    for _ in gen:
        if time.time() >= end:
            return


if __name__ == "__main__":
    main()
