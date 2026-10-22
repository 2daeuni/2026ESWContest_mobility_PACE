"""
CAN timing analysis for a CANoe ASC log (Logging block, ASCII format).

Reports per message ID: frame count, cycle time min / mean / max / std,
Alive Counter duplicates / skips / backward steps, DLC errors, and the
command -> status response times:
  - Motor : MOTOR_COMMAND (0x120) Alive -> first MOTOR_STATUS (0x210) whose
            Byte6 (Applied Command Counter) echoes it
  - Haptic: HAPTIC_COMMAND (0x110) risk/output change -> first HAPTIC_STATUS (0x200)
            reporting the same risk, vibration, intensity and frequency

Usage:
    python canoe/tools/asc_timing.py log.asc [--csv result.csv]
"""
import argparse
import csv
import re
import statistics
import sys

LINE = re.compile(r"^\s*(\d+\.\d+)\s+(\d+)\s+([0-9A-Fa-f]+)x?\s+(Rx|Tx)\s+d\s+(\d+)\s*((?:[0-9A-Fa-f]{2}\s*)*)")
NAMES = {0x100: "PEDAL_STATUS", 0x110: "HAPTIC_COMMAND", 0x120: "MOTOR_COMMAND",
         0x200: "HAPTIC_STATUS", 0x210: "MOTOR_STATUS"}


def read_asc(path):
    frames = []
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = LINE.match(line)
            if not m:
                continue
            t, _ch, cid, direction, dlc, data = m.groups()
            frames.append((float(t), int(cid, 16), direction, int(dlc),
                           [int(b, 16) for b in data.split()]))
    return frames


def stats_ms(values):
    if not values:
        return ("", "", "", "")
    ms = [v * 1000.0 for v in values]
    std = statistics.pstdev(ms) if len(ms) > 1 else 0.0
    return (f"{min(ms):.2f}", f"{statistics.mean(ms):.2f}", f"{max(ms):.2f}", f"{std:.2f}")


def analyse(frames):
    rows = []
    for cid, name in NAMES.items():
        fr = [f for f in frames if f[1] == cid]
        times = [f[0] for f in fr]
        gaps = [b - a for a, b in zip(times, times[1:])]
        dup = skip = back = 0
        prev = None
        for f in fr:
            if len(f[4]) < 8:
                continue
            alive = f[4][7]
            if prev is not None:
                d = (alive - prev) & 0xFF
                if d == 0:
                    dup += 1
                elif d >= 128:
                    back += 1
                elif d >= 2:
                    skip += 1
            prev = alive
        bad_dlc = sum(1 for f in fr if f[3] != 8)
        dirs = "/".join(sorted({f[2] for f in fr}))
        rows.append(dict(id=f"0x{cid:03X}", message=name, dir=dirs, frames=len(fr),
                         cycle_min_ms=stats_ms(gaps)[0], cycle_mean_ms=stats_ms(gaps)[1],
                         cycle_max_ms=stats_ms(gaps)[2], cycle_std_ms=stats_ms(gaps)[3],
                         alive_dup=dup, alive_skip=skip, alive_back=back, dlc_error=bad_dlc))
    return rows


def motor_response(frames):
    pending = {}
    delays = []
    for t, cid, _d, dlc, data in frames:
        if dlc != 8 or len(data) < 8:
            continue
        if cid == 0x120:
            pending.setdefault(data[7], t)
        elif cid == 0x210 and data[6] in pending:
            delays.append(t - pending.pop(data[6]))
            pending = {k: v for k, v in pending.items() if t - v < 1.0}
    return delays


def haptic_response(frames):
    delays = []
    last_cmd = None
    waiting = None
    for t, cid, _d, dlc, data in frames:
        if dlc != 8 or len(data) < 8:
            continue
        if cid == 0x110:
            cmd = tuple(data[0:4])
            if cmd != last_cmd:
                waiting = (t, cmd)
                last_cmd = cmd
        elif cid == 0x200 and waiting is not None:
            if tuple(data[0:4]) == waiting[1]:
                delays.append(t - waiting[0])
                waiting = None
    return delays


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("asc")
    ap.add_argument("--csv")
    a = ap.parse_args()
    frames = read_asc(a.asc)
    if not frames:
        sys.exit("no CAN frames found (ASC export from the CANoe Logging block expected)")
    rows = analyse(frames)
    print(f"{'ID':6} {'Message':15} {'Dir':6} {'Frames':>7} {'min':>7} {'mean':>7} {'max':>7} {'std':>6}  dup skip back dlc")
    for r in rows:
        print(f"{r['id']:6} {r['message']:15} {r['dir']:6} {r['frames']:7} {r['cycle_min_ms']:>7} {r['cycle_mean_ms']:>7} "
              f"{r['cycle_max_ms']:>7} {r['cycle_std_ms']:>6}  {r['alive_dup']:3} {r['alive_skip']:4} {r['alive_back']:4} {r['dlc_error']:3}")
    for label, d in (("Motor  0x120 -> 0x210 (Applied Counter echo)", motor_response(frames)),
                     ("Haptic 0x110 change -> 0x200 applied", haptic_response(frames))):
        s = stats_ms(d)
        print(f"{label}: n={len(d)} min {s[0]} mean {s[1]} max {s[2]} std {s[3]} ms")
    if a.csv:
        with open(a.csv, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print("saved", a.csv)


if __name__ == "__main__":
    main()
