"""
PACE risk reference vectors from the real Safety ECU Python code.

Runs safety_ecu/risk (BNS, PMS, RiskManager) and control/command_mapper with
the same inputs that CAPL/PACE_Test.can injects into the CANoe Safety node,
and writes the expected results to CAPL/PACE_RefVectors.cin.

Usage (from the repository root):
    python canoe/tools/risk_reference.py

Each vector is held for HOLD_EVALS evaluations (main-loop calls), so the
3-up / 5-down confirmation has settled. The CANoe test holds every vector for
HOLD_MS with a 1 ms evaluation period.
"""
import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "safety_ecu"))

from risk.bns import BNSCalculator            # noqa: E402
from risk.pms import PMSCalculator            # noqa: E402
from risk.risk_manager import RiskManager     # noqa: E402
from control.command_mapper import CommandMapper  # noqa: E402

HOLD_EVALS = 60

# id, tc, description, obstacle_present, dist_m, speed_kmh, closing_mps, ttc_s (None),
# accel %, brake %, accel_rate %/s
VECTORS = [
    ("V01", "TC01", "normal driving, no condition",          1, 40.0, 20.0, 5.56, 7.2,  30, 0, 0.0),
    ("V02", "TC02", "L1: distance 20 m + TTC 3.5 s",         1, 20.0, 20.0, 5.56, 3.5,  30, 0, 0.0),
    ("V03", "TC03", "L2: distance 14 m + TTC 2.0 s",         1, 14.0, 25.0, 6.94, 2.0,  30, 0, 0.0),
    ("V04", "TC04", "L3: distance 8 m + TTC 1.0 s",          1,  8.0, 29.0, 8.00, 1.0,  30, 0, 0.0),
    ("V05", "TC05", "release: risk descends one level per 5 evaluations", 1, 40.0, 0.0, 0.0, None, 0, 0, 0.0),
    ("V06", "TC06", "distance only 5 m (TTC None, no pedal)", 1,  5.0, 0.0, 0.0, None,  0, 0, 0.0),
    ("V07", "TC06", "TTC only 1.0 s (distance 40 m)",        1, 40.0, 30.0, 40.0, 1.0,  0, 0, 0.0),
    ("V08", "TC06", "pedal only accelerator 95 %",           1, 40.0, 0.0, 0.0, None,  95, 0, 0.0),
    ("V09", "TC07", "TTC 2.0 s + accelerator 80 %",          1, 40.0, 30.0, 20.0, 2.0, 80, 0, 0.0),
    ("V10", "TC07", "TTC 1.2 s + accelerator rate 700 %/s",  1, 40.0, 30.0, 33.3, 1.2, 30, 0, 700.0),
    ("V11", "TC07", "distance 12 m + rate 460 %/s",          1, 12.0, 0.0, 0.0, None,  30, 0, 460.0),
    ("V12", "TC08", "brake 10 % suppresses pedal (TTC 2.0 + acc 80)", 1, 40.0, 30.0, 20.0, 2.0, 80, 10, 0.0),
    ("V13", "TC08", "brake 4 % does not suppress",           1, 40.0, 30.0, 20.0, 2.0, 80, 4, 0.0),
    ("V14", "TC08", "pedal conflict acc 30 / brake 30 + TTC 2.0", 1, 40.0, 30.0, 20.0, 2.0, 30, 30, 0.0),
    ("V15", "TC08", "pedal conflict + distance 8 m + TTC 1.0 (L3 by BNS)", 1, 8.0, 30.0, 8.0, 1.0, 30, 30, 0.0),
    ("V16", "EDGE", "distance 25.0 m boundary + acc 60",     1, 25.0, 0.0, 0.0, None,  60, 0, 0.0),
    ("V17", "EDGE", "distance 25.01 m + acc 60",             1, 25.01, 0.0, 0.0, None, 60, 0, 0.0),
    ("V18", "EDGE", "TTC 2.5 s boundary + acc 75",           1, 40.0, 30.0, 16.0, 2.5, 75, 0, 0.0),
    ("V19", "EDGE", "TTC 2.51 s + acc 75",                   1, 40.0, 30.0, 15.9, 2.51, 75, 0, 0.0),
    ("V20", "EDGE", "rate 649.9 %/s + distance 9 m",         1,  9.0, 0.0, 0.0, None,  10, 0, 649.9),
    ("V21", "EDGE", "rate 650.0 %/s + distance 9 m",         1,  9.0, 0.0, 0.0, None,  10, 0, 650.0),
    ("V22", "EDGE", "negative closing: TTC ignored",         1,  9.0, 30.0, -2.0, None, 95, 0, 0.0),
    ("V23", "EDGE", "no obstacle + acc 95",                  0,  0.0, 50.0, 0.0, None,  95, 0, 0.0),
    ("V24", "TC05", "back to normal",                        1, 40.0, 0.0, 0.0, None,   0, 0, 0.0),
]


def run():
    bns, pms, rm, cm = BNSCalculator(), PMSCalculator(), RiskManager(), CommandMapper()
    rows = []
    for vid, tc, desc, present, dist, spd, closing, ttc, acc, brk, rate in VECTORS:
        if not present:
            closing, ttc = 0.0, None
        trace = []
        # Pdl_AccelRate is int16 * 0.1 on CAN: decode exactly like canbus/decoder.py
        rate_can = int(round(rate * 10)) * 0.1
        for _ in range(HOLD_EVALS):
            bns.calculate(dist * 100.0 if present else None, bool(present), spd, ttc,
                          closing_speed_mps=closing, environment_connected=True)
            pms.calculate(acc, rate_can, True, 0x00, brake=brk, brake_rate=0.0)
            total, level = rm.evaluate(
                bns.score, bns.available, pms.score, pms.available,
                (bns.distance_l1, bns.ttc_l1, pms.pedal_l1),
                (bns.distance_l2, bns.ttc_l2, pms.pedal_l2),
                (bns.distance_l3, bns.ttc_l3, pms.pedal_l3))
            if not trace or trace[-1] != level:
                trace.append(level)
        cmd = cm.map(level, rm.available, False)
        h, m = cmd["haptic"], cmd["motor"]
        rows.append(dict(id=vid, tc=tc, desc=desc, present=present, dist=dist, speed=spd,
                         closing=closing, ttc=-1.0 if ttc is None else ttc,
                         accel=acc, brake=brk, rate=rate,
                         bns=round(bns.score, 6), pms=round(pms.score, 6), total=round(total, 6),
                         cond_l1=sum(map(bool, (bns.distance_l1, bns.ttc_l1, pms.pedal_l1))),
                         cond_l2=sum(map(bool, (bns.distance_l2, bns.ttc_l2, pms.pedal_l2))),
                         cond_l3=sum(map(bool, (bns.distance_l3, bns.ttc_l3, pms.pedal_l3))),
                         level=level, levels=" > ".join(f"L{x}" for x in trace),
                         vib=h["vibration_command"], intensity=h["intensity"], frequency=h["frequency"],
                         mode=m["control_mode"], limit=m["output_limit"], flags=m["command_flags"]))
    return rows


def write_cin(rows, path):
    n = len(rows)
    def arr(ctype, name, key, fmt="{}"):
        return f"  {ctype} {name}[{n}] = {{" + ", ".join(fmt.format(r[key]) for r in rows) + "};"
    lines = [
        "/*@!Encoding:65001*/",
        "// Generated by canoe/tools/risk_reference.py from safety_ecu (do not edit by hand)",
        "variables",
        "{",
        f"  const long REF_COUNT = {n};",
        arr("long", "refPresent", "present"),
        arr("float", "refDist", "dist", "{:.4f}"),
        arr("float", "refSpeed", "speed", "{:.4f}"),
        arr("float", "refClosing", "closing", "{:.4f}"),
        arr("float", "refTtc", "ttc", "{:.4f}"),
        arr("long", "refAccel", "accel"),
        arr("long", "refBrake", "brake"),
        arr("float", "refRate", "rate", "{:.1f}"),
        arr("float", "refBns", "bns", "{:.6f}"),
        arr("float", "refPms", "pms", "{:.6f}"),
        arr("float", "refTotal", "total", "{:.6f}"),
        arr("long", "refLevel", "level"),
        arr("long", "refVib", "vib"),
        arr("long", "refIntensity", "intensity"),
        arr("long", "refFrequency", "frequency"),
        arr("long", "refMode", "mode"),
        arr("long", "refLimit", "limit"),
        arr("long", "refFlags", "flags"),
        "}",
        "",
    ]
    Path(path).write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    rows = run()
    here = Path(__file__).resolve().parent
    write_cin(rows, here.parent / "CAPL" / "PACE_RefVectors.cin")
    with open(here / "risk_reference.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(f'{r["id"]} {r["tc"]:5} L{r["level"]} ({r["levels"]:<17}) cond {r["cond_l1"]}/{r["cond_l2"]}/{r["cond_l3"]} '
              f'BNS {r["bns"]:.3f} PMS {r["pms"]:.3f} HAP {r["vib"]}/{r["intensity"]}/{r["frequency"]} '
              f'MOT {r["mode"]}/{r["limit"]}/0x{r["flags"]:02X}  {r["desc"]}')
