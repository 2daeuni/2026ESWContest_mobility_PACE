# Safety risk policies

CARLA mode now uses `DirectionalRiskPolicy`; hardware mode retains the initial
BNS/PMS 2-of-3 policy described below. CARLA telemetry must include `gear`
(`D`, `R`, or `P`) and report the earliest predicted vehicle/pedestrian collision
in the selected travel direction. The CARLA sender predicts short-horizon
oriented-box overlap from actor position, velocity, yaw rate and ego steering;
it also sends signed speed and the prior target's gap for escape confirmation.
The Windows control receiver also requires a matching gear in each
Safety command, and stops during a gear/command mismatch or stale command.
Invalid CARLA risk inputs command STOP rather than normal output.

For a closing target, CARLA L3 is immediate if TTC is at most 1.5 s or gap is
at most `closing_speed * 0.3 + closing_speed**2 / (2 * 6.0) + 1.5` metres.
L2 applies within 1.5 times that distance or TTC <= 2.5 s; L1 applies within
2.5 times that distance or TTC <= 4 s. L1/L2 require 0.2 s of continuous
evidence; lowering one level requires 0.5 s, shortened to 0.25 s only when
braking plus measured deceleration and improving gap/TTC/closing speed, or
verified movement away from the previous target, are observed. Gear changes
alone do not clear risk. Closing speeds at or below 0.4 m/s (apart from contact)
do not raise collision level; BNS ignores non-closing distance. BNS is reduced
15% for confirmed braking improvement outside L3. Receding targets do not raise
collision risk. Pedal misuse score remains visible but does not gate collision
intervention. L2 retains 30% output and 30 km/h CARLA speed limits; L3 retains
zero output and full CARLA braking. Rule reason, gear and required stopping
distance are logged by the Safety ECU.
Haptic commands continue at the control cadence; HMI snapshots are sent
immediately on a risk-level change even between normal 10 Hz updates.

In CARLA mode, validated braking (brake >=5%, accelerator <=3%) can clear
L1/L2/L3 directly to L0 if the current collision candidate is L0 and the
vehicle is stopped (<=0.5 km/h with relative closing <=0.1 m/s), or measured
deceleration is >=0.2 m/s² and any remaining target is not approaching faster
than 0.4 m/s. A controlled stop also ignores a zero TTC caused solely by
stationary bounding-box contact; a target still approaching above 0.1 m/s,
continued accelerator input, or invalid/stale telemetry blocks that exception.
After a gear change with a tracked prior hazard, braking alone does not take
the direct-release path; the existing escape/downward confirmation applies.
These release values are class constants in `DirectionalRiskPolicy`, not changes
to the existing L1/L2/L3 hazard thresholds. Without this evidence, ordinary
downward hysteresis remains unchanged. The Haptic ECU receives an explicit
0x110 OFF command and its 0x200 applied-OFF status is logged; HMI UDP publishes
the new level without waiting for the 10 Hz interval and stops active audio.
Motor E-STOP recovery and all Fail-Safe overrides remain independent of L0.

The 6.0 m/s² deceleration, 0.3 s delay and 1.5 m clearance are provisional
simulation parameters, not measured braking performance or a safety
certification. Calibrate in CARLA before relying on these thresholds. Actor
detection does not cover walls, unmodeled static geometry or occluded objects.
The 6 s constant-turn/constant-speed forecast is a conservative simulation
approximation, not a validated intersection planner. PC and Pi source updates
must be installed together; older telemetry omits escape evidence and therefore
uses the ordinary 0.5 s downward hold.

## Initial hardware policy

All scores are 0..1. These are the requested initial test thresholds, not calibrated
vehicle stopping-distance models. No vehicle-speed multiplier has been invented.

| Level | Distance <= m | TTC <= s | Accelerator >= % | Accelerator rate >= %/s | Score anchor |
| --- | --- | --- | --- | --- | --- |
| L1 | 25 | 4.0 | 60 | 250 | .35 |
| L2 | 15 | 2.5 | 75 | 450 | .65 |
| L3 | 10 | 1.5 | 90 | 650 | .90 |

BNS uses the larger interpolated distance/TTC score. Distance and TTC conditions
remain independent. The main adapter passes distance in cm (converted back to m
inside BNS), CARLA speed in km/h, closing in m/s and TTC in seconds. Speed must be
finite/nonnegative. A supplied closing speed <=0 cannot activate a TTC condition.
Missing TTC is not inferred; distance can still combine with pedal risk.

PMS uses accelerator OR positive accelerator-rate thresholds at each level. CAN
0x100 decoder already converts both signed int16 rates with scale 0.1 to %/s;
there is no second rate conversion. Brake >=5% suppresses acceleration misuse.
Accelerator >=20% AND brake >=20% instead raises pedal_conflict, score .65 and
pedal L1/L2 (not L3). Brake rate is validated but is not independently misuse.
Sensor faults set PMS unavailable/score zero and remain handled by FailSafeManager.

Distance/TTC scores interpolate toward 1 at zero. PMS scores interpolate between
anchors and saturate at 1 one further L2-to-L3 interval above L3. These score
interpolation anchors do not change the boolean level thresholds.

The RiskManager keeps its 2-of-3 rule (distance, TTC, pedal), highest level first.
Total risk is max(BNS, PMS), bounded to [0,1]. A high total score alone cannot
raise a level. Upward confirmation is still 3 identical candidates; downward
confirmation is 5 identical lower candidates, one level per confirmation.
These are main-loop evaluation counts, not distinct CAN/UDP frame counts.

CARLA freshness is passed separately from distance_valid. A fresh no-obstacle
packet is BNS available with zero score/all conditions false. A fresh obstacle
with None TTC is also available. Stale CARLA is unavailable and existing failsafe
handling is unchanged. Pedal connection, status=0 and valid decoded inputs make
PMS available. Invalid inputs clear all cached conditions.

CommandMapper motor policy: L0/L1 Normal 100%, L2 Output Limit 30%, L3 Stop 0%
with Command Valid and E-STOP ON. Fail-Safe takes priority over every risk
level: Stop 0% with Command Valid and E-STOP ON. After E-STOP, recovery
requires released accelerator and a stopped motor before normal output resumes.
CAN layout and timeouts are unchanged.
MOTOR_COMMAND remains standard CAN ID `0x120`, DLC 8. Byte0=mode,
Byte1=limit %, Byte2=risk, Byte3=flags, Byte4..6=0, Byte7=alive counter.
Flags bit0=Command Valid, bit1=Emergency Stop.

| Policy | Mode | Limit % | Flags | CAN payload (AA = alive, RR = current risk) |
| --- | --- | --- | --- | --- |
| L0 | 0 | 100 | 0x01 | `00 64 00 01 00 00 00 AA` |
| L1 | 0 | 100 | 0x01 | `00 64 01 01 00 00 00 AA` |
| L2 | 1 | 30 | 0x01 | `01 1E 02 01 00 00 00 AA` |
| L3 | 2 | 0 | 0x03 | `02 00 03 03 00 00 00 AA` |
| Fail-Safe | 2 | 0 | 0x03 | `02 00 RR 03 00 00 00 AA` |

HMI keeps bns/pms/total_risk/risk_level, plus its already supported score_max=1.0
metadata so normalized values display correctly. [RISK] logs are limited to 2Hz.

Offline checks (no hardware or sockets):

```
.venv/bin/python -B -m unittest discover -s safety_ecu/tests -p test_risk_pipeline.py -v
.venv/bin/python -B safety_ecu/tests/test_risk_manager.py
.venv/bin/python -B safety_ecu/tests/test_command_mapper.py
.venv/bin/python -B safety_ecu/tests/test_failsafe.py
```

The new tests exercise A-H, threshold edges, unavailable reset, brake/conflict,
JSON CARLA decode/freshness, CAN rate decoding, and the actual main loop with
in-memory CAN/CARLA receivers, mocked CSV/HMI workers and a mocked CAN bus.
They inspect final CAN 0x120 bytes and HMI packets/normalization. They do not run
CARLA, CAN interfaces, network listeners, or real motors.
