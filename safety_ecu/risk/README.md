# Initial BNS/PMS trial policy

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
with Command Valid ON and E-STOP OFF. Fail-Safe takes priority over every risk
level: Stop 0% with Command Valid ON and E-STOP ON. Motor recovery, CAN layout and timeouts are unchanged.
MOTOR_COMMAND remains standard CAN ID `0x120`, DLC 8. Byte0=mode,
Byte1=limit %, Byte2=risk, Byte3=flags, Byte4..6=0, Byte7=alive counter.
Flags bit0=Command Valid, bit1=Emergency Stop.

| Policy | Mode | Limit % | Flags | CAN payload (AA = alive, RR = current risk) |
| --- | --- | --- | --- | --- |
| L0 | 0 | 100 | 0x01 | `00 64 00 01 00 00 00 AA` |
| L1 | 0 | 100 | 0x01 | `00 64 01 01 00 00 00 AA` |
| L2 | 1 | 30 | 0x01 | `01 1E 02 01 00 00 00 AA` |
| L3 | 2 | 0 | 0x01 | `02 00 03 01 00 00 00 AA` |
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
