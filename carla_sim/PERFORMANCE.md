# CARLA safety-path performance check

The CARLA control loop is asynchronous. It does not call `world.tick()` each
frame. `world.wait_for_tick(2.0)` runs once at startup. This optimization does
not change `synchronous_mode`, `fixed_delta_seconds`, Traffic Manager settings,
telemetry frequency, or safety-control behavior.

The first live profile showed `safety_telemetry` taking about 291-347 ms while
`predicted_ttc` took about 1.5-1.9 ms and `get_actors` under 1.1 ms. Escape
tracking now reuses that actor list instead of calling `world.get_actor()`.
However, a later profile still showed 166-232 ms in `safety_telemetry`, so
that lookup was not the only bottleneck. The profile now separately measures
speed extraction, signed speed, escape tracking, actor type and bounding-box
access, and the complete safety scan. The remaining delay must be attributed
from those live measurements before changing detection frequency or thresholds.

The detailed profile then identified `get_bounding_box` at about 6 ms per
lookup, repeated roughly 24 times per safety scan. The bounding boxes are
actor-local geometry; they are now cached by actor ID and removed when an
actor disappears from the world actor list. This does not change collision
geometry or the 10 Hz telemetry rate. Confirm with the
`2026-10-23-bbox-cache` profile on the CARLA PC.

Run the lightweight synthetic comparison on the Raspberry Pi:

```sh
cd /home/pi/PACE
python3 carla_sim/benchmark_safety_prediction.py \
  --baseline-archive /home/pi/PACE_risk_v3_before_deploy_20261023.tar.gz
```

That archive is the pre-risk-v3 version. To compare against the slow risk-v3
version, pass its saved script with `--baseline-source PATH`. The benchmark
reports Python computation time and getter call counts, **not** CARLA FPS.

On the CARLA PC, set `PACE_PERF_PROFILE=1` before launching the driving script.
Every five seconds `[PACE PERF]` reports:

- `server_tick_fps`: CARLA frame-number change per wall-clock second.
- `client_loop_fps`: completed pygame loop iterations per second.
- `client_work`, `client_pacing`, `camera_render_flip`, `apply_control`,
  `safety_telemetry`, `predicted_ttc`, `safety_udp`, `hmi_udp`, and
  `status_output`: mean/maximum milliseconds and sample count.
- `get_actors`, `get_location`, `get_transform`, `get_velocity`, and
  `get_angular_velocity`: calls and time inside the safety scan.

The displayed FPS in the pygame title is client-loop FPS, not server FPS.
`get_snapshot` is sampled only for diagnostics. `world.tick()` is not called by
the driving loop. A low `server_tick_fps` with normal client-work time suggests
a CARLA server, rendering, traffic, physics, or GPU bottleneck. A high
`safety_telemetry` or `predicted_ttc` time indicates the safety scan. A high
`camera_render_flip` time indicates the pygame rendering path.

For HMI rendering, launch the Pi HMI with `PACE_HMI_FPS_LOG=1`; the separate
`[PACE HMI PERF] render_fps` line counts Qt frame swaps every five seconds.
Compare all four rates while CARLA is running, with the same map, traffic,
camera resolution, weather, and HMI page. Inspect CARLA-PC CPU/GPU utilization
with Task Manager or `nvidia-smi` if the PC has an NVIDIA GPU. These GPU
measurements cannot be inferred from Pi CPU readings.
