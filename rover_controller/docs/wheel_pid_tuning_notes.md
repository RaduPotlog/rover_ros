# Wheel-speed PID tuning notes (session handoff)

Working notes from the **wheels-lifted** tuning session (2026-09-20), kept so the next
session can pick up without re-deriving anything. Gains from this session are already in
`config/wheel_01_controller.yaml` (commit `a516610`).

**Status: step 1 of the README "Drive-train tuning" list is done for lifted wheels only.**
Steps 2 (acceleration limits) and 3 (skid-steer calibration) are untouched.
**Next action: re-run the same procedure with the wheels on the ground.**

---

## 1. Result

Final gains (all four wheels share p/i/d; `i_clamp` differs front/rear):

| wheels | p | i | d | feedforward_gain | i_clamp |
|--------|------|-----|------|------|---------|
| fl, fr | 0.05 | 1.0 | 0.04 | 1.0 | ±0.25 |
| rl, rr | 0.05 | 1.0 | 0.04 | 1.0 | ±0.33 |

`u_clamp ±12.58`, `antiwindup_strategy: back_calculation`, `save_i_term: false` — all unchanged.

Target was overshoot < 10 % and speed error < 3 %. Measured over **3 consecutive runs**,
median of 4 wheels, wheels lifted — worst case **overshoot 9.6 %, |speed error| 2.3 %**:

| step | overshoot (3 runs) | speed error (3 runs) |
|------|--------------------|----------------------|
| linear +0.20 | 1.3 / 0.8 / 2.4 % | −1.80 / −2.17 / −2.30 % |
| linear +0.40 | 4.6 / 4.6 / 5.3 % | +0.04 / −0.12 / −0.40 % |
| linear +0.60 | 5.8 / 6.2 / 6.0 % | +0.32 / +0.69 / +0.53 % |
| linear +0.80 | 7.2 / 6.9 / 7.0 % | +0.75 / +0.29 / +0.65 % |
| linear −0.40 | 9.2 / 8.9 / 8.7 % | +0.68 / −0.25 / +0.01 % |
| angular +0.50 | 2.8 / 3.3 / 2.4 % | −0.21 / −0.69 / −0.53 % |
| angular +1.00 | **9.6 / 8.5 / 9.0 %** | +0.90 / +0.52 / +0.58 % |
| angular −1.00 | 6.2 / 5.3 / 5.8 % | −0.22 / −0.15 / 0.00 % |

`angular +1.00` is the binding step, with only ~0.4 points of margin. Watch it on the ground.

Baseline before tuning (`p=0.05, i=1.0, d=0, i_clamp=0.4`) was already fine on speed error
(≤1.2 %) and failed only on overshoot: 6.6 / 10.2 / 11.6 / 11.8 / 12.9 / 8.9 / **14.4** / 12.3 %.

---

## 2. The model that explains the plant — read this before turning any knob

Four findings, each backed by a run. They are what make the gains above non-obvious.

### 2.1 Encoders are 20 Hz, so dead time is ~0.2 s and loop gain must stay tiny
Measured dead time was 0.15–0.26 s on every run (README says read these as ±50 ms).
The controllers run at 100 Hz, so each PID sees a new measurement only every ~5 cycles.

**P is nearly useless here.** `p=1.0, i=0.6, i_clamp=0.2` rang the loop to **39–87 % overshoot**
and doubled wheel accelerations (32–65 rad/s²). Do not raise `p` above ~0.1. All the
steady-state work has to be done by the integral.

### 2.2 The plant barely overshoots on its own — the controller causes the overshoot
Open loop (`p=i=d=0`, `ff=1.0`) is the single most informative run. Do it first on the ground too.

| step | open-loop overshoot | open-loop speed error |
|------|--------------------|------------------------|
| linear +0.20 | 0.0 % | **−21.3 %** |
| linear +0.40 | 1.9 % | −6.0 % |
| linear +0.60 | 3.8 % | −0.2 % |
| linear +0.80 | 4.5 % | +2.8 % |
| linear −0.40 | 2.3 % | −5.0 % |
| angular +0.50 | 1.6 % | −2.0 % |
| angular +1.00 | 8.4 % | +5.4 % |
| angular −1.00 | 0.0 % | −10.9 % |

### 2.3 Feedforward cannot be fixed with a single scale factor
Note the speed-error column above: the feedforward is **~21 % slow at 0.2 m/s** but
**~3 % fast at 0.8 m/s**. That is motor deadband / static friction at low speed, not a
scale error. Lowering `feedforward_gain` fixes the fast end and worsens the slow end.
**The integral is genuinely required.** Don't waste a run trying to trim `ff`.

### 2.4 `i_clamp`, not `i`, is what bounds overshoot
The integral winds up through the dead time (error is the full step amplitude the whole
time) and saturates. So:

```
peak overshoot  ~=  i_clamp / step amplitude          (+ the plant's own few %)
```

Evidence: overshoot barely moved between `i=1.0` (14.4 %) and `i=0.4` (12.3 %) on the big
steps, because the I-term hits the clamp either way. Dropping `i` only wrecked the speed
error (−7.0 % at 0.2 m/s) without fixing overshoot.

Direct confirmation, from the `output` column of `samples.csv` (`output − reference` ≈ I-term),
`i=1.5, i_clamp=0.25`:

```
linear +0.20   fl corr +0.255 err −1.83 %   fr corr +0.213 err +0.47 %
               rl corr +0.255 err −7.90 %   rr corr +0.258 err −10.65 %   <- pinned, still slow
linear +0.80   fl corr −0.001 err +0.36 %   fr corr −0.256 err +2.92 %    <- pinned, still fast
               rl corr +0.002 err −0.36 %   rr corr +0.030 err −0.06 %
```

**This is why `i_clamp` is per wheel.** The required trim is roughly *constant in rad/s*
(~0.26–0.39 to beat low-speed friction), while the overshoot budget is a *percentage of
amplitude*. One global clamp cannot serve both ends. Per-wheel analysis showed the wheels
needing authority (rl, rr) are exactly the ones with overshoot headroom:

Per-wheel overshoot at `d=0.04, i_clamp=0.4` (the 4-wheel median hides this):

| step | fl | **fr** | rl | rr |
|------|----|--------|----|----|
| linear +0.20 | 1.8 | **19.0** | 0.5 | 5.4 |
| linear +0.40 | 8.2 | **13.0** | 8.8 | 3.7 |
| linear +0.60 | 6.6 | **13.9** | 9.4 | 6.6 |
| linear +0.80 | 8.9 | **17.8** | 9.3 | 7.9 |
| linear −0.40 | 16.6 | **21.4** | 12.5 | 10.8 |
| angular +0.50 | 0.1 | **21.7** | 1.1 | 4.9 |
| angular +1.00 | 15.4 | **15.9** | 11.5 | 6.7 |
| angular −1.00 | 13.0 | **17.9** | 9.1 | 9.1 |

`fr` is the worst wheel in all 8 steps; `rr` never exceeds 10.8 %. Reverse is worse than
forward for every wheel. **If the on-ground numbers look odd, always check per wheel before
changing a gain** — use `perwheel.py` in §5.

### 2.5 `d=0.04` is at the noise limit — do not raise it
There is no derivative filter in `pid_controller`. `d` cuts overshoot without touching
steady state (exactly as theory predicts), but it amplifies 20 Hz encoder quantisation:

| d (with p=0.05, i=1.0) | result |
|------|--------|
| 0.00 | worst overshoot 14.4 % |
| 0.01 | worst 13.7 % |
| **0.04** | **linear steps 3.6–9.1 %, best value found** |
| 0.055 (front only) | `linear −0.40` got *worse*: 11.6 % |
| 0.08 | much worse — up to **29.5 %**, noise-driven |

---

## 3. Every run from this session (so nothing gets retried)

| # | p | i | d | i_clamp | worst overshoot | worst speed err | verdict |
|---|---|---|---|---------|-----------------|-----------------|---------|
| baseline | 0.05 | 1.0 | 0 | 0.4 | 14.4 % | 1.2 % | overshoot fails |
| 2 | 0.05 | 0.4 | 0 | 0.4 | 14.4 % | 7.0 % | both fail |
| 3 | 0 | 0 | 0 | — | 8.4 % | 21.3 % | open-loop reference |
| 4 | 1.0 | 0.6 | 0 | 0.2 | 87.2 % | 6.8 % | P far too hot |
| 5 | 0.05 | 1.0 | 0.01 | 0.4 | 13.7 % | 1.1 % | d helps a little |
| 6 | 0.05 | 1.0 | 0.04 | 0.4 | 14.5 % | 1.2 % | linear ok, angular fails |
| 7 | 0.05 | 1.0 | 0.08 | 0.3 | 29.5 % | 3.0 % | d noise, much worse |
| 8 | 0.05 | 1.0 | 0.04 | 0.25 | 8.9 % | **4.3 %** | overshoot ok, low-speed err fails |
| 9 | 0.05 | 1.5 | 0.04 | 0.25 | 8.9 % | **4.9 %** | raising i did not fix it (clamp-bound) |
| **10** | **0.05** | **1.0** | **0.04** | **0.25 / 0.33** | **9.6 %** | **1.8 %** | **PASS — shipped** |
| 11 | 0.05 | 1.0 | 0.04 | 0.22 / 0.33 | 10.0 % | 4.0 % | tighter front is worse on both |
| 12 | 0.05 | 1.0 | 0.055f/0.04r | 0.25 / 0.33 | 11.6 % | 1.9 % | more front d is worse |

Run-to-run variance is **~1–1.5 points of overshoot**, so treat anything within ~1.5 points
of 10 % as "not proven" and repeat the run. Runs 10/11 differ by one clamp step and landed
9.6 % vs 10.0 %.

---

## 4. Reproducing the rig (container was rebuilt — do these in order)

```bash
cd /root/ros2_ws/rover_a1
source /opt/ros/lyrical/setup.bash
source install/setup.bash
export ROVER_NAMESPACE=rover
ros2 launch rover_bringup rover_bringup.launch.py
```

Wait for `Configured and activated all the parsed controllers list : [...]` in the log.

### The E-Stop latch blocks all motion at startup — this will bite you
`motion_lock` is `true` (**true = motion inhibited**) after every boot because
`gpio_pin_sw_e_stop_latch_status` is latched. The step-response tool will publish happily
and the wheels will not move. Clear it:

```bash
ros2 service call /rover/hardware_interface/sw_e_stop_latch_reset std_srvs/srv/Trigger "{}"

# verify: latch_status false, motor_contactor_engaged true, motion_lock false
ros2 topic echo /rover/hardware_interface/gpio_state --once
ros2 topic echo /rover/motion_lock --once
```

Re-engage when finished: `ros2 service call /rover/hardware_interface/sw_user_e_stop_set std_srvs/srv/Trigger "{}"`

### Environment gotchas
- **`ros2 control ...` CLI is not installed** in this image. Use `ros2 param`/`ros2 topic`/
  `ros2 service`, or the ros-mcp tools, to inspect controllers.
- **rosbridge is on `127.0.0.1:9090`** for ros-mcp (`connect_to_robot`).
- `ros2 param get` occasionally prints a harmless
  `zenoh ... Received ResponseFinal for unknown Request` warning and returns nothing —
  just retry; the helper scripts below re-read and print every gain so you can eyeball it.
- The battery node logs `levelOneCellVoltageTooHigh` / `levelOnePackVoltageTooHigh`
  continuously at 100 % charge, and the modbus driver logs `Coil engage is not allowed`
  while the latch is held. Both are unrelated to tuning — **not** caused by these changes.

### Running the tool
```bash
ros2 run rover_controller wheel_step_response --ros-args -r __ns:=/rover \
  -p enable_motion:=true -p output_dir:=/tmp/tuning/run_name
```
Without `enable_motion:=true` it is a dry run and moves nothing. Default schedule is
5 linear + 3 angular steps, 3 s hold / 2 s rest, ~45 s total. Results: `summary.yaml`
and `samples.csv` (columns `wheel,t,reference,feedback,output`).

---

## 5. Helper scripts (were in a temp dir; re-create as needed)

`setpw.sh` — set all four wheels, per-wheel `i_clamp`, then read every gain back:

```bash
#!/bin/bash
# usage: setpw.sh <p> <i> <d> <ff> <clamp_fl> <clamp_fr> <clamp_rl> <clamp_rr>
source /opt/ros/lyrical/setup.bash
source /root/ros2_ws/rover_a1/install/setup.bash
P=$1; I=$2; D=$3; FF=$4
declare -A C=( [fl]=$5 [fr]=$6 [rl]=$7 [rr]=$8 )
for w in fl fr rl rr; do
  n=/rover/pid_controller_${w}_wheel_base_to_${w}_wheel_joint
  j=${w}_wheel_base_to_${w}_wheel_joint
  for kv in p:$P i:$I d:$D feedforward_gain:$FF i_clamp_max:${C[$w]} i_clamp_min:-${C[$w]}; do
    timeout 10 ros2 param set $n gains.$j.${kv%%:*} ${kv##*:} >/dev/null 2>&1
  done
done
for w in fl fr rl rr; do
  n=/rover/pid_controller_${w}_wheel_base_to_${w}_wheel_joint
  j=${w}_wheel_base_to_${w}_wheel_joint
  printf "  %s: " $w
  for g in p i d feedforward_gain i_clamp_max; do
    printf "%s=%s " $g "$(timeout 10 ros2 param get $n gains.$j.$g 2>/dev/null | grep -oE '[-0-9.]+$')"
  done; echo
done
```

`perwheel.py` — per-wheel overshoot from a run directory, reusing the repo's own
`analyze_step` so the numbers match the tool exactly. **This is the one that found `fr`.**

```python
import csv, sys, os
sys.path.insert(0, '/root/ros2_ws/rover_a1/src/rover_ros/rover_controller')
from rover_controller.response_analysis import analyze_step, Sample

run = sys.argv[1]
rows = {}
with open(os.path.join(run, 'samples.csv')) as f:
    for r in csv.DictReader(f):
        rows.setdefault(r['wheel'], []).append(
            (float(r['t']), float(r['reference']), float(r['feedback'])))

labels = ['linear +0.20','linear +0.40','linear +0.60','linear +0.80','linear -0.40',
          'angular +0.50','angular +1.00','angular -1.00']
hold, rest = 3.0, 2.0
t, segs = 0.0, []
for lab in labels:
    t += rest
    segs.append((lab, t, t + hold))
    t += hold

print(f"{'step':16s} " + " ".join(f"{w.split('_')[2]:>16s}" for w in sorted(rows)))
for lab, s, e in segs:
    cells = []
    for w in sorted(rows):
        win = [Sample(tt, ref, fb) for tt, ref, fb in rows[w] if s - rest/2 <= tt < e]
        try:
            m = analyze_step(win, s)
            cells.append(f"os{m.overshoot*100:5.1f} t{m.target:+5.2f}")
        except ValueError:
            cells.append(" " * 16)
    print(f"{lab:16s} " + " ".join(f"{c:>16s}" for c in cells))
```

`iterm.py` — is the integral pinned at the clamp? Prints `output − reference` per wheel over
the last 0.5 s of a window. Usage: `iterm.py <run_dir> <seg_start_s> <seg_end_s>`
(e.g. `2.0 5.0` for `linear +0.20`, `17.0 20.0` for `linear +0.80`).

```python
import csv, sys, os
run, s, e = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
rows = {}
with open(os.path.join(run, 'samples.csv')) as f:
    for r in csv.DictReader(f):
        rows.setdefault(r['wheel'], []).append(
            (float(r['t']), float(r['reference']), float(r['feedback']), float(r['output'])))
print(f"window {s}-{e}s   (output-reference ~= I-term contribution)")
for w in sorted(rows):
    win = [x for x in rows[w] if e - 0.5 <= x[0] < e]
    if not win: continue
    ref = sum(x[1] for x in win)/len(win)
    fb  = sum(x[2] for x in win)/len(win)
    out = sum(x[3] for x in win)/len(win)
    print(f"  {w.split('_')[2]:3s} ref={ref:+6.3f} fb={fb:+6.3f} out={out:+6.3f} "
          f"corr={out-ref:+6.3f} err%={(fb-ref)/abs(ref)*100:+6.2f}")
```

---

## 6. Plan for the on-ground run

Gains here are fitted to **lifted, unloaded wheels**. On the ground, inertia and load rise,
low-speed friction changes, and slip appears. Expect the numbers to move.

1. **Clear the area** — on the ground the rover drives ~0.2–0.8 m/s forwards and back and
   spins in place. The schedule is ~45 s.
2. **Run the shipped gains first** as the on-ground baseline, twice (variance is ~1.5 points).
3. **Run open loop** (`p=i=d=0`) once. §2.2's table is the lifted reference; the on-ground
   version tells you how much of any new overshoot is the plant vs the controller, and
   re-measures the low-speed friction that sets the required `i_clamp`.
4. **If overshoot rises**, lower `i_clamp` (front first — `fr` is the worst wheel).
   Do **not** reach for `p`, and do **not** raise `d` above 0.04. See §2.1 and §2.5.
5. **If low-speed speed error rises**, raise `i_clamp` on the wheels that `iterm.py` shows
   pinned. Loaded wheels need more friction trim, so `rl/rr` 0.33 may need to go up.
6. **Always check `perwheel.py`** before changing a gain — the median hides single-wheel outliers.

### Then step 2 of the README (acceleration limits) — not started
The tool reports the largest wheel acceleration it saw. Lifted-wheel numbers are **not**
usable for this; they need the on-ground run. For reference, the shipped gains reported
`wheel_accel 13.5–14.9 rad/s²` → `linear.x 1.79–1.93 m/s²`, `angular.z 3.81–4.11 rad/s²`,
while the config currently limits `linear.x 2.7` / `angular.z 3.74`. Since the measured
value sits at/below the configured limit, **the controller ramp was likely the bottleneck,
not the wheels** — per the README, relax those limits for one measurement run and repeat
before setting anything.

### Then step 3 (skid-steer calibration) — not started
`wheel_odom_calibration`, on the real surface. `wheel_separation_multiplier` is currently 1.63.
