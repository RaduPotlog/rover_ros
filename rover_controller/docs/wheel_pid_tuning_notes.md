# Wheel-speed PID tuning notes (session handoff)

Working notes from the **wheels-lifted** tuning session (2026-09-20), kept so the next
session can pick up without re-deriving anything. Gains from this session are already in
`config/wheel_01_controller.yaml` (commit `a516610`).

> **Partly superseded (2026-10-02).** The `p`, `i` and `d` findings (§2.1–§2.3, §2.5) still
> hold. The `i_clamp` values and §2.4 / §6 describe the *plain* integral. On the ground
> (2026-09-29) in-place turns ran 37–56 % under the reference with every I-term pinned, so
> `SeededPidController` gained wheel loop options (`stop_at_zero_reference`, a model-reference
> integral `integral_reference_delay` / `_time_constant`, `scale_integral_with_reference`) and
> `i_clamp` became ±2.0 on all four wheels. The ground tune and its results are in the comment
> above the gains in `config/wheel_01_controller.yaml`; the options are in the README.
> `wheel_separation_multiplier` is now 1.659: the calibrated 1.63, rescaled to a measured
> separation of 0.615 m (see the comment in the controller config).

> **Update (2026-10-06, wheels lifted, 50 Hz).** Shipped gains (`i_clamp` ±2.0, model-reference
> integral) re-measured: median worst overshoot 4.9-5.7 %, speed error <= 1.7 %. Open loop only
> `fr` overshoots (5.2 % at +0.60, 7.8 % at +0.80); closed loop `fr` peaked at 9.7-11.8 % on
> +0.80. `fr` `feedforward_gain` 0.96 (others 1.0) brought that to 7.7 / 6.2 %, median worst
> 4.7-4.9 %, speed error <= 1.1 % (two runs). Ground re-check still pending. Gotcha: `ros2 param
> set` needs doubles (`0.0`, not `0`) or it fails with exit code 0; the CLI needs `--no-daemon`.

> **Skid-steer calibration (2026-10-06/07, ground; README step 3). No config change.** Spin
> (`wheel_odom_calibration mode:=spin`, `imu_yaw_sign` -1): `wheel_separation_multiplier` 1.672 (median
> of 6; mean of the four 0.6 / 1.0 rad/s segments 1.666, five-segment mean 1.654) against the
> shipped 1.659. The -0.30 rad/s segment barely turned the rover (stiction, 5.0) and is not real; the
> slow +/-0.3 segments are unreliable. Straight (2.0 m at 0.3 m/s, two runs, tape-measured 2.2 m
> both): wheel odometry 2.196 / 2.228 m after coasting (2.007 / 2.004 m at the stop command, so
> compare the tape with the post-coast figure) -> radius multiplier 1.002 / 0.987, mean 0.995. That is
> inside the tape resolution (~1 cm), so left/right radius multipliers stay 1.0. The tool's own distance
> readout stops at the stop command and misses the coast; capture the odometry pose after the rover
> has stopped (watchdog script) before dividing.

> **Long-hold baseline (2026-10-10, ground, 10 m lane, battery 57 %, 25 Hz, `motor_acceleration` 2.0).**
> Shipped gains, two identical runs: 6 s holds / 2.5 s rest, straights alternating +-0.4 / +-0.6 /
> +-0.8 m/s (out and back, at most ~5 m from the start), then spins +-0.6 / +-1.0 rad/s. *Straights:*
> median overshoot 1.8-4.2 % / 2.1-3.4 %, worst wheel 8.4 % (fr -0.40, run 1 only; 2.8 % in run 2) /
> 4.3 %; settled error (mean of the last 2 s) <= 0.7 % on every wheel; rise 1.6-1.7 s, dead time
> 0.22-0.30 s; output / reference 1.10-1.14 at 0.4 m/s, ~1.0 at 0.8 m/s (fr 0.93-0.99, its `ff` 0.96).
> The 3.5 s holds' -1.4..+1.2 % end error was partly the slow rise. *Spins:* median overshoot 6.6-8.2 %
> / 3.9-11.9 %, worst wheel 17.6 % (fl +0.6) / 14.7 %; settled -1.0..-5.3 % / -2.5..-7.2 % under,
> per wheel -8..+11 %; output / reference 1.8-2.1x at +-0.6 rad/s with fr / rl at 2.09-2.10, i.e. the
> integral pinned at `i_clamp` 2.1, and 1.5-1.7x at +-1.0. Spins vary ~4 points run to run (straights
> ~1). The tool's suggested limits (0.50 m/s^2 / 0.97 rad/s^2) come from the spin rise and do not
> apply (see phases 2-3). **Shipped gains kept**; spins still need the turn feed-forward (pending).
> *Open loop, same schedule* (p = i = d = 0 live, `ff` unchanged, gains restored after; battery 53 %):
> straights settle 11-14 % slow at +-0.4 m/s, 3-6 % at +-0.6, 0-3 % at +-0.8, the four wheels within
> ~0.5 % of each other; overshoot 0-2.7 %, so the closed loop's 2-4 % comes from the integral through
> the ~0.3 s dead time, as on lifted wheels. Spins: the wheels reach 0-12 % of the reference at
> +-0.6 rad/s (wheel reference 1.86 rad/s, i.e. stalled) and ~45 % at +-1.0 (3.1 rad/s). Both fit a
> **constant scrub offset of ~1.7 rad/s of wheel command, independent of the spin rate**, and the
> closed-loop runs agree: the controller added 1.8-1.9 rad/s above the reference at both rates. So
> the spin overshoot is all controller: the integral must build ~1.8 of its 2.1 clamp before the
> wheels break loose. Starting value for the turn feed-forward: ~1.8 rad/s per wheel, signed by the
> wheel's turn component, faded in with the angular share of the command (design pending).

> **Turn feed-forward lane runs (2026-10-11, ground, battery 49-46 %, same 6 s schedule).** Implemented
> as the `turn_feedforward` wheel-loop option (`72cee8a`): `turn_side * sign(w) * turn_feedforward`,
> ramped in to 0.3 rad/s, scaled by the turn's share of the wheel speed, fed from
> `rover_drive_controller/cmd_vel_out` (25 Hz confirmed during the runs, peak |w| 1.00). One run each,
> set live on all four wheels, restored to 0 after. Median of 4 wheels:
>
> | | off (A) | 1.0 (B) | 1.8 (C) |
> |---|---|---|---|
> | +-0.6 rad/s overshoot | 10.4 / 14.8 % | 7.6 / 9.0 % | 8.5 / 5.5 % |
> | +-0.6 rad/s rise | 2.07 / 2.06 s | 1.90 / 1.96 s | 1.08 / 1.52 s |
> | +-0.6 rad/s settled error | -3.9 / -6.9 % | -0.6 / -2.0 % | -1.3 / -1.1 % |
> | +-1.0 rad/s overshoot | 4.2 / 4.2 % | 4.6 / 4.6 % | 7.0 / 8.6 % |
> | spin dead time | 0.37-0.47 s | 0.31-0.37 s | 0.29-0.33 s |
> | worst single wheel (spins) | 19.3 % | 10.7 % | 10.5 % |
>
> Straights unchanged in all three (overshoot <= 3.9 %, one 6.6 % wheel in B; settled <= 0.6 %); A
> reproduces the 2026-10-10 baseline, so the option is inert at 0. At 1.8 the end-of-spin output equals
> reference + feed-forward (integral ~0), confirming the ~1.8 scrub offset, but +-1.0 rad/s spins
> over-drive. **Shipped 1.0**: better at 0.6 rad/s, no change elsewhere. B vs C is within the ~4-point
> spin variance; repeat both before moving towards 1.8 (or try 1.4).

> **Ground campaign at `motor_acceleration` 2.0, phases 4-5 (2026-10-09, battery 47 %).** *Top speed:* +-0.95 m/s
> is reached within 0.5 % on all wheels (t50 0.67 s, output 1.00-1.01x the reference). *Spin breakaway*
> (3.5 s holds, IMU sign-corrected): every rate turns closed loop, down to 0.3 rad/s (IMU 0.28-0.29, t50
> 2.7-2.9 s, output 3.0x the reference); output / reference falls to 1.6x at 1.0 rad/s, i.e. a roughly
> constant breakaway effort. IMU / command 0.58-1.02 between 0.45 and 1.0 rad/s. Per-wheel end errors show
> a diagonal pattern (+ spins: fl / rr -27..-46 %, fr / rl ahead; - spins mostly fr / rl behind) - a
> diagonal pair unloading (chassis rocking on the floor or mass distribution), not a gain issue. The
> `rover_nav_params.yaml` note that the rover stalls below ~1.0 rad/s (2026-09-26) no longer holds.
> *Skid-steer calibration* (`wheel_odom_calibration` spin, 7 s holds, +-0.6 / 1.0 / 1.5 rad/s, 0 rejected):
> `wheel_separation_multiplier` 1.644 (segments 1.616-1.723) against the shipped 1.659 - within the spread
> and the 2026-10-06/07 range, **kept**. With 7 s holds spins reach 90-96 % of the command. *Nav 2 limits*
> (velocity smoother, MPPI ax / az, behaviour server rotational_acc_lim) all match 1.4 / 1.3: unchanged.

> **Ground campaign at `motor_acceleration` 2.0, phases 2-3 (2026-10-09, battery 45-50 %, 3.5 s holds,
> +-0.2..0.8 m/s and +-0.6 / +-1.0 rad/s, alternating directions).** *Gains:* two shipped baselines agree:
> straights t50 0.36-0.62 s, t90 1.9-2.1 s, peak overshoot <= 5.6 % (worst wheel 9.3 %, only at 0.2 m/s),
> end error <= 1.7 %. Open loop all four wheels match within ~1 % (DC gain 0.63 / 0.86 / 0.93 / 0.965 at
> 0.2 / 0.4 / 0.6 / 0.8 m/s), so no per-wheel feed-forward change (`fr` 0.96 is not worse closed loop).
> Spins +-0.6 rad/s: open loop the wheels do not move at all; closed loop output ~2.0x the reference,
> stick-slip, single wheels +41 % / -59 %. `i_clamp` 3.0 (live): straights unchanged, spin +-0.6 end error
> better (+0.1 / -6.8 %) but overshoot worse (median 10-16 %, worst 46 %), +-1.0 overshoot 6-9 %: rejected,
> as on 2026-10-06; the spin output only needed ~1.9 of integral. **Shipped gains kept.**
> *Acceleration limits* (relaxed to 20 / 40, wheel odometry + IMU, two runs): the response is lag-limited,
> not acceleration-limited - the 10-90 % rise is ~1.3-1.4 s at every step while the peak (0.2 s window)
> grows with step size (1.2 / 1.9 / 2.5 m/s^2 at 0.4 / 0.6 / 0.8 m/s; spins 1.1-1.5 rad/s^2 at 1.0, 2.7-3.3
> at 1.5 rad/s). The tool's "recommended" limits come from the slowest 10-90 % average (~0.2 m/s^2) and do
> not apply. **1.4 m/s^2 / 1.3 rad/s^2 kept.** Spins at 1.5 rad/s reach 1.39-1.45 rad/s and IMU yaw rate
> matches wheel odometry within 5 % (separation multiplier holds); at 1.0 rad/s they reach only 0.75-0.94 and
> IMU / wheel yaw rate scatters 0.76-1.18 (slip). The IMU `linear_acceleration.x` stayed ~0 +- 0.06 m/s^2
> during every acceleration, so the straight-line slip check was inconclusive (axis / frame to check).

> **In-place turns are slow to start (2026-10-09, ground, shipped gains, 3.5 s holds).** Operators report
> that spinning from the joystick / RC is sluggish and then overshoots. Spin +-1.0 rad/s (wheel reference
> 3.1 rad/s): the reference reaches 90 % in 0.71-0.76 s (the 1.3 rad/s^2 limit), the wheels reach 50 %
> only after 0.9-1.8 s and 90 % after 2.2-3.0 s (straight +0.6 m/s: 0.47-0.54 s / 1.9-2.0 s). Peak
> controller output is 1.5-1.9x the reference on spins against 1.05-1.12x on straights, i.e. a spin
> needs ~0.6x the reference of extra drive (skid scrub) that only the clamped integral (2.1) supplies, and
> it only starts after the integral model's ~0.5 s hold-off. After release the wheels drop below 10 % in
> 0.65-0.81 s and then kick back 0.12-0.31 rad/s (straights: 1.2 s, no kick). The acceleration limit is
> not the bottleneck. **Rejected:** `i` 2.0 (all wheels, live): spin time-to-50 % unchanged (1.33-1.49 s),
> spins oscillate (peak +21..+36 %, end -9..-21 %, worst at +-0.6 rad/s), straights 90 % in 1.6 s but
> overshoot 6.5 %. Gains cannot fix this; it needs a turn feed-forward (design pending), not a faster or
> larger integral.

> **Integral model re-check at `motor_acceleration` 2.0 (2026-10-09, ground).** Shipped gains, 3.5 s holds,
> +-0.4 / +-0.6 m/s and +-1.0 rad/s, median of the four wheels: end-of-hold error -1.4 .. +1.2 % and peak
> overshoot <= 3.4 % on the straight steps, spins 5.5 / 9.7 % under, t90 ~2.0-2.4 s. A 2 s hold (first
> baseline) is shorter than the rise and shows a false -13 % "steady-state" error. Open loop (p=i=d=0,
> ff 1.0) gave DC gains 0.86-1.02 on straight steps and 0.2-0.45 on spins (skid-steer friction); a
> delay + first-order fit was poor (RMSE 0.14-0.40 rad/s, lag 0.24-0.54 s depending on step), so open-loop
> delay/lag fits do not transfer to this model. The 0.14 / 0.35 candidate was worse closed loop (spins
> 5.4 / 13.7 % under, one 6.7 % overshoot, slower t90). Model left at 0.40 / 0.12. The acceleration limits
> are not re-measured yet.

> **Acceleration limits (2026-10-06, ground).** Measured with limits relaxed to 20 / 40: the controller
> ramp (16 rad/s^2) was not the bottleneck, the wheels were (sustained 10-90 % slope 1.5-11 rad/s^2, best
> 10.2 at 0.75 m/s). Shipped `linear.x` 1.4 m/s^2, `angular.z` 1.3 rad/s^2 (were 2.7 / 3.74). The
> tool's peak "wheel accel" is encoder quantisation noise - use the rise slope.

> **Rejected (2026-10-06):** `i_clamp` 3.0 + `linear.x` 0.75 m/s. Straight steps unchanged; turns got
> worse (single-wheel overshoot 22-38 % vs <= 10 % at 2.1, error +-20 % both signs). Kept 2.1 / 0.95.
> **Ground re-fit (2026-10-06).** `motor_acceleration` is now 1.0 (was 10.0 when the integral model
> was fitted), so the 0.15 / 0.08 s model was ~0.3 s too fast and straight steps overshot 9-21 %.
> Measured plant ~0.45 s delay + 0.14 s lag; `integral_reference_delay` 0.40 /
> `_time_constant` 0.12 gave straight overshoot <= 2.6 %, |error| <= 2.0 %. `i_clamp` 3.0 helped turns
> but exceeds the full-duty guard (<= 2.17), so the shipped clamp is 2.1. Turn per-wheel scatter
> (+-12-25 %) is skid-steer noise, not a gain limit.

**Status at the time: step 1 of the README "Drive-train tuning" list was done for lifted
wheels only.** Steps 2 (acceleration limits) and 3 (skid-steer calibration) were untouched.

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

### 2.1 Encoders are 10 Hz, so dead time is ~0.2 s and loop gain must stay tiny
Measured dead time was 0.15–0.26 s on every run (README says read these as ±50 ms).
The gains in this file were tuned with the controllers at 100 Hz, where each PID saw a new
measurement only every ~10 cycles. The manager now runs at 50 Hz (~5 cycles per measurement).
The integral is rate-independent, but the D term's spike on each fresh sample is about half as
large, so re-run the step response at 50 Hz before trusting the overshoot numbers here.

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
steady state (exactly as theory predicts), but it amplifies 10 Hz encoder quantisation:

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
export ROVER_SYSTEM_NAMESPACE=rover
ros2 launch rover_bringup rover_bringup.launch.py
```

Wait for `Configured and activated all the parsed controllers list : [...]` in the log.

### The E-Stop latch blocks all motion at startup — this will bite you
`motion_lock` is `true` (**true = motion inhibited**) after every boot because the safety
latch is set. The step-response tool will publish happily and the wheels will not move.
Clear it:

```bash
ros2 service call /rover/hardware_interface/sw_e_stop_latch_reset std_srvs/srv/Trigger "{}"

# verify: latch_active false, motor_contactor_engaged true, motion_lock false
ros2 topic echo /rover/hardware_interface/safety_status --once
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
`wheel_odom_calibration`, on the real surface. `wheel_separation_multiplier` was 1.63 at the
time; it is now 1.659, the calibrated value rescaled to a measured separation of 0.615 m.
