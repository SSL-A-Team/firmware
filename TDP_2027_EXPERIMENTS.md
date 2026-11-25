# TDP 2027 motion experiments: runbook

How to collect the data and figures for the TDP motion section. The controller design is in
[MOTION_CONTROL_REPORT.md](MOTION_CONTROL_REPORT.md). Scripts are in
[controls/analysis/tdp/](controls/analysis/tdp/); **every command runs from `controls/`** in a
ROS-sourced shell unless noted.

## Plan

Two items in about 10 LNCS pages. TDPs weigh reproducibility, so each item needs equations,
parameter values, procedure and validation.

| Item | Claim | Figures | Experiments |
|---|---|---|---|
| **A. Feed-forward controller and how to calibrate it** (~4 pp) | One pipeline (profile, fit, A/B on tracking error) identifies friction, efficiency and slip without roll-outs. Friction scales with omni-roller speed k(dir); excess wheel speed is linear in current, and compensating for it cuts tracking error | `friction_direction`, `fit_efficiency`, `fit_slip`, `sweep_slip`, `sweep_slip_fleet`, per-robot parameter table | E1, E3, E5 |
| **B. Current limiting matched to the motors** (~3 pp) | A direction-preserving current budget, an I²t thermal mirror and feasible-acceleration planning keep the robot on its path where per-wheel clipping curves it | `feasible_accel_polar`, `budget_vs_clip` (done, model); `field_accel_polar`, `field_deviation`, `field_thermal`, `sweep_caps`, `ablation_*` | E0, E2, F1–F3, E6 |

Pages: shared context (structure, relation to TIGERs) 1.5, A 4, B 3, results/limitations/lessons
1 (including the motor-controller commutation and interrupt-load bugs), slack 0.5.

**Order:** bench smoke test (§2b) → E1, E3, E2 → F1–F3 → E6 → E5. **Everything is untethered.**
If neither F2 nor E6 happens, drop B: run A at ~7 pages with a fuller calibration walkthrough.

Previous plan: see the appendix.

---

## 0. Ask Nick for his logs

None of the November 2026 data is committed. If Nick has these, some figures only need plotting:

| What | Where | Makes |
|---|---|---|
| Breakaway runs | `analysis/data/friction_profile/*.npz` (or `../data/...`) | `friction_direction` |
| Constant-speed runs | `analysis/data/plateau_profile/*.npz` | `friction_direction` |
| Move-set logs | `analysis/data/small_move/*.npz` | paths, tracking (candidate per file not recorded) |
| Calibration recordings | `/tmp/calib.npz` (last run only) | one `fit_*` figure |

---

## 1. Nick's code

```
controls/analysis/
  ros_scripts/            ROS nodes that drive the robot; all share small_move_test.py's safety monitor
    small_move_test.py    out-and-back BCM_GLOBAL_POSITION moves; the test harness
    theta_ramp_test.py    rotation ramps
    friction_profile.py   breakaway: open-loop accel ramp per direction
    plateau_profile.py    kinetic: constant velocity per direction
    telem_record.py       extended telemetry -> .npz
  calibration/            fit scripts (common.py: move set, recorder, param get/set, move_tracking)
  tdp/                    TDP scripts and figures
```

**Safety monitor** (`small_move_test.py`, 200 Hz): sends `BCM_OFF` and latches if the robot
leaves the area (`TDP_AREA`, §2; ±0.25 m around field (0, 0) without it), turns > 95° from
`--heading-ref`, exceeds 0.8 m/s (`TDP_MAX_SPEED`) or 4 rad/s, a wheel exceeds 4.2 A or 60 rad/s,
a motor reports an error, or telemetry/vision is stale > 0.8 s.

**Radio parameters** are live until reboot, so they A/B values without reflashing:

| ID | Name | Values (compiled) |
|---|---|---|
| 6 | `PHYS_FRICTION_MODEL` | coulomb_xy, viscous_xy, coulomb_w, viscous_w, eff_xy, eff_w (4.5, 0, 0.1, 0, 0.75, 0.8) |
| 15 | `POSE_FB_XY` | K, MAX, anti-jitter (8, 0.3, 0.01) |
| 16 | `POSE_FB_W` | K, MAX, anti-jitter, K_vw (8, 2, 0.01, 2) |
| 17 | `WHEEL_VEL_FB` | kP mA/(rad/s), kI, anti-jitter, total current cap A (400, 0, 1.5, 4.0) |
| 18 | `WHEEL_FF_CURRENT_LIMIT` | FF cap A (3.5) |
| 19 | `WHEEL_SLIP_COMP` | K (rad/s)/A (1.2) |

`config.py` handles them by name: `show`, `set`, `defaults`, `friction-off`, `snapshot`, plus
`wheel-loop on|off` (a bridge parameter), the play override and play parameters.

**Tracking metric** (`common.move_tracking`): RMS of |trajectory − estimate| while moving, and
time to within 5 mm. It compares against the *estimate*, not vision; say so in the paper.

---

## 2. Setup

### One-time (per machine)

- ROS 2 **Jazzy** on Ubuntu 24.04. ROS `software` repo on **`dev/will/tdp-2027-motion`** (Nick's
  `dev/nick/model-search` plus `TdpMotionPlay`), then `git submodule update --init --recursive`.
- Build in a plain terminal, not `nix develop`. `rosdep` silently skips packages on 24.04, so
  install by hand: `sudo apt install -y python3-clang ros-jazzy-ompl nlohmann-json3-dev
  libprotobuf-dev protobuf-compiler`; Rust ≥ 1.85 via rustup. Then
  `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`.
- Use the **firmware's** `controls` checkout for everything: the ROS branch pins Nick's
  cleanup commit, which deletes `friction_profile.py`, `plateau_profile.py` and `rollout_test.py`.
- scipy in the ROS environment (for Nick's `efficiency_friction.py`).

### Per robot

- Flash this branch's firmware (`util/tdp-images.sh flash shipped`, §E6); older firmware
  doesn't match the bridge's protocol.
- Extended telemetry on (control board DIP switch 0).
- IMU calibrated: hold ENTER while powering on, upright and still. Stored in flash.

### Per session

```bash
source /opt/ros/jazzy/setup.bash; source ~/ateam/ateam_ws/install/setup.bash
cd <firmware>/controls && mkdir -p analysis/data/tdp
eval "$(python3 analysis/tdp/config.py area field)"     # or: area patch; in every test shell
ros2 launch ateam_bringup bringup_physical_core.launch.py    # + kenobi for F1–F3: bringup_physical_game
ros2 topic hz /robot_feedback/extended/robot2               # ~100 Hz
analysis/tdp/bag.sh e1_slip_r2 2                            # own terminal; Ctrl-C after the last test
```

- **Area:** `config.py area NAME` exports `TDP_AREA` (the safety monitor's box, for every script
  and subprocess), `TDP_HOME` (area center, heading 180°) and `TDP_HEADING`, and sets the play's
  area if kenobi is running. `--max-speed 1.5` raises the monitor's speed trip. Areas are in
  `analysis/tdp/areas.json`: `field` (lines 4470 × 2615 mm) and `patch`.
  **TODO: patch size, center (vision coordinates) and yaw**; `area patch` refuses to run until
  they're filled in. Vision reports the full field either way.
- **Untethered safety:** the backstops are the monitor's `BCM_OFF` (robot coasts), the firmware's
  0.5 s command timeout, and the e-stop or GC HALT in your hand.
- **Bag:** start it after the stack and before the first test. It writes
  `analysis/data/tdp/bags/NAME_<date>_<time>/` (zstd MCAP: feedback, commands, vision, referee,
  kenobi state, `/parameter_events`, `/rosout`) with `params_start.json` / `params_end.json`
  (firmware parameters aren't ROS parameters, so the bag alone misses them). The start snapshot
  takes ~20 s. Convert slices for the figure scripts:
  ```bash
  python3 analysis/tdp/bag2npz.py <bag> --list
  python3 analysis/tdp/bag2npz.py <bag> -r 2 --start 14:07:30 --end 14:10 --out part.npz   # also unix t_start/t_end from sweep.py
  ```
- Record battery voltage, robot, field and firmware commit (`git -C .. describe --always --dirty`)
  in `--note`.
- File naming: `analysis/data/tdp/<experiment>_r<robot>.{npz,json}`. `sweep.py` appends to an
  existing JSON; use a new name after changing the robot.
- The `fig_*.py` scripts need only numpy and matplotlib and run anywhere.

---

## 2b. Bench smoke test

Before the first session and after any firmware or stack change. Robot on a stand, all wheels
clear, bench supply at pack voltage with a few amps of current limit, kicker never armed.
~1.5 min.

```bash
ros2 launch ateam_bringup bringup_physical_core.launch.py   # vision off, or robot out of view
python3 analysis/tdp/bench_smoke.py -r 2                    # asks before spinning the wheels
python3 analysis/tdp/bench_smoke.py -r 2 --no-motion        # comms and logging only
```

Output: `analysis/data/tdp/bench/<date>_r<robot>/` (`report.json`, `bench_live.npz`, the bag).
Exit code 1 on any FAIL.

| Phase | Checks |
|---|---|
| preflight | bridge, telemetry, radio connected, vision seen |
| link (idle and moving) | telemetry rate ≥ 80 Hz, gaps (~0.5 s radio stalls are WARN), packet loss from sequence numbers, error flags, bus voltage |
| parameters | all six read and equal to compiled; stand values set and read back; service round trip |
| motion | 8 `BCM_LOCAL_VELOCITY` segments (±x, ±y, ±w, ±diagonal), `BCM_OFF` between. Setpoints vs kinematic model (frame/order/sign: FAIL), wheel tracking, `torque_limited`, current, command round trip, supply sag |
| restore | compiled values restored and verified, also after Ctrl-C or a trip |
| logging | bag closed, ≥ 99 % of live telemetry, runs to the end, commands recorded, both snapshots complete and the end one compiled |

**On a stand the estimate diverges from the trajectory:** the gyro sees no rotation and vision (if
it sees the robot) pins the position, so the firmware's divergence check (1 m, 1.57 rad, 5 m/s,
10 rad/s) can brake mid-move. Hence velocity mode only (position modes need vision), segments of
0.45 m and 1.2 rad, and `BCM_OFF` between them. A segment that collapses anyway reports *held*
(WARN), not a wheel fault. Friction FF and slip compensation are zeroed during the test, since
unloaded wheels only fight them.

Stops on > 40 rad/s, > 4.2 A, a motor error, telemetry stale 0.5 s, or Ctrl-C. Tested against a
simulated robot only.

---

## 3. Experiments

Times are rough (a move is 10–15 s with holds).

### E0. Model figures, item B (done)

```bash
python3 analysis/tdp/fig_model.py --out analysis/tdp/figures
```

| | |
|---|---|
| ![feasible acceleration](controls/analysis/tdp/figures/feasible_accel_polar.png) | ![budget vs clip](controls/analysis/tdp/figures/budget_vs_clip.png) |

- **Feasible acceleration:** forward 1.86 m/s², sideways 0.95, diagonal 0.84. Derated to 1.9 A,
  sideways is **0** (friction uses the whole budget; only the planner's 0.3 m/s² floor is left).
  Worth a sentence, and a check on a hot robot.
- **Budget vs clipping** at 3 m/s²: clipping gets more acceleration but up to **51° off direction**
  with **7.7 rad/s²** of unwanted yaw; the budget stays on direction. `--limit 2.5`: 80°.
- `friction_direction_model` is the fallback if E3 doesn't happen.

Model predictions until F2/E6 back them up.

### E1. Wheel slip, item A (~25 min) → `fit_slip`, `sweep_slip`

```bash
python3 analysis/tdp/record.py slip-linear   --home="$TDP_HOME" --heading-ref "$TDP_HEADING" --out analysis/data/tdp/slip_linear_r2.npz
python3 analysis/tdp/record.py slip-rotation --home="$TDP_HOME" --heading-ref "$TDP_HEADING" --out analysis/data/tdp/slip_rotation_r2.npz
python3 analysis/tdp/sweep.py --home="$TDP_HOME" --heading-ref "$TDP_HEADING" --repeats 3 \
    --out analysis/data/tdp/slip_sweep_r2.json --note "robot 2, lab carpet, 24.8 V" \
    --candidate 0:19=0 --candidate 0.8:19=0.8 --candidate 1.2:19=1.2 --candidate 1.6:19=1.6
python3 analysis/tdp/fig_fits.py slip analysis/data/tdp/slip_linear_r2.npz --rotation analysis/data/tdp/slip_rotation_r2.npz
python3 analysis/tdp/fig_sweep.py analysis/data/tdp/slip_sweep_r2.json --metric xy_rms_mm --xlabel "slip gain K ((rad/s)/A)" --name sweep_slip
```

Expected (Nick, robot 2): linear K ≈ 1.0, rotation ≈ 0; XY RMS 10.0 mm at K=0, 7.3 mm at K=1.2.
Cross-check K with `python3 analysis/calibration/wheel_slip.py --home="$TDP_HOME" --heading-ref "$TDP_HEADING"`.
If K=0 vs 1.2 is within the error bars, add repeats.

### E2. Current caps, item B (~20 min) → `sweep_caps`

FF cap (18) and the wheel loop's total cap (4th value of 17) change together. Quote the
candidates (`;`). Don't exceed 3.5/4.0: 4.0/4.5 hit the 4.2 A trip.

```bash
python3 analysis/tdp/sweep.py --home="$TDP_HOME" --heading-ref "$TDP_HEADING" --repeats 3 \
    --out analysis/data/tdp/caps_sweep_r2.json \
    --candidate "2.7/3.0:18=2.7;17=400,0,1.5,3.0" \
    --candidate "3.0/3.5:18=3.0;17=400,0,1.5,3.5" \
    --candidate "3.5/4.0:18=3.5;17=400,0,1.5,4.0"
python3 analysis/tdp/fig_sweep.py analysis/data/tdp/caps_sweep_r2.json \
    --metric reach_5mm_s slip_p95_mps --xlabel "FF / total cap (A)" --name sweep_caps
```

Expected: 15–25 % faster moves from 2.7/3.0 to 3.5/4.0.

### E3. Friction and efficiency, item A (~30 min) → `friction_direction`, `fit_efficiency`

The profilers need friction FF zeroed and have no `--home`: place the robot near `$TDP_HOME`.
Pass `--out` (their default lands outside `controls/`).

```bash
python3 analysis/tdp/config.py friction-off -r 2
python3 analysis/ros_scripts/friction_profile.py -r 2 --dirs 0,30,45,60,90,120,135,150,w --reps 4 \
    --out analysis/data/friction_profile
python3 analysis/ros_scripts/plateau_profile.py -r 2 --dirs 0,30,45,60,90,120,135,150,w --speeds 0.1,0.2 \
    --out analysis/data/plateau_profile
python3 analysis/tdp/config.py defaults -r 2
python3 analysis/tdp/config.py show -r 2         # all compiled, wheel_vel True
python3 analysis/tdp/record.py fit-moves --home="$TDP_HOME" --heading-ref "$TDP_HEADING" --out analysis/data/tdp/fit_moves_r2.npz
python3 analysis/tdp/fig_friction.py --breakaway analysis/data/friction_profile/*.npz --plateau analysis/data/plateau_profile/*.npz
python3 analysis/tdp/fig_fits.py efficiency analysis/data/tdp/fit_moves_r2.npz
python3 analysis/calibration/efficiency_friction.py --home="$TDP_HOME" --heading-ref "$TDP_HEADING" \
    --no-run --out analysis/data/tdp/fit_moves_r2.npz      # cross-check η and C
```

- `friction_profile.py` turns the wheel loop off and back on. If it crashes, `config.py wheel-loop on`;
  otherwise every later test runs feed-forward only.
- The claim is that force / k(dir) is flat (Nick: ~5.3 N). The script prints raw vs normalized spread.
- The fit gives C ≈ 3.7–3.9 N; the shipped 4.5 N won on tracking. To show that A/B:
  `sweep.py ... --candidate 3.9:6=3.9,0,0.1,0,0.75,0.8 --candidate 4.5:6=4.5,0,0.1,0,0.75,0.8`.

### E5. More robots, item A (~1 day) → `sweep_slip_fleet`, parameter table

E1 and E3 with `-r N` and `_rN` names (IMU calibration first). Report each robot's η, C and K;
if they differ by more than the fit spread, say per-robot calibration is needed.

```bash
python3 analysis/tdp/fig_sweep.py analysis/data/tdp/slip_sweep_r{2,3,5}.json --labels "robot 2" "robot 3" "robot 5" \
    --metric xy_rms_mm --xlabel "slip gain K ((rad/s)/A)" --name sweep_slip_fleet
```

### E6. Ablation with small moves, item B (~½ day) → `ablation_tracking`, `ablation_paths`

The planner caps acceleration at 0.9 × feasible, so the budget rarely saturates; the ablation
also needs the cap off. Three firmware images (cargo features; ablation images log
`TDP ABLATION BUILD` at boot):

| Image | Planner cap | FF budget |
|---|---|---|
| `shipped` | on | direction-preserving |
| `nocap` | off | direction-preserving |
| `clip` | off | per-wheel clip (slip compensation still sees the unclipped command; say so) |

From the firmware root, in `nix develop`, probe attached:

```bash
util/tdp-images.sh build           # build/tdp-images/<commit>/control-{shipped,nocap,clip}.{elf,bin}
util/tdp-images.sh flash nocap
util/tdp-images.sh flash shipped   # always finish with this
```

Per image (same label as the image):

```bash
CFG=nocap
python3 analysis/tdp/sweep.py --home="$TDP_HOME" --heading-ref "$TDP_HEADING" --repeats 3 --amax 3.0 \
    --moves 0,0.2,0 0.15,0.15,90 0.15,-0.15,-90 \
    --out analysis/data/tdp/ablation_r2.json --candidate "$CFG:" --note "ablation $CFG"
python3 analysis/tdp/fig_sweep.py analysis/data/tdp/ablation_r2.json \
    --metric xy_rms_mm heading_rms_deg --xlabel "config" --name ablation_tracking
python3 analysis/tdp/fig_paths.py "shipped=<file>" "no planner cap=<file>" "per-wheel clip=<file>" --name ablation_paths
```

`nocap` and `clip` should track worse and `clip` should yaw: start with `--repeats 1`. A lower FF
cap on every config (`--candidate "$CFG:18=2.7"`) saturates sooner. Run the images back to back
on one battery.

### F1–F3. Field scale, item B (~½ day) → `field_*`

**`TdpMotionPlay`** (kenobi, `ateam_kenobi/src/plays/test_plays/tdp_motion_play.*`) drives one robot
through straight `GlobalPosition` moves, planner skipped, inside the area minus `margin_m` (0.3 m).
Patterns and `heading_deg` are in the area's frame; kenobi logs the usable rectangle at start
(field, defense areas avoided: ±1.44 × ±1.01 m).

| Pattern | Moves | Use |
|---|---|---|
| `star` | center → radius → center, `star_directions` directions | acceleration vs direction (F1, F2) |
| `sprint_x` / `_y` / `_diag` | end to end | thermal soak (F3) |
| `tour` | corners, turning 90° per side | translation + rotation (optional) |

Parameters: `config.py play-params --show`. They're read when the play is selected.

- **Defense areas:** kenobi stops non-goalie robots entering them. The play avoids them by
  default; to use them (the patch overlaps one), make the robot our goalie in the GC and set
  `avoid_defense_areas=false`. The play refuses to start otherwise.
- **Safety:** only the test robot on the field. The play sends Stop and latches *ABORTED* above
  `abort_speed_mps` (3 m/s), outside the area, or with the robot lost > 0.3 s; re-select to restart.
  GC HALT always wins. GC in **Force Start** (Stop imposes 1.25 m/s). First run: `accel=1.5 vel=1.0`.

```bash
ros2 launch ateam_bringup bringup_physical_game.launch.py          # terminal 1
analysis/tdp/bag.sh f1_star_shipped 2                              # terminal 2; Ctrl-C when "done"
eval "$(python3 analysis/tdp/config.py area field)"                # terminal 3
python3 analysis/tdp/config.py play-params robot_id=2 pattern=star star_directions=24 accel=3.0 vel=2.0 dwell_s=1.0
python3 analysis/tdp/config.py play TdpMotionPlay
python3 analysis/tdp/config.py play                                # clear the override
python3 analysis/tdp/bag2npz.py analysis/data/tdp/bags/f1_star_shipped_<date>_<time> -r 2 \
    --out analysis/data/tdp/field_star_shipped_r2.npz
```

(`field_record.py -r 2 --out ...` records the same format live, without a bag.)

- **F1** (`shipped`, ~3 min): `star`, 24 directions (12 miss the notches near 45°/135°),
  `accel=3.0`. Achieved acceleration should sit on 0.9 × model; forward ≈ 2× sideways.
- **F2** (~10 min + flashing): F1 with `nocap` and `clip`. `nocap` should reach the full model value
  but track worse; `clip` should deviate where `budget_vs_clip` predicts. Name them
  `field_star_{shipped,nocap,clip}_r2.npz`. **TODO:** `fig_field.py` doesn't yet flag moves cut short
  by divergence recovery.
- **F3** (`shipped`, ~3 min): `pattern=sprint_x cycles=15 dwell_s=0 accel=3.0 vel=2.0`. The replayed
  I²t should climb and the FF limit derate toward 1.9 A, with `torque_limited` staying off. Stop if
  a motor is too hot to touch.

```bash
python3 analysis/tdp/fig_field.py shipped=analysis/data/tdp/field_star_shipped_r2.npz \
    nocap=analysis/data/tdp/field_star_nocap_r2.npz clip=analysis/data/tdp/field_star_clip_r2.npz
python3 analysis/tdp/fig_field.py soak=analysis/data/tdp/field_soak_r2.npz --thermal --name-prefix soak_
```

`fig_field.py` prints and saves (`field_moves_<label>.json`) one row per move: direction in the
robot frame, planned/achieved/model acceleration, cross-track and heading deviation, tracking RMS,
time to target, peak current. The thermal replay uses 100 Hz currents (the firmware integrates at 1 kHz).

---

## 4. Figures

| Item | Figure (`controls/analysis/tdp/figures/`) | Script | Status |
|---|---|---|---|
| B | `feasible_accel_polar`, `budget_vs_clip` | `fig_model.py` | done (model) |
| A | `friction_direction_model` | `fig_model.py` | done (fallback) |
| A | `fit_slip`, `sweep_slip` | `fig_fits.py slip`, `fig_sweep.py` | E1 |
| B | `sweep_caps` | `fig_sweep.py` | E2 |
| A | `friction_direction`, `fit_efficiency` | `fig_friction.py`, `fig_fits.py efficiency` | E3 |
| A | `sweep_slip_fleet`, parameter table | `fig_sweep.py` | E5 |
| B | `ablation_tracking`, `ablation_paths` | `fig_sweep.py`, `fig_paths.py` | E6 |
| B | `field_accel_polar`, `field_deviation`, `field_paths` | `fig_field.py` | F1, F2 |
| B | `soak_field_thermal` | `fig_field.py --thermal` | F3 |

All write `.pdf` and `.png` at LNCS width and print their numbers; quote the printed numbers.

## 5. Gotchas

- **Radio parameters are lost on reboot.** `sweep.py` and `bench_smoke.py` restore them;
  Nick's `common.validate()` doesn't.
- **"telemetry stale" trips:** the radio stalls ~0.5 s every 1–2 min. Rerun; `sweep.py` counts them.
- **No telemetry:** check the DIP switch, then restart the bridge.
- **Bag QoS:** kenobi's commands are `TRANSIENT_LOCAL`, the scripts' `VOLATILE`; a bare
  `ros2 bag record` drops one of them. Use `bag.sh` (per-topic overrides).
- **Stopping a recorder from a script:** background jobs ignore `SIGINT`; use `SIGTERM`.
- **Don't use Nick's `telem_bag2np.py`:** float32 jitters the robot clock ±2.5 % after ~80 min uptime.
  `bag2npz.py` keeps float64.
- `tdp_common.py` copies constants from `defaults.rs`, `controller_params.rs` and `wheel_thermal.rs`;
  update it if they change.
- For a reviewer-proof hold number, also give the vision-measured error (vision noise at rest:
  0.2 mm / 0.08°).

---

## Appendix: previous plan (2026-11-22)

Three highlights instead of two items: current limiting (~2 pp; E0, E2, E6), friction/efficiency
identification (~2 pp; E3, E5), wheel-slip feed-forward (~1 pp; E1, E5), and an optional estimator
section (~1 pp; E4). Superseded 2026-11-24: slip moved into item A, the estimator was dropped, and
E7 (untethered, full field) became F1–F3.

**E4 (dropped): vision latency** → `fit_latency`. Heading residual vs assumed latency; gyro and
encoder minima should agree (Nick: 56 ms). Depends on the camera setup.

```bash
python3 analysis/tdp/record.py latency --home="$TDP_HOME" --heading-ref "$TDP_HEADING" --out analysis/data/tdp/latency_r2.npz
python3 analysis/tdp/fig_fits.py latency analysis/data/tdp/latency_r2.npz
python3 analysis/calibration/vision_latency.py --home="$TDP_HOME" --heading-ref "$TDP_HEADING" --no-run --out analysis/data/tdp/latency_r2.npz
```
