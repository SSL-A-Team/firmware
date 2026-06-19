# Control Board

Firmware binaries for the STM32-based control board.

## Firmware Images

| Binary | Description |
|--------|-------------|
| [control](#control) | Production robot firmware: radio, motion control, kicker, dribbler |
| [hwtest-drib](#hwtest-drib) | Interactive dribbler motor test via kicker board link |
| [hwtest-kicker](#hwtest-kicker) | Interactive kicker and dribbler current test with power board integration |
| [hwtest-torque](#hwtest-torque) | Wheel motor torque characterization with USB data output |

---

### control

Production firmware. Connects to the AI software stack over Wi-Fi, runs full robot control: body motion (velocity/position/acceleration modes), kicker charging and firing, and dribbler speed control. No interactive buttons.

---

### hwtest-drib

Tests the dribbler motor through all supported control modes. Setpoints are normalized 0.0–1.0; the kicker board CCM driver maps them to raw rad/s (velocity) or mA (current). Telemetry (velocity, average current, ball detection) prints every 200 ms when enabled.

**Modes (cycled with LEFT/RIGHT):**

| Mode | Description |
|------|-------------|
| HARD_RECEIVE | Smart receive — hard stop (stubbed) |
| SOFT_RECEIVE | Smart receive — compliant (stubbed) |
| DRIBBLE | Active dribble (stubbed) |
| VELOCITY | Velocity PID control (0.0–1.0 → 0–500 rad/s) |
| CURRENT | Direct current control (0.0–1.0 → 0–2646 mA) |

**Buttons:**

| Button | Function |
|--------|----------|
| BACK | Toggle telemetry print |
| CENTER | Toggle motor enable/disable |
| LEFT | Previous mode |
| RIGHT | Next mode |
| UP | Increase setpoint (step 0.1 smart modes, 0.01 velocity/current) |
| DOWN | Decrease setpoint; at 0.0 while disabled, resets to first mode |

---

### hwtest-kicker

Tests the full kicker path: capacitor charging, kick firing, and dribbler current. Integrates with the power board for safe discharge on shutdown. Waits for the kicker board to load dribbler firmware before starting.

**Buttons:**

| Button | Function |
|--------|----------|
| BACK | Toggle telemetry print |
| CENTER | Fire kick at current speed |
| LEFT | Decrease dribbler current setpoint (step 0.01, range 0.0–1.0) |
| RIGHT | Increase dribbler current setpoint (step 0.01, range 0.0–1.0) |
| UP | Increase kick speed (step 0.5 m/s, max 6.5 m/s) |
| DOWN | Decrease kick speed (step 0.5 m/s) |

---

### hwtest-torque

Characterizes wheel motor torque by running a controlled current ramp (0 to 500 mA over ~125 s) on one wheel at a time. Measures the angular displacement to detect when the wheel breaks free and computes stall torque. Torque feedback packets stream to USB for offline analysis.

Wheels: FL (front-left), FR (front-right), BR (back-right), BL (back-left).

**Buttons:**

| Button | Function |
|--------|----------|
| CENTER | Start / restart a test round on the active wheel |
| LEFT | Select previous wheel (counter-clockwise order) |
| RIGHT | Select next wheel (clockwise order) |
| UP | Increase pulley radius (6 / 10 / 20 mm) |
| DOWN | Decrease pulley radius |
