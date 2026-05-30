"""
Encoder lag compensation comparison.

Models the nonlinear first-order lag:
    T_eff(u) = T_PARAM * |u| + 0.001
    dv/dt = (K/T_eff)*u - (1/T_eff)*v

Compares four inversion strategies:
  - No compensation (open loop)
  - Linearized fixed-point (T_eff frozen from desired) — matches model_enc.c
  - Analytical closed-form (T_eff frozen, infinite iterations)
  - True nonlinear fixed-point (T_eff recomputed from optVel each step)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── Physical parameters (DD robot, X-axis) ─────────────────────────────────
K       = 0.95      # encoder model gain
T_PARAM = 0.095     # T coefficient [s/(m/s)]
DT      = 0.001     # timestep [s]
CMD_CLAMP = 30.0    # wheel command saturation [m/s]

# ── Forward model (nonlinear first-order lag) ───────────────────────────────
def model_step(state: float, cmd: float) -> float:
    T_eff = T_PARAM * abs(cmd) + 0.001
    vdot  = (K / T_eff) * cmd - (1.0 / T_eff) * state
    return state + DT * vdot


# ── Inversion strategies ────────────────────────────────────────────────────

def invert_none(state: float, desired: float, **_) -> float:
    """No compensation — pass desired through directly."""
    return desired


def invert_linear(state: float, desired: float, n: int, **_) -> float:
    """
    Linearized fixed-point iteration (matches model_enc.c ModelEncInvert).
    T_eff is frozen from |desired|, not updated as optVel changes.
    """
    T_eff = T_PARAM * abs(desired) + 0.001
    a     = K / T_eff
    b     = 1.0 / T_eff
    cmd   = desired
    for _ in range(n):
        future = state + DT * (a * cmd - b * state)
        cmd   += desired - future
    return cmd


def invert_analytical(state: float, desired: float, **_) -> float:
    """
    Closed-form solution with T_eff frozen from |desired| (n → ∞).
    Derived from: desired = state + dt*(a*cmd - b*state)
    → cmd = (desired - state*(1 - dt*b)) / (dt*a)
    """
    T_eff = T_PARAM * abs(desired) + 0.001
    a     = K / T_eff
    b     = 1.0 / T_eff
    denom = DT * a
    if abs(denom) < 1e-12:
        return desired
    return np.clip((desired - state * (1.0 - DT * b)) / denom, -CMD_CLAMP, CMD_CLAMP)


def invert_nonlinear(state: float, desired: float, n: int, **_) -> float:
    """
    True nonlinear fixed-point: T_eff recomputed from |optVel| each step.
    This is self-consistent with the forward model.
    """
    cmd = desired
    for _ in range(n):
        T_eff  = T_PARAM * abs(cmd) + 0.001
        a      = K / T_eff
        b      = 1.0 / T_eff
        future = state + DT * (a * cmd - b * state)
        cmd   += desired - future
        cmd    = float(np.clip(cmd, -CMD_CLAMP, CMD_CLAMP))
    return cmd


# ── Simulation harness ──────────────────────────────────────────────────────

def simulate(desired_traj, invert_fn, **kwargs):
    """
    Run the closed-loop simulation.
    Returns (body_vel, cmd_vel) arrays of length len(desired_traj).
    """
    state   = 0.0
    vel_out = np.zeros(len(desired_traj))
    cmd_out = np.zeros(len(desired_traj))
    for k, desired in enumerate(desired_traj):
        cmd          = invert_fn(state, desired, **kwargs)
        cmd          = float(np.clip(cmd, -CMD_CLAMP, CMD_CLAMP))
        state        = model_step(state, cmd)
        vel_out[k]   = state
        cmd_out[k]   = cmd
    return vel_out, cmd_out


# ── Input profiles ──────────────────────────────────────────────────────────

T_SIM   = 0.5          # total simulation time [s]
N       = int(T_SIM / DT)
t       = np.arange(N) * DT

# Step: 0 → 1 m/s at t = 50 ms
STEP_VEL   = 1.0
STEP_START = 0.05
step_traj  = np.where(t >= STEP_START, STEP_VEL, 0.0)

# Trapezoid: ramp at 10 m/s² up to 2 m/s, hold, ramp down
MAX_ACC  = 10.0   # m/s²   (== K/T_PARAM — physical ceiling)
PEAK_VEL = 2.0    # m/s
trap_traj = np.zeros(N)
v = 0.0
for k in range(N):
    if k > 0:
        target = PEAK_VEL if t[k] < 0.35 else 0.0
        v = v + np.clip(target - v, -MAX_ACC * DT, MAX_ACC * DT)
    trap_traj[k] = v


# ── Theoretical time constants ──────────────────────────────────────────────

vel_range  = np.linspace(0.05, 5.0, 200)   # operating velocity [m/s]
T_eff_vec  = T_PARAM * vel_range + 0.001

def tau_linear(n):
    return T_eff_vec / (1.0 + n * K) * 1e3  # [ms]

def dc_gain_linear(n):
    return K * (n + 1) / (1.0 + n * K)


# ── Build variants ──────────────────────────────────────────────────────────

linear_ns    = [1, 2, 5, 10, 20]
nonlinear_ns = [10, 20]

lin_colors    = plt.cm.Blues(np.linspace(0.35, 0.95, len(linear_ns)))
nonlin_colors = ['darkorange', 'firebrick']

# Simulate everything
results = {}

results['none'] = {
    'label': 'No compensation',
    'step':  simulate(step_traj, invert_none),
    'trap':  simulate(trap_traj, invert_none),
    'color': 'gray', 'ls': '--',
}
results['analytical'] = {
    'label': 'Analytical (frozen T_eff, n→∞)',
    'step':  simulate(step_traj, invert_analytical),
    'trap':  simulate(trap_traj, invert_analytical),
    'color': 'black', 'ls': ':',
}
for i, n in enumerate(linear_ns):
    results[f'lin_{n}'] = {
        'label': f'Linear fixed-point n={n}',
        'step':  simulate(step_traj, invert_linear, n=n),
        'trap':  simulate(trap_traj, invert_linear, n=n),
        'color': lin_colors[i], 'ls': '-',
    }
for i, n in enumerate(nonlinear_ns):
    results[f'nl_{n}'] = {
        'label': f'Nonlinear fixed-point n={n}',
        'step':  simulate(step_traj, invert_nonlinear, n=n),
        'trap':  simulate(trap_traj, invert_nonlinear, n=n),
        'color': nonlin_colors[i], 'ls': '-.',
    }


# ── Plot ────────────────────────────────────────────────────────────────────

fig = plt.figure(figsize=(16, 14))
fig.suptitle(
    f'Encoder lag compensation: model comparison\n'
    f'K={K}, T_param={T_PARAM} s/(m/s), dt={DT*1000:.1f} ms',
    fontsize=13, fontweight='bold'
)
gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.42, wspace=0.32)

ax_sv  = fig.add_subplot(gs[0, 0])   # step – body velocity
ax_sc  = fig.add_subplot(gs[1, 0])   # step – cmd velocity
ax_tv  = fig.add_subplot(gs[0, 1])   # trap – body velocity
ax_tc  = fig.add_subplot(gs[1, 1])   # trap – cmd velocity
ax_tau = fig.add_subplot(gs[2, 0])   # theoretical τ vs velocity
ax_dc  = fig.add_subplot(gs[2, 1])   # theoretical DC gain vs velocity


# ── Step response ─────────────────────────────────────────────────────────
ax_sv.set_title('Step response — body velocity')
ax_sv.plot(t * 1e3, step_traj, 'k-', lw=1.2, alpha=0.4, label='Desired')
ax_sc.set_title('Step response — wheel command')

ax_tv.set_title('Trapezoid response — body velocity')
ax_tv.plot(t * 1e3, trap_traj, 'k-', lw=1.2, alpha=0.4, label='Desired')
ax_tc.set_title('Trapezoid response — wheel command')

order = (
    ['none', 'analytical']
    + [f'lin_{n}' for n in linear_ns]
    + [f'nl_{n}' for n in nonlinear_ns]
)

for key in order:
    r    = results[key]
    sv, sc = r['step']
    tv, tc = r['trap']
    kw   = dict(label=r['label'], color=r['color'], ls=r['ls'], lw=1.5)

    ax_sv.plot(t * 1e3, sv, **kw)
    ax_sc.plot(t * 1e3, sc, **kw)
    ax_tv.plot(t * 1e3, tv, **kw)
    ax_tc.plot(t * 1e3, tc, **kw)

# Annotations on step body-vel
t_step_ms = STEP_START * 1e3
ax_sv.axvline(t_step_ms, color='gray', lw=0.8, ls=':')
ax_sc.axvline(t_step_ms, color='gray', lw=0.8, ls=':')

for ax in (ax_sv, ax_tv):
    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('Body velocity [m/s]')
    ax.legend(fontsize=7, loc='lower right')
    ax.grid(True, alpha=0.3)

for ax in (ax_sc, ax_tc):
    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('Wheel command [m/s]')
    ax.set_ylim(-CMD_CLAMP * 0.4, CMD_CLAMP * 0.4)
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True, alpha=0.3)

# Zoom step plots to interesting window
ax_sv.set_xlim(40, 200)
ax_sc.set_xlim(40, 200)


# ── Theoretical τ vs operating velocity ──────────────────────────────────
ax_tau.set_title('Theoretical τ_eff vs operating velocity\n(linearized, T_eff frozen)')
ax_tau.plot(vel_range, T_eff_vec * 1e3, color='gray', ls='--', lw=1.5,
            label='Open-loop τ (no comp)')
for i, n in enumerate(linear_ns):
    tau_n = tau_linear(n)
    ax_tau.plot(vel_range, tau_n, color=lin_colors[i], lw=1.8,
                label=f'n={n}  (τ = T_eff/{1+n*K:.2f})')
ax_tau.set_xlabel('Operating velocity [m/s]')
ax_tau.set_ylabel('Time constant [ms]')
ax_tau.legend(fontsize=8)
ax_tau.grid(True, alpha=0.3)
ax_tau.set_yscale('log')
ax_tau.set_ylim(0.5, 500)
ax_tau.annotate('1 m/s → T_eff=96ms', xy=(1.0, T_eff_vec[np.argmin(abs(vel_range-1))] * 1e3),
                xytext=(1.5, 150), fontsize=8,
                arrowprops=dict(arrowstyle='->', color='gray'))


# ── DC gain vs operating velocity ─────────────────────────────────────────
ax_dc.set_title('DC gain vs n\n(linearized, frozen T_eff — velocity-independent)')
n_vec = np.arange(0, 25)
dc_vec = K * (n_vec + 1) / (1.0 + n_vec * K)
ax_dc.plot(n_vec, dc_vec, 'o-', color='steelblue', lw=2)
ax_dc.axhline(1.0, color='gray', ls='--', lw=1, label='Unity gain')
ax_dc.set_xlabel('Number of iterations n')
ax_dc.set_ylabel('DC gain  K(n+1)/(1+nK)')
ax_dc.set_xticks(n_vec)
ax_dc.legend(fontsize=8)
ax_dc.grid(True, alpha=0.3)
for n in linear_ns:
    g = dc_gain_linear(n)
    ax_dc.annotate(f'n={n}\n{g:.4f}', xy=(n, g),
                   xytext=(n + 0.3, g - 0.03), fontsize=7,
                   arrowprops=dict(arrowstyle='->', color='gray', lw=0.8))


plt.savefig('lag_compensation_comparison.png', dpi=150, bbox_inches='tight')
print('Saved: lag_compensation_comparison.png')
plt.show()
