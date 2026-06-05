"""
Overlaid normalized step responses for each inversion strategy.

For each strategy variant one subplot is produced, showing all five velocity
steps (0→1, 1→2, 2→3, 3→4, 4→5 m/s) overlaid and normalized to [0, 1].

Normalization: 0 = actual settled state at v_from, 1 = v_to.
No-compensation plateaus at K=0.95 (DC gain < 1) — this is intentional and
visible in the plot as curves that don't reach the dashed unity line.

Steps are simulated independently: the model is pre-settled at v_from before
each step, so each curve is a clean, properly initialised transient.
"""

import numpy as np
import matplotlib.pyplot as plt

# ── Physical parameters (DD robot, X-axis) ─────────────────────────────────
K         = 0.95
T_PARAM   = 0.095
DT        = 0.001
CMD_CLAMP = 30.0

# ── Forward model ───────────────────────────────────────────────────────────
def model_step(state: float, cmd: float) -> float:
    T_eff = T_PARAM * abs(cmd) + 0.001
    return state + DT * ((K / T_eff) * cmd - (1.0 / T_eff) * state)

# ── Inversion strategies ────────────────────────────────────────────────────
def invert_none(state: float, desired: float, **_) -> float:
    return desired

def invert_linear(state: float, desired: float, n: int, **_) -> float:
    T_eff = T_PARAM * abs(desired) + 0.001
    a, b  = K / T_eff, 1.0 / T_eff
    cmd   = desired
    for _ in range(n):
        cmd += desired - (state + DT * (a * cmd - b * state))
    return cmd

def invert_analytical(state: float, desired: float, **_) -> float:
    T_eff = T_PARAM * abs(desired) + 0.001
    a, b  = K / T_eff, 1.0 / T_eff
    denom = DT * a
    if abs(denom) < 1e-12:
        return desired
    return float(np.clip((desired - state * (1.0 - DT * b)) / denom, -CMD_CLAMP, CMD_CLAMP))

def invert_nonlinear(state: float, desired: float, n: int, **_) -> float:
    cmd = desired
    for _ in range(n):
        T_eff = T_PARAM * abs(cmd) + 0.001
        a, b  = K / T_eff, 1.0 / T_eff
        cmd  += desired - (state + DT * (a * cmd - b * state))
        cmd   = float(np.clip(cmd, -CMD_CLAMP, CMD_CLAMP))
    return cmd

# ── Per-step simulation ─────────────────────────────────────────────────────
N_SETTLE = 4000   # > 6τ for no-comp at 5 m/s (τ≈476ms → need ~3s)
N_WINDOW = 1000   # 1000 ms response window
t_window = np.arange(N_WINDOW) * DT * 1e3  # [ms]

def simulate_step(invert_fn, v_from: float, v_to: float, **kwargs):
    """Pre-settle at v_from, step to v_to.

    Returns:
        body_norm  -- body velocity normalized: 0 = settled v_from, 1 = v_to
        cmd_norm   -- wheel command normalized by v_to (1.0 = commanding target directly)
    """
    state = 0.0
    for _ in range(N_SETTLE):
        cmd   = float(np.clip(invert_fn(state, v_from, **kwargs), -CMD_CLAMP, CMD_CLAMP))
        state = model_step(state, cmd)
    v_init   = state
    vel      = np.zeros(N_WINDOW)
    cmd_out  = np.zeros(N_WINDOW)
    for k in range(N_WINDOW):
        cmd        = float(np.clip(invert_fn(state, v_to, **kwargs), -CMD_CLAMP, CMD_CLAMP))
        state      = model_step(state, cmd)
        vel[k]     = state
        cmd_out[k] = cmd
    span      = v_to - v_init
    body_norm = np.zeros(N_WINDOW) if abs(span) < 1e-9 else (vel - v_init) / span
    cmd_norm  = cmd_out
    return body_norm, cmd_norm

# ── Variants ─────────────────────────────────────────────────────────────────
variants = [
    ('No compensation',         invert_none,    {}),
    ('Linear n=1',              invert_linear,  {'n': 1}),
    ('Linear n=2',              invert_linear,  {'n': 2}),
    ('Linear n=5',              invert_linear,  {'n': 5}),
    ('Linear n=10  ★ firmware', invert_linear,  {'n': 10}),
    ('Linear n=20',             invert_linear,  {'n': 20}),
]

# ── Step pairs and colors ─────────────────────────────────────────────────────
STEP_PAIRS  = [(0.0, 1.0), (0.0, 2.0), (0.0, 3.0), (0.0, 4.0), (0.0, 5.0)]
step_colors = plt.cm.plasma(np.linspace(0.1, 0.85, len(STEP_PAIRS)))

# ── Pre-compute ───────────────────────────────────────────────────────────────
print('Simulating...')
responses = {}
for vi, (title, fn, kw) in enumerate(variants):
    for si, (v_from, v_to) in enumerate(STEP_PAIRS):
        responses[(vi, si)] = simulate_step(fn, v_from, v_to, **kw)
    print(f'  {title}')

# ── Plot ─────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 3, figsize=(15, 9), sharey=False)
fig.suptitle(
    'Normalized step responses — overlaid across target velocities\n'
    '5 steps per subplot: 0→1, 0→2, 0→3, 0→4, 0→5 m/s  |  '
    f'K={K}, T_param={T_PARAM} s/(m/s), dt={DT*1000:.0f} ms\n'
    'Solid = body velocity (norm.)    Dashed = wheel command / target vel',
    fontsize=11, fontweight='bold'
)

for vi, (title, _fn, _kw) in enumerate(variants):
    ax  = axes.flatten()[vi]
    ax2 = ax.twinx()

    for si, ((v_from, v_to), color) in enumerate(zip(STEP_PAIRS, step_colors)):
        body_norm, cmd_norm = responses[(vi, si)]
        ax.plot( t_window, body_norm, color=color, lw=1.8,  label=f'0→{v_to:.0f} m/s')
        ax2.plot(t_window, cmd_norm,  color=color, lw=0.9, ls='--', alpha=0.5)

    ax.axhline(1.0, color='black', lw=0.8, ls='--', alpha=0.35)
    ax.axhline(0.9, color='gray',  lw=0.6, ls=':',  alpha=0.5)
    ax.text(N_WINDOW * DT * 1e3 - 2, 0.91, '90%',
            fontsize=7, color='gray', ha='right', va='bottom')

    # clip command axis to show transient without extreme scaling
    cmd_max = max(responses[(vi, si)][1].max() for si in range(len(STEP_PAIRS)))
    ax2.set_ylim(0, min(cmd_max * 1.1, 15))
    ax2.tick_params(axis='y', labelsize=7, colors='gray')
    ax2.set_ylabel('Wheel cmd [m/s]', fontsize=7, color='gray')

    fw = 'bold' if 'firmware' in title else 'normal'
    ax.set_title(title, fontsize=10, fontweight=fw)
    ax.set_xlim(0, N_WINDOW * DT * 1e3)
    ax.set_ylim(-0.05, 1.22)
    ax.set_xlabel('Time since step [ms]', fontsize=8)
    if vi % 3 == 0:
        ax.set_ylabel('Normalized body velocity', fontsize=8)
    ax.tick_params(labelsize=8)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc='lower right', handlelength=1.2)

# Shared legend for step colors
handles = [plt.Line2D([0], [0], color=c, lw=2,
                      label=f'0→{v1:.0f} m/s  (T_eff≈{int((T_PARAM*v1+0.001)*1e3)}ms)')
           for (_, v1), c in zip(STEP_PAIRS, step_colors)]
fig.legend(handles=handles, loc='lower center', ncol=5, fontsize=9,
           title='Step target  (uncompensated T_eff at that velocity)',
           title_fontsize=9, framealpha=0.9,
           bbox_to_anchor=(0.5, -0.02))

plt.tight_layout(rect=[0, 0.07, 1, 1])
plt.savefig('lag_multistep_comparison.png', dpi=150, bbox_inches='tight')
print('Saved: lag_multistep_comparison.png')
plt.show()
