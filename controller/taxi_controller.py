"""
taxi_controller.py
==================
External Python controller for the Unity taxiing environment.
Connects via the ML-Agents Python API, runs MPPI + HOCBF-QP, and sends
[a, delta] actions back to Unity each decision step.

Observation contract (must match TaxiAgent.cs CollectObservations order):
  obs[0]  x_ego   — forward position along taxiway (Unity Z) [m]
  obs[1]  y_ego   — lateral offset (Unity X) [m]
  obs[2]  theta   — heading error from +Z axis [rad], normalised to [-π, π]
  obs[3]  v       — speed [m/s]
  obs[4]  obs_dx  — obstacle forward separation (Unity Z) [m]
  obs[5]  obs_dy  — obstacle lateral separation (Unity X) [m]
  obs[6]  obs_vx  — obstacle velocity along Z [m/s]
  obs[7]  obs_vy  — obstacle velocity along X [m/s]
  obs[8]  goal_dx — distance to goal along Z [m]
  obs[9]  cbf_h   — barrier value h = dist²-D²  (logging only)

Action contract:
  act[0]  a_cmd     — acceleration [m/s²], clipped to [A_MIN, A_MAX]
  act[1]  delta_cmd — steering [rad],      clipped to [-DELTA_LIM, DELTA_LIM]

COORDINATE NOTE:
  Python baseline uses X=forward, Y=lateral.
  Unity scene uses Z=forward (taxiway), X=lateral (airplane frontIsNegativeZ=false).
  obs_to_state() is the single bridge: it maps Unity obs indices to Python state
  convention. Do NOT touch MPPI/CBF logic if only the axis mapping changes — fix
  obs_to_state() only.
"""

import numpy as np
import argparse
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.environment_parameters_channel import (
    EnvironmentParametersChannel,
)
import quadprog

# ── Parameters — keep in sync with TaxiAgent.cs inspector fields ─────────────
DT        = 0.1          # Fixed Timestep in Unity (Project Settings → Time)
L         = 6.0          # wheelbase [m]
V_DES     = 8.0          # desired taxi speed [m/s]
W_HALF    = 10.0         # taxiway half-width [m]
D_SAFE    = 6.0          # keep-out radius [m]
D_INFL    = 10.0         # MPPI obstacle influence radius [m]
A_MIN     = -4.0
A_MAX     =  1.5
DELTA_LIM = 0.5
ALPHA1    = 1.8          # HOCBF first-order gain
ALPHA2    = 1.8          # HOCBF second-order gain
ALPHA_W   = 6.0          # QP acceleration vs steering weight

H_MPPI    = 25           # planning horizon (steps)
K_MPPI    = 1500         # rollout samples
LAMBDA    = 1.0          # MPPI temperature
SIG_A     = 1.0          # noise std for acceleration samples
SIG_D     = 0.12         # noise std for steering samples

# MPPI stage costs
W_LAT, W_HEAD, W_V, W_CTRL = 3.0, 6.0, 1.2, 0.05
W_OBS, W_OFF, W_PROG        = 12.0, 200.0, 0.4
BIG = 300.0              # CBF handles hard safety — MPPI just needs a soft nudge

rng = np.random.default_rng(42)


# ── Observation unpacking ────────────────────────────────────────────────────

def obs_to_state(obs: np.ndarray):
    """
    Map the Unity observation vector to the Python state representation.

    Python state convention: s = [x_forward, y_lateral, theta, v]
    Obstacle:  obs_xy = [x_forward, y_lateral] (absolute world position)
               obs_v  = [vx_forward, vy_lateral]

    Unity sends SEPARATIONS in obs[4]/obs[5] (obstacle minus ego), so we
    recover absolute obstacle position as ego position + separation.

    If you change the axis mapping in TaxiAgent.cs, update ONLY this function.
    """
    x_ego  = float(obs[0])   # Unity Z
    y_ego  = float(obs[1])   # Unity X
    theta  = float(obs[2])   # yaw, normalised [-π, π]
    v      = float(obs[3])
    obs_dx = float(obs[4])   # obstacle forward separation
    obs_dy = float(obs[5])   # obstacle lateral separation
    obs_vx = float(obs[6])   # obstacle forward velocity
    obs_vy = float(obs[7])   # obstacle lateral velocity
    goal_dx = float(obs[8])

    s      = np.array([x_ego, y_ego, theta, v])
    obs_xy = np.array([x_ego + obs_dx, y_ego + obs_dy])
    obs_v  = np.array([obs_vx, obs_vy])
    return s, obs_xy, obs_v, goal_dx


# ── MPPI ─────────────────────────────────────────────────────────────────────

def mppi(s0, mean, obs_xy, obs_v, goal_dx):
    """
    Sample K_MPPI rollouts around 'mean' control sequence.
    Returns (u_nom, new_mean) where u_nom is the next action to apply.
    """
    noise = rng.normal(0, [SIG_A, SIG_D], (K_MPPI, H_MPPI, 2))
    na    = mean + noise
    na[:, :, 0] = np.clip(na[:, :, 0], A_MIN, A_MAX)
    na[:, :, 1] = np.clip(na[:, :, 1], -DELTA_LIM, DELTA_LIM)

    cost = np.zeros(K_MPPI)
    st   = np.tile(s0, (K_MPPI, 1)).astype(float)

    for k in range(H_MPPI):
        v = st[:, 3]
        st[:, 0] += v * np.cos(st[:, 2]) * DT
        st[:, 1] += v * np.sin(st[:, 2]) * DT
        st[:, 2] += v / L * np.tan(na[:, k, 1]) * DT
        st[:, 3]  = np.maximum(0., v + na[:, k, 0] * DT)

        y, th, vv = st[:, 1], st[:, 2], st[:, 3]
        cost += W_LAT  * y**2
        cost += W_HEAD * th**2
        cost += W_V    * (vv - V_DES)**2
        cost += W_CTRL * (na[:, k, 0]**2 + 4. * na[:, k, 1]**2)
        cost += np.where(np.abs(y) > W_HALF, W_OFF * (np.abs(y) - W_HALF)**2, 0.)

        obs_pred = obs_xy + obs_v * (k + 1) * DT
        d = np.hypot(st[:, 0] - obs_pred[0], st[:, 1] - obs_pred[1])
        cost += np.where(d < D_INFL, W_OBS * (D_INFL - d)**2, 0.)
        cost += np.where(d < D_SAFE, BIG, 0.)

    # Remaining-distance proxy: penalise rollouts that don't close on the goal
    cost += W_PROG * (goal_dx - (st[:, 0] - s0[0]))

    w   = np.exp(-(cost - cost.min()) / LAMBDA)
    w  /= w.sum()
    opt = (w[:, None, None] * na).sum(axis=0)   # (H_MPPI, 2)

    u_nom    = opt[0].copy()
    new_mean = np.vstack([opt[1:], opt[-1]])     # shift horizon by one step
    return u_nom, new_mean


# ── HOCBF-QP ─────────────────────────────────────────────────────────────────

def hocbf_constraint(s, u_nom, obs_xy, obs_v):
    """
    Compute the HOCBF linear constraint row [A_cbf] and right-hand side b_cbf
    for the QP:  A_cbf @ u ≥ b_cbf
    where u = [a, delta].

    h     = dist² - D_SAFE²
    ḣ     = 2(dx·rel_vx + dy·rel_vy)
    ḧ_nom is the second derivative of h evaluated at u_nom.

    HOCBF (second-order): ḧ + (α₁+α₂)ḣ + α₁α₂h ≥ 0
    """
    x, y, th, v  = s
    a_nom  = float(np.clip(u_nom[0], A_MIN, A_MAX))
    d_nom  = float(np.clip(u_nom[1], -DELTA_LIM, DELTA_LIM))
    px, py = obs_xy
    vx, vy = obs_v

    dx     = x  - px
    dy     = y  - py
    rel_vx = v * np.cos(th) - vx
    rel_vy = v * np.sin(th) - vy

    h    = dx**2 + dy**2 - D_SAFE**2
    hdot = 2. * (dx * rel_vx + dy * rel_vy)

    tand    = np.tan(d_nom)
    sec2d   = 1. + tand**2
    thdot_n = v / L * tand

    # ḧ terms
    hh_kin = 2. * (rel_vx**2 + rel_vy**2)
    hh_th  = 2.*dx*(-v*np.sin(th)*thdot_n) + 2.*dy*( v*np.cos(th)*thdot_n)
    hh_a   = (2.*dx*np.cos(th) + 2.*dy*np.sin(th)) * a_nom
    hh_nom = hh_kin + hh_th + hh_a

    # Gradient of ḧ w.r.t. u = [a, delta]
    dHH_da  = 2.*dx*np.cos(th) + 2.*dy*np.sin(th)
    dthd_dd = v / L * sec2d
    dHH_dd  = 2.*dx*(-v*np.sin(th))*dthd_dd + 2.*dy*(v*np.cos(th))*dthd_dd

    rhs = (-(ALPHA1 + ALPHA2)*hdot - ALPHA1*ALPHA2*h
           - hh_nom + dHH_da*a_nom + dHH_dd*d_nom)

    return np.array([dHH_da, dHH_dd]), rhs


def cbf_qp(s, u_nom, obs_xy, obs_v):
    """
    Solve the safety QP:
        min  (u - u_nom)ᵀ W (u - u_nom)
        s.t. A_cbf @ u ≥ b_cbf   (HOCBF safety constraint)
             A_MIN ≤ a ≤ A_MAX
             |delta| ≤ DELTA_LIM

    Returns (u_cmd, cbf_engaged) where cbf_engaged is True when the
    safety filter overrode the nominal action.
    """
    a_nom  = float(np.clip(u_nom[0], A_MIN, A_MAX))
    d_nom  = float(np.clip(u_nom[1], -DELTA_LIM, DELTA_LIM))
    u_n    = np.array([a_nom, d_nom])

    W = np.diag([1., ALPHA_W])   # weight acceleration vs steering deviation
    c = W @ u_n                  # linear term: minimise (u-u_n)ᵀ W (u-u_n)

    A_cbf, b_cbf = hocbf_constraint(s, u_nom, obs_xy, obs_v)

    # quadprog convention: C @ u ≥ b
    # Stack: [CBF, a≥A_MIN, a≤A_MAX, delta≥-DELTA_LIM, delta≤DELTA_LIM]
    C = np.column_stack([A_cbf,
                         [ 1., 0.], [-1., 0.],
                         [ 0., 1.], [ 0.,-1.]])
    b = np.array([b_cbf, A_MIN, -A_MAX, -DELTA_LIM, -DELTA_LIM])

    try:
        u_star   = quadprog.solve_qp(W, c, C, b, 0)[0]
        engaged  = bool(np.linalg.norm(u_star - u_n) > 1e-3)
        return u_star, engaged
    except Exception:
        # QP infeasible — apply maximum braking as a safe fallback
        return np.array([A_MIN, d_nom]), True


# ── System identification ─────────────────────────────────────────────────────

def identify_bicycle_model(env, behavior_name, n_steps=200):
    """
    Drive the aircraft at fixed throttle for n_steps to measure Unity's actual
    acceleration response. Prints a warning if the measured value deviates
    significantly from the commanded value.

    In kinematic mode (TaxiAgent sets velocity directly) this should match
    exactly — the check is most useful if you later switch to force-based.
    """
    print("[SysID] Probing bicycle dynamics...")
    decision_steps, _ = env.get_steps(behavior_name)
    speeds = []

    for _ in range(n_steps):
        n = len(decision_steps)
        if n == 0:
            env.step()
            decision_steps, _ = env.get_steps(behavior_name)
            continue

        action = ActionTuple(
            continuous=np.tile([A_MAX * 0.5, 0.0], (n, 1)).astype(np.float32)
        )
        env.set_actions(behavior_name, action)
        env.step()
        decision_steps, _ = env.get_steps(behavior_name)

        if len(decision_steps) > 0:
            speeds.append(float(decision_steps.obs[0][0][3]))

    if len(speeds) > 2:
        empirical = (speeds[-1] - speeds[0]) / (len(speeds) * DT)
        print(f"[SysID] Commanded a={A_MAX*0.5:.2f} m/s²,  "
              f"measured accel≈{empirical:.3f} m/s²")
        if abs(empirical - A_MAX * 0.5) > 0.2 * A_MAX * 0.5:
            print("[SysID] WARNING: >20% mismatch — check Fixed Timestep or "
                  "ApplyBicycleDynamics in TaxiAgent.cs")
    else:
        print("[SysID] Not enough data — is the Unity environment running?")


# ── Scenario sweep ────────────────────────────────────────────────────────────

# Encounter geometry is parameterized by a single variable: the incursion agent's
# time-to-conflict offset Δt [s]. Δt=0 means the airplane and the crosser reach the
# conflict point simultaneously (worst case). The dangerous band is roughly
# Δt ∈ [-DT_SPAN, +DT_SPAN]; outside it they miss naturally.
DT_SPAN     = 3.0        # half-width of the swept Δt band [s]
JITTER_DT   = 0.15       # seeded per-episode Δt jitter [s]
SPEED_JIT   = 0.10       # ±10% ambulance speed jitter
BASE_SEED   = 1234       # sweep reproducibility seed


def make_scenarios(n_episodes, base_seed=BASE_SEED):
    """
    Build a reproducible list of per-episode scenario parameter dicts.

    Layer 1 — deterministic Δt grid over [-DT_SPAN, +DT_SPAN] for coverage of the
              dangerous encounter band (same every run → regression-friendly).
    Layer 2 — seeded jitter on Δt and ambulance speed for off-grid robustness.

    Each dict maps directly to ML-Agents EnvironmentParameters keys consumed by
    TaxiAgent.OnEpisodeBegin.
    """
    grid = np.linspace(-DT_SPAN, DT_SPAN, n_episodes)
    scenarios = []
    for i, dt in enumerate(grid):
        r = np.random.default_rng(base_seed + i)
        scenarios.append({
            "incursion_dt":    float(dt + r.uniform(-JITTER_DT, JITTER_DT)),
            "ambulance_speed": float(5.0 * (1.0 + r.uniform(-SPEED_JIT, SPEED_JIT))),
            # spawn_lateral left unset → TaxiAgent uses its own spawnLateralRange noise
        })
    return scenarios


# ── Main control loop ─────────────────────────────────────────────────────────

def run(unity_exec_path=None, port=5004, run_sysid=True, n_episodes=20):
    """
    Connect to Unity (Editor in Play mode, or a built executable) and run the
    MPPI + HOCBF-QP controller for n_episodes.

    Args:
        unity_exec_path: path to built Unity .x86_64 / .exe.
                         Pass None to connect to the Unity Editor (must be in
                         Play mode before running this script).
        port:            ML-Agents communication port (default 5004).
        run_sysid:       run bicycle system-ID step before the episode loop.
        n_episodes:      number of episodes to run.
    """
    print(f"[Controller] Connecting to Unity on port {port} ...")
    env_params = EnvironmentParametersChannel()
    env = UnityEnvironment(
        file_name=unity_exec_path,
        base_port=port,
        seed=42,
        no_graphics=False,
        side_channels=[env_params],
    )
    env.reset()

    behavior_name = list(env.behavior_specs.keys())[0]
    spec          = env.behavior_specs[behavior_name]
    print(f"[Controller] Behavior  : {behavior_name}")
    print(f"[Controller] Obs shape : {spec.observation_specs[0].shape}")
    print(f"[Controller] Act size  : {spec.action_spec.continuous_size}")

    obs_size = spec.observation_specs[0].shape[0]
    act_size = spec.action_spec.continuous_size
    assert obs_size == 10, (
        f"Expected 10 observations, got {obs_size}. "
        "Check TaxiAgent.cs CollectObservations and BehaviorParameters."
    )
    assert act_size == 2, (
        f"Expected 2 continuous actions, got {act_size}. "
        "Check TaxiAgent.cs BehaviorParameters."
    )

    if run_sysid:
        identify_bicycle_model(env, behavior_name)
        env.reset()

    scenarios     = make_scenarios(n_episodes)
    episode_stats = []

    for ep in range(n_episodes):
        mean    = np.zeros((H_MPPI, 2))
        sc      = scenarios[ep]
        ep_log  = {"min_h": np.inf, "min_dist": np.inf,
                   "collided": False, "reached": False, "steps": 0,
                   "incursion_dt": sc["incursion_dt"]}

        # Push this episode's scenario parameters BEFORE reset so TaxiAgent.
        # OnEpisodeBegin reads them when Unity rebuilds the episode.
        for key, val in sc.items():
            env_params.set_float_parameter(key, val)

        print(f"\n[Ep {ep+1:3d}] scenario  Δt={sc['incursion_dt']:+.2f}s  "
              f"v_amb={sc['ambulance_speed']:.2f} m/s")

        env.reset()
        decision_steps, terminal_steps = env.get_steps(behavior_name)

        while True:
            if len(decision_steps) == 0:
                # Episode ended — terminal_steps contains the final step info
                break

            obs = decision_steps.obs[0][0]           # shape (10,)
            s, obs_xy, obs_v, goal_dx = obs_to_state(obs)

            # ── Step 7 coordinate-verification debug ────────────────────────
            if ep_log["steps"] % 20 == 0:
                print(f"[DEBUG] x={s[0]:.1f} y={s[1]:.2f} th={s[2]:.3f} "
                      f"v={s[3]:.2f} obs_xy={obs_xy}  goal_dx={goal_dx:.1f}")

            # MPPI nominal action
            u_nom, mean = mppi(s, mean, obs_xy, obs_v, goal_dx)

            # HOCBF-QP safety filter
            u_cmd, cbf_engaged = cbf_qp(s, u_nom, obs_xy, obs_v)
            a_cmd     = float(np.clip(u_cmd[0], A_MIN, A_MAX))
            delta_cmd = float(np.clip(u_cmd[1], -DELTA_LIM, DELTA_LIM))

            # Logging
            h_val = float(obs[9])
            dist  = float(np.sqrt(max(0., h_val + D_SAFE**2)))
            ep_log["min_h"]    = min(ep_log["min_h"], h_val)
            ep_log["min_dist"] = min(ep_log["min_dist"], dist)
            ep_log["steps"]   += 1

            if cbf_engaged:
                print(f"  [CBF] t={ep_log['steps']*DT:.1f}s  "
                      f"h={h_val:.1f}  dist={dist:.2f}m  "
                      f"a_nom={u_nom[0]:.2f}→a_cmd={a_cmd:.2f}  "
                      f"δ_nom={u_nom[1]:.3f}→δ_cmd={delta_cmd:.3f}")

            action = ActionTuple(
                continuous=np.array([[a_cmd, delta_cmd]], dtype=np.float32)
            )
            env.set_actions(behavior_name, action)
            env.step()
            decision_steps, terminal_steps = env.get_steps(behavior_name)

        ep_log["reached"]  = ep_log["min_dist"] > D_SAFE and ep_log["steps"] > 0
        ep_log["collided"] = ep_log["min_h"] < 0.
        episode_stats.append(ep_log)

        verdict = "COLLISION" if ep_log["collided"] else "safe"
        print(f"[Ep {ep+1:3d}] Δt={ep_log['incursion_dt']:+.2f}s  "
              f"steps={ep_log['steps']:4d}  "
              f"min_dist={ep_log['min_dist']:5.2f}m  "
              f"min_h={ep_log['min_h']:8.2f}  → {verdict}")

    env.close()

    print("\n=== Summary ===")
    n   = len(episode_stats)
    col = sum(1 for e in episode_stats if e["collided"])
    print(f"Collision rate : {col}/{n} = {col/n:.1%}")
    print(f"Mean min_dist  : {np.mean([e['min_dist'] for e in episode_stats]):.2f} m")
    print(f"Worst min_dist : {np.min([e['min_dist'] for e in episode_stats]):.2f} m "
          f"(target ≥ {D_SAFE:.1f} m)")
    print(f"Worst min_h    : {np.min([e['min_h'] for e in episode_stats]):.2f} "
          "(target ≥ 0)")

    # Per-Δt breakdown — shows which encounter geometries (if any) stress the barrier
    print("\n  Δt[s]   min_dist[m]   min_h     result")
    for e in sorted(episode_stats, key=lambda d: d["incursion_dt"]):
        print(f"  {e['incursion_dt']:+5.2f}   {e['min_dist']:8.2f}   "
              f"{e['min_h']:8.2f}   {'COLLISION' if e['collided'] else 'safe'}")
    return episode_stats


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--exec",     default=None,
                   help="Path to Unity build executable (omit to connect to Editor)")
    p.add_argument("--port",     default=5004,  type=int)
    p.add_argument("--sysid",    default=True,  type=lambda x: x.lower() == "true")
    p.add_argument("--episodes", default=20,    type=int)
    args = p.parse_args()

    run(unity_exec_path=args.exec if args.exec != "None" else None,
        port=args.port,
        run_sysid=args.sysid,
        n_episodes=args.episodes)
