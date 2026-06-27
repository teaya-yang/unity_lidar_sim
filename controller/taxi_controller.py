"""
taxi_controller.py
==================
External Python controller for the Unity taxiing environment.
Connects via the ML-Agents Python API, runs MPPI + HOCBF-QP, and sends
[a, delta] actions back to Unity each decision step.

Observation contract (must match TaxiAgent.cs CollectObservations order):
  obs[0]   x_ego    — forward position along taxiway (Unity Z) [m]
  obs[1]   y_ego    — lateral offset (Unity X) [m]
  obs[2]   theta    — heading error from +Z axis [rad], normalised to [-pi, pi]
  obs[3]   v        — speed [m/s]
  obs[4]   obs0_dx  — nearest obstacle: forward separation (Unity Z) [m]
  obs[5]   obs0_dy  — nearest obstacle: lateral separation (Unity X) [m]
  obs[6]   obs0_vx  — nearest obstacle: velocity along Z [m/s]
  obs[7]   obs0_vy  — nearest obstacle: velocity along X [m/s]
  obs[8]   obs1_dx  — 2nd obstacle (zero-padded if absent)
  obs[9]   obs1_dy
  obs[10]  obs1_vx
  obs[11]  obs1_vy
  obs[12]  obs2_dx  — 3rd obstacle (zero-padded if absent)
  obs[13]  obs2_dy
  obs[14]  obs2_vx
  obs[15]  obs2_vy
  obs[16]  goal_dx  — distance to goal along Z [m]
  obs[17]  cbf_h    — barrier value h = dist^2 - D^2 of nearest obstacle (logging)

Action contract:
  act[0]  a_cmd     — acceleration [m/s^2], clipped to [A_MIN, A_MAX]
  act[1]  delta_cmd — steering [rad],       clipped to [-DELTA_LIM, DELTA_LIM]

COORDINATE NOTE:
  Python baseline uses X=forward, Y=lateral.
  Unity scene uses Z=forward (taxiway), X=lateral.
  obs_to_state() is the single bridge. Do NOT touch MPPI/CBF logic if only the
  axis mapping changes — fix obs_to_state() only.
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

# ── Realistic kinematic extensions — must match TaxiAgent.cs inspector values ─
DRAG_COEFF        = 0.04   # aerodynamic + rolling drag [1/s]
ACCEL_TAU         = 0.5    # thrust/brake lag time constant [s]
MAX_STEER_RATE    = 0.6    # nose-wheel steering rate limit [rad/s]
STEER_ROLLOFF_SPD = 15.0   # speed at which steering authority starts rolling off [m/s]
STEER_ROLLOFF_MIN = 0.25   # minimum steering authority fraction at high speed

H_MPPI    = 25           # planning horizon (steps)
K_MPPI    = 1500         # rollout samples
LAMBDA    = 1.0          # MPPI temperature
SIG_A     = 1.0          # noise std for acceleration samples
SIG_D     = 0.12         # noise std for steering samples

# MPPI stage costs
W_LAT, W_HEAD, W_V, W_CTRL = 3.0, 6.0, 1.2, 0.05
W_OBS, W_OFF, W_PROG        = 12.0, 200.0, 0.4
BIG = 300.0

# Number of obstacles packed in the observation vector (must match TaxiAgent K_OBS)
K_OBS    = 3
OBS_SIZE = 4 + K_OBS * 4 + 2   # 18: ego(4) + K_OBS*4 + goal_dx(1) + cbf_h(1)

# Scenario types — int value pushed as 'scenario_type' side channel
SCENARIO_STANDARD    = 0
SCENARIO_STATIONARY  = 1
SCENARIO_HEADON      = 2
SCENARIO_HIGHSPEED   = 3
SCENARIO_ACCELERATING= 4
SCENARIO_NAMES = ['standard','stationary','headon','highspeed','accelerating']

V_DES_HIGH = 14.0   # high-speed scenario [m/s] (~27 knots)

rng = np.random.default_rng(42)


# ── Observation unpacking ────────────────────────────────────────────────────

def obs_to_state(obs: np.ndarray, prev_delta: float = 0.0, prev_accel: float = 0.0):
    """
    Unpack the Unity observation vector.

    Returns
    -------
    s        : np.ndarray shape (6,)  — [x_fwd, y_lat, theta, v, delta_actual, accel_actual]
    obstacles: list of (obs_xy, obs_v) — up to K_OBS entries, padded slots omitted
    goal_dx  : float

    delta_actual and accel_actual are NOT sent as observations (Unity owns them internally).
    We carry them as Python-side state across steps, passed in as prev_delta / prev_accel.

    A padded slot has obs_dy == 999 (sentinel from TaxiAgent zero-padding).
    We skip those so MPPI/CBF only sees real obstacles.
    """
    x_ego   = float(obs[0])
    y_ego   = float(obs[1])
    theta   = float(obs[2])
    v       = float(obs[3])
    goal_dx = float(obs[16])

    s = np.array([x_ego, y_ego, theta, v, prev_delta, prev_accel])

    obstacles = []
    for i in range(K_OBS):
        base   = 4 + i * 4
        dx     = float(obs[base + 0])
        dy     = float(obs[base + 1])
        vx_obs = float(obs[base + 2])
        vy_obs = float(obs[base + 3])

        # Skip zero-padded slots (dy == 999 sentinel)
        if abs(dy) > 900:
            continue

        obs_xy = np.array([x_ego + dx, y_ego + dy])
        obs_v  = np.array([vx_obs, vy_obs])
        obstacles.append((obs_xy, obs_v))

    return s, obstacles, goal_dx


def inject_sensor_noise(obs: np.ndarray, noise_std: float, rng_local) -> np.ndarray:
    """
    Add Gaussian noise to the obstacle observation slots [4..15].
    Ego state [0..3] and goal/cbf [16..17] are not corrupted
    (they come from onboard sensors assumed more reliable).
    """
    if noise_std <= 0.0:
        return obs
    noisy = obs.copy()
    noisy[4:16] += rng_local.normal(0.0, noise_std, 12).astype(obs.dtype)
    return noisy


# ── MPPI ─────────────────────────────────────────────────────────────────────

def _rollout_step(st, a_cmd, delta_cmd):
    """
    One-step realistic bicycle dynamics for MPPI rollouts.
    st columns: [x, y, theta, v, delta_actual, accel_actual]
    Mirrors TaxiAgent.ApplyBicycleDynamics exactly.
    """
    v            = st[:, 3]
    delta_actual = st[:, 4]
    accel_actual = st[:, 5]

    # 1+2: rate-limited, speed-dependent steering
    speed_frac    = np.clip(v / max(STEER_ROLLOFF_SPD, 1e-3), 0., 1.)
    eff_limit     = DELTA_LIM * (1. - speed_frac * (1. - STEER_ROLLOFF_MIN))
    delta_target  = np.clip(delta_cmd, -eff_limit, eff_limit)
    max_delta_step = MAX_STEER_RATE * DT
    delta_new     = delta_actual + np.clip(delta_target - delta_actual,
                                           -max_delta_step, max_delta_step)

    # 3: first-order acceleration lag
    a_clamped  = np.clip(a_cmd, A_MIN, A_MAX)
    if ACCEL_TAU > 1e-3:
        accel_new = accel_actual + (a_clamped - accel_actual) * (DT / ACCEL_TAU)
    else:
        accel_new = a_clamped

    # 4: drag + speed integration
    drag  = DRAG_COEFF * v
    v_new = np.maximum(0., v + (accel_new - drag) * DT)

    # Bicycle geometry
    dtheta = v_new / L * np.tan(delta_new) * DT
    x_new  = st[:, 0] + v_new * np.cos(st[:, 2]) * DT
    y_new  = st[:, 1] + v_new * np.sin(st[:, 2]) * DT
    th_new = st[:, 2] + dtheta

    st_new = np.stack([x_new, y_new, th_new, v_new, delta_new, accel_new], axis=1)
    return st_new


def mppi(s0, mean, obstacles, goal_dx):
    """
    Sample K_MPPI rollouts with realistic 6D state dynamics.
    s0 shape: (6,) — [x, y, theta, v, delta_actual, accel_actual]
    Returns (u_nom, new_mean).
    """
    noise = rng.normal(0, [SIG_A, SIG_D], (K_MPPI, H_MPPI, 2))
    na    = mean + noise
    na[:, :, 0] = np.clip(na[:, :, 0], A_MIN, A_MAX)
    na[:, :, 1] = np.clip(na[:, :, 1], -DELTA_LIM, DELTA_LIM)

    cost = np.zeros(K_MPPI)
    st   = np.tile(s0, (K_MPPI, 1)).astype(float)

    for k in range(H_MPPI):
        st = _rollout_step(st, na[:, k, 0], na[:, k, 1])

        y, th, vv = st[:, 1], st[:, 2], st[:, 3]
        cost += W_LAT  * y**2
        cost += W_HEAD * th**2
        cost += W_V    * (vv - V_DES)**2
        cost += W_CTRL * (na[:, k, 0]**2 + 4. * na[:, k, 1]**2)
        cost += np.where(np.abs(y) > W_HALF, W_OFF * (np.abs(y) - W_HALF)**2, 0.)

        for obs_xy, obs_v in obstacles:
            obs_pred = obs_xy + obs_v * (k + 1) * DT
            d = np.hypot(st[:, 0] - obs_pred[0], st[:, 1] - obs_pred[1])
            cost += np.where(d < D_INFL, W_OBS * (D_INFL - d)**2, 0.)
            cost += np.where(d < D_SAFE, BIG, 0.)

    cost += W_PROG * (goal_dx - (st[:, 0] - s0[0]))

    w   = np.exp(-(cost - cost.min()) / LAMBDA)
    w  /= w.sum()
    opt = (w[:, None, None] * na).sum(axis=0)

    u_nom    = opt[0].copy()
    new_mean = np.vstack([opt[1:], opt[-1]])
    return u_nom, new_mean


# ── HOCBF-QP ─────────────────────────────────────────────────────────────────

def hocbf_constraint(s, u_nom, obs_xy, obs_v):
    """
    Compute one HOCBF constraint row for a single obstacle.
    Returns (A_row, b_row) for the QP inequality A @ u >= b.
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

    hh_kin = 2. * (rel_vx**2 + rel_vy**2)
    hh_th  = 2.*dx*(-v*np.sin(th)*thdot_n) + 2.*dy*(v*np.cos(th)*thdot_n)
    hh_a   = (2.*dx*np.cos(th) + 2.*dy*np.sin(th)) * a_nom
    hh_nom = hh_kin + hh_th + hh_a

    dHH_da  = 2.*dx*np.cos(th) + 2.*dy*np.sin(th)
    dthd_dd = v / L * sec2d
    dHH_dd  = 2.*dx*(-v*np.sin(th))*dthd_dd + 2.*dy*(v*np.cos(th))*dthd_dd

    rhs = (-(ALPHA1 + ALPHA2)*hdot - ALPHA1*ALPHA2*h
           - hh_nom + dHH_da*a_nom + dHH_dd*d_nom)

    return np.array([dHH_da, dHH_dd]), rhs


def cbf_qp(s, u_nom, obstacles):
    """
    Solve the safety QP with one constraint row per obstacle:
        min  (u - u_nom)^T W (u - u_nom)
        s.t. A_cbf[i] @ u >= b_cbf[i]  for each obstacle i
             A_MIN <= a <= A_MAX
             |delta| <= DELTA_LIM

    Returns (u_cmd, cbf_engaged).
    """
    a_nom = float(np.clip(u_nom[0], A_MIN, A_MAX))
    d_nom = float(np.clip(u_nom[1], -DELTA_LIM, DELTA_LIM))
    u_n   = np.array([a_nom, d_nom])

    W = np.diag([1., ALPHA_W])
    c = W @ u_n

    # Box constraints: a >= A_MIN, a <= A_MAX, delta >= -DELTA_LIM, delta <= DELTA_LIM
    C_box = np.array([[ 1., 0.], [-1., 0.], [0.,  1.], [0., -1.]]).T   # (2, 4)
    b_box = np.array([A_MIN, -A_MAX, -DELTA_LIM, -DELTA_LIM])

    if obstacles:
        cbf_rows = []
        cbf_rhs  = []
        for obs_xy, obs_v in obstacles:
            row, rhs = hocbf_constraint(s, u_nom, obs_xy, obs_v)
            cbf_rows.append(row)
            cbf_rhs.append(rhs)
        C_cbf = np.array(cbf_rows).T          # (2, n_obs)
        b_cbf = np.array(cbf_rhs)
        C = np.hstack([C_cbf, C_box])          # (2, n_obs + 4)
        b = np.concatenate([b_cbf, b_box])
    else:
        C = C_box
        b = b_box

    try:
        u_star  = quadprog.solve_qp(W, c, C, b, 0)[0]
        engaged = bool(np.linalg.norm(u_star - u_n) > 1e-3)
        return u_star, engaged
    except Exception:
        return np.array([A_MIN, d_nom]), True


# ── System identification ─────────────────────────────────────────────────────

def identify_bicycle_model(env, behavior_name, n_steps=200):
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
        print(f"[SysID] Commanded a={A_MAX*0.5:.2f} m/s^2, "
              f"measured accel≈{empirical:.3f} m/s^2")
        if abs(empirical - A_MAX * 0.5) > 0.2 * A_MAX * 0.5:
            print("[SysID] WARNING: >20% mismatch — check Fixed Timestep or "
                  "ApplyBicycleDynamics in TaxiAgent.cs")
    else:
        print("[SysID] Not enough data — is the Unity environment running?")


# ── Scenario sweep ────────────────────────────────────────────────────────────

DT_SPAN        = 3.0
JITTER_DT      = 0.15
SPEED_JIT      = 0.10
BASE_SEED      = 1234
CONFLICT_Z_MAX = 20.0   # max Z shift of conflict point along taxiway [m]


def make_scenarios(n_episodes, base_seed=BASE_SEED, min_difficulty=0.0, max_difficulty=1.0,
                   scenario_type=SCENARIO_STANDARD):
    """
    Build a reproducible list of per-episode scenario parameter dicts.

    Each dict is pushed to Unity via EnvironmentParametersChannel before env.reset().
    Keys:
      incursion_dt    — Δt offset for the primary (agent[0]) incursion [s]
      ambulance_speed — crossing speed for agent[0] [m/s]
      difficulty      — [0, 1] curriculum difficulty sent to ScenarioManager

    min_difficulty / max_difficulty clamp the ramp so you can test a specific
    difficulty band. E.g. min_difficulty=0.85 forces 3-agent Erratic scenarios.
    """
    grid = np.linspace(-DT_SPAN, DT_SPAN, n_episodes)
    scenarios = []
    for i, dt in enumerate(grid):
        r = np.random.default_rng(base_seed + i)
        t = i / max(1, n_episodes - 1)   # 0 → 1
        difficulty = float(min_difficulty + t * (max_difficulty - min_difficulty))
        # Choose scenario type if a specific one isn't forced
        stype = float(scenario_type) if scenario_type >= 0 else float(r.integers(0, 5))
        desired_spd = V_DES_HIGH if stype == SCENARIO_HIGHSPEED else -1.0
        scenarios.append({
            "incursion_dt":      float(dt + r.uniform(-JITTER_DT, JITTER_DT)),
            "ambulance_speed":   float(5.0 * (1.0 + r.uniform(-SPEED_JIT, SPEED_JIT))),
            "difficulty":        difficulty,
            "conflict_z_offset": float(r.uniform(-CONFLICT_Z_MAX, CONFLICT_Z_MAX)),
            "cross_dir_sign":    float(r.choice([-1.0, 1.0])),
            "scenario_type":     stype,
            "desired_speed":     desired_spd,
            "head_on_prob":      0.3 if stype == SCENARIO_HEADON else 0.0,
        })
    return scenarios


# ── Main control loop ─────────────────────────────────────────────────────────

def run(unity_exec_path=None, port=5004, run_sysid=True, n_episodes=20,
        min_difficulty=0.0, max_difficulty=1.0,
        noise_std=0.0, scenario_type=SCENARIO_STANDARD):
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
    assert obs_size == OBS_SIZE, (
        f"Expected {OBS_SIZE} observations, got {obs_size}. "
        f"Set BehaviorParameters Vector Observations = {OBS_SIZE} in Unity Inspector "
        f"and ensure TaxiAgent.cs K_OBS = {K_OBS}."
    )
    assert act_size == 2, (
        f"Expected 2 continuous actions, got {act_size}."
    )

    if run_sysid:
        identify_bicycle_model(env, behavior_name)
        env.reset()

    scenarios     = make_scenarios(n_episodes, min_difficulty=min_difficulty,
                                              max_difficulty=max_difficulty,
                                              scenario_type=scenario_type)
    episode_stats = []

    for ep in range(n_episodes):
        mean   = np.zeros((H_MPPI, 2))
        sc     = scenarios[ep]
        ep_log = {"min_h": np.inf, "min_dist": np.inf,
                  "collided": False, "reached": False, "steps": 0,
                  "incursion_dt": sc["incursion_dt"],
                  "difficulty": sc["difficulty"],
                  "scenario": SCENARIO_NAMES[int(sc["scenario_type"])]}

        for key, val in sc.items():
            env_params.set_float_parameter(key, val)

        sname = SCENARIO_NAMES[int(sc["scenario_type"])]
        print(f"\n[Ep {ep+1:3d}] [{sname}] Δt={sc['incursion_dt']:+.2f}s  "
              f"v_amb={sc['ambulance_speed']:.2f} m/s  "
              f"diff={sc['difficulty']:.2f}  "
              f"noise={noise_std:.3f}  "
              f"dir={'L' if sc['cross_dir_sign'] < 0 else 'R'}")

        env.reset()
        decision_steps, terminal_steps = env.get_steps(behavior_name)

        prev_x        = None
        ep_steps      = 0
        delta_actual  = 0.0   # tracked Python-side to feed into MPPI state
        accel_actual  = 0.0

        while True:
            if len(decision_steps) == 0:
                break

            obs   = decision_steps.obs[0][0]          # shape (OBS_SIZE,)
            x_now = float(obs[0])

            # Detect episode reset: x_ego jumps backward
            if prev_x is not None and x_now < prev_x - 10.0 and ep_steps > 10:
                break
            prev_x   = x_now
            ep_steps += 1

            obs_n = inject_sensor_noise(obs, noise_std, rng)
            s, obstacles, goal_dx = obs_to_state(obs_n, delta_actual, accel_actual)

            if ep_log["steps"] % 20 == 0:
                n_obs_str = f"{len(obstacles)} obstacle(s)"
                print(f"[DEBUG] x={s[0]:.1f} y={s[1]:.2f} th={s[2]:.3f} "
                      f"v={s[3]:.2f} δ={s[4]:.3f} a_act={s[5]:.2f}  "
                      f"{n_obs_str}  goal_dx={goal_dx:.1f}")

            u_nom, mean = mppi(s, mean, obstacles, goal_dx)
            u_cmd, cbf_engaged = cbf_qp(s[:4], u_nom, obstacles)  # CBF uses [x,y,th,v] only
            a_cmd     = float(np.clip(u_cmd[0], A_MIN, A_MAX))
            delta_cmd = float(np.clip(u_cmd[1], -DELTA_LIM, DELTA_LIM))

            # Advance Python-side kinematic state to match what Unity will compute
            v = s[3]
            speed_frac   = min(v / max(STEER_ROLLOFF_SPD, 1e-3), 1.0)
            eff_limit    = DELTA_LIM * (1.0 - speed_frac * (1.0 - STEER_ROLLOFF_MIN))
            delta_target = float(np.clip(delta_cmd, -eff_limit, eff_limit))
            delta_actual = delta_actual + float(np.clip(
                delta_target - delta_actual, -MAX_STEER_RATE * DT, MAX_STEER_RATE * DT))
            if ACCEL_TAU > 1e-3:
                accel_actual += (float(np.clip(a_cmd, A_MIN, A_MAX)) - accel_actual) * (DT / ACCEL_TAU)
            else:
                accel_actual = float(np.clip(a_cmd, A_MIN, A_MAX))

            # Log nearest obstacle distance
            h_val = float(obs[17])
            dist  = float(np.sqrt(max(0., h_val + D_SAFE**2)))
            ep_log["min_h"]    = min(ep_log["min_h"], h_val)
            ep_log["min_dist"] = min(ep_log["min_dist"], dist)
            ep_log["steps"]    = ep_steps

            if cbf_engaged:
                print(f"  [CBF] t={ep_steps*DT:.1f}s  h={h_val:.1f}  dist={dist:.2f}m  "
                      f"a_nom={u_nom[0]:.2f}→{a_cmd:.2f}  "
                      f"d_nom={u_nom[1]:.3f}→{delta_cmd:.3f}  "
                      f"n_obs={len(obstacles)}")

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
              f"diff={ep_log['difficulty']:.2f}  "
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
          f"(target >= {D_SAFE:.1f} m)")
    print(f"Worst min_h    : {np.min([e['min_h'] for e in episode_stats]):.2f} "
          "(target >= 0)")

    print("\n  Δt[s]  scenario      diff  min_dist[m]   min_h     result")
    for e in sorted(episode_stats, key=lambda d: d["incursion_dt"]):
        print(f"  {e['incursion_dt']:+5.2f}  {e['scenario']:<12s}  {e['difficulty']:.2f}  "
              f"{e['min_dist']:8.2f}   {e['min_h']:8.2f}   "
              f"{'COLLISION' if e['collided'] else 'safe'}")
    return episode_stats


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--exec",           default=None)
    p.add_argument("--port",           default=5004,  type=int)
    p.add_argument("--sysid",          default=True,  type=lambda x: x.lower() == "true")
    p.add_argument("--episodes",       default=20,    type=int)
    p.add_argument("--min-difficulty", default=0.0,   type=float)
    p.add_argument("--max-difficulty", default=1.0,   type=float)
    p.add_argument("--noise-std",      default=0.0,   type=float,
                   help="Std-dev of Gaussian noise injected into obstacle obs [m]. 0=off.")
    p.add_argument("--scenario",       default="standard",
                   choices=SCENARIO_NAMES,
                   help="Force a specific scenario type for all episodes.")
    args = p.parse_args()

    sc_int = SCENARIO_NAMES.index(args.scenario)
    run(unity_exec_path=args.exec if args.exec != "None" else None,
        port=args.port,
        run_sysid=args.sysid,
        n_episodes=args.episodes,
        min_difficulty=args.min_difficulty,
        max_difficulty=args.max_difficulty,
        noise_std=args.noise_std,
        scenario_type=sc_int)
