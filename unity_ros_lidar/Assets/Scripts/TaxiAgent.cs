using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// ML-Agents Agent for aircraft taxiing.
/// External Python process sends actions [a, delta] each decision step.
///
/// COORDINATE MAPPING (Unity ↔ Python baseline):
///   Python X (forward along taxiway) = Unity +Z
///   Python Y (lateral offset)        = Unity +X
///   Python θ (heading error)         = Unity eulerAngles.Y in radians, normalised to [-π, π]
///   The airplane's visual front is +Z (frontIsNegativeZ = false on EgoRouteFollower).
///
/// This script replaces EgoRouteFollower for ML-Agents episodes. Disable or
/// remove EgoRouteFollower from the airplane when using TaxiAgent.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class TaxiAgent : Unity.MLAgents.Agent
{
    [Header("Aircraft parameters — must match Python DT and L")]
    public float wheelbase    = 6.0f;   // L [m]
    public float maxAccel     = 1.5f;   // A_MAX [m/s²]
    public float maxBrake     = 4.0f;   // |A_MIN| [m/s²]
    public float maxSteer     = 0.5f;   // DELTA_LIM [rad]
    public float desiredSpeed = 8.0f;   // V_DES [m/s]

    [Header("Scene references")]
    public Transform goalMarker;        // empty GameObject at the end of the taxiway
    public Transform incursionAgent;    // the crossing obstacle (Vehicle NPC)
    public float taxiwayHalfWidth = 10f;
    public float dSafe            = 6.0f;

    [Header("Spawn randomisation (set ranges to 0 to disable)")]
    public float spawnLateralRange = 0.0f;
    public float spawnHeadingRange = 0.0f;

    [Header("Parameterized incursion (scenario generation)")]
    [Tooltip("Controller that moves the incursion agent at constant velocity. " +
             "If assigned, the agent is reset deterministically each episode so the " +
             "encounter geometry is reproducible. Leave null to keep free-running NPC motion.")]
    public IncursionAgentController incursionController;
    [Tooltip("Empty GameObject where the incursion path crosses the taxiway centreline. " +
             "The agent is placed so it arrives here Δt seconds relative to the airplane.")]
    public Transform conflictPoint;
    [Tooltip("Default time-to-conflict offset Δt [s] when no side-channel value is supplied. " +
             "Δt=0 → airplane and agent reach the conflict point simultaneously (worst case).")]
    public float defaultIncursionDt = 0.0f;

    public float maxEpisodeSeconds = 60f;

    // ── private state ──────────────────────────────────────────────────────
    Rigidbody _rb;
    float     _speed;
    bool      _collided;
    float     _episodeTime;

    // Captured spawn pose so resets work in any scene (taxiway need not be at origin)
    Vector3 _spawnPos;

    // Per-episode scenario parameters (from ML-Agents EnvironmentParameters side channel)
    int _episodeIndex;

    // Obstacle velocity estimated from Transform delta each FixedUpdate
    Vector3 _obsPosPrev;
    Vector3 _obsVel;

    // ── Unity / ML-Agents lifecycle ────────────────────────────────────────

    public override void Initialize()
    {
        _rb = GetComponent<Rigidbody>();
        _rb.isKinematic = true;   // bicycle model drives position; Unity physics is passive
        _rb.constraints = RigidbodyConstraints.FreezeRotationX
                        | RigidbodyConstraints.FreezeRotationZ
                        | RigidbodyConstraints.FreezePositionY;

        // Remember the authored spawn pose so OnEpisodeBegin can reset to it
        // regardless of where the taxiway sits in world space.
        _spawnPos = transform.position;
    }

    public override void OnEpisodeBegin()
    {
        _collided     = false;
        _speed        = desiredSpeed;
        _episodeTime  = 0f;

        // ── Per-episode scenario parameters from the Python side channel ──────
        // Python pushes these via EnvironmentParametersChannel.set_float_parameter
        // before each env.reset(). Falls back to Inspector defaults if absent.
        var ep = Academy.Instance.EnvironmentParameters;
        float incursionDt    = ep.GetWithDefault("incursion_dt",    defaultIncursionDt);
        float spawnLateral   = ep.GetWithDefault("spawn_lateral",   float.NaN);
        float ambulanceSpeed = ep.GetWithDefault("ambulance_speed", -1f);

        // ── Place airplane at its authored spawn pose with optional noise ────
        float latOff = float.IsNaN(spawnLateral)
            ? Random.Range(-spawnLateralRange, spawnLateralRange)
            : spawnLateral;
        float hdgOff = Random.Range(-spawnHeadingRange, spawnHeadingRange);

        transform.position    = _spawnPos + new Vector3(latOff, 0f, 0f);
        transform.eulerAngles = new Vector3(0f, hdgOff * Mathf.Rad2Deg, 0f);

        // ── Deterministic incursion placement (Δt-parameterized) ─────────────
        // The airplane travels along +Z at desiredSpeed, so its time to reach the
        // conflict point is (conflict.z - spawn.z) / v. We place the crosser so it
        // reaches the conflict point at egoTimeToConflict + Δt:
        //   start = conflict - dir * speed * (egoTimeToConflict + Δt)
        if (incursionController != null && conflictPoint != null)
        {
            float egoTimeToConflict =
                Mathf.Max(0f, (conflictPoint.position.z - transform.position.z) / desiredSpeed);

            float speed = ambulanceSpeed > 0f ? ambulanceSpeed : incursionController.crossSpeed;
            Vector3 dir = incursionController.CrossDirectionNormalized;
            Vector3 start = conflictPoint.position - dir * speed * (egoTimeToConflict + incursionDt);

            incursionController.ResetCrossing(start, speed);
        }

        if (incursionAgent != null)
        {
            _obsPosPrev = incursionAgent.position;
            _obsVel     = Vector3.zero;
        }

        _episodeIndex++;
    }

    // ── Observations (10 floats) sent to Python ────────────────────────────
    //   [0] x_ego   — forward progress along taxiway (Unity Z) [m]
    //   [1] y_ego   — lateral offset from centreline (Unity X) [m]
    //   [2] theta   — heading error from +Z axis [rad], normalised to [-π, π]
    //   [3] v       — speed [m/s]
    //   [4] obs_dx  — obstacle Z separation (Unity) = Python forward sep [m]
    //   [5] obs_dy  — obstacle X separation (Unity) = Python lateral sep [m]
    //   [6] obs_vx  — obstacle velocity along Z [m/s]
    //   [7] obs_vy  — obstacle velocity along X [m/s]
    //   [8] goal_dx — distance to goal along Z [m]
    //   [9] cbf_h   — barrier value h = dist²-D²  (debug / logging)
    public override void CollectObservations(VectorSensor sensor)
    {
        // Ego state
        float x_ego = transform.position.z;    // Python X = Unity Z
        float y_ego = transform.position.x;    // Python Y = Unity X
        float theta = transform.eulerAngles.y * Mathf.Deg2Rad;
        theta = Mathf.Atan2(Mathf.Sin(theta), Mathf.Cos(theta));   // → [-π, π]
        // _speed is maintained by ApplyBicycleDynamics; no Rigidbody velocity to read

        // Obstacle state
        float obs_dx = 0f, obs_dy = 0f, obs_vx = 0f, obs_vy = 0f;
        float cbf_h = float.MaxValue;

        if (incursionAgent != null)
        {
            Vector3 obsPos = incursionAgent.position;
            _obsVel = (obsPos - _obsPosPrev) / Time.fixedDeltaTime;
            _obsPosPrev = obsPos;

            obs_dx = obsPos.z - transform.position.z;   // forward separation (Z)
            obs_dy = obsPos.x - transform.position.x;   // lateral separation (X)
            obs_vx = _obsVel.z;
            obs_vy = _obsVel.x;
            cbf_h  = obs_dx * obs_dx + obs_dy * obs_dy - dSafe * dSafe;
        }

        float goal_dx = goalMarker != null
            ? goalMarker.position.z - transform.position.z
            : 999f;

        sensor.AddObservation(x_ego);
        sensor.AddObservation(y_ego);
        sensor.AddObservation(theta);
        sensor.AddObservation(_speed);
        sensor.AddObservation(obs_dx);
        sensor.AddObservation(obs_dy);
        sensor.AddObservation(obs_vx);
        sensor.AddObservation(obs_vy);
        sensor.AddObservation(goal_dx);
        sensor.AddObservation(cbf_h);
    }

    // ── Actions received from Python ───────────────────────────────────────
    //   [0] a_cmd     — acceleration [m/s²] ∈ [A_MIN, A_MAX]
    //   [1] delta_cmd — steering [rad]       ∈ [-DELTA_LIM, DELTA_LIM]
    public override void OnActionReceived(ActionBuffers actions)
    {
        float a_cmd     = actions.ContinuousActions[0];
        float delta_cmd = actions.ContinuousActions[1];

        ApplyBicycleDynamics(a_cmd, delta_cmd);
        AddReward(ComputeReward());

        _episodeTime += Time.fixedDeltaTime;

        float goal_dx = goalMarker != null
            ? goalMarker.position.z - transform.position.z
            : 999f;

        bool reached = goal_dx < 2.0f;
        bool timeout = _episodeTime >= maxEpisodeSeconds;
        bool offRoad = Mathf.Abs(transform.position.x) > taxiwayHalfWidth + 2f;

        if (reached)   AddReward( 10f);
        if (_collided) AddReward(-20f);

        if (reached || _collided || timeout || offRoad)
            EndEpisode();
    }

    // ── Bicycle kinematic model ────────────────────────────────────────────
    // Sets velocity directly (kinematic Rigidbody) so the Python model is
    // authoritative. The rotation sign convention matches Unity's left-handed
    // Y-up coordinate system: positive delta steers left (+Z turning toward +X),
    // which is the same as positive θ̇ in the Python baseline (counter-clockwise
    // when viewed from above in right-handed convention is CW in Unity Y-up).
    // The Python controller is consistent with this because the obs mapping
    // passes eulerAngles.y directly.
    void ApplyBicycleDynamics(float a, float delta)
    {
        a     = Mathf.Clamp(a,     -maxBrake, maxAccel);
        delta = Mathf.Clamp(delta, -maxSteer, maxSteer);

        _speed = Mathf.Max(0f, _speed + a * Time.fixedDeltaTime);

        // dθ/dt = (v/L) * tan(δ)  — positive δ turns left (Unity: rotate Y positively)
        float dTheta = (_speed / wheelbase) * Mathf.Tan(delta) * Time.fixedDeltaTime;
        transform.Rotate(Vector3.up, dTheta * Mathf.Rad2Deg);

        transform.position += transform.forward * (_speed * Time.fixedDeltaTime);
    }

    // ── Reward (used for logging; MPPI drives actions, not gradient) ───────
    float ComputeReward()
    {
        float y_ego   = transform.position.x;
        float theta   = transform.eulerAngles.y * Mathf.Deg2Rad;
        theta = Mathf.Atan2(Mathf.Sin(theta), Mathf.Cos(theta));

        float r = -0.01f * y_ego * y_ego
                  -0.04f * theta * theta
                  -0.01f * Mathf.Pow(_speed - desiredSpeed, 2f)
                  + 0.02f * _speed;

        if (Mathf.Abs(y_ego) > taxiwayHalfWidth) r -= 0.5f;
        return r;
    }

    // ── Collision detection ────────────────────────────────────────────────
    void OnTriggerEnter(Collider other)
    {
        if (other.gameObject == gameObject) return;

        // Ignore the ground plane and static environment meshes
        if (other.gameObject.name == "Plane" || other.gameObject.isStatic) return;

        _collided = true;
        Debug.LogWarning($"[TaxiAgent] COLLISION with '{other.gameObject.name}' at t={_episodeTime:F2}s", this);
    }

    // ── Heuristic — keyboard control for testing without Python ───────────
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var ca = actionsOut.ContinuousActions;
        ca[0] = Input.GetAxis("Vertical")   * maxAccel;
        ca[1] = Input.GetAxis("Horizontal") * maxSteer;
    }
}
