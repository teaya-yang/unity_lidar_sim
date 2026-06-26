using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// ML-Agents Agent for aircraft taxiing.
/// External Python process sends actions [a, delta] each decision step.
///
/// OBSERVATION VECTOR (18 floats — must match Python obs_to_state):
///   [0]  x_ego    — forward position along taxiway (Unity Z) [m]
///   [1]  y_ego    — lateral offset (Unity X) [m]
///   [2]  theta    — heading error from +Z axis [rad], normalised to [-pi, pi]
///   [3]  v        — speed [m/s]
///   [4..7]         nearest obstacle 0: [dx, dy, vx, vy]
///   [8..11]        nearest obstacle 1: [dx, dy, vx, vy]  (zero-padded if absent)
///   [12..15]       nearest obstacle 2: [dx, dy, vx, vy]  (zero-padded if absent)
///   [16] goal_dx  — distance to goal along Z [m]
///   [17] cbf_h    — barrier value h = dist^2 - D^2 of nearest obstacle (logging)
///
/// BACKWARD COMPATIBILITY:
///   If scenarioManager is null, the legacy incursionAgent/incursionController
///   single-agent path is used. Obstacle slots [4..15] are filled: slot 0 with the
///   single obstacle, slots 1-2 zero-padded. The Inspector BehaviorParameters must
///   be set to 18 continuous observations regardless.
///
/// COORDINATE MAPPING:
///   Python X (forward) = Unity +Z
///   Python Y (lateral)  = Unity +X
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class TaxiAgent : Unity.MLAgents.Agent
{
    // ── Aircraft parameters ────────────────────────────────────────────────────

    [Header("Aircraft parameters — must match Python DT and L")]
    public float wheelbase    = 6.0f;
    public float maxAccel     = 1.5f;
    public float maxBrake     = 4.0f;
    public float maxSteer     = 0.5f;
    public float desiredSpeed = 8.0f;

    // ── Scene references ───────────────────────────────────────────────────────

    [Header("Scene references")]
    public Transform goalMarker;
    public float taxiwayHalfWidth = 10f;
    public float dSafe            = 6.0f;

    [Header("Spawn randomisation (set ranges to 0 to disable)")]
    public float spawnLateralRange = 0.0f;
    public float spawnHeadingRange = 0.0f;

    // ── Multi-agent scenario (new path) ───────────────────────────────────────

    [Header("Multi-agent scenario (assign ScenarioManager for multi-obstacle support)")]
    [Tooltip("Assign to enable multi-obstacle observations and difficulty curriculum. " +
             "Leave null to fall back to the single-agent incursionController path.")]
    public TaxiScenarioManager scenarioManager;

    // ── Legacy single-agent fields (kept for backward compatibility) ───────────

    [Header("Legacy single-agent (used only when scenarioManager is null)")]
    public IncursionAgentController incursionController;
    [Tooltip("Transform of the crossing obstacle. Used for observation when scenarioManager is null.")]
    public Transform incursionAgent;
    [Tooltip("Empty GameObject where the incursion path crosses the taxiway centreline.")]
    public Transform conflictPoint;
    public float defaultIncursionDt   = 0.0f;

    public float maxEpisodeSeconds = 60f;

    // ── Constants ─────────────────────────────────────────────────────────────

    const int K_OBS = 3;   // nearest obstacles packed into the observation vector

    // ── Private state ──────────────────────────────────────────────────────────

    Rigidbody _rb;
    float     _speed;
    bool      _collided;
    float     _episodeTime;
    Vector3   _spawnPos;
    int       _episodeIndex;

    // Legacy single-agent velocity estimation
    Vector3 _obsPosPrev;
    Vector3 _obsVel;

    // ── Unity / ML-Agents lifecycle ────────────────────────────────────────────

    public override void Initialize()
    {
        _rb = GetComponent<Rigidbody>();
        _rb.isKinematic = true;
        _rb.constraints = RigidbodyConstraints.FreezeRotationX
                        | RigidbodyConstraints.FreezeRotationZ
                        | RigidbodyConstraints.FreezePositionY;
        _spawnPos = transform.position;
    }

    public override void OnEpisodeBegin()
    {
        _collided    = false;
        _speed       = desiredSpeed;
        _episodeTime = 0f;

        var ep = Academy.Instance.EnvironmentParameters;

        // ── Spawn airplane ─────────────────────────────────────────────────────
        float latOff = ep.GetWithDefault("spawn_lateral", float.NaN);
        if (float.IsNaN(latOff))
            latOff = Random.Range(-spawnLateralRange, spawnLateralRange);
        float hdgOff = Random.Range(-spawnHeadingRange, spawnHeadingRange);

        transform.position    = _spawnPos + new Vector3(latOff, 0f, 0f);
        transform.eulerAngles = new Vector3(0f, hdgOff * Mathf.Rad2Deg, 0f);

        // ── Multi-agent path (TaxiScenarioManager) ────────────────────────────────
        if (scenarioManager != null)
        {
            float difficulty     = ep.GetWithDefault("difficulty",      scenarioManager.defaultDifficulty);
            float incursionDt    = ep.GetWithDefault("incursion_dt",    scenarioManager.defaultIncursionDt);
            float ambulanceSpeed = ep.GetWithDefault("ambulance_speed", scenarioManager.defaultAmbulanceSpeed);

            scenarioManager.ResetEpisode(difficulty, desiredSpeed, transform, incursionDt, ambulanceSpeed);
        }
        // ── Legacy single-agent path ──────────────────────────────────────────
        else if (incursionController != null && conflictPoint != null)
        {
            float incursionDt    = ep.GetWithDefault("incursion_dt",    defaultIncursionDt);
            float ambulanceSpeed = ep.GetWithDefault("ambulance_speed", -1f);

            float egoTtc = Mathf.Max(0f,
                (conflictPoint.position.z - transform.position.z) / desiredSpeed);
            float speed  = ambulanceSpeed > 0f ? ambulanceSpeed : incursionController.crossSpeed;
            Vector3 dir  = incursionController.CrossDirectionNormalized;
            Vector3 start = conflictPoint.position - dir * speed * (egoTtc + incursionDt);
            incursionController.ResetCrossing(start, speed);
        }

        // Legacy velocity estimator seed
        if (incursionAgent != null) { _obsPosPrev = incursionAgent.position; _obsVel = Vector3.zero; }

        _episodeIndex++;
    }

    // ── Observations (18 floats) ───────────────────────────────────────────────

    public override void CollectObservations(VectorSensor sensor)
    {
        float x_ego = transform.position.z;
        float y_ego = transform.position.x;
        float theta = transform.eulerAngles.y * Mathf.Deg2Rad;
        theta = Mathf.Atan2(Mathf.Sin(theta), Mathf.Cos(theta));

        sensor.AddObservation(x_ego);
        sensor.AddObservation(y_ego);
        sensor.AddObservation(theta);
        sensor.AddObservation(_speed);

        // ── Nearest-K obstacle observations ───────────────────────────────────
        List<(float dx, float dy, float vx, float vy, float dist2)> slots
            = new List<(float, float, float, float, float)>();

        if (scenarioManager != null)
        {
            foreach (var a in scenarioManager.ActiveAgents)
            {
                Vector3 pos = a.transform.position;
                float dx    = pos.z - transform.position.z;
                float dy    = pos.x - transform.position.x;
                Vector3 vel = a.Velocity;
                float dist2 = dx * dx + dy * dy;
                slots.Add((dx, dy, vel.z, vel.x, dist2));
            }
        }
        else if (incursionAgent != null)
        {
            // Legacy: update velocity estimate
            Vector3 obsPos = incursionAgent.position;
            _obsVel     = (obsPos - _obsPosPrev) / Time.fixedDeltaTime;
            _obsPosPrev = obsPos;

            float dx    = obsPos.z - transform.position.z;
            float dy    = obsPos.x - transform.position.x;
            float dist2 = dx * dx + dy * dy;
            slots.Add((dx, dy, _obsVel.z, _obsVel.x, dist2));
        }

        // Sort nearest first
        slots.Sort((a, b) => a.dist2.CompareTo(b.dist2));

        float cbf_h_nearest = float.MaxValue;

        for (int i = 0; i < K_OBS; i++)
        {
            if (i < slots.Count)
            {
                var s = slots[i];
                sensor.AddObservation(s.dx);
                sensor.AddObservation(s.dy);
                sensor.AddObservation(s.vx);
                sensor.AddObservation(s.vy);
                if (i == 0)
                    cbf_h_nearest = s.dist2 - dSafe * dSafe;
            }
            else
            {
                // Zero-pad: far away on lateral axis, zero velocity
                sensor.AddObservation(0f);
                sensor.AddObservation(999f);
                sensor.AddObservation(0f);
                sensor.AddObservation(0f);
            }
        }

        float goal_dx = goalMarker != null
            ? goalMarker.position.z - transform.position.z
            : 999f;

        sensor.AddObservation(goal_dx);
        sensor.AddObservation(cbf_h_nearest == float.MaxValue ? 999f : cbf_h_nearest);
    }

    // ── Actions received from Python ───────────────────────────────────────────

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

    // ── Bicycle kinematic model ────────────────────────────────────────────────

    void ApplyBicycleDynamics(float a, float delta)
    {
        a     = Mathf.Clamp(a,     -maxBrake, maxAccel);
        delta = Mathf.Clamp(delta, -maxSteer, maxSteer);

        _speed = Mathf.Max(0f, _speed + a * Time.fixedDeltaTime);

        float dTheta = (_speed / wheelbase) * Mathf.Tan(delta) * Time.fixedDeltaTime;
        transform.Rotate(Vector3.up, dTheta * Mathf.Rad2Deg);
        transform.position += transform.forward * (_speed * Time.fixedDeltaTime);
    }

    // ── Reward ────────────────────────────────────────────────────────────────

    float ComputeReward()
    {
        float y_ego = transform.position.x;
        float theta = transform.eulerAngles.y * Mathf.Deg2Rad;
        theta = Mathf.Atan2(Mathf.Sin(theta), Mathf.Cos(theta));

        float r = -0.01f * y_ego * y_ego
                  -0.04f * theta * theta
                  -0.01f * Mathf.Pow(_speed - desiredSpeed, 2f)
                  + 0.02f * _speed;

        if (Mathf.Abs(y_ego) > taxiwayHalfWidth) r -= 0.5f;
        return r;
    }

    // ── Collision detection ────────────────────────────────────────────────────

    void OnTriggerEnter(Collider other)
    {
        if (other.gameObject == gameObject) return;
        if (other.gameObject.name == "Plane" || other.gameObject.isStatic) return;
        _collided = true;
        Debug.LogWarning($"[TaxiAgent] COLLISION with '{other.gameObject.name}' at t={_episodeTime:F2}s", this);
    }

    // ── Heuristic (keyboard testing without Python) ────────────────────────────

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var ca = actionsOut.ContinuousActions;
        ca[0] = Input.GetAxis("Vertical")   * maxAccel;
        ca[1] = Input.GetAxis("Horizontal") * maxSteer;
    }
}
