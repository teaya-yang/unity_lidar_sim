using UnityEngine;

/// <summary>
/// Trajectory mode for this incursion agent.
/// Set per-episode by ScenarioManager (or manually in the Inspector for testing).
/// </summary>
public enum TrajectoryMode
{
    /// <summary>Constant-velocity straight crossing (original behaviour).</summary>
    Straight,
    /// <summary>Constant-speed arc; turn rate = speed / curveRadius.</summary>
    Curved,
    /// <summary>Stops near the taxiway edge, waits, then continues.</summary>
    StopGo,
    /// <summary>Random heading and speed perturbations each update interval.</summary>
    Erratic,
}

/// <summary>
/// Deterministic incursion crosser for parameterized scenario generation.
///
/// TaxiAgent.OnEpisodeBegin (or ScenarioManager.ResetEpisode) calls ResetCrossing()
/// to teleport the agent to its start position and set a TrajectoryMode for the
/// episode. The agent then moves autonomously in FixedUpdate.
///
/// All four modes share the same public API: ResetCrossing / StopCrossing / Velocity.
/// Disable any Vehicle / AmbulanceTrajectorySubscriber on the same GameObject so
/// they don't fight this controller for the Transform.
/// </summary>
public class IncursionAgentController : MonoBehaviour
{
    // ── Shared settings ────────────────────────────────────────────────────────

    [Header("Crossing motion")]
    [Tooltip("World-space direction the agent travels across the taxiway. " +
             "Will be normalised. Typically perpendicular to the taxiway (±X).")]
    public Vector3 crossDirection = Vector3.right;

    [Tooltip("Crossing speed [m/s]. Overridden per-episode by ScenarioManager/TaxiAgent.")]
    public float crossSpeed = 5.0f;

    [Tooltip("If true, rotate to face the travel direction each reset.")]
    public bool faceTravelDirection = true;

    [Tooltip("Tick if the model's visual front points down -Z (like the ambulance/NPC cars). " +
             "Untick for models whose front is +Z.")]
    public bool frontIsNegativeZ = true;

    // ── Trajectory mode ────────────────────────────────────────────────────────

    [Header("Trajectory mode (set by ScenarioManager each episode)")]
    public TrajectoryMode trajectoryMode = TrajectoryMode.Straight;

    // ── Curved mode ────────────────────────────────────────────────────────────

    [Header("Curved mode")]
    [Tooltip("Arc radius [m]. Smaller = tighter curve. Agent turns left (CCW from above).")]
    public float curveRadius = 20f;

    // ── StopGo mode ────────────────────────────────────────────────────────────

    [Header("StopGo mode")]
    [Tooltip("Distance to conflictPoint (along crossing direction) at which the agent stops.")]
    public float stopDistanceFromConflict = 6f;

    [Tooltip("How long the agent waits before resuming [s].")]
    public float stopWaitTime = 2.0f;

    [Tooltip("Conflict point transform. Required for StopGo distance check. " +
             "Assign the same ConflictPoint used by ScenarioManager/TaxiAgent.")]
    public Transform conflictPoint;

    // ── Erratic mode ───────────────────────────────────────────────────────────

    [Header("Erratic mode")]
    [Tooltip("Speed perturbation +/- [m/s] applied each update interval.")]
    public float erraticSpeedJitter = 0.8f;

    [Tooltip("Heading perturbation +/- [rad] applied each update interval.")]
    public float erraticHeadingJitter = 0.12f;

    [Tooltip("How often the erratic perturbation refreshes [s].")]
    public float erraticUpdateInterval = 0.5f;

    // ── Private state ──────────────────────────────────────────────────────────

    bool    _moving;
    Vector3 _dir;          // current travel direction (unit vector; may rotate in Curved/Erratic)
    float   _speed;        // base crossing speed

    // StopGo
    bool  _stopped;
    float _stopTimer;

    // Erratic
    float _erraticTimer;
    float _erraticSpeedOffset;

    // ── Public API ─────────────────────────────────────────────────────────────

    public Vector3 CrossDirectionNormalized =>
        crossDirection.sqrMagnitude > 1e-6f ? crossDirection.normalized : Vector3.right;

    /// <summary>Current world velocity. Zero when stopped or idle.</summary>
    public Vector3 Velocity => (_moving && !_stopped) ? _dir * (_speed + _erraticSpeedOffset) : Vector3.zero;

    /// <summary>
    /// Teleport to startPos and begin crossing. Called by ScenarioManager or TaxiAgent.
    /// </summary>
    public void ResetCrossing(Vector3 startPos, float speed = -1f)
    {
        transform.position = startPos;
        _dir    = CrossDirectionNormalized;
        _speed  = speed > 0f ? speed : crossSpeed;
        _moving = true;

        _stopped            = false;
        _stopTimer          = 0f;
        _erraticTimer       = 0f;
        _erraticSpeedOffset = 0f;

        if (faceTravelDirection && _dir.sqrMagnitude > 1e-6f)
        {
            Vector3 faceDir = frontIsNegativeZ ? -_dir : _dir;
            transform.rotation = Quaternion.LookRotation(faceDir, Vector3.up);
        }
    }

    public void StopCrossing() => _moving = false;

    // ── FixedUpdate ────────────────────────────────────────────────────────────

    void FixedUpdate()
    {
        if (!_moving) return;
        switch (trajectoryMode)
        {
            case TrajectoryMode.Straight: StepStraight(); break;
            case TrajectoryMode.Curved:   StepCurved();   break;
            case TrajectoryMode.StopGo:   StepStopGo();   break;
            case TrajectoryMode.Erratic:  StepErratic();  break;
        }
    }

    // ── Trajectory implementations ─────────────────────────────────────────────

    void StepStraight()
    {
        transform.position += _dir * (_speed * Time.fixedDeltaTime);
    }

    void StepCurved()
    {
        if (curveRadius > 0.1f)
        {
            float omega = _speed / curveRadius * Time.fixedDeltaTime;
            _dir = Quaternion.AngleAxis(omega * Mathf.Rad2Deg, Vector3.up) * _dir;
        }
        transform.position += _dir * (_speed * Time.fixedDeltaTime);
        if (faceTravelDirection && _dir.sqrMagnitude > 1e-6f)
        {
            Vector3 faceDir = frontIsNegativeZ ? -_dir : _dir;
            transform.rotation = Quaternion.LookRotation(faceDir, Vector3.up);
        }
    }

    void StepStopGo()
    {
        if (_stopped)
        {
            _stopTimer += Time.fixedDeltaTime;
            if (_stopTimer >= stopWaitTime) { _stopped = false; _stopTimer = 0f; }
            return;
        }
        if (conflictPoint != null)
        {
            float along = Vector3.Dot(conflictPoint.position - transform.position, _dir);
            if (along > 0f && along < stopDistanceFromConflict)
            {
                _stopped = true; _stopTimer = 0f; return;
            }
        }
        transform.position += _dir * (_speed * Time.fixedDeltaTime);
    }

    void StepErratic()
    {
        _erraticTimer += Time.fixedDeltaTime;
        if (_erraticTimer >= erraticUpdateInterval)
        {
            _erraticTimer       = 0f;
            _erraticSpeedOffset = Random.Range(-erraticSpeedJitter, erraticSpeedJitter);
            float hdg           = Random.Range(-erraticHeadingJitter, erraticHeadingJitter);
            _dir = Quaternion.AngleAxis(hdg * Mathf.Rad2Deg, Vector3.up) * _dir;
        }
        float eff = Mathf.Max(0f, _speed + _erraticSpeedOffset);
        transform.position += _dir * (eff * Time.fixedDeltaTime);
        if (faceTravelDirection && _dir.sqrMagnitude > 1e-6f)
        {
            Vector3 faceDir = frontIsNegativeZ ? -_dir : _dir;
            transform.rotation = Quaternion.LookRotation(faceDir, Vector3.up);
        }
    }

    // ── Gizmo ─────────────────────────────────────────────────────────────────

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.magenta;
        Vector3 d = CrossDirectionNormalized;
        Gizmos.DrawLine(transform.position - d * 20f, transform.position + d * 20f);
        Gizmos.DrawWireSphere(transform.position, 1f);
    }
}
