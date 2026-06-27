using UnityEngine;

/// <summary>
/// Trajectory mode for this incursion agent.
/// </summary>
public enum TrajectoryMode
{
    Straight,    // constant-velocity straight crossing
    Curved,      // constant-speed arc
    StopGo,      // stop near taxiway edge, wait, resume
    Erratic,     // random heading/speed perturbations
    Stationary,  // placed on taxiway and never moves (parked vehicle / FOD)
    Accelerating,// starts slow, accelerates toward conflict point
    FollowPath,  // drive along an assigned TaxiwayPath (GeoJSON map-based scenario)
}

/// <summary>
/// Deterministic incursion crosser supporting six trajectory modes.
/// All modes share the same public API: ResetCrossing / StopCrossing / Velocity.
/// </summary>
public class IncursionAgentController : MonoBehaviour
{
    [Header("Crossing motion")]
    public Vector3 crossDirection = Vector3.right;
    public float   crossSpeed     = 5.0f;
    public bool    faceTravelDirection = true;
    public bool    frontIsNegativeZ    = true;

    [Header("Trajectory mode")]
    public TrajectoryMode trajectoryMode = TrajectoryMode.Straight;

    [Header("Curved")]
    public float curveRadius = 20f;

    [Header("StopGo")]
    public float     stopDistanceFromConflict = 6f;
    public float     stopWaitTime             = 2.0f;
    public Transform conflictPoint;

    [Header("Erratic")]
    public float erraticSpeedJitter      = 0.8f;
    public float erraticHeadingJitter    = 0.12f;
    public float erraticUpdateInterval   = 0.5f;

    [Header("Accelerating")]
    [Tooltip("Speed at spawn (fraction of crossSpeed). Agent accelerates to crossSpeed.")]
    public float accelStartFraction = 0.2f;
    [Tooltip("Acceleration rate [m/s²].")]
    public float accelRate          = 1.0f;

    [Header("FollowPath (GeoJSON map mode)")]
    [Tooltip("Set by TaxiScenarioManager. The agent follows these waypoints in order.")]
    public TaxiwayPath assignedPath;
    [Tooltip("Waypoint-reached radius [m].")]
    public float pathWaypointRadius = 3f;

    // ── private ────────────────────────────────────────────────────────────────

    bool    _moving;
    Vector3 _dir;
    float   _speed;        // current speed (may change in Accelerating/Erratic)
    float   _topSpeed;     // target speed for Accelerating mode

    // StopGo
    bool  _stopped;
    float _stopTimer;

    // Erratic
    float _erraticTimer;
    float _erraticSpeedOffset;

    // FollowPath
    int _pathWpIndex;

    // ── public API ─────────────────────────────────────────────────────────────

    public Vector3 CrossDirectionNormalized =>
        crossDirection.sqrMagnitude > 1e-6f ? crossDirection.normalized : Vector3.right;

    public Vector3 Velocity => (_moving && !_stopped)
        ? _dir * (_speed + _erraticSpeedOffset)
        : Vector3.zero;

    public void ResetCrossing(Vector3 startPos, float speed = -1f)
    {
        transform.position  = startPos;
        _topSpeed           = speed > 0f ? speed : crossSpeed;
        _speed              = trajectoryMode == TrajectoryMode.Accelerating
                                ? _topSpeed * accelStartFraction
                                : _topSpeed;
        _moving             = true;
        _stopped            = false;
        _stopTimer          = 0f;
        _erraticTimer       = 0f;
        _erraticSpeedOffset = 0f;
        _pathWpIndex        = 0;

        if (trajectoryMode == TrajectoryMode.FollowPath && assignedPath != null
            && assignedPath.Waypoints.Count > 0)
        {
            // Start direction is toward the first waypoint ahead of startPos
            _pathWpIndex = FindNearestWaypointAhead(startPos);
            Vector3 toNext = assignedPath.Waypoints[_pathWpIndex] - startPos;
            toNext.y = 0f;
            _dir = toNext.sqrMagnitude > 1e-6f ? toNext.normalized : Vector3.forward;
        }
        else
        {
            _dir = CrossDirectionNormalized;
        }

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
            case TrajectoryMode.Straight:     StepStraight();     break;
            case TrajectoryMode.Curved:       StepCurved();       break;
            case TrajectoryMode.StopGo:       StepStopGo();       break;
            case TrajectoryMode.Erratic:      StepErratic();      break;
            case TrajectoryMode.Stationary:                       break; // intentionally idle
            case TrajectoryMode.Accelerating: StepAccelerating(); break;
            case TrajectoryMode.FollowPath:   StepFollowPath();   break;
        }
    }

    // ── modes ──────────────────────────────────────────────────────────────────

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
            transform.rotation = Quaternion.LookRotation(frontIsNegativeZ ? -_dir : _dir, Vector3.up);
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
            { _stopped = true; _stopTimer = 0f; return; }
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
            transform.rotation = Quaternion.LookRotation(frontIsNegativeZ ? -_dir : _dir, Vector3.up);
    }

    void StepAccelerating()
    {
        _speed = Mathf.Min(_topSpeed, _speed + accelRate * Time.fixedDeltaTime);
        transform.position += _dir * (_speed * Time.fixedDeltaTime);
    }

    void StepFollowPath()
    {
        if (assignedPath == null || assignedPath.Waypoints.Count == 0) return;

        var wps = assignedPath.Waypoints;
        if (_pathWpIndex >= wps.Count) { _moving = false; return; }

        Vector3 target = wps[_pathWpIndex];
        Vector3 toTarget = target - transform.position;
        toTarget.y = 0f;

        if (toTarget.sqrMagnitude < pathWaypointRadius * pathWaypointRadius)
        {
            _pathWpIndex++;
            if (_pathWpIndex >= wps.Count) { _moving = false; return; }
            toTarget = wps[_pathWpIndex] - transform.position;
            toTarget.y = 0f;
        }

        if (toTarget.sqrMagnitude > 1e-6f) _dir = toTarget.normalized;

        transform.position += _dir * (_speed * Time.fixedDeltaTime);

        if (faceTravelDirection && _dir.sqrMagnitude > 1e-6f)
            transform.rotation = Quaternion.LookRotation(frontIsNegativeZ ? -_dir : _dir, Vector3.up);
    }

    // Returns the index of the waypoint nearest to pos that is still ahead
    // (or 0 if the path has just started).
    int FindNearestWaypointAhead(Vector3 pos)
    {
        if (assignedPath == null || assignedPath.Waypoints.Count == 0) return 0;
        float best = float.MaxValue;
        int   idx  = 0;
        var   wps  = assignedPath.Waypoints;
        for (int i = 0; i < wps.Count; i++)
        {
            float d = Vector3.Distance(pos, wps[i]);
            if (d < best) { best = d; idx = i; }
        }
        return Mathf.Min(idx + 1, wps.Count - 1);
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
