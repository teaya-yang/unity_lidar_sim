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

    // ── public API ─────────────────────────────────────────────────────────────

    public Vector3 CrossDirectionNormalized =>
        crossDirection.sqrMagnitude > 1e-6f ? crossDirection.normalized : Vector3.right;

    public Vector3 Velocity => (_moving && !_stopped)
        ? _dir * (_speed + _erraticSpeedOffset)
        : Vector3.zero;

    public void ResetCrossing(Vector3 startPos, float speed = -1f)
    {
        transform.position  = startPos;
        _dir                = CrossDirectionNormalized;
        _topSpeed           = speed > 0f ? speed : crossSpeed;
        _speed              = trajectoryMode == TrajectoryMode.Accelerating
                                ? _topSpeed * accelStartFraction
                                : _topSpeed;
        _moving             = true;
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
            case TrajectoryMode.Straight:     StepStraight();     break;
            case TrajectoryMode.Curved:       StepCurved();       break;
            case TrajectoryMode.StopGo:       StepStopGo();       break;
            case TrajectoryMode.Erratic:      StepErratic();      break;
            case TrajectoryMode.Stationary:                       break; // intentionally idle
            case TrajectoryMode.Accelerating: StepAccelerating(); break;
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

    // ── Gizmo ─────────────────────────────────────────────────────────────────

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.magenta;
        Vector3 d = CrossDirectionNormalized;
        Gizmos.DrawLine(transform.position - d * 20f, transform.position + d * 20f);
        Gizmos.DrawWireSphere(transform.position, 1f);
    }
}
