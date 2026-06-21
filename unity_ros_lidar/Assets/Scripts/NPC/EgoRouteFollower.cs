using UnityEngine;

// Drives the ego airplane along a fixed sequence of TaxiwayLanes.
// Attach to the ego GameObject. Assign a route in the Inspector and press Play.
//
// The ego is treated as an Vehicle peer for the purpose of intersection
// right-of-way: NPCs on give-way lanes will detect the ego on priority lanes
// and yield accordingly.
//
// Controls:
//   Space (hold)  — pause / resume
//   R             — restart from lane 0
[RequireComponent(typeof(Rigidbody))]
public class EgoRouteFollower : MonoBehaviour
{
    [Header("Route")]
    [Tooltip("Ordered sequence of TaxiwayLanes the ego follows. Loops when complete.")]
    public TaxiwayLane[] route;

    [Tooltip("If true, restarts from lane 0 after reaching the end of the route.")]
    public bool loop = true;

    [Header("Movement")]
    public float speed = 6f;
    [Tooltip("Degrees per second the model can turn. Keep high enough to track sharp corners.")]
    public float rotationSpeed = 90f;
    public float waypointReachedDistance = 2f;

    [Tooltip("Tick if the model's visual front points down -Z (like the NPC cars). " +
             "Leave unticked for models whose front is +Z (default for the airplane).")]
    public bool frontIsNegativeZ = false;

    [Header("Intersection yielding")]
    [Tooltip("When the ego is on a give-way lane (one with YieldToLanes set), it holds at the " +
             "junction while a priority lane is occupied within this radius (m).")]
    public float junctionConflictRadius = 20f;

    [Header("Debug")]
    public bool showGizmos = true;

    // ── Runtime ───────────────────────────────────────────────────────────────

    TaxiwayLane _currentLane;
    int         _laneIndex;
    int         _waypointIndex;
    Vector3     _target;
    bool        _paused;
    bool        _finished;
    bool        _yielding;

    // Static registry so Vehicle.AnyVehicleNearOnLanes can also check the ego.
    public static EgoRouteFollower Instance { get; private set; }

    Rigidbody _rb;

    void Awake()
    {
        Instance = this;
        _rb = GetComponent<Rigidbody>();
        _rb.isKinematic = true;   // we drive position directly; no physics integration needed
    }

    void OnDestroy() { if (Instance == this) Instance = null; }

    void Start()
    {
        if (route == null || route.Length == 0)
        {
            Debug.LogWarning("[EgoRouteFollower] No route assigned — ego will not move.", this);
            enabled = false;
            return;
        }

        Restart();
    }

    void Update()
    {
        HandleInput();
        if (_paused || _finished) return;

        // Intersection right-of-way: if on a give-way lane near the junction, hold while a
        // priority lane is occupied. Re-evaluated every frame so we proceed the instant it clears.
        if (ShouldYieldAtJunction())
        {
            _yielding = true;
            return;
        }
        _yielding = false;

        MoveTowardTarget();

        if (Vector3.Distance(transform.position, _target) < waypointReachedDistance)
            AdvanceWaypoint();
    }

    // ── Public API ────────────────────────────────────────────────────────────

    public bool  IsPaused   => _paused;
    public bool  IsFinished => _finished;
    public bool  IsYielding => _yielding;
    public int   LaneIndex  => _laneIndex;
    public int   WaypointIndex => _waypointIndex;
    public string CurrentLaneName => _currentLane != null ? _currentLane.name : "none";

    public void Restart()
    {
        _laneIndex     = 0;
        _waypointIndex = 0;
        _paused        = false;
        _finished      = false;
        EnterLane(route[0]);
        transform.position = route[0].Waypoints[0];
    }

    // ── Movement ──────────────────────────────────────────────────────────────

    void MoveTowardTarget()
    {
        Vector3 dir = _target - transform.position;
        dir.y = 0f;
        if (dir.sqrMagnitude < 0.001f) return;
        dir.Normalize();

        // The model's forward axis depends on how the mesh was authored. The airplane's visual
        // front is +Z; the NPC cars are -Z. frontIsNegativeZ selects which.
        Vector3    faceDir = frontIsNegativeZ ? -dir : dir;
        Quaternion look    = Quaternion.LookRotation(faceDir);
        transform.rotation = Quaternion.RotateTowards(
            transform.rotation, look, rotationSpeed * Time.deltaTime);

        // Advance only when roughly aligned so the plane doesn't crab sideways.
        Vector3 heading   = frontIsNegativeZ ? -transform.forward : transform.forward;
        float   alignment = Mathf.Clamp01(Vector3.Dot(heading, dir));

        float effectiveSpeed = Mathf.Min(speed, _currentLane != null ? _currentLane.SpeedLimit : speed);
        transform.position += heading * (effectiveSpeed * alignment * Time.deltaTime);
    }

    // Intersection right-of-way for the ego: hold near the end of a give-way lane while a
    // priority lane (the current lane's YieldToLanes) is occupied by an NPC near the junction.
    bool ShouldYieldAtJunction()
    {
        if (_currentLane == null || !_currentLane.HasYieldRule) return false;

        Vector3[] wps = _currentLane.Waypoints;
        if (wps == null || wps.Length == 0) return false;

        // Only yield on the final approach segment to the junction stop point.
        if (_waypointIndex < wps.Length - 1) return false;

        Vector3 junction = wps[^1];
        return Vehicle.AnyVehicleNearOnLanes(
            _currentLane.YieldToLanes, junction, junctionConflictRadius, null);
    }

    void AdvanceWaypoint()
    {
        _waypointIndex++;

        if (_currentLane != null && _waypointIndex < _currentLane.Waypoints.Length)
        {
            _target = _currentLane.Waypoints[_waypointIndex];
            return;
        }

        // Reached end of current lane — move to next.
        _laneIndex++;
        if (_laneIndex < route.Length)
        {
            EnterLane(route[_laneIndex]);
            return;
        }

        if (loop)
        {
            Restart();
        }
        else
        {
            _finished = true;
            Debug.Log("[EgoRouteFollower] Route complete.", this);
        }
    }

    void EnterLane(TaxiwayLane lane)
    {
        _currentLane   = lane;
        _waypointIndex = 0;

        if (lane.Waypoints != null && lane.Waypoints.Length > 0)
            _target = lane.Waypoints[0];
    }

    // ── Input ─────────────────────────────────────────────────────────────────

    void HandleInput()
    {
        if (Input.GetKeyDown(KeyCode.Space)) _paused = !_paused;
        if (Input.GetKeyDown(KeyCode.R))     Restart();
    }

    // ── Gizmos ────────────────────────────────────────────────────────────────

    void OnDrawGizmos()
    {
        if (!showGizmos || route == null) return;

        // Draw the full planned route in blue.
        Gizmos.color = new Color(0.3f, 0.6f, 1f, 0.7f);
        foreach (var lane in route)
        {
            if (lane == null || lane.Waypoints == null) continue;
            for (int i = 0; i < lane.Waypoints.Length - 1; i++)
                Gizmos.DrawLine(lane.Waypoints[i], lane.Waypoints[i + 1]);
        }
    }

    void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying || !showGizmos) return;

        // Line to current waypoint target.
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position, _target);
        Gizmos.DrawSphere(_target, 0.5f);

#if UNITY_EDITOR
        UnityEditor.Handles.Label(
            transform.position + Vector3.up * 3f,
            $"EGO  lane:{CurrentLaneName} [{_laneIndex}]  wp:{_waypointIndex}" +
            (_yielding ? "  [YIELDING]" : "") +
            (_paused   ? "  [PAUSED]"   : "") +
            (_finished ? "  [DONE]"     : ""),
            new GUIStyle { normal = { textColor = Color.cyan }, fontSize = 12, fontStyle = FontStyle.Bold });
#endif
    }
}
