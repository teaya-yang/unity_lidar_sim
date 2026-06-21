using System.Collections.Generic;
using UnityEngine;

// Ground vehicle NPC — stochastic movement along the TaxiwayLane graph.
// Implements INpc so TrafficManager can spawn, cull, and despawn it.
//
// Routing modes (set before OnNpcInitialize / Initialize is called):
//   fixedRoute != null  → follows the ordered lane sequence, then continues randomly
//   fixedRoute == null  → picks a random NextLane at every junction
//
// Holds at IsHoldingPosition lanes (e.g. runway crossings) until the ego clears.
public class ErraticVehicle : MonoBehaviour, INpc
{
    static readonly List<ErraticVehicle> s_All = new();

    void OnEnable()  => s_All.Add(this);
    void OnDisable() => s_All.Remove(this);

    // True if any active vehicle is within 'radius' (m) of 'pos' on the horizontal plane.
    // Used by the traffic simulators to avoid spawning a new vehicle on top of an existing
    // one at a shared lane entry point (which otherwise gridlocks the car-following logic).
    public static bool AnyVehicleWithin(Vector3 pos, float radius)
    {
        float r2 = radius * radius;
        foreach (ErraticVehicle v in s_All)
        {
            if (v == null) continue;
            Vector3 d = v.transform.position - pos;
            d.y = 0f;
            if (d.sqrMagnitude < r2) return true;
        }
        return false;
    }

    // True if any active vehicle (other than 'except') is currently on one of 'lanes' AND within
    // 'radius' (m) of 'junctionPoint'. Used for intersection right-of-way: a give-way vehicle
    // holds while a priority lane is occupied near the junction. The radius bounds the conflict
    // so a vehicle far down a long priority lane doesn't block entry indefinitely.
    public static bool AnyVehicleNearOnLanes(List<TaxiwayLane> lanes, Vector3 junctionPoint, float radius, ErraticVehicle except)
    {
        if (lanes == null || lanes.Count == 0) return false;
        float r2 = radius * radius;

        foreach (ErraticVehicle v in s_All)
        {
            if (v == null || v == except || v._currentLane == null) continue;
            if (!lanes.Contains(v._currentLane)) continue;
            Vector3 d = v.transform.position - junctionPoint;
            d.y = 0f;
            if (d.sqrMagnitude <= r2) return true;
        }

        return false;
    }

    // ── INpc ─────────────────────────────────────────────────────────────

    public bool ShouldDespawn => _shouldDespawn;

    public Bounds Bounds
    {
        get
        {
            Renderer[] rs = GetComponentsInChildren<Renderer>();
            if (rs.Length == 0) return new Bounds(Vector3.zero, Vector3.one * 2f);
            Bounds b = rs[0].bounds;
            for (int i = 1; i < rs.Length; i++) b.Encapsulate(rs[i].bounds);
            return new Bounds(Vector3.zero, b.size);
        }
    }

    public void OnNpcInitialize(TaxiwayLane startLane, Transform[] egoVehicles)
    {
        _currentLane = startLane;
        airplane     = egoVehicles != null && egoVehicles.Length > 0 ? egoVehicles[0] : null;
        Initialize();
    }

    // ── Routing mode (readable in Inspector at runtime and by GroundTruthPublisher) ──

    public enum RoutingMode { Wandering, RandomLane, FixedRoute }

    public RoutingMode CurrentRoutingMode =>
        _currentLane == null ? RoutingMode.Wandering :
        fixedRoute   != null ? RoutingMode.FixedRoute :
                               RoutingMode.RandomLane;

    public bool   IsStopped   => _stopped;
    public bool   IsYielding  => _yielding;
    public string CurrentLaneName => _currentLane != null ? _currentLane.name : "none";
    public int    WaypointIndex   => _waypointIndex;

    // ── Inspector ─────────────────────────────────────────────────────────────

    [Header("Movement")]
    public float minSpeed                = 4f;
    public float maxSpeed                = 12f;
    public float rotationSpeed           = 3f;
    public float waypointReachedDistance = 2f;
    public float wanderRadius            = 60f;

    [Header("Micro-stops")]
    [Range(0f, 1f)] public float stopProbabilityPerSecond = 0.04f;
    public float minStopDuration = 1f;
    public float maxStopDuration = 5f;

    [Header("Separation")]
    public float separationRadius   = 6f;
    public float separationStrength = 3f;

    [Header("Car-following")]
    [Tooltip("Gap (m) to the vehicle ahead at which this vehicle begins slowing.")]
    public float followDistance = 12f;
    [Tooltip("Gap (m) to the vehicle ahead at which this vehicle comes to a full stop.")]
    public float stopDistance = 4f;
    [Tooltip("Half-width (m) of the corridor ahead used to decide whether another vehicle " +
             "counts as 'in my lane'. Wider = vehicles queue from farther to the side.")]
    public float laneHalfWidth = 3f;

    [Header("Ego proximity")]
    [Tooltip("The ego aircraft. Used only to hold at IsHoldingPosition lanes (runway crossings) " +
             "until the ego clears. NPCs no longer startle / pull over for the ego.")]
    public Transform airplane;
    [Tooltip("Distance (m) within which the ego counts as 'nearby' for holding-position lanes.")]
    public float reactionRadius = 30f;

    [Header("Intersection yielding")]
    [Tooltip("A vehicle on a priority lane (listed in the current lane's YieldToLanes) within this " +
             "distance of the junction (m) makes this vehicle hold at the junction entry.")]
    public float junctionConflictRadius = 20f;

    // Set by RouteTrafficSimulator before Initialize() — vehicle follows this lane
    // sequence then continues randomly from the last lane's NextLanes.
    [HideInInspector] public TaxiwayLane[] fixedRoute;

    // ── Runtime state ─────────────────────────────────────────────────────────

    float       _speed;
    Vector3     _target;
    float       _stopTimer;
    bool        _stopped;
    bool        _yielding;
    bool        _initialized;
    bool        _shouldDespawn;

    TaxiwayLane _currentLane;
    int         _waypointIndex;
    int         _routeIndex;

    Vector3 _lastPos;
    bool    _hasLastPos;

    float   _lastGap = float.PositiveInfinity; // gap to leader last frame, for gizmo/state
    public bool IsFollowing => _lastGap < followDistance;

    // ── Unity messages ────────────────────────────────────────────────────────

    void Start()
    {
        if (!_initialized) Initialize();
    }

    void Update()
    {
        if (_stopped)
        {
            // While stopped at a holding position, stay stopped as long as ego is near.
            if (_currentLane != null && _currentLane.IsHoldingPosition && IsEgoNearby())
                return;

            _stopTimer -= Time.deltaTime;
            if (_stopTimer <= 0f) ExitStop();
            return;
        }

        // Intersection right-of-way: hold at the junction entry while a priority lane is occupied.
        // Unlike a micro-stop this is re-evaluated every frame, so the vehicle proceeds the instant
        // the conflict clears.
        if (ShouldYieldAtJunction())
        {
            _yielding   = true;
            _hasLastPos = false;   // suppress the jump warning when we resume moving
            return;
        }
        _yielding = false;

        if (_hasLastPos)
        {
            float jump = Vector3.Distance(transform.position, _lastPos);
            if (jump > maxSpeed * Time.deltaTime + 1f)
                Debug.LogWarning($"[ErraticVehicle] '{name}' jumped {jump:F1} m between frames — " +
                                 "another component may also be moving this object.", this);
        }

        MoveTowardTarget();

        if (Vector3.Distance(transform.position, _target) < waypointReachedDistance)
            AdvanceWaypoint();

        _lastPos    = transform.position;
        _hasLastPos = true;

        if (Roll(stopProbabilityPerSecond)) EnterStop();
    }

    // ── Initialization ────────────────────────────────────────────────────────

    public void Initialize()
    {
        _initialized   = true;
        _shouldDespawn = false;
        _speed         = Random.Range(minSpeed, maxSpeed);
        _stopped       = false;
        _yielding      = false;
        _hasLastPos    = false;
        _routeIndex    = 0;
        _waypointIndex = 0;

        if (_currentLane != null)
            EnterLane(_currentLane);
        else
            PickWanderTarget();
    }

    // ── Lane graph traversal ──────────────────────────────────────────────────

    void EnterLane(TaxiwayLane lane)
    {
        _currentLane   = lane;
        _waypointIndex = 0;
        _speed         = Mathf.Min(Random.Range(minSpeed, maxSpeed), lane.SpeedLimit);

        if (lane.Waypoints != null && lane.Waypoints.Length > 0)
            _target = lane.Waypoints[0];
    }

    void AdvanceWaypoint()
    {
        if (_currentLane == null || _currentLane.Waypoints == null)
        {
            PickWanderTarget();
            return;
        }

        _waypointIndex++;

        if (_waypointIndex < _currentLane.Waypoints.Length)
        {
            _target = _currentLane.Waypoints[_waypointIndex];
            return;
        }

        OnLaneComplete();
    }

    void OnLaneComplete()
    {
        // Block at holding positions (runway crossings, give-way) until ego clears.
        if (_currentLane != null && _currentLane.IsHoldingPosition && IsEgoNearby())
        {
            EnterStop(float.MaxValue);
            return;
        }

        TaxiwayLane next = PickNextLane();
        if (next != null)
            EnterLane(next);
        else
            EnterStop(maxStopDuration); // dead-end: pull over
    }

    // Intersection right-of-way (explicit lane-priority rule, ported from AWSIM).
    // Returns true when this vehicle, nearing the end of a give-way lane, must hold because a
    // priority lane (listed in the current lane's YieldToLanes) is occupied near the junction.
    bool ShouldYieldAtJunction()
    {
        if (_currentLane == null || !_currentLane.HasYieldRule) return false;

        var wps = _currentLane.Waypoints;
        if (wps == null || wps.Length == 0) return false;

        // Only yield on the final segment — the approach to the junction stop point.
        if (_waypointIndex < wps.Length - 1) return false;

        Vector3 junction = wps[wps.Length - 1];
        return AnyVehicleNearOnLanes(_currentLane.YieldToLanes, junction, junctionConflictRadius, this);
    }

    TaxiwayLane PickNextLane()
    {
        // Fixed route still has lanes ahead — follow them before going random.
        if (fixedRoute != null && _routeIndex + 1 < fixedRoute.Length)
        {
            _routeIndex++;
            return fixedRoute[_routeIndex];
        }

        // Random traversal: pick any outgoing lane.
        if (_currentLane != null &&
            _currentLane.NextLanes != null &&
            _currentLane.NextLanes.Count > 0)
            return _currentLane.NextLanes[Random.Range(0, _currentLane.NextLanes.Count)];

        return null;
    }

    void PickWanderTarget()
    {
        Vector2 disk = Random.insideUnitCircle * wanderRadius;
        _target = transform.position + new Vector3(disk.x, 0f, disk.y);
    }

    // ── Movement ──────────────────────────────────────────────────────────────

    void MoveTowardTarget()
    {
        Vector3 dir = _target - transform.position;
        dir.y = 0f;
        if (dir.sqrMagnitude < 0.001f) return;
        dir.Normalize();

        // Model's visual front is -Z.
        Quaternion look = Quaternion.LookRotation(-dir);
        transform.rotation = Quaternion.RotateTowards(
            transform.rotation, look, rotationSpeed * 60f * Time.deltaTime);

        Vector3 heading = -transform.forward;
        float alignment = Mathf.Clamp01(Vector3.Dot(heading, dir));

        // Car-following: slow (or stop) for the nearest vehicle ahead in our lane corridor.
        // Travel direction is the desired direction toward the target (dir), so "ahead"
        // means farther along the path — this is what enforces queue order.
        _lastGap = GapToLeader(dir);
        float followFactor = FollowSpeedFactor(_lastGap);

        // Forward motion is scaled by followFactor: at stopDistance it is 0, so the follower
        // physically cannot advance past the leader. Separation is lateral-only, so it never
        // shoves a vehicle forward/backward (the old omnidirectional force caused overtaking).
        Vector3 move = heading * (_speed * alignment * followFactor) + LateralSeparation(dir);
        move.y = 0f;
        transform.position += move * Time.deltaTime;
    }

    // Longitudinal gap (m) to the nearest vehicle ahead within the lane corridor, or
    // +infinity if the path ahead is clear. "Ahead" = positive projection onto travelDir;
    // "in corridor" = lateral offset from the travel axis below laneHalfWidth.
    float GapToLeader(Vector3 travelDir)
    {
        float best = float.PositiveInfinity;
        foreach (ErraticVehicle other in s_All)
        {
            if (other == this) continue;
            Vector3 toOther = other.transform.position - transform.position;
            toOther.y = 0f;

            float ahead = Vector3.Dot(toOther, travelDir);   // longitudinal distance
            if (ahead <= 0f) continue;                       // behind us — not a leader

            float lateral = (toOther - travelDir * ahead).magnitude;
            if (lateral > laneHalfWidth) continue;           // beside us, not in our lane

            if (ahead < best) best = ahead;
        }
        return best;
    }

    // Linear speed ramp from car-following: full speed beyond followDistance, zero at or
    // below stopDistance, linearly interpolated in between (a simplified IDM gap policy).
    float FollowSpeedFactor(float gap)
    {
        if (gap >= followDistance) return 1f;
        if (gap <= stopDistance)   return 0f;
        return (gap - stopDistance) / Mathf.Max(followDistance - stopDistance, 0.001f);
    }

    // Lane-aware separation: keeps only the component perpendicular to the travel axis, so
    // crowded vehicles nudge sideways without pushing each other along the lane (which would
    // let a follower slip past its leader). Returns a velocity offset in world space.
    Vector3 LateralSeparation(Vector3 travelDir)
    {
        Vector3 offset = Vector3.zero;
        foreach (ErraticVehicle other in s_All)
        {
            if (other == this) continue;
            Vector3 diff = transform.position - other.transform.position;
            diff.y = 0f;
            float dist = diff.magnitude;
            if (dist >= separationRadius || dist <= 0.001f) continue;

            // Project out the longitudinal part, keep the lateral (perpendicular) push.
            Vector3 lateral = diff - travelDir * Vector3.Dot(diff, travelDir);
            if (lateral.sqrMagnitude < 0.0001f) continue;    // directly in line — no sideways push
            offset += lateral.normalized * ((separationRadius - dist) / separationRadius);
        }
        return offset * separationStrength;
    }

    // ── Ego proximity (holding positions only) ──────────────────────────────────

    bool IsEgoNearby() =>
        airplane != null && Vector3.Distance(transform.position, airplane.position) < reactionRadius;

    // ── Micro-stops ───────────────────────────────────────────────────────────

    void EnterStop(float duration = -1f)
    {
        _stopped   = true;
        _stopTimer = duration > 0f ? duration : Random.Range(minStopDuration, maxStopDuration);
    }

    void ExitStop()
    {
        _stopped = false;
        _speed   = Random.Range(minSpeed, maxSpeed);

        // If we were holding at the end of a lane, proceed to the next.
        if (_currentLane != null && _waypointIndex >= _currentLane.Waypoints.Length)
            OnLaneComplete();
        else
            AdvanceWaypoint();
    }

    bool Roll(float probabilityPerSecond) =>
        Random.value < probabilityPerSecond * Time.deltaTime;

    // ── Gizmos ────────────────────────────────────────────────────────────────

    void OnDrawGizmosSelected()
    {
        // Reaction radius
        if (airplane != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(transform.position, reactionRadius);
        }

        if (!Application.isPlaying) return;

        // Current target — red line from vehicle to next waypoint
        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position, _target);
        Gizmos.DrawSphere(_target, 0.4f);

        // Draw remaining waypoints in the current lane
        if (_currentLane != null && _currentLane.Waypoints != null)
        {
            Gizmos.color = CurrentRoutingMode == RoutingMode.FixedRoute
                ? Color.cyan : Color.yellow;

            for (int i = _waypointIndex; i < _currentLane.Waypoints.Length - 1; i++)
                Gizmos.DrawLine(_currentLane.Waypoints[i], _currentLane.Waypoints[i + 1]);
        }

        // Routing mode label above vehicle
#if UNITY_EDITOR
        UnityEditor.Handles.Label(
            transform.position + Vector3.up * 2f,
            $"{CurrentRoutingMode}  lane:{CurrentLaneName}  wp:{_waypointIndex}" +
            (_stopped ? "  [STOPPED]" : "") +
            (_yielding ? "  [YIELDING]" : "") +
            (IsFollowing ? $"  [FOLLOWING gap:{_lastGap:F1}m]" : ""),
            new GUIStyle { normal = { textColor = Color.white }, fontSize = 11 });
#endif
    }
}
