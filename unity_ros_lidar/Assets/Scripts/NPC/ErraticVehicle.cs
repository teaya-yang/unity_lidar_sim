using System.Collections.Generic;
using UnityEngine;

// Ground vehicle NPC — stochastic movement along the TaxiwayLane graph.
// Implements INpc so TrafficManager can spawn, cull, and despawn it.
//
// Routing modes (set before OnNpcInitialize / Initialize is called):
//   fixedRoute != null  → follows the ordered lane sequence, then continues randomly
//   fixedRoute == null  → picks a random NextLane at every junction
//
// Reacts to the ego aircraft: pulls over or rushes depending on pullOverOnReaction.
public class ErraticVehicle : MonoBehaviour, INpc
{
    static readonly List<ErraticVehicle> s_All = new();

    void OnEnable()  => s_All.Add(this);
    void OnDisable() => s_All.Remove(this);

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

    public bool   IsReacting  => _reacting;
    public bool   IsStopped   => _stopped;
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

    [Header("Ego reaction")]
    public Transform airplane;
    public float reactionRadius = 30f;
    [Tooltip("True = pull over and stop when ego approaches; False = rush (speed boost).")]
    public bool pullOverOnReaction = true;

    // Set by RouteTrafficSimulator before Initialize() — vehicle follows this lane
    // sequence then continues randomly from the last lane's NextLanes.
    [HideInInspector] public TaxiwayLane[] fixedRoute;

    // ── Runtime state ─────────────────────────────────────────────────────────

    float       _speed;
    Vector3     _target;
    float       _stopTimer;
    bool        _stopped;
    bool        _reacting;
    bool        _initialized;
    bool        _shouldDespawn;

    TaxiwayLane _currentLane;
    int         _waypointIndex;
    int         _routeIndex;

    Vector3 _lastPos;
    bool    _hasLastPos;

    // ── Unity messages ────────────────────────────────────────────────────────

    void Start()
    {
        if (!_initialized) Initialize();
    }

    void Update()
    {
        CheckAirplaneReaction();

        if (_stopped)
        {
            // While stopped at a holding position, stay stopped as long as ego is near.
            if (_currentLane != null && _currentLane.IsHoldingPosition && IsEgoNearby())
                return;

            _stopTimer -= Time.deltaTime;
            if (_stopTimer <= 0f) ExitStop();
            return;
        }

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

        if (!_reacting && Roll(stopProbabilityPerSecond)) EnterStop();
    }

    // ── Initialization ────────────────────────────────────────────────────────

    public void Initialize()
    {
        _initialized   = true;
        _shouldDespawn = false;
        _speed         = Random.Range(minSpeed, maxSpeed);
        _stopped       = false;
        _reacting      = false;
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
        Vector3 move    = heading * (_speed * alignment) + Separation();
        move.y = 0f;
        transform.position += move * Time.deltaTime;
    }

    Vector3 Separation()
    {
        Vector3 offset = Vector3.zero;
        foreach (ErraticVehicle other in s_All)
        {
            if (other == this) continue;
            Vector3 diff = transform.position - other.transform.position;
            diff.y = 0f;
            float dist = diff.magnitude;
            if (dist < separationRadius && dist > 0.001f)
                offset += diff.normalized * ((separationRadius - dist) / separationRadius);
        }
        return offset * separationStrength;
    }

    // ── Ego reaction ──────────────────────────────────────────────────────────

    void CheckAirplaneReaction()
    {
        if (airplane == null) return;
        bool inRange = Vector3.Distance(transform.position, airplane.position) < reactionRadius;

        if (inRange && !_reacting)
        {
            _reacting = true;
            if (pullOverOnReaction)
                EnterStop(maxStopDuration * 2f);
            else
                _speed = maxSpeed * 1.5f;
        }

        if (!inRange && _reacting)
        {
            _reacting = false;
            _speed    = Random.Range(minSpeed, maxSpeed);
        }
    }

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
            (_stopped ? "  [STOPPED]" : "") + (_reacting ? "  [REACTING]" : ""),
            new GUIStyle { normal = { textColor = Color.white }, fontSize = 11 });
#endif
    }
}
