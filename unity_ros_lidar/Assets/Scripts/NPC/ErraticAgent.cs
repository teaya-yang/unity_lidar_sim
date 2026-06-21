using UnityEngine;
using UnityEngine.AI;

// NavMesh-based NPC pedestrian / ground crew.
// Mirrors AWSIM's NPC pattern: follows a lane's waypoints, pauses randomly,
// and reacts to the ego aircraft.
// Implements INpc so TrafficManager can spawn and wire it.
[RequireComponent(typeof(NavMeshAgent))]
public class ErraticAgent : MonoBehaviour, INpc
{
    public enum AgentType { Pedestrian, Animal }

    [Header("Agent Type")]
    public AgentType agentType = AgentType.Pedestrian;

    [Header("Movement")]
    public float minSpeed = 0.5f;
    public float maxSpeed = 3.5f;
    public float waypointReachedDistance = 1f;
    public bool  loopPatrol = true;

    [Header("Micro-stops")]
    [Range(0f, 1f)] public float pauseProbabilityPerSecond = 0.06f;
    public float minPauseDuration = 0.5f;
    public float maxPauseDuration = 3f;

    [Header("Ego Vehicle Reaction")]
    public Transform[] egoVehicles;
    public float startleRadius = 12f;
    [Tooltip("Positive = flee, Negative = approach (curious)")]
    public float reactionBias = 1f;
    public float reactionSpeedMultiplier = 2f;

    public bool ShouldDespawn => false;

    public Bounds Bounds
    {
        get
        {
            Renderer[] rs = GetComponentsInChildren<Renderer>();
            if (rs.Length == 0) return new Bounds(transform.position, Vector3.one);
            Bounds b = rs[0].bounds;
            for (int i = 1; i < rs.Length; i++) b.Encapsulate(rs[i].bounds);
            return b;
        }
    }

    // Called by NavMeshTrafficSimulator after spawn.
    // startLane: patrol lane assigned by NavMeshTrafficSimulatorConfig.patrolLane.
    public void OnNpcInitialize(TaxiwayLane startLane, Transform[] egoVehicles)
    {
        this.egoVehicles = egoVehicles;
        if (startLane != null && startLane.Waypoints != null && startLane.Waypoints.Length > 0)
            _laneWaypoints = startLane.Waypoints;
    }

    // True when a patrol path is available — used by GroundTruthPublisher.
    public bool HasPatrolPath => _laneWaypoints != null && _laneWaypoints.Length > 0;

    NavMeshAgent _nav;
    Vector3[]    _laneWaypoints;
    int          _waypointIndex;
    int          _patrolDir = 1;
    float        _pauseTimer;

    enum State { Patrolling, Paused, Reacting }
    State _state = State.Patrolling;

    void Start()
    {
        _nav = GetComponent<NavMeshAgent>();
        _nav.speed = Random.Range(minSpeed, maxSpeed);

        if (HasPatrolPath)
            EnterPatrol();
        else
            Debug.LogWarning($"[ErraticAgent] '{name}' has no patrol lane — assign one via " +
                             "NavMeshTrafficSimulatorConfig.patrolLane or it will stand still.", this);
    }

    void Update()
    {
        CheckEgoReaction();

        switch (_state)
        {
            case State.Patrolling: UpdatePatrolling(); break;
            case State.Paused:     UpdatePaused();     break;
            case State.Reacting:   UpdateReacting();   break;
        }
    }

    void EnterPatrol()
    {
        _state = State.Patrolling;
        _nav.isStopped = false;
        _nav.speed     = Random.Range(minSpeed, maxSpeed);
        SetDestinationToWaypoint();
    }

    void UpdatePatrolling()
    {
        if (!_nav.pathPending && _nav.remainingDistance < waypointReachedDistance)
            AdvanceWaypoint();

        if (Roll(pauseProbabilityPerSecond)) EnterPause();
    }

    void AdvanceWaypoint()
    {
        if (!HasPatrolPath) return;

        if (loopPatrol)
        {
            _waypointIndex = (_waypointIndex + 1) % _laneWaypoints.Length;
        }
        else
        {
            _waypointIndex += _patrolDir;
            if (_waypointIndex >= _laneWaypoints.Length - 1 || _waypointIndex <= 0)
                _patrolDir *= -1;
        }

        SetDestinationToWaypoint();
    }

    void SetDestinationToWaypoint()
    {
        if (!HasPatrolPath || _waypointIndex >= _laneWaypoints.Length) return;
        TrySetDestination(_laneWaypoints[_waypointIndex], 5f);
    }

    void EnterPause()
    {
        if (_state == State.Reacting) return;
        _state           = State.Paused;
        _nav.isStopped   = true;
        _pauseTimer      = Random.Range(minPauseDuration, maxPauseDuration);
    }

    void UpdatePaused()
    {
        _pauseTimer -= Time.deltaTime;
        if (_pauseTimer > 0f) return;
        EnterPatrol();
    }

    void CheckEgoReaction()
    {
        Transform closest = ClosestEgoInRange();
        if (closest != null && _state != State.Reacting) EnterReaction(closest);
        if (closest == null && _state == State.Reacting) ExitReaction();
    }

    void EnterReaction(Transform ego)
    {
        _state     = State.Reacting;
        _nav.isStopped = false;
        _nav.speed = Random.Range(minSpeed, maxSpeed) * reactionSpeedMultiplier;
        SetReactionTarget(ego);
    }

    void UpdateReacting()
    {
        Transform closest = ClosestEgoInRange();
        if (closest == null) { ExitReaction(); return; }

        if (!_nav.pathPending && _nav.remainingDistance < waypointReachedDistance)
            SetReactionTarget(closest);
    }

    void SetReactionTarget(Transform ego)
    {
        if (ego == null) return;
        Vector3 dir       = (transform.position - ego.position).normalized * Mathf.Sign(reactionBias);
        Vector3 candidate = transform.position + dir * startleRadius * 1.5f;
        TrySetDestination(candidate, startleRadius * 2f);
    }

    void ExitReaction()
    {
        _nav.speed = Random.Range(minSpeed, maxSpeed);
        EnterPatrol();
    }

    Transform ClosestEgoInRange()
    {
        if (egoVehicles == null) return null;
        Transform best   = null;
        float     bestDist = startleRadius;
        foreach (Transform t in egoVehicles)
        {
            if (t == null) continue;
            float d = Vector3.Distance(transform.position, t.position);
            if (d < bestDist) { bestDist = d; best = t; }
        }
        return best;
    }

    bool TrySetDestination(Vector3 candidate, float searchRadius)
    {
        if (!_nav.isOnNavMesh) return false;
        NavMeshHit hit;
        if (!NavMesh.SamplePosition(candidate, out hit, searchRadius, NavMesh.AllAreas))
            return false;
        _nav.SetDestination(hit.position);
        return true;
    }

    bool Roll(float probabilityPerSecond) =>
        Random.value < probabilityPerSecond * Time.deltaTime;

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, startleRadius);

        if (_laneWaypoints == null) return;
        Gizmos.color = Color.yellow;
        for (int i = 0; i < _laneWaypoints.Length - 1; i++)
            Gizmos.DrawLine(_laneWaypoints[i], _laneWaypoints[i + 1]);

        if (Application.isPlaying && _waypointIndex < _laneWaypoints.Length)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(transform.position, _laneWaypoints[_waypointIndex]);
        }
    }
}
