using UnityEngine;
using UnityEngine.AI;

// NavMesh-based NPC pedestrian / ground crew.
// Mirrors AWSIM's NPC pattern: follows a lane's waypoints and pauses randomly.
// Implements INpc so TrafficManager can spawn and wire it.
[RequireComponent(typeof(NavMeshAgent))]
public class Agent : MonoBehaviour, INpc
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
    // egoVehicles is part of the INpc contract but unused — agents no longer react to the ego.
    public void OnNpcInitialize(TaxiwayLane startLane, Transform[] egoVehicles)
    {
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

    enum State { Patrolling, Paused }
    State _state = State.Patrolling;

    void Start()
    {
        _nav = GetComponent<NavMeshAgent>();
        _nav.speed = Random.Range(minSpeed, maxSpeed);

        if (HasPatrolPath)
            EnterPatrol();
        else
            Debug.LogWarning($"[Agent] '{name}' has no patrol lane — assign one via " +
                             "NavMeshTrafficSimulatorConfig.patrolLane or it will stand still.", this);
    }

    void Update()
    {
        switch (_state)
        {
            case State.Patrolling: UpdatePatrolling(); break;
            case State.Paused:     UpdatePaused();     break;
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
