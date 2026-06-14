using UnityEngine;
using UnityEngine.AI;

// Four-state pedestrian/animal AI: Patrolling (follows a street path), Wandering (fully
// erratic), Paused, Crossing, and Reacting (to ego vehicle).
// Emergency vehicles (ambulance) trigger an immediate switch from Patrol → Wandering.
// After erraticDuration seconds the agent returns to its patrol path automatically.
[RequireComponent(typeof(NavMeshAgent))]
public class ErraticAgent : MonoBehaviour
{
    public enum AgentType { Pedestrian, Animal }

    [Header("Agent Type")]
    public AgentType agentType = AgentType.Pedestrian;

    [Header("Wandering")]
    public float wanderRadius = 20f;
    public float waypointReachedDistance = 1f;

    [Header("Speed")]
    public float minSpeed = 0.5f;
    public float maxSpeed = 3.5f;

    [Header("Micro-stops")]
    [Range(0f, 1f)] public float pauseProbabilityPerSecond = 0.08f;
    public float minPauseDuration = 0.5f;
    public float maxPauseDuration = 4f;

    [Header("Waypoint Jitter")]
    [Range(0f, 1f)] public float jitterProbabilityPerSecond = 0.12f;
    public float jitterRadius = 4f;

    [Header("Direction Reversal")]
    [Range(0f, 1f)] public float reversalProbabilityPerSecond = 0.04f;

    // ── Patrol path ───────────────────────────────────────────────────────────
    [Header("Patrol Path")]
    [Tooltip("Assign street waypoints to make this agent walk a predictable route. Leave empty for fully erratic.")]
    public Transform[] patrolWaypoints;
    public bool loopPatrol = true;

    // ── Emergency switch ──────────────────────────────────────────────────────
    [Header("Emergency Switch")]
    [Tooltip("Vehicles whose proximity triggers panic (e.g. ambulance). Separate from ego vehicle.")]
    public Transform[] emergencyVehicles;
    public float emergencyRadius = 20f;
    [Range(0f, 1f)]
    [Tooltip("Per-second chance of spontaneously going erratic even without an emergency.")]
    public float erraticSwitchChancePerSecond = 0.01f;
    [Tooltip("Seconds of erratic wandering before returning to patrol. -1 = never return.")]
    public float erraticDuration = 15f;

    // ── Ego vehicle reaction ──────────────────────────────────────────────────
    [Header("Ego Vehicle Reaction")]
    public Transform[] egoVehicles;
    public float startleRadius = 12f;
    [Tooltip("Positive = flee, Negative = approach (curious)")]
    public float reactionBias = 1f;
    public float reactionSpeedMultiplier = 2f;

    NavMeshAgent navAgent;
    Vector3 currentTarget;
    float pauseTimer;
    float erraticTimer;
    int m_PatrolIndex;
    int m_PatrolDir = 1;

    enum State { Wandering, Paused, Crossing, Reacting, Patrolling }
    State state = State.Wandering;

    void Start()
    {
        navAgent = GetComponent<NavMeshAgent>();
        navAgent.speed = Random.Range(minSpeed, maxSpeed);

        if (patrolWaypoints != null && patrolWaypoints.Length > 0)
            EnterPatrol();
        else
            PickNewWaypoint();
    }

    void Update()
    {
        CheckEmergency();
        CheckEgoReaction();

        switch (state)
        {
            case State.Wandering:  UpdateWandering();  break;
            case State.Paused:     UpdatePaused();     break;
            case State.Crossing:   UpdateCrossing();   break;
            case State.Reacting:   UpdateReacting();   break;
            case State.Patrolling: UpdatePatrolling(); break;
        }
    }

    // ── Patrol ────────────────────────────────────────────────────────────────

    void EnterPatrol()
    {
        state = State.Patrolling;
        navAgent.isStopped = false;
        navAgent.speed = Random.Range(minSpeed, maxSpeed);
        SetPatrolDestination();
    }

    void UpdatePatrolling()
    {
        if (!navAgent.pathPending && navAgent.remainingDistance < waypointReachedDistance)
            AdvancePatrol();

        if (Roll(pauseProbabilityPerSecond)) EnterPause();
        if (Roll(erraticSwitchChancePerSecond)) EnterErratic();

        if (agentType == AgentType.Animal) AnimateAnimalSpeed();
    }

    void AdvancePatrol()
    {
        if (patrolWaypoints == null || patrolWaypoints.Length == 0) return;

        if (loopPatrol)
        {
            m_PatrolIndex = (m_PatrolIndex + 1) % patrolWaypoints.Length;
        }
        else
        {
            m_PatrolIndex += m_PatrolDir;
            if (m_PatrolIndex >= patrolWaypoints.Length - 1 || m_PatrolIndex <= 0)
                m_PatrolDir *= -1;
        }

        SetPatrolDestination();
    }

    void SetPatrolDestination()
    {
        if (patrolWaypoints == null || m_PatrolIndex >= patrolWaypoints.Length) return;
        Transform wp = patrolWaypoints[m_PatrolIndex];
        if (wp != null) TrySetDestination(wp.position, 5f);
    }

    // ── Emergency switch ──────────────────────────────────────────────────────

    void CheckEmergency()
    {
        if (emergencyVehicles == null || state == State.Reacting || state == State.Wandering) return;
        foreach (Transform t in emergencyVehicles)
        {
            if (t == null) continue;
            if (Vector3.Distance(transform.position, t.position) < emergencyRadius)
            {
                EnterErratic();
                return;
            }
        }
    }

    // Switch from patrol to full erratic; restores patrol after erraticDuration.
    void EnterErratic()
    {
        state = State.Wandering;
        navAgent.isStopped = false;
        navAgent.speed = Random.Range(minSpeed, maxSpeed);
        erraticTimer = erraticDuration;
        PickNewWaypoint();
    }

    // ── State updates ──────────────────────────────────────────────────────────

    void UpdateWandering()
    {
        if (!navAgent.pathPending && navAgent.remainingDistance < waypointReachedDistance)
            PickNewWaypoint();

        if (Roll(pauseProbabilityPerSecond))    EnterPause();
        if (Roll(jitterProbabilityPerSecond))   JitterTarget();
        if (Roll(reversalProbabilityPerSecond)) ReverseDirection();

        if (agentType == AgentType.Animal) AnimateAnimalSpeed();

        // count down back to patrol path
        if (patrolWaypoints != null && patrolWaypoints.Length > 0 && erraticDuration >= 0f)
        {
            erraticTimer -= Time.deltaTime;
            if (erraticTimer <= 0f) EnterPatrol();
        }
    }

    void UpdatePaused()
    {
        pauseTimer -= Time.deltaTime;
        if (pauseTimer > 0f) return;

        navAgent.isStopped = false;
        navAgent.speed = Random.Range(minSpeed, maxSpeed);

        bool shouldPatrol = patrolWaypoints != null && patrolWaypoints.Length > 0
                            && (erraticDuration < 0f || erraticTimer > 0f);
        if (shouldPatrol)
            EnterPatrol();
        else
        {
            state = State.Wandering;
            PickNewWaypoint();
        }
    }

    void UpdateCrossing()
    {
        if (!navAgent.pathPending && navAgent.remainingDistance < waypointReachedDistance)
            ExitCrossing();
    }

    void UpdateReacting()
    {
        Transform closest = ClosestEgoInRange();
        if (closest == null) { ExitReaction(); return; }

        if (!navAgent.pathPending && navAgent.remainingDistance < waypointReachedDistance)
            SetReactionTarget(closest);
    }

    // ── Ego vehicle reaction ───────────────────────────────────────────────────

    void CheckEgoReaction()
    {
        if (state == State.Crossing) return;

        Transform closest = ClosestEgoInRange();
        if (closest != null && state != State.Reacting) EnterReaction(closest);
        if (closest == null && state == State.Reacting) ExitReaction();
    }

    void EnterReaction(Transform ego)
    {
        state = State.Reacting;
        navAgent.speed = Random.Range(minSpeed, maxSpeed) * reactionSpeedMultiplier;
        SetReactionTarget(ego);
    }

    void SetReactionTarget(Transform ego)
    {
        if (ego == null) return;
        Vector3 dir = (transform.position - ego.position).normalized * Mathf.Sign(reactionBias);
        Vector3 candidate = transform.position + dir * wanderRadius * 0.5f;
        TrySetDestination(candidate, wanderRadius);
    }

    Transform ClosestEgoInRange()
    {
        Transform best = null;
        float bestDist = startleRadius;
        if (egoVehicles == null) return null;
        foreach (Transform t in egoVehicles)
        {
            if (t == null) continue;
            float d = Vector3.Distance(transform.position, t.position);
            if (d < bestDist) { bestDist = d; best = t; }
        }
        return best;
    }

    void ExitReaction()
    {
        navAgent.speed = Random.Range(minSpeed, maxSpeed);
        if (patrolWaypoints != null && patrolWaypoints.Length > 0)
            EnterPatrol();
        else
        {
            state = State.Wandering;
            PickNewWaypoint();
        }
    }

    // ── Stochastic behaviours ─────────────────────────────────────────────────

    void EnterPause()
    {
        if (state != State.Wandering && state != State.Patrolling) return;
        state = State.Paused;
        navAgent.isStopped = true;
        pauseTimer = Random.Range(minPauseDuration, maxPauseDuration);
    }

    void JitterTarget()
    {
        Vector3 jittered = currentTarget + new Vector3(
            Random.Range(-jitterRadius, jitterRadius), 0f,
            Random.Range(-jitterRadius, jitterRadius));
        if (TrySetDestination(jittered, jitterRadius))
            currentTarget = navAgent.destination;
    }

    void ReverseDirection()
    {
        Vector3 behind = transform.position - transform.forward * Random.Range(3f, wanderRadius * 0.4f);
        TrySetDestination(behind, wanderRadius);
    }

    void PickNewWaypoint()
    {
        Vector3 randomDir = Random.insideUnitSphere * wanderRadius;
        randomDir.y = 0f;
        Vector3 candidate = transform.position + randomDir;
        if (TrySetDestination(candidate, wanderRadius))
            currentTarget = navAgent.destination;
    }

    void AnimateAnimalSpeed()
    {
        if (Roll(0.03f))
            navAgent.speed = maxSpeed * 1.5f;
        else
            navAgent.speed = Mathf.Lerp(navAgent.speed, Random.Range(minSpeed, maxSpeed), Time.deltaTime * 0.4f);
    }

    // ── Street crossing (called externally by StreetCrossingZone) ─────────────

    public void TriggerCrossing(Vector3 crossPoint)
    {
        // Already crossing — don't restart. Reacting is allowed to be overridden:
        // crossing the street takes priority over startling at the ego vehicle.
        if (state == State.Crossing) return;

        // A NavMeshAgent only moves if it's actually placed on the navmesh. If the agent
        // was spawned/dropped off the baked mesh, SetDestination silently does nothing.
        if (navAgent == null || !navAgent.isOnNavMesh)
        {
            Debug.LogWarning($"[ErraticAgent] '{name}' is NOT on the NavMesh — can't cross. " +
                             "Place agents on baked navmesh (snap them to the ground).", this);
            return;
        }

        navAgent.isStopped = false;
        navAgent.speed = Random.Range(minSpeed * 0.8f, maxSpeed * 1.3f);

        // search a wide radius so a slightly-off crossTarget still resolves onto the navmesh
        if (!TrySetDestination(crossPoint, 15f))
        {
            Debug.LogWarning($"[ErraticAgent] '{name}' could not reach crossTarget {crossPoint} " +
                             "— no NavMesh within 15 m. Move the target onto baked navmesh.", this);
            return;
        }

        Debug.Log($"[ErraticAgent] '{name}' crossing to {crossPoint}.", this);
        state = State.Crossing;
    }

    void ExitCrossing()
    {
        navAgent.speed = Random.Range(minSpeed, maxSpeed);
        if (patrolWaypoints != null && patrolWaypoints.Length > 0)
            EnterPatrol();
        else
        {
            state = State.Wandering;
            PickNewWaypoint();
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    bool TrySetDestination(Vector3 candidate, float searchRadius)
    {
        NavMeshHit hit;
        if (!NavMesh.SamplePosition(candidate, out hit, searchRadius, NavMesh.AllAreas))
            return false;
        navAgent.SetDestination(hit.position);
        return true;
    }

    bool Roll(float probabilityPerSecond) =>
        Random.value < probabilityPerSecond * Time.deltaTime;
}
