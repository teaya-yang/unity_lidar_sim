using UnityEngine;
using UnityEngine.AI;

// Generate an agent that try to mimic an erratic agents, by dividing the agents that keeps moving accross four multiple states
// (wandering, paused, cross, reacting) wandering = roll three dice for being paused, nudge by a random offset the current destination, or pick a point behind the agent and walk there
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

    [Header("Ego Vehicle Reaction")]
    public Transform egoVehicle;
    public float startleRadius = 12f;
    [Tooltip("Positive = flee, Negative = approach (curious)")]
    public float reactionBias = 1f;
    public float reactionSpeedMultiplier = 2f;

    private NavMeshAgent navAgent;
    private Vector3 currentTarget;
    private float pauseTimer;

    private enum State { Wandering, Paused, Crossing, Reacting }
    private State state = State.Wandering;

    void Start()
    {
        navAgent = GetComponent<NavMeshAgent>();
        navAgent.speed = Random.Range(minSpeed, maxSpeed);
        PickNewWaypoint();
    }

    void Update()
    {
        // at each timestamp run checkego reaction, to check how far is the agent from the ego, if distance smaller than set range, updaUpdatete the velocity of the target to generate random behaviour
        CheckEgoReaction();


        switch (state)
        {
            case State.Wandering: UpdateWandering(); break;
            case State.Paused:   UpdatePaused();    break;
            case State.Crossing: UpdateCrossing();  break;
            case State.Reacting: UpdateReacting();  break;
        }
    }

    // ── State updates ──────────────────────────────────────────────────────────

    void UpdateWandering()
    {
        if (!navAgent.pathPending && navAgent.remainingDistance < waypointReachedDistance)
            PickNewWaypoint();

        if (Roll(pauseProbabilityPerSecond))  EnterPause();
        if (Roll(jitterProbabilityPerSecond)) JitterTarget();
        if (Roll(reversalProbabilityPerSecond)) ReverseDirection();

        if (agentType == AgentType.Animal)
            AnimateAnimalSpeed();
    }

    void UpdatePaused()
    {
        pauseTimer -= Time.deltaTime;
        if (pauseTimer > 0f) return;

        navAgent.isStopped = false;
        navAgent.speed = Random.Range(minSpeed, maxSpeed);
        state = State.Wandering;
        PickNewWaypoint();
    }

    void UpdateCrossing()
    {
        if (!navAgent.pathPending && navAgent.remainingDistance < waypointReachedDistance)
            ExitCrossing();
    }

    void UpdateReacting()
    {
        if (egoVehicle == null) { ExitReaction(); return; }

        // compute distance between transform.position (agents) and egoVehicle (ambulance)
        float dist = Vector3.Distance(transform.position, egoVehicle.position);
        if (dist >= startleRadius)
            ExitReaction();
        else if (!navAgent.pathPending && navAgent.remainingDistance < waypointReachedDistance)
            SetReactionTarget();
    }

    // ── Ego vehicle reaction ───────────────────────────────────────────────────

    void CheckEgoReaction()
    {
        if (egoVehicle == null || state == State.Crossing) return;

        // check distance from ego
        float dist = Vector3.Distance(transform.position, egoVehicle.position);
        bool inRange = dist < startleRadius;

        if (inRange && state != State.Reacting)  EnterReaction();
        if (!inRange && state == State.Reacting) ExitReaction();
    }

    void EnterReaction()
    {
        state = State.Reacting;
        navAgent.speed = Random.Range(minSpeed, maxSpeed) * reactionSpeedMultiplier;
        SetReactionTarget();
    }

    void SetReactionTarget()
    {
        if (egoVehicle == null) return;
        // reactionBias > 0 → flee away; < 0 → move toward (curious animal)
        Vector3 dir = (transform.position - egoVehicle.position).normalized * Mathf.Sign(reactionBias);
        Vector3 candidate = transform.position + dir * wanderRadius * 0.5f;
        TrySetDestination(candidate, wanderRadius);
    }

    void ExitReaction()
    {
        state = State.Wandering;
        navAgent.speed = Random.Range(minSpeed, maxSpeed);
        PickNewWaypoint();
    }

    // ── Stochastic behaviours ─────────────────────────────────────────────────

    void EnterPause()
    {
        if (state != State.Wandering) return;
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

    // Animals get occasional speed bursts that decay back to normal
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
        if (state == State.Crossing || state == State.Reacting) return;
        state = State.Crossing;
        navAgent.isStopped = false;
        navAgent.speed = Random.Range(minSpeed * 0.8f, maxSpeed * 1.3f);
        TrySetDestination(crossPoint, 5f);
    }

    void ExitCrossing()
    {
        state = State.Wandering;
        navAgent.speed = Random.Range(minSpeed, maxSpeed);
        PickNewWaypoint();
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    bool TrySetDestination(Vector3 candidate, float searchRadius)
    {
        NavMeshHit hit;
        // navmesh sample position from candidate baked navmesh - if there is no one return false
        if (!NavMesh.SamplePosition(candidate, out hit, searchRadius, NavMesh.AllAreas))
            return false;
        navAgent.SetDestination(hit.position);
        return true;
    }

    // Probability event scaled to frame rate (Poisson approximation)
    bool Roll(float probabilityPerSecond) =>
        Random.value < probabilityPerSecond * Time.deltaTime;
}
