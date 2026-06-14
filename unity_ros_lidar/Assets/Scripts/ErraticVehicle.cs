using UnityEngine;

/// <summary>
/// Stochastic vehicle movement without NavMesh — picks random waypoints, occasionally
/// stops, and reacts when the airplane enters reactionRadius (pull over or rush).
/// Attach to the ambulance prefab instead of AmbulanceTrajectorySubscriber for
/// fully autonomous, ROS-free behavior.
/// </summary>
public class ErraticVehicle : MonoBehaviour
{
    [Header("Patrol Route")]
    [Tooltip("Assign street waypoints for a predictable route. Leave empty for random wandering.")]
    public Transform[] patrolWaypoints;
    public bool loopPatrol = true;

    [Header("Movement")]
    public float minSpeed               = 4f;
    public float maxSpeed               = 12f;
    public float rotationSpeed          = 3f;
    public float waypointReachedDistance = 2f;
    public float wanderRadius           = 60f;

    [Header("Micro-stops")]
    [Range(0f, 1f)] public float stopProbabilityPerSecond = 0.04f;
    public float minStopDuration = 1f;
    public float maxStopDuration = 5f;

    [Header("Airplane reaction")]
    public Transform airplane;
    public float reactionRadius = 30f;
    [Tooltip("True = pull over and stop; False = rush (speed boost)")]
    public bool pullOverOnReaction = true;

    float   m_Speed;
    Vector3 m_Target;
    float   m_StopTimer;
    bool    m_Stopped;
    bool    m_Reacting;
    int     m_PatrolIndex;
    int     m_PatrolDir = 1;

    void Start()
    {
        m_Speed = Random.Range(minSpeed, maxSpeed);
        if (patrolWaypoints != null && patrolWaypoints.Length > 0)
            m_Target = patrolWaypoints[0].position;
        else
            PickWaypoint();
    }

    void Update()
    {
        CheckAirplaneReaction();

        if (m_Stopped)
        {
            m_StopTimer -= Time.deltaTime;
            if (m_StopTimer <= 0f) ExitStop();
            return;
        }

        MoveTowardTarget();

        if (Vector3.Distance(transform.position, m_Target) < waypointReachedDistance)
            AdvanceTarget();

        if (!m_Reacting && Roll(stopProbabilityPerSecond))
            EnterStop();
    }

    void MoveTowardTarget()
    {
        Vector3 dir = m_Target - transform.position;
        dir.y = 0f;
        if (dir.sqrMagnitude < 0.001f) return;
        dir.Normalize();

        // model's visual front is -Z, so face -dir toward the target
        Quaternion look = Quaternion.LookRotation(-dir);
        transform.rotation = Quaternion.RotateTowards(
            transform.rotation, look, rotationSpeed * 60f * Time.deltaTime);

        // drive along the car's own heading, not straight at the target — turn first, then accelerate
        Vector3 heading = -transform.forward;            // visual front
        float alignment = Mathf.Clamp01(Vector3.Dot(heading, dir));
        transform.position += heading * (m_Speed * alignment * Time.deltaTime);
    }

    void CheckAirplaneReaction()
    {
        if (airplane == null) return;
        bool inRange = Vector3.Distance(transform.position, airplane.position) < reactionRadius;

        if (inRange && !m_Reacting)
        {
            m_Reacting = true;
            if (pullOverOnReaction)
                EnterStop(maxStopDuration * 2f);
            else
                m_Speed = maxSpeed * 1.5f;
        }

        if (!inRange && m_Reacting)
        {
            m_Reacting = false;
            m_Speed = Random.Range(minSpeed, maxSpeed);
        }
    }

    void EnterStop(float duration = -1f)
    {
        m_Stopped   = true;
        m_StopTimer = duration > 0f ? duration : Random.Range(minStopDuration, maxStopDuration);
    }

    void ExitStop()
    {
        m_Stopped = false;
        m_Speed   = Random.Range(minSpeed, maxSpeed);
        AdvanceTarget();
    }

    void AdvanceTarget()
    {
        if (patrolWaypoints != null && patrolWaypoints.Length > 0)
        {
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
            Transform wp = patrolWaypoints[m_PatrolIndex];
            if (wp != null) m_Target = wp.position;
        }
        else
        {
            PickWaypoint();
        }
    }

    void PickWaypoint()
    {
        Vector2 disk = Random.insideUnitCircle * wanderRadius;
        m_Target = transform.position + new Vector3(disk.x, 0f, disk.y);
    }

    bool Roll(float probabilityPerSecond) =>
        Random.value < probabilityPerSecond * Time.deltaTime;

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, wanderRadius);
        if (airplane != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(transform.position, reactionRadius);
        }
    }
}
