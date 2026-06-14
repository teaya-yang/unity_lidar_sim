using UnityEngine;

/// <summary>
/// Stochastic vehicle movement without NavMesh — picks random waypoints, occasionally
/// stops, and reacts when the airplane enters reactionRadius (pull over or rush).
/// Attach to the ambulance prefab instead of AmbulanceTrajectorySubscriber for
/// fully autonomous, ROS-free behavior.
/// </summary>
public class ErraticVehicle : MonoBehaviour
{
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

    void Start()
    {
        m_Speed = Random.Range(minSpeed, maxSpeed);
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
            PickWaypoint();

        if (!m_Reacting && Roll(stopProbabilityPerSecond))
            EnterStop();
    }

    void MoveTowardTarget()
    {
        transform.position = Vector3.MoveTowards(
            transform.position, m_Target, m_Speed * Time.deltaTime);

        Vector3 dir = m_Target - transform.position;
        if (dir.sqrMagnitude > 0.001f)
        {
            Quaternion look = Quaternion.LookRotation(dir);
            transform.rotation = Quaternion.Slerp(transform.rotation, look, rotationSpeed * Time.deltaTime);
        }
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
        PickWaypoint();
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
