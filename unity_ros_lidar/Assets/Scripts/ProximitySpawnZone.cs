using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

/// <summary>
/// Spawns and culls ErraticAgents around a moving ego (e.g. the airplane).
/// Agents beyond cullRadius are destroyed; new ones fill up to targetCount.
/// Attach to any GameObject in the scene and drag the airplane into ego.
/// </summary>
public class ProximitySpawnZone : MonoBehaviour
{
    [Header("Ego")]
    public Transform ego;

    [Header("Prefabs")]
    public GameObject[] agentPrefabs;        // person / animal prefabs with ErraticAgent

    [Header("Spawn settings")]
    public int   targetCount      = 8;
    public float minSpawnDistance = 20f;     // don't spawn closer than this (keep outside LiDAR view)
    public float spawnRadius      = 60f;     // spawn within this distance of ego
    public float lidarRange       = 80f;     // match this to PointCloudPublisher.maxRange
    [Tooltip("How far behind the plane an agent must be before it can be culled. " +
             "0 = exactly perpendicular, 1 = directly behind. 0.2 is a safe margin.")]
    [Range(0f, 1f)]
    public float behindThreshold  = 0.2f;
    public float checkInterval    = 3f;      // seconds between spawn/cull passes
    [Range(10f, 360f)]
    [Tooltip("Total angular width of the spawn arc.")]
    public float spawnArc         = 180f;
    [Range(0f, 360f)]
    [Tooltip("Degrees offset from ego.forward. 0 = in front, 180 = directly behind.")]
    public float spawnArcOffset   = 0f;

    [Header("Agent config")]
    public float minSpeed      = 0.5f;
    public float maxSpeed      = 3.5f;
    public float startleRadius = 50f;        // must be >= spawnRadius so agents react on spawn
    [Tooltip("Positive = flee, Negative = curious/approach")]
    public float reactionBias  = -1f;

    readonly List<GameObject> m_Agents = new();
    float m_NextCheck;

    void Update()
    {
        if (Time.time < m_NextCheck) return;
        m_NextCheck = Time.time + checkInterval;
        CullDistant();
        SpawnMissing();
    }

    void CullDistant()
    {
        if (ego == null) return;
        Vector3 fwd = ego.forward; fwd.y = 0f;
        bool hasFwd = fwd.sqrMagnitude > 0.001f;
        if (hasFwd) fwd.Normalize();

        for (int i = m_Agents.Count - 1; i >= 0; i--)
        {
            if (m_Agents[i] == null) { m_Agents.RemoveAt(i); continue; }

            Vector3 toAgent = m_Agents[i].transform.position - ego.position;
            float dist = toAgent.magnitude;

            // keep anything the LiDAR can still reach
            if (dist <= lidarRange) continue;

            // beyond LiDAR range: only cull if clearly behind the plane,
            // not just beside it — behindThreshold > 0 adds a margin past 90°
            toAgent.y = 0f;
            float dot = hasFwd ? Vector3.Dot(fwd, toAgent.normalized) : -1f;
            if (dot < -behindThreshold)
            {
                Destroy(m_Agents[i]);
                m_Agents.RemoveAt(i);
            }
        }
    }

    void SpawnMissing()
    {
        if (agentPrefabs == null || agentPrefabs.Length == 0 || ego == null) return;
        int toSpawn = targetCount - m_Agents.Count;
        for (int i = 0; i < toSpawn; i++)
            TrySpawnOne();
    }

    void TrySpawnOne()
    {
        Vector3 forward = ego.forward;
        forward.y = 0f;
        if (forward.sqrMagnitude < 0.001f) forward = Vector3.forward;
        forward.Normalize();

        // rotate forward by the arc offset to get the spawn zone's centre direction
        float centerRad = spawnArcOffset * Mathf.Deg2Rad;
        float cos0 = Mathf.Cos(centerRad), sin0 = Mathf.Sin(centerRad);
        Vector3 center = new Vector3(
            forward.x * cos0 - forward.z * sin0,
            0f,
            forward.x * sin0 + forward.z * cos0);

        // random angle within the arc around that centre
        float halfArc = spawnArc * 0.5f * Mathf.Deg2Rad;
        float angle   = Random.Range(-halfArc, halfArc);
        float cosA = Mathf.Cos(angle), sinA = Mathf.Sin(angle);
        Vector3 dir = new Vector3(
            center.x * cosA - center.z * sinA,
            0f,
            center.x * sinA + center.z * cosA);

        float dist    = Random.Range(minSpawnDistance, spawnRadius);
        Vector3 candidate = ego.position + dir * dist;

        if (!NavMesh.SamplePosition(candidate, out NavMeshHit hit, spawnRadius, NavMesh.AllAreas))
            return;

        GameObject prefab = agentPrefabs[Random.Range(0, agentPrefabs.Length)];
        if (prefab == null) return;

        GameObject go = Instantiate(prefab, hit.position, Quaternion.identity);
        go.name = $"ProxAgent_{m_Agents.Count}";

        ErraticAgent agent = go.GetComponent<ErraticAgent>();
        if (agent != null)
        {
            agent.minSpeed      = minSpeed;
            agent.maxSpeed      = maxSpeed;
            agent.startleRadius = startleRadius;
            agent.reactionBias  = reactionBias;
            agent.egoVehicles   = new Transform[] { ego };
        }

        m_Agents.Add(go);
    }

    void OnDestroy()
    {
        foreach (GameObject go in m_Agents)
            if (go != null) Destroy(go);
        m_Agents.Clear();
    }

    void OnDrawGizmosSelected()
    {
        if (ego == null) return;

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(ego.position, lidarRange);

        Vector3 fwd = ego.forward;
        fwd.y = 0f;
        if (fwd.sqrMagnitude > 0.001f) fwd.Normalize();

        // draw the two edges of the spawn arc
        Vector3 left  = Quaternion.Euler(0f, spawnArcOffset - spawnArc * 0.5f, 0f) * fwd;
        Vector3 right = Quaternion.Euler(0f, spawnArcOffset + spawnArc * 0.5f, 0f) * fwd;
        Gizmos.color = Color.green;
        Gizmos.DrawRay(ego.position, left  * spawnRadius);
        Gizmos.DrawRay(ego.position, right * spawnRadius);
        // inner (min distance) boundary markers
        Gizmos.color = Color.yellow;
        Gizmos.DrawRay(ego.position, left  * minSpawnDistance);
        Gizmos.DrawRay(ego.position, right * minSpawnDistance);
    }
}
