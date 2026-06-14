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
    public int   targetCount   = 8;
    public float spawnRadius   = 40f;        // spawn within this distance of ego
    public float cullRadius    = 80f;        // destroy agents beyond this distance
    public float checkInterval = 3f;         // seconds between spawn/cull passes

    [Header("Agent config")]
    public float minSpeed      = 0.5f;
    public float maxSpeed      = 3.5f;
    public float startleRadius = 15f;
    [Tooltip("Positive = flee, Negative = curious")]
    public float reactionBias  = 1f;

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
        for (int i = m_Agents.Count - 1; i >= 0; i--)
        {
            if (m_Agents[i] == null) { m_Agents.RemoveAt(i); continue; }
            if (Vector3.Distance(m_Agents[i].transform.position, ego.position) > cullRadius)
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
        Vector2 disk = Random.insideUnitCircle * spawnRadius;
        Vector3 candidate = ego.position + new Vector3(disk.x, 0f, disk.y);

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
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(ego.position, spawnRadius);
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(ego.position, cullRadius);
    }
}
