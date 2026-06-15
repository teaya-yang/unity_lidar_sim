using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class ScenarioManager : MonoBehaviour
{
    [Header("Config")]
    public ScenarioConfig config;

    [Header("References")]
    public GameObject agentPrefab;
    public Transform[] egoVehicles;

    [Header("Optional wiring (S2 emergency / patrol scenes)")]
    [Tooltip("Vehicles whose proximity makes patrolling agents go erratic (e.g. ambulance).")]
    public Transform[] emergencyVehicles;
    [Tooltip("Shared street route. If set, spawned agents patrol it instead of pure wandering.")]
    public Transform[] patrolWaypoints;

    [Header("Ambulance spawning (S2)")]
    [Tooltip("Ambulance prefab to spawn at runtime. Leave null to use scene-placed emergencyVehicles only.")]
    public GameObject ambulancePrefab;
    [Tooltip("How many ambulances to spawn. They share patrolWaypoints, staggered by equal offsets.")]
    public int ambulanceCount = 0;

    readonly List<GameObject> m_SpawnedAgents = new();
    readonly List<Transform>  m_SpawnedAmbulances = new();

    void Start() => ResetEpisode(config != null ? config.seed : 0);

    public void ResetEpisode(int seed)
    {
        Random.InitState(seed);
        DestroyAgents();

        SpawnAmbulances();

        if (config == null) { Debug.LogError("[ScenarioManager] No ScenarioConfig assigned.", this); return; }
        if (agentPrefab == null) {  Debug.LogError("[ScenarioManager] No agentPrefab assigned.", this); return; }

        Debug.Log($"[ScenarioManager] ResetEpisode seed={seed} | config={config.name} | " +
                  $"agentCount={config.agentCount} spawnCenter={config.spawnCenter} spawnRadius={config.spawnRadius}", this);

        // Warn early if the spawn center is far from any NavMesh
        if (!NavMesh.SamplePosition(config.spawnCenter, out _, config.spawnRadius * 2f, NavMesh.AllAreas))
            Debug.LogWarning($"[ScenarioManager] spawnCenter {config.spawnCenter} has NO NavMesh within " +
                             $"{config.spawnRadius * 2f} m. Bake a NavMesh that covers this area, " +
                             "or move spawnCenter onto the baked surface.", this);

        int spawned = 0;
        for (int i = 0; i < config.agentCount; i++)
            if (SpawnAgent(i)) spawned++;

        Debug.Log($"[ScenarioManager] Spawned {spawned}/{config.agentCount} agents.", this);
    }

    bool SpawnAgent(int index)
    {
        Vector3 candidate = config.spawnCenter +
            new Vector3(Random.Range(-config.spawnRadius, config.spawnRadius), 0f,
                        Random.Range(-config.spawnRadius, config.spawnRadius));

        if (!NavMesh.SamplePosition(candidate, out NavMeshHit hit, config.spawnRadius, NavMesh.AllAreas))
        {
            Debug.LogWarning($"[ScenarioManager] Agent_{index}: NavMesh sample FAILED " +
                             $"at candidate {candidate} (searchRadius={config.spawnRadius}). " +
                             "Move spawnCenter onto baked NavMesh, increase spawnRadius, or re-bake.", this);
            return false;
        }

        Debug.Log($"[ScenarioManager] Agent_{index}: candidate={candidate} → snapped to {hit.position}", this);

        GameObject go = Instantiate(agentPrefab, hit.position, Quaternion.identity);
        go.name = $"Agent_{index}";

        ErraticAgent agent = go.GetComponent<ErraticAgent>();
        if (agent == null)
        {
            Debug.LogError($"[ScenarioManager] agentPrefab '{agentPrefab.name}' has no ErraticAgent component. " +
                           "Add ErraticAgent to the prefab.", this);
            return false;
        }

        agent.agentType = config.agentTypes.Length > 0
            ? config.agentTypes[Random.Range(0, config.agentTypes.Length)]
            : ErraticAgent.AgentType.Pedestrian;

        agent.minSpeed          = config.minSpeed;
        agent.maxSpeed          = config.maxSpeed;
        agent.startleRadius     = config.startleRadius;
        agent.reactionBias      = config.reactionBias;
        agent.egoVehicles       = egoVehicles;
        agent.emergencyVehicles = emergencyVehicles;
        agent.patrolWaypoints   = patrolWaypoints;

        m_SpawnedAgents.Add(go);
        return true;
    }

    void SpawnAmbulances()
    {
        foreach (Transform t in m_SpawnedAmbulances)
            if (t != null) Destroy(t.gameObject);
        m_SpawnedAmbulances.Clear();

        if (ambulancePrefab == null || ambulanceCount <= 0) return;

        // Build combined list: scene-placed + runtime-spawned
        var allEmergency = new List<Transform>(emergencyVehicles ?? System.Array.Empty<Transform>());

        int wpCount = patrolWaypoints != null ? patrolWaypoints.Length : 0;
        for (int i = 0; i < ambulanceCount; i++)
        {
            // Start each ambulance at a different waypoint offset so they spread out
            Vector3 spawnPos = wpCount > 0
                ? patrolWaypoints[(i * (wpCount / Mathf.Max(ambulanceCount, 1))) % wpCount].position
                : config.spawnCenter + new Vector3(i * 5f, 0f, 0f);

            GameObject go = Instantiate(ambulancePrefab, spawnPos, Quaternion.identity);
            go.name = $"Ambulance_{i}";

            ErraticVehicle ev = go.GetComponent<ErraticVehicle>();
            if (ev != null)
            {
                ev.patrolWaypoints = patrolWaypoints;
                ev.airplane = egoVehicles != null && egoVehicles.Length > 0 ? egoVehicles[0] : null;
            }

            m_SpawnedAmbulances.Add(go.transform);
            allEmergency.Add(go.transform);
        }

        // Rebuild the array so spawned agents react to runtime ambulances too
        emergencyVehicles = allEmergency.ToArray();
    }

    void DestroyAgents()
    {
        foreach (GameObject go in m_SpawnedAgents)
            if (go != null) Destroy(go);
        m_SpawnedAgents.Clear();
    }
}
