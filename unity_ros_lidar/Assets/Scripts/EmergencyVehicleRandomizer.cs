using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

// Spawns and wires emergency vehicles (ambulances, police cars, etc.) each episode.
// Must run AFTER AgentPlacementRandomizer so it can wire emergencyVehicles onto agents.
// Add to the ScenarioManager randomizer pipeline after AgentPlacementRandomizer.
public class EmergencyVehicleRandomizer : EpisodeRandomizer
{
    [Header("Vehicle prefabs")]
    [Tooltip("Pool of emergency vehicle prefabs — each spawn picks one at random.")]
    public GameObject[] vehiclePrefabs;

    [Header("Count")]
    public int minVehicles = 1;
    public int maxVehicles = 3;

    [Header("Patrol route")]
    [Tooltip("Waypoints the vehicles patrol. If empty they wander randomly.")]
    public Transform[] patrolWaypoints;

    [Header("References")]
    public ScenarioManager manager;

    readonly List<GameObject> m_Spawned = new();

    void Reset() => manager = GetComponent<ScenarioManager>();

    public override void Randomize(int episodeSeed, int randomizerIndex)
    {
        if (manager == null) manager = GetComponent<ScenarioManager>();

        SeedStream(episodeSeed, randomizerIndex);
        ClearVehicles();

        if (vehiclePrefabs == null || vehiclePrefabs.Length == 0)
        {
            Debug.LogWarning("[EmergencyVehicleRandomizer] No vehiclePrefabs assigned.", this);
            return;
        }

        int count = Random.Range(minVehicles, maxVehicles + 1);
        var spawnedTransforms = new List<Transform>();

        for (int i = 0; i < count; i++)
        {
            GameObject prefab = vehiclePrefabs[Random.Range(0, vehiclePrefabs.Length)];
            Vector3 spawnPos = PickSpawnPosition(i, count);

            GameObject go = Object.Instantiate(prefab, spawnPos, Quaternion.identity);
            go.name = $"EmergencyVehicle_{i}";

            ErraticVehicle ev = go.GetComponent<ErraticVehicle>();
            if (ev != null)
            {
                ev.patrolWaypoints = patrolWaypoints;
                ev.airplane = manager.egoVehicles != null && manager.egoVehicles.Length > 0
                    ? manager.egoVehicles[0] : null;
            }

            m_Spawned.Add(go);
            spawnedTransforms.Add(go.transform);
        }

        // Wire spawned vehicles onto every agent as emergency triggers
        foreach (GameObject agentGO in manager.SpawnedAgents)
        {
            ErraticAgent agent = agentGO.GetComponent<ErraticAgent>();
            if (agent != null)
                agent.emergencyVehicles = spawnedTransforms.ToArray();
        }

        Debug.Log($"[EmergencyVehicleRandomizer] Spawned {count} emergency vehicle(s).");
    }

    Vector3 PickSpawnPosition(int index, int total)
    {
        if (patrolWaypoints != null && patrolWaypoints.Length > 0)
        {
            int wpIndex = (index * (patrolWaypoints.Length / Mathf.Max(total, 1))) % patrolWaypoints.Length;
            return patrolWaypoints[wpIndex].position;
        }

        // Fallback: spread around spawn center
        if (manager.config != null)
            return manager.config.spawnCenter + new Vector3(index * 8f, 0f, 0f);

        return Vector3.zero;
    }

    void ClearVehicles()
    {
        foreach (GameObject go in m_Spawned)
            if (go != null) Destroy(go);
        m_Spawned.Clear();
    }

    void OnDestroy() => ClearVehicles();
}
