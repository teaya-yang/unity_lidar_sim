using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

// Generates a unique random patrol path for every spawned ErraticAgent each episode.
// Must run AFTER AgentPlacementRandomizer in the randomizer pipeline (index > placement).
//
// Strategy: random walk — each waypoint steps from the previous one in a random direction,
// clamped to the NavMesh. This produces natural-looking routes rather than a random scatter
// of disconnected points.
public class PatrolPathRandomizer : EpisodeRandomizer
{
    [Header("Path shape")]
    [Tooltip("Number of waypoints per agent path.")]
    [Range(2, 12)] public int waypointCount = 4;

    [Tooltip("Min/max step distance between consecutive waypoints (metres).")]
    public float minStepDistance = 5f;
    public float maxStepDistance = 15f;

    [Tooltip("How far from the NavMesh a candidate can land and still snap (metres).")]
    public float navMeshSnapRadius = 5f;

    [Tooltip("Agents loop back to waypoint 0 after reaching the last one.")]
    public bool loopPath = true;

    [Header("References")]
    public ScenarioManager manager;

    // Waypoint GameObjects created this episode — destroyed on next Randomize call.
    readonly List<GameObject> m_WaypointRoots = new();

    void Reset() => manager = GetComponent<ScenarioManager>();

    public override void Randomize(int episodeSeed, int randomizerIndex)
    {
        if (manager == null) manager = GetComponent<ScenarioManager>();
        if (manager == null)
        {
            Debug.LogError("[PatrolPathRandomizer] No ScenarioManager assigned.", this);
            return;
        }

        SeedStream(episodeSeed, randomizerIndex);
        ClearWaypoints();

        int assigned = 0;
        foreach (GameObject agentGO in manager.SpawnedAgents)
        {
            ErraticAgent agent = agentGO.GetComponent<ErraticAgent>();
            if (agent == null) continue;

            // patrolWaypoints removed — agents now follow TaxiwayLane via OnApronInitialize.
            assigned++;
        }

        Debug.Log($"[PatrolPathRandomizer] Assigned random paths to {assigned}/{manager.SpawnedAgents.Count} agents " +
                  $"({waypointCount} waypoints each).");
    }

    Transform[] GeneratePath(Vector3 startPos)
    {
        var waypoints = new List<Transform>();
        Vector3 current = startPos;

        for (int i = 0; i < waypointCount; i++)
        {
            Vector3 candidate = SampleNextPoint(current);
            // float.IsPositiveInfinity, not `== Vector3.positiveInfinity`: Vector3's == compares
            // (a-b).sqrMagnitude which is NaN (never true) for infinity, so the sentinel would slip
            // through and add a waypoint at (∞,∞,∞).
            if (float.IsPositiveInfinity(candidate.x))
            {
                Debug.LogWarning($"[PatrolPathRandomizer] Could not find NavMesh point after {current}. " +
                                 "Stopping path early.");
                break;
            }

            GameObject wp = new($"RandWP_{waypoints.Count}");
            wp.transform.position = candidate;
            m_WaypointRoots.Add(wp);
            waypoints.Add(wp.transform);
            current = candidate;
        }

        if (waypoints.Count < 2) return null;
        return waypoints.ToArray();
    }

    // Step in a random direction from `from`, snap to NavMesh.
    Vector3 SampleNextPoint(Vector3 from)
    {
        for (int attempt = 0; attempt < 8; attempt++)
        {
            Vector2 disk   = Random.insideUnitCircle.normalized;
            float   dist   = Random.Range(minStepDistance, maxStepDistance);
            Vector3 candidate = from + new Vector3(disk.x, 0f, disk.y) * dist;

            if (NavMesh.SamplePosition(candidate, out NavMeshHit hit, navMeshSnapRadius, NavMesh.AllAreas))
                return hit.position;
        }
        return Vector3.positiveInfinity;
    }

    void ClearWaypoints()
    {
        foreach (GameObject wp in m_WaypointRoots)
            if (wp != null) Destroy(wp);
        m_WaypointRoots.Clear();
    }

    void OnDestroy() => ClearWaypoints();
}
