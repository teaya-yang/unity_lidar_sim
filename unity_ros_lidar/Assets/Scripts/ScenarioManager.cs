using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

// Maps a SpawnEntry label to a set of scene waypoints.
// Set label to match exactly the SpawnEntry.label in your ScenarioConfig.
[System.Serializable]
public class WaypointSet
{
    public string label;
    public Transform[] waypoints;
}

public class ScenarioManager : MonoBehaviour
{
    [Header("Config")]
    public ScenarioConfig config;

    [Header("References")]
    [Tooltip("One or more ego vehicles respawned to their start pose at each episode reset.")]
    public Transform[] egoVehicles;


    [Header("Waypoints per entry type")]
    [Tooltip("Match label to SpawnEntry.label. Waypoints are assigned to spawned agents at runtime.")]
    public WaypointSet[] waypointSets;

    [Header("Spawn exclusion")]
    [Tooltip("Agents won't spawn closer than this to any ego vehicle (prevents spawning under wings).")]
    public float minEgoDistance = 10f;

    [Tooltip("Agents won't spawn closer than this (centre-to-centre, metres) to another already-placed " +
             "agent/vehicle, so they don't stack. 0 disables the check. Set ~ the agent footprint.")]
    public float minAgentSpacing = 1.5f;

    [Header("Randomizer pipeline (optional)")]
    [Tooltip("Ordered domain-randomization steps run at the start of each episode " +
             "(Perception-style). Leave empty to use built-in inline placement. Order defines " +
             "each randomizer's RNG stream — don't reorder mid-dataset or seeds stop reproducing.")]
    public EpisodeRandomizer[] randomizers;

    readonly List<GameObject> m_SpawnedAgents = new();

    // Live agents for the current episode — randomizers (heading, scale, …) act on these.
    public IReadOnlyList<GameObject> SpawnedAgents => m_SpawnedAgents;

    // Ego vehicle home poses — captured once in Start(), restored every ResetEpisode().
    struct Pose { public Vector3 pos; public Quaternion rot; }
    Pose[] m_EgoPoses;

    void Start()
    {
        CaptureEgoPoses();
        ResetEpisode(config != null ? config.seed : 0);
    }

    public void ResetEpisode(int seed)
    {
        DestroyAgents();
        RestoreEgoPoses();
        Random.InitState(seed);

        if (config == null) { Debug.LogError("[ScenarioManager] No ScenarioConfig assigned.", this); return; }

        Debug.Log($"[ScenarioManager] ResetEpisode seed={seed} | config={config.name} | " +
                  $"spawnCenter={config.spawnCenter} spawnRadius={config.spawnRadius}", this);

        if (randomizers != null && randomizers.Length > 0)
        {
            for (int i = 0; i < randomizers.Length; i++)
            {
                EpisodeRandomizer r = randomizers[i];
                if (r == null || !r.enabledInSweep) continue;
                r.Randomize(seed, i);
            }
        }
        else
        {
            SpawnAllAgents();
        }

        // Reseed so spawned agents' runtime behavior (ErraticAgent speed/pause/jitter draws)
        // reproduces from seed alone, independent of how many randomizers ran.
        Random.InitState(seed);

        Debug.Log($"[ScenarioManager] Spawned {m_SpawnedAgents.Count} agents.", this);
    }

    // Spawns all entries defined in config.spawnEntries.
    // Called by AgentPlacementRandomizer and by the inline path when no randomizers are assigned.
    public void SpawnAllAgents()
    {
        if (config.spawnEntries == null || config.spawnEntries.Length == 0)
        {
            Debug.LogWarning("[ScenarioManager] No spawnEntries defined in config. Nothing to spawn.", this);
            return;
        }

        if (!NavMesh.SamplePosition(config.spawnCenter, out _, config.spawnRadius * 2f, NavMesh.AllAreas))
            Debug.LogWarning($"[ScenarioManager] spawnCenter {config.spawnCenter} has NO NavMesh within " +
                             $"{config.spawnRadius * 2f} m. Bake a NavMesh that covers this area, " +
                             "or move spawnCenter onto the baked surface.", this);

        config.ResolveEntryCounts();
        int globalIndex = 0;
        foreach (SpawnEntry entry in config.spawnEntries)
        {
            Debug.Log($"[ScenarioManager] Entry '{entry.label}': spawning {entry.resolvedCount}.");
            for (int i = 0; i < entry.resolvedCount; i++)
                SpawnAgentFromEntry(globalIndex++, entry);
        }

        WireEmergencyVehicles();
    }

    // Spawns one agent chosen randomly from entry.prefabs.
    // Handles both ErraticAgent (NavMesh pedestrian/animal) and ErraticVehicle prefabs.
    bool SpawnAgentFromEntry(int index, SpawnEntry entry)
    {
        if (entry.prefabs == null || entry.prefabs.Length == 0)
        {
            Debug.LogWarning($"[ScenarioManager] Entry '{entry.label}' has no prefabs assigned.", this);
            return false;
        }

        GameObject prefab = entry.prefabs[Random.Range(0, entry.prefabs.Length)];
        Transform[] entryWaypoints = FindWaypoints(entry.label);

        bool isVehicle = prefab.GetComponent<ErraticVehicle>() != null;

        if (isVehicle && (entryWaypoints == null || entryWaypoints.Length == 0))
            Debug.LogWarning($"[ScenarioManager] Vehicle entry '{entry.label}' has no waypoints. " +
                             $"Add a WaypointSet whose label is exactly '{entry.label}', " +
                             "or it will spawn near the ego and immediately pull over.", this);

        // Vehicles drive a waypoint route off the pedestrian NavMesh, so spawn them on the
        // route (first waypoint). Spawning them near spawnCenter would land them on top of
        // the ego vehicle after a reset and make them immediately pull over.
        Vector3 spawnPos;
        if (isVehicle && entryWaypoints != null && entryWaypoints.Length > 0 && entryWaypoints[0] != null)
        {
            spawnPos = entryWaypoints[0].position;
        }
        else
        {
            spawnPos = SampleNavMeshPosition(index, entry.label);
            // Use float.IsPositiveInfinity, not `== Vector3.positiveInfinity`: Vector3's == is an
            // approximate (a-b).sqrMagnitude compare that is NaN (never true) for infinity, which
            // would let a failed placement spawn the agent at (∞,∞,∞).
            if (float.IsPositiveInfinity(spawnPos.x)) return false;
        }

        GameObject go = Instantiate(prefab, spawnPos, Quaternion.identity);
        go.name = $"{entry.label}_{index}";

        ErraticAgent agent = go.GetComponent<ErraticAgent>();
        if (agent != null)
        {
            // Speed is kept from the prefab so each type can move at its own pace.
            // Only scenario-wide reaction settings are applied.
            agent.startleRadius  = config.startleRadius;
            agent.reactionBias   = config.reactionBias;
            agent.egoVehicles    = egoVehicles;
        }

        ErraticVehicle vehicle = go.GetComponent<ErraticVehicle>();
        if (vehicle != null)
        {
            vehicle.airplane = egoVehicles != null && egoVehicles.Length > 0 ? egoVehicles[0] : null;
            vehicle.Initialize();
        }

        if (agent == null && vehicle == null)
            Debug.LogWarning($"[ScenarioManager] '{go.name}' has neither ErraticAgent nor ErraticVehicle.", this);

        m_SpawnedAgents.Add(go);
        return true;
    }

    // Builds the combined emergency vehicle list (scene-placed + runtime-spawned ErraticVehicles)
    // and assigns it to every spawned ErraticAgent.
    void WireEmergencyVehicles()
    {
        var all = new List<Transform>();

        foreach (GameObject go in m_SpawnedAgents)
        {
            if (go != null && go.GetComponent<ErraticVehicle>() != null)
                all.Add(go.transform);
        }

        if (all.Count == 0) return;

        // emergencyVehicles wiring removed — ErraticAgent no longer has that field.
        Debug.Log($"[ScenarioManager] Found {all.Count} emergency vehicle(s) (wiring skipped — use ApronTrafficManager).");
    }

    // Returns the waypoints assigned to a given entry label, or null if none defined.
    Transform[] FindWaypoints(string label)
    {
        if (waypointSets == null) return null;
        foreach (WaypointSet ws in waypointSets)
            if (ws.label == label) return ws.waypoints;
        return null;
    }

    // Returns a NavMesh-snapped position near spawnCenter that is at least minEgoDistance
    // from every ego vehicle. Returns Vector3.positiveInfinity if no valid position is found.
    Vector3 SampleNavMeshPosition(int index, string label)
    {
        const int maxAttempts = 10;
        for (int attempt = 0; attempt < maxAttempts; attempt++)
        {
            Vector3 candidate = config.spawnCenter +
                new Vector3(Random.Range(-config.spawnRadius, config.spawnRadius), 0f,
                            Random.Range(-config.spawnRadius, config.spawnRadius));

            if (!NavMesh.SamplePosition(candidate, out NavMeshHit hit, config.spawnRadius, NavMesh.AllAreas))
                continue;

            if (IsTooCloseToEgo(hit.position))
                continue;

            // Reject candidates that stack on an already-placed agent/vehicle. NavMesh.SamplePosition
            // can collapse several random candidates onto the same nearby point, so without this two
            // agents can share a spot; nothing pushes them apart at spawn (no Rigidbody sim, and
            // NavMesh avoidance only acts later, during navigation).
            if (IsTooCloseToAgents(hit.position))
                continue;

            Debug.Log($"[ScenarioManager] {label}_{index}: snapped to {hit.position} (attempt {attempt + 1})");
            return hit.position;
        }

        Debug.LogWarning($"[ScenarioManager] {label}_{index}: failed to find a valid spawn position " +
                         $"after {maxAttempts} attempts. Increase spawnRadius or reduce minEgoDistance.", this);
        return Vector3.positiveInfinity;
    }

    bool IsTooCloseToEgo(Vector3 pos)
    {
        if (egoVehicles == null) return false;
        foreach (Transform ego in egoVehicles)
        {
            if (ego == null) continue;
            if (Vector3.Distance(pos, ego.position) < minEgoDistance)
                return true;
        }
        return false;
    }

    // True if pos is within minAgentSpacing of any agent/vehicle already spawned this episode.
    // Checked during placement so independently-sampled agents don't end up stacked.
    bool IsTooCloseToAgents(Vector3 pos)
    {
        if (minAgentSpacing <= 0f) return false;
        foreach (GameObject go in m_SpawnedAgents)
        {
            if (go == null) continue;
            if (Vector3.Distance(pos, go.transform.position) < minAgentSpacing)
                return true;
        }
        return false;
    }

    void CaptureEgoPoses()
    {
        if (egoVehicles == null) return;
        m_EgoPoses = new Pose[egoVehicles.Length];
        for (int i = 0; i < egoVehicles.Length; i++)
            if (egoVehicles[i] != null)
                m_EgoPoses[i] = new Pose { pos = egoVehicles[i].position, rot = egoVehicles[i].rotation };
    }

    void RestoreEgoPoses()
    {
        if (egoVehicles == null || m_EgoPoses == null) return;
        for (int i = 0; i < egoVehicles.Length; i++)
        {
            if (egoVehicles[i] == null) continue;

            Rigidbody rb = egoVehicles[i].GetComponent<Rigidbody>();
            if (rb != null) { rb.linearVelocity = Vector3.zero; rb.angularVelocity = Vector3.zero; }

            egoVehicles[i].SetPositionAndRotation(m_EgoPoses[i].pos, m_EgoPoses[i].rot);
        }
        Debug.Log("[ScenarioManager] Ego vehicles reset to spawn pose.");
    }

    void DestroyAgents()
    {
        foreach (GameObject go in m_SpawnedAgents)
            if (go != null) Destroy(go);
        m_SpawnedAgents.Clear();
    }
}
