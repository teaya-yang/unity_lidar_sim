using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class ScenarioManager : MonoBehaviour
{
    [Header("Config")]
    public ScenarioConfig config;

    [Header("References")]
    [Tooltip("One or more ego vehicles respawned to their start pose at each episode reset.")]
    public Transform[] egoVehicles;

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

        Vector3 spawnPos = SampleNavMeshPosition(index, entry.label);
        if (spawnPos == Vector3.positiveInfinity) return false;

        GameObject prefab = entry.prefabs[Random.Range(0, entry.prefabs.Length)];
        GameObject go = Instantiate(prefab, spawnPos, Quaternion.identity);
        go.name = $"{entry.label}_{index}";

        ErraticAgent agent = go.GetComponent<ErraticAgent>();
        if (agent != null)
        {
            agent.minSpeed      = config.minSpeed;
            agent.maxSpeed      = config.maxSpeed;
            agent.startleRadius = config.startleRadius;
            agent.reactionBias  = config.reactionBias;
            agent.egoVehicles   = egoVehicles;
        }

        ErraticVehicle vehicle = go.GetComponent<ErraticVehicle>();
        if (vehicle != null)
            vehicle.airplane = egoVehicles != null && egoVehicles.Length > 0 ? egoVehicles[0] : null;

        if (agent == null && vehicle == null)
            Debug.LogWarning($"[ScenarioManager] '{go.name}' has neither ErraticAgent nor ErraticVehicle.", this);

        m_SpawnedAgents.Add(go);
        return true;
    }

    // Returns a NavMesh-snapped position near spawnCenter, or Vector3.positiveInfinity on failure.
    Vector3 SampleNavMeshPosition(int index, string label)
    {
        Vector3 candidate = config.spawnCenter +
            new Vector3(Random.Range(-config.spawnRadius, config.spawnRadius), 0f,
                        Random.Range(-config.spawnRadius, config.spawnRadius));

        if (!NavMesh.SamplePosition(candidate, out NavMeshHit hit, config.spawnRadius, NavMesh.AllAreas))
        {
            Debug.LogWarning($"[ScenarioManager] {label}_{index}: NavMesh sample FAILED at {candidate}. " +
                             "Move spawnCenter onto baked NavMesh, increase spawnRadius, or re-bake.", this);
            return Vector3.positiveInfinity;
        }

        Debug.Log($"[ScenarioManager] {label}_{index}: candidate={candidate} → snapped to {hit.position}");
        return hit.position;
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
