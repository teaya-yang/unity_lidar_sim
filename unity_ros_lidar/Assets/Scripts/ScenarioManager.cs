using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class ScenarioManager : MonoBehaviour
{
    [Header("Config")]
    public ScenarioConfig config;

    [Header("References")]
    [Tooltip("One or more agent prefabs — each spawn picks one at random.")]
    public GameObject[] agentPrefabs;
    public Transform[] egoVehicles;

    [Header("Randomizer pipeline (optional)")]
    [Tooltip("Ordered domain-randomization steps run at the start of each episode " +
             "(Perception-style). Leave empty to use built-in inline placement. Order defines " +
             "each randomizer's RNG stream — don't reorder mid-dataset or seeds stop reproducing.")]
    public EpisodeRandomizer[] randomizers;

    readonly List<GameObject> m_SpawnedAgents = new();

    // Live agents for the current episode — randomizers (heading, scale, …) act on these.
    public IReadOnlyList<GameObject> SpawnedAgents => m_SpawnedAgents;

    void Start() => ResetEpisode(config != null ? config.seed : 0);

    public void ResetEpisode(int seed)
    {
        DestroyAgents();
        Random.InitState(seed);

        if (config == null) { Debug.LogError("[ScenarioManager] No ScenarioConfig assigned.", this); return; }
        if (agentPrefabs == null || agentPrefabs.Length == 0) { Debug.LogError("[ScenarioManager] No agentPrefabs assigned.", this); return; }

        Debug.Log($"[ScenarioManager] ResetEpisode seed={seed} | config={config.name} | " +
                  $"agentCount={config.agentCount} spawnCenter={config.spawnCenter} spawnRadius={config.spawnRadius}", this);

        if (randomizers != null && randomizers.Length > 0)
        {
            // Perception-style pipeline: each randomizer owns one aspect of the scene and
            // runs in a fixed order, each with its own deterministic RNG stream.
            for (int i = 0; i < randomizers.Length; i++)
            {
                EpisodeRandomizer r = randomizers[i];
                if (r == null || !r.enabledInSweep) continue;
                r.Randomize(seed, i);
            }
        }
        else
        {
            // No pipeline assigned — legacy inline placement (keeps existing scenes working).
            SpawnAllAgents();
        }

        // Reseed from the episode seed so spawned agents' RUNTIME behavior (ErraticAgent
        // speed/pause/jitter draws) reproduces from `seed` alone, independent of how many
        // randomizers ran or in what order.
        Random.InitState(seed);

        Debug.Log($"[ScenarioManager] Spawned {m_SpawnedAgents.Count}/{config.agentCount} agents.", this);
    }

    // Placement implementation, shared by the legacy path and AgentPlacementRandomizer.
    // Single source of truth for "where agents go" — new placement variants just reseed
    // their RNG stream and call this.
    // Previous function used to spawn all agents, before using specific randomizers
    public void SpawnAllAgents()
    {
        // Warn early if the spawn center is far from any NavMesh
        if (!NavMesh.SamplePosition(config.spawnCenter, out _, config.spawnRadius * 2f, NavMesh.AllAreas))
            Debug.LogWarning($"[ScenarioManager] spawnCenter {config.spawnCenter} has NO NavMesh within " +
                             $"{config.spawnRadius * 2f} m. Bake a NavMesh that covers this area, " +
                             "or move spawnCenter onto the baked surface.", this);

        for (int i = 0; i < config.agentCount; i++)
            SpawnAgent(i);
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

        GameObject prefab = agentPrefabs[Random.Range(0, agentPrefabs.Length)];
        GameObject go = Instantiate(prefab, hit.position, Quaternion.identity);
        go.name = $"Agent_{index}";

        ErraticAgent agent = go.GetComponent<ErraticAgent>();
        if (agent == null)
        {
            Debug.LogError($"[ScenarioManager] agentPrefab '{prefab.name}' has no ErraticAgent component. " +
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
        agent.egoVehicles = egoVehicles;

        m_SpawnedAgents.Add(go);
        return true;
    }

    void DestroyAgents()
    {
        foreach (GameObject go in m_SpawnedAgents)
            if (go != null) Destroy(go);
        m_SpawnedAgents.Clear();
    }
}
