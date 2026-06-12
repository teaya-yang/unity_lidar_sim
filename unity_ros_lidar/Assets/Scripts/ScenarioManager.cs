using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class ScenarioManager : MonoBehaviour
{
    [Header("Config")]
    public ScenarioConfig config;

    [Header("References")]
    public GameObject agentPrefab;
    public Transform egoVehicle;

    readonly List<GameObject> m_SpawnedAgents = new();

    void Start() => ResetEpisode(config != null ? config.seed : 0);

    public void ResetEpisode(int seed)
    {
        Random.InitState(seed);
        DestroyAgents();

        if (config == null || agentPrefab == null) return;

        for (int i = 0; i < config.agentCount; i++)
            SpawnAgent(i);
    }

    void SpawnAgent(int index)
    {
        Vector3 candidate = config.spawnCenter +
            new Vector3(Random.Range(-config.spawnRadius, config.spawnRadius), 0f,
                        Random.Range(-config.spawnRadius, config.spawnRadius));

        if (!NavMesh.SamplePosition(candidate, out NavMeshHit hit, config.spawnRadius, NavMesh.AllAreas))
            return;

        GameObject go = Instantiate(agentPrefab, hit.position, Quaternion.identity);
        go.name = $"Agent_{index}";

        ErraticAgent agent = go.GetComponent<ErraticAgent>();
        if (agent == null) return;

        agent.agentType = config.agentTypes.Length > 0
            ? config.agentTypes[Random.Range(0, config.agentTypes.Length)]
            : ErraticAgent.AgentType.Pedestrian;

        agent.minSpeed       = config.minSpeed;
        agent.maxSpeed       = config.maxSpeed;
        agent.startleRadius  = config.startleRadius;
        agent.reactionBias   = config.reactionBias;
        agent.egoVehicle     = egoVehicle;

        m_SpawnedAgents.Add(go);
    }

    void DestroyAgents()
    {
        foreach (GameObject go in m_SpawnedAgents)
            if (go != null) Destroy(go);
        m_SpawnedAgents.Clear();
    }
}
