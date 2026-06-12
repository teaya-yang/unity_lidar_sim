using UnityEngine;

[CreateAssetMenu(fileName = "ScenarioConfig", menuName = "Simulation/Scenario Config")]
public class ScenarioConfig : ScriptableObject
{
    [Header("Agents")]
    public int agentCount = 5;
    public ErraticAgent.AgentType[] agentTypes = { ErraticAgent.AgentType.Pedestrian };

    [Header("Speed")]
    public float minSpeed = 0.5f;
    public float maxSpeed = 3.5f;

    [Header("Spawn")]
    public float spawnRadius = 20f;
    public Vector3 spawnCenter = Vector3.zero;

    [Header("Reaction")]
    public float startleRadius = 12f;
    public float reactionBias = 1f;

    [Header("Reproducibility")]
    public int seed = 0;
}
