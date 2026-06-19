using System;
using UnityEngine;

// One population group: a pool of prefabs and how many to spawn this episode.
// Count is sampled uniformly in [minCount, maxCount] at episode start.
[Serializable]
public class SpawnEntry
{
    [Tooltip("Human-readable label shown in the Inspector and sweep log.")]
    public string label = "Pedestrian";

    [Tooltip("Pool of prefabs — one is picked at random per spawn. " +
             "Prefab must have either ErraticAgent or ErraticVehicle.")]
    public GameObject[] prefabs;

    [Tooltip("Minimum number to spawn (inclusive).")]
    [Min(0)] public int minCount = 1;

    [Tooltip("Maximum number to spawn (inclusive).")]
    [Min(0)] public int maxCount = 5;

    [Tooltip("Relative spawn weight when distributing a shared total budget. " +
             "Ignored when totalAgentBudget == 0 (each entry uses its own min/max).")]
    [Min(0f)] public float spawnWeight = 1f;

    // Resolved at runtime by ScenarioManager — not serialized.
    [NonSerialized] public int resolvedCount;
}

[CreateAssetMenu(fileName = "ScenarioConfig", menuName = "Simulation/Scenario Config")]
public class ScenarioConfig : ScriptableObject
{
    // ── Population ──────────────────────────────────────────────────────────
    [Header("Population")]
    [Tooltip("Each entry is one agent group (pedestrians, animals, ambulances, …).")]
    public SpawnEntry[] spawnEntries;

    [Tooltip("Optional shared budget. When > 0 each entry's count is scaled proportionally " +
             "to its spawnWeight so the total never exceeds this number. " +
             "Set to 0 to use each entry's own minCount/maxCount independently.")]
    [Min(0)] public int totalAgentBudget = 0;

    // ── Spawn area ───────────────────────────────────────────────────────────
    [Header("Spawn area")]
    public float spawnRadius = 20f;
    public Vector3 spawnCenter = Vector3.zero;

    // ── Reaction ─────────────────────────────────────────────────────────────
    [Header("Reaction")]
    public float startleRadius = 12f;
    public float reactionBias = 1f;

    // ── Reproducibility ───────────────────────────────────────────────────────
    [Header("Reproducibility")]
    public int seed = 0;

    // ── Helpers ──────────────────────────────────────────────────────────────

    // Rolls resolvedCount for every entry using the current UnityEngine.Random state.
    // Call this after seeding Random so results are reproducible.
    public void ResolveEntryCounts()
    {
        if (spawnEntries == null) return;

        if (totalAgentBudget > 0)
        {
            float totalWeight = 0f;
            foreach (var e in spawnEntries) totalWeight += Mathf.Max(e.spawnWeight, 0f);

            int remaining = totalAgentBudget;
            for (int i = 0; i < spawnEntries.Length; i++)
            {
                SpawnEntry e = spawnEntries[i];
                if (i == spawnEntries.Length - 1)
                {
                    e.resolvedCount = Mathf.Clamp(remaining, e.minCount, e.maxCount);
                }
                else
                {
                    float share = totalWeight > 0f ? (e.spawnWeight / totalWeight) * totalAgentBudget : 0f;
                    int count = Mathf.RoundToInt(share);
                    e.resolvedCount = Mathf.Clamp(count, e.minCount, e.maxCount);
                    remaining -= e.resolvedCount;
                }
            }
        }
        else
        {
            foreach (var e in spawnEntries)
                e.resolvedCount = UnityEngine.Random.Range(e.minCount, e.maxCount + 1);
        }
    }
}
