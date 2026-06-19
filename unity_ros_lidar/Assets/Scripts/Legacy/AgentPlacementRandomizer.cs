using UnityEngine;

// places the episode's agents. Thin by design — it reseeds its own
// RNG stream, then delegates to ScenarioManager.SpawnAllAgents(), which owns the scene
// references (prefab, ego/emergency wiring, the active ScenarioConfig).
//
// Use this file as the template for the next randomizers (heading, scale, prop-scatter,
// sensor-jitter): subclass EpisodeRandomizer, call SeedStream() first, then act on
// manager.SpawnedAgents.
public class AgentPlacementRandomizer : EpisodeRandomizer
{
    [Tooltip("Scenario whose agents this places. Defaults to a ScenarioManager on the same GameObject.")]
    public ScenarioManager manager;

    void Reset() => manager = GetComponent<ScenarioManager>();

    public override void Randomize(int episodeSeed, int randomizerIndex)
    {
        if (manager == null) manager = GetComponent<ScenarioManager>();
        if (manager == null)
        {
            Debug.LogError("[AgentPlacementRandomizer] No ScenarioManager assigned.", this);
            return;
        }

        SeedStream(episodeSeed, randomizerIndex);
        manager.SpawnAllAgents();
    }
}
