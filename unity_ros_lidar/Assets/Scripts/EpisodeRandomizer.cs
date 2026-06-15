using UnityEngine;

// Domain-randomization building block, modeled on Unity Perception's Randomizer/Scenario
// pattern (arXiv:2107.04259) but specialized for geometric / LiDAR randomization.
//
// ScenarioManager acts as the "Scenario": it holds an ordered list of these and calls
// Randomize() on each at the start of every episode. Each randomizer reseeds the global
// UnityEngine.Random from a stream derived from (episodeSeed, its own index), so adding,
// removing, or disabling one randomizer does NOT shift the random draws of the others —
// the property that keeps an old seed reproducible as the pipeline grows.
//
// For a pure raycast LiDAR, only GEOMETRIC and SENSOR randomizers move the needle.
// Appearance randomizers (texture/color/lighting) are pointless — Physics.Raycast can't
// see them. Build placement, heading, scale, prop-scatter, and sensor-jitter; skip the rest.
public interface IEpisodeRandomizer
{
    void Randomize(int episodeSeed, int randomizerIndex);
}

public abstract class EpisodeRandomizer : MonoBehaviour, IEpisodeRandomizer
{
    [Tooltip("Skip this randomizer for the run without removing it from the pipeline " +
             "(keeps every other randomizer's index — and therefore its RNG stream — stable).")]
    public bool enabledInSweep = true;

    public abstract void Randomize(int episodeSeed, int randomizerIndex);

    // Seed the global RNG with an independent per-(episode, randomizer) stream.
    // Call this FIRST in every Randomize() override.
    protected void SeedStream(int episodeSeed, int randomizerIndex)
    {
        Random.InitState(Mix(episodeSeed, randomizerIndex));
    }

    // Deterministic integer mix (FNV-1a style). Stable across runs and platforms, unlike
    // string.GetHashCode which is randomized per-process on some .NET runtimes.
    static int Mix(int a, int b)
    {
        unchecked
        {
            uint h = 2166136261u;
            h = (h ^ (uint)a) * 16777619u;
            h = (h ^ (uint)b) * 16777619u;
            return (int)h;
        }
    }
}
