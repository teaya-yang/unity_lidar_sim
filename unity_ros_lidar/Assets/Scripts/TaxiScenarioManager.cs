using System.Collections.Generic;
using System.Linq;
using UnityEngine;

/// <summary>
/// Centralized per-episode scenario configurator.
///
/// Owns a pool of IncursionAgentControllers and activates/positions them each
/// episode based on a difficulty scalar [0, 1] pushed by the Python training loop
/// via EnvironmentParameters ("difficulty").
///
/// Difficulty mapping
/// ──────────────────
///   [0.00 – 0.33]  1 agent   Straight
///   [0.33 – 0.55]  1 agent   StopGo
///   [0.55 – 0.70]  2 agents  Straight + StopGo
///   [0.70 – 0.85]  2 agents  Straight + Curved
///   [0.85 – 1.00]  3 agents  Straight + Curved + Erratic
///
/// Spawn geometry
/// ──────────────
/// The aircraft travels along +Z at aircraftSpeed. Its TTC to the conflict point is
///   ttc_ego = (conflict.z - spawn.z) / aircraftSpeed
/// Each incursion agent i is placed so it reaches the conflict point at
///   ttc_ego + incursionDt_i
/// where incursionDt_i is a per-agent offset set by the Python side channel or
/// defaulted from the Inspector.
///
/// Call ResetEpisode(difficulty, aircraftSpeed, aircraftTransform, incursionDtOffset)
/// from TaxiAgent.OnEpisodeBegin. Then read ActiveAgents for the observation pack.
/// </summary>
public class TaxiScenarioManager : MonoBehaviour
{
    [Header("Agent pool — assign all IncursionAgentControllers in the scene")]
    public List<IncursionAgentController> agentPool = new List<IncursionAgentController>();

    [Header("Scene reference")]
    [Tooltip("Empty GameObject where incursion paths cross the taxiway centreline.")]
    public Transform conflictPoint;

    [Header("Defaults (overridden by Python side-channel per episode)")]
    public float defaultDifficulty    = 0.0f;
    public float defaultIncursionDt   = 0.0f;   // applied to agent[0]; others get ±offset
    public float defaultAmbulanceSpeed = 5.0f;

    // ── read-only state ────────────────────────────────────────────────────────
    public IReadOnlyList<IncursionAgentController> ActiveAgents => _active;

    readonly List<IncursionAgentController> _active = new List<IncursionAgentController>();

    // ── per-episode configuration ──────────────────────────────────────────────

    /// <summary>
    /// Configure and reset all incursion agents for a new episode.
    ///
    /// Parameters
    /// ----------
    /// difficulty      [0, 1] — controls how many agents are active and which
    ///                          trajectory modes are used.
    /// aircraftSpeed   used to compute time-to-conflict for spawn placement.
    /// aircraftTransform  the aircraft's current (reset) transform, used to
    ///                    measure distance to conflictPoint.
    /// baseDt          primary Δt offset [s] for agent[0] (from side channel).
    /// ambulanceSpeed  crossing speed for agent[0]; subsequent agents offset slightly.
    /// </summary>
    public void ResetEpisode(float difficulty,
                             float aircraftSpeed,
                             Transform aircraftTransform,
                             float baseDt,
                             float ambulanceSpeed)
    {
        _active.Clear();

        if (agentPool == null || agentPool.Count == 0 || conflictPoint == null)
        {
            Debug.LogWarning("[ScenarioManager] agentPool is empty or conflictPoint not assigned.");
            return;
        }

        // Disable all agents first
        foreach (var a in agentPool)
            a.gameObject.SetActive(false);

        // Decide how many agents to activate and their modes
        int nActive;
        TrajectoryMode[] modes;
        ResolveLayout(difficulty, agentPool.Count, out nActive, out modes);
        nActive = Mathf.Min(nActive, agentPool.Count);

        float egoTtc = Mathf.Max(0f,
            (conflictPoint.position.z - aircraftTransform.position.z) / Mathf.Max(1f, aircraftSpeed));

        for (int i = 0; i < nActive; i++)
        {
            var agent = agentPool[i];
            agent.gameObject.SetActive(true);

            // Each subsequent agent gets a small additional Δt offset so they
            // stagger through the conflict zone rather than arriving together.
            float dtOffset  = baseDt + i * 1.5f;
            float spd       = ambulanceSpeed * (1f + i * 0.05f); // slight speed variation

            // Spawn position: agent starts far enough back that it reaches
            // conflictPoint at (egoTtc + dtOffset) seconds from now.
            Vector3 dir   = agent.CrossDirectionNormalized;
            Vector3 start = conflictPoint.position - dir * spd * (egoTtc + dtOffset);

            agent.trajectoryMode = modes[i];
            agent.ResetCrossing(start, spd);
            _active.Add(agent);
        }
    }

    // ── difficulty → layout table ──────────────────────────────────────────────

    static void ResolveLayout(float d, int poolSize,
                              out int nActive, out TrajectoryMode[] modes)
    {
        if (d < 0.33f)
        {
            nActive = 1;
            modes   = new[] { TrajectoryMode.Straight };
        }
        else if (d < 0.55f)
        {
            nActive = 1;
            modes   = new[] { TrajectoryMode.StopGo };
        }
        else if (d < 0.70f)
        {
            nActive = 2;
            modes   = new[] { TrajectoryMode.Straight, TrajectoryMode.StopGo };
        }
        else if (d < 0.85f)
        {
            nActive = 2;
            modes   = new[] { TrajectoryMode.Straight, TrajectoryMode.Curved };
        }
        else
        {
            nActive = 3;
            modes   = new[] { TrajectoryMode.Straight, TrajectoryMode.Curved, TrajectoryMode.Erratic };
        }

        // Clamp to actual pool size
        nActive = Mathf.Min(nActive, poolSize);
        if (modes.Length > nActive)
        {
            var trimmed = new TrajectoryMode[nActive];
            System.Array.Copy(modes, trimmed, nActive);
            modes = trimmed;
        }
    }

    // ── Gizmo: draw conflict-point radius ─────────────────────────────────────
    void OnDrawGizmosSelected()
    {
        if (conflictPoint == null) return;
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(conflictPoint.position, 6f);
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(conflictPoint.position - Vector3.right * 30f,
                        conflictPoint.position + Vector3.right * 30f);
    }
}
