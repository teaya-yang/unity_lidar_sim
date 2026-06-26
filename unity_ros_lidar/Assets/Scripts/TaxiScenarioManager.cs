using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Centralized per-episode scenario configurator.
///
/// SETUP:
///   - Assign one incursionPrefab (IncursionAgentController + Rigidbody + Collider).
///   - Assign one or more conflictPoints (empty GameObjects at taxiway intersections).
///   - Set maxAgents. The manager pre-instantiates that many copies in Awake().
///
/// Each episode, active agents are each assigned a conflict point from the list
/// (round-robin or random). Their spawn position is computed so they arrive at
/// their assigned conflict point at (egoTtc + dtOffset).
///
/// Difficulty mapping
/// ──────────────────
///   [0.00 – 0.33]  1 agent   Straight
///   [0.33 – 0.55]  1 agent   StopGo
///   [0.55 – 0.70]  2 agents  Straight + StopGo
///   [0.70 – 0.85]  2 agents  Straight + Curved
///   [0.85 – 1.00]  3 agents  Straight + Curved + Erratic
/// </summary>
public class TaxiScenarioManager : MonoBehaviour
{
    [Header("Prefab — assign ONE ambulance/vehicle prefab")]
    [Tooltip("Must have IncursionAgentController, a kinematic Rigidbody, and a Collider.")]
    public GameObject incursionPrefab;

    [Tooltip("Maximum agents ever active simultaneously. This many copies are pre-instantiated.")]
    public int maxAgents = 3;

    [Header("Conflict points — one per taxiway intersection")]
    [Tooltip("Add one empty GameObject per intersection. Each active agent is assigned one. " +
             "If fewer points than agents, assignment wraps round-robin.")]
    public List<Transform> conflictPoints = new List<Transform>();

    [Header("Defaults (overridden by Python side-channel per episode)")]
    public float defaultDifficulty     = 0.0f;
    public float defaultIncursionDt    = 0.0f;
    public float defaultAmbulanceSpeed = 5.0f;

    [Header("Spawn randomisation")]
    [Tooltip("Max Z-axis jitter applied to each conflict point this episode [m].")]
    public float conflictZJitter = 10f;
    [Tooltip("Randomly flip crossing direction (left vs right) each episode.")]
    public bool  randomiseCrossDirection = true;
    [Tooltip("Lateral offset between agents assigned to the same conflict point [m].")]
    public float lateralSpread = 3f;
    [Tooltip("Speed variation between agents (fraction of base speed).")]
    public float speedVariation = 0.05f;

    // ── public read-only ───────────────────────────────────────────────────────
    public IReadOnlyList<IncursionAgentController> ActiveAgents => _active;

    // ── internal ───────────────────────────────────────────────────────────────
    readonly List<IncursionAgentController> _pool   = new List<IncursionAgentController>();
    readonly List<IncursionAgentController> _active = new List<IncursionAgentController>();

    // ── Awake: pre-instantiate pool ────────────────────────────────────────────

    void Awake()
    {
        if (incursionPrefab == null)
        {
            Debug.LogError("[TaxiScenarioManager] incursionPrefab not assigned.", this);
            return;
        }
        if (conflictPoints == null || conflictPoints.Count == 0)
        {
            Debug.LogError("[TaxiScenarioManager] No conflictPoints assigned.", this);
            return;
        }

        for (int i = 0; i < maxAgents; i++)
        {
            GameObject go = Instantiate(incursionPrefab, Vector3.zero, Quaternion.identity, transform);
            go.name = $"IncursionAgent_{i}";
            go.SetActive(false);

            var ctrl = go.GetComponent<IncursionAgentController>();
            if (ctrl == null)
            {
                Debug.LogError("[TaxiScenarioManager] Prefab missing IncursionAgentController.", this);
                Destroy(go); continue;
            }
            _pool.Add(ctrl);
        }

        Debug.Log($"[TaxiScenarioManager] Pool ready: {_pool.Count} agents, " +
                  $"{conflictPoints.Count} conflict point(s).");
    }

    // ── per-episode reset ──────────────────────────────────────────────────────

    /// <summary>
    /// Called by TaxiAgent.OnEpisodeBegin each episode.
    ///
    /// Each active agent is assigned a conflict point from the list in order:
    ///   agent 0 → conflictPoints[0], agent 1 → conflictPoints[1], …
    /// wrapping round-robin if there are fewer points than agents.
    ///
    /// conflictZOffset — per-episode Z shift applied to ALL conflict points [m].
    ///                   NaN → uniform random within conflictZJitter.
    /// crossDirSign    — +1/-1 to force crossing direction; 0 → random.
    /// </summary>
    public void ResetEpisode(float difficulty,
                             float aircraftSpeed,
                             Transform aircraftTransform,
                             float baseDt,
                             float ambulanceSpeed,
                             float conflictZOffset = float.NaN,
                             float crossDirSign    = 0f)
    {
        _active.Clear();

        if (_pool.Count == 0 || conflictPoints == null || conflictPoints.Count == 0)
        {
            Debug.LogWarning("[TaxiScenarioManager] Pool or conflict points missing.");
            return;
        }

        foreach (var a in _pool)
            a.gameObject.SetActive(false);

        // ── Global Z jitter (same offset for all conflict points this episode) ─
        float zOffset = float.IsNaN(conflictZOffset)
            ? Random.Range(-conflictZJitter, conflictZJitter)
            : conflictZOffset;

        // ── Crossing direction ────────────────────────────────────────────────
        float dirSign = crossDirSign != 0f
            ? Mathf.Sign(crossDirSign)
            : (randomiseCrossDirection && Random.value < 0.5f ? -1f : 1f);

        // ── Layout ────────────────────────────────────────────────────────────
        int nActive;
        TrajectoryMode[] modes;
        ResolveLayout(difficulty, _pool.Count, out nActive, out modes);

        for (int i = 0; i < nActive; i++)
        {
            var agent = _pool[i];
            agent.gameObject.SetActive(true);

            // ── Assign conflict point (round-robin across the list) ────────────
            Transform cp            = conflictPoints[i % conflictPoints.Count];
            Vector3   effectiveCP   = cp.position + new Vector3(0f, 0f, zOffset);
            agent.conflictPoint     = cp;   // StopGo mode uses this

            // ── TTC from aircraft spawn to this agent's conflict point ─────────
            float egoTtc = Mathf.Max(0f,
                (effectiveCP.z - aircraftTransform.position.z) / Mathf.Max(1f, aircraftSpeed));

            float dtOffset     = baseDt + i * 1.5f;
            float spd          = ambulanceSpeed * (1f + i * speedVariation);
            float agentDirSign = (i % 2 == 0) ? dirSign : -dirSign;

            Vector3 baseDir    = agent.CrossDirectionNormalized * agentDirSign;
            Vector3 lateralAxis = Vector3.Cross(baseDir, Vector3.up).normalized;
            float   lateralOff  = (i - (nActive - 1) * 0.5f) * lateralSpread;

            Vector3 start = effectiveCP - baseDir * spd * (egoTtc + dtOffset)
                          + lateralAxis * lateralOff;

            agent.crossDirection = baseDir;
            agent.trajectoryMode = modes[i];
            agent.ResetCrossing(start, spd);
            _active.Add(agent);

            Debug.Log($"[TaxiScenarioManager] Agent {i}: mode={modes[i]} " +
                      $"cp='{cp.name}' egoTtc={egoTtc:F1}s dtOff={dtOffset:+.1f}s " +
                      $"spd={spd:F1} dir={agentDirSign:+0}");
        }

        Debug.Log($"[TaxiScenarioManager] Episode reset: diff={difficulty:F2} " +
                  $"n={nActive} zOff={zOffset:+.1f}m dir={dirSign:+0}");
    }

    // ── difficulty → layout ────────────────────────────────────────────────────

    static void ResolveLayout(float d, int poolSize,
                              out int nActive, out TrajectoryMode[] modes)
    {
        if (d < 0.33f)
        {
            nActive = 1; modes = new[] { TrajectoryMode.Straight };
        }
        else if (d < 0.55f)
        {
            nActive = 1; modes = new[] { TrajectoryMode.StopGo };
        }
        else if (d < 0.70f)
        {
            nActive = 2; modes = new[] { TrajectoryMode.Straight, TrajectoryMode.StopGo };
        }
        else if (d < 0.85f)
        {
            nActive = 2; modes = new[] { TrajectoryMode.Straight, TrajectoryMode.Curved };
        }
        else
        {
            nActive = 3; modes = new[] { TrajectoryMode.Straight, TrajectoryMode.Curved, TrajectoryMode.Erratic };
        }

        nActive = Mathf.Min(nActive, poolSize);
        if (modes.Length > nActive)
        {
            var t = new TrajectoryMode[nActive];
            System.Array.Copy(modes, t, nActive); modes = t;
        }
    }

    // ── Gizmos ────────────────────────────────────────────────────────────────

    void OnDrawGizmosSelected()
    {
        if (conflictPoints == null) return;
        for (int i = 0; i < conflictPoints.Count; i++)
        {
            if (conflictPoints[i] == null) continue;
            // Each conflict point gets a distinct colour
            Gizmos.color = Color.HSVToRGB(i / (float)Mathf.Max(conflictPoints.Count, 1), 0.9f, 1f);
            Gizmos.DrawWireSphere(conflictPoints[i].position, 6f);
            Gizmos.DrawLine(conflictPoints[i].position - Vector3.right * 40f,
                            conflictPoints[i].position + Vector3.right * 40f);
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(conflictPoints[i].position - Vector3.forward * conflictZJitter,
                            conflictPoints[i].position + Vector3.forward * conflictZJitter);
        }
    }
}
