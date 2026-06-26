using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Centralized per-episode scenario configurator.
///
/// SETUP: assign one incursionPrefab (a GameObject with IncursionAgentController,
/// Rigidbody, and Collider). The manager pre-instantiates maxAgents copies in Awake()
/// and manages their lifecycle — you never place ambulances manually in the scene.
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
/// Each agent is placed so it reaches effectiveConflict at (egoTtc + dtOffset).
/// The conflict point position is jittered each episode along Z.
/// Crossing direction (left/right) is randomised or controlled by Python side channel.
/// </summary>
public class TaxiScenarioManager : MonoBehaviour
{
    [Header("Prefab — assign ONE ambulance/vehicle prefab")]
    [Tooltip("Must have IncursionAgentController, a Rigidbody (kinematic), and a Collider.")]
    public GameObject incursionPrefab;

    [Tooltip("Maximum number of incursion agents ever active simultaneously. " +
             "This many copies are pre-instantiated at startup.")]
    public int maxAgents = 3;

    [Header("Scene reference")]
    [Tooltip("Empty GameObject where incursion paths cross the taxiway centreline.")]
    public Transform conflictPoint;

    [Header("Defaults (overridden by Python side-channel per episode)")]
    public float defaultDifficulty     = 0.0f;
    public float defaultIncursionDt    = 0.0f;
    public float defaultAmbulanceSpeed = 5.0f;

    [Header("Spawn randomisation")]
    [Tooltip("Max Z-axis shift of the conflict point each episode [m].")]
    public float conflictZJitter = 20f;
    [Tooltip("Randomly flip the crossing direction (left vs right) each episode.")]
    public bool  randomiseCrossDirection = true;
    [Tooltip("Per-agent lateral offset jitter [m] so agents don't stack on the same line.")]
    public float lateralSpread = 3f;
    [Tooltip("Speed variation between agents (fraction of base speed).")]
    public float speedVariation = 0.05f;

    // ── read-only ──────────────────────────────────────────────────────────────
    public IReadOnlyList<IncursionAgentController> ActiveAgents => _active;

    // ── internal pool ──────────────────────────────────────────────────────────
    readonly List<IncursionAgentController> _pool   = new List<IncursionAgentController>();
    readonly List<IncursionAgentController> _active = new List<IncursionAgentController>();

    // ── Unity lifecycle ────────────────────────────────────────────────────────

    void Awake()
    {
        if (incursionPrefab == null)
        {
            Debug.LogError("[TaxiScenarioManager] incursionPrefab is not assigned.", this);
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
                Debug.LogError($"[TaxiScenarioManager] Prefab has no IncursionAgentController.", this);
                Destroy(go);
                continue;
            }

            // Wire the shared conflict point so StopGo mode knows where to stop
            ctrl.conflictPoint = conflictPoint;
            _pool.Add(ctrl);
        }

        Debug.Log($"[TaxiScenarioManager] Pre-instantiated {_pool.Count} incursion agents.");
    }

    // ── per-episode reset ──────────────────────────────────────────────────────

    /// <summary>
    /// Called by TaxiAgent.OnEpisodeBegin. Activates 1-maxAgents agents from the
    /// pool, assigns trajectory modes and spawn positions based on difficulty.
    ///
    /// conflictZOffset — Z shift [m] for this episode (NaN → Unity random).
    /// crossDirSign    — +1/-1 to force crossing direction (0 → Unity random).
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

        if (_pool.Count == 0 || conflictPoint == null)
        {
            Debug.LogWarning("[TaxiScenarioManager] Pool empty or conflictPoint missing.");
            return;
        }

        // Disable all pooled agents
        foreach (var a in _pool)
            a.gameObject.SetActive(false);

        // ── Effective conflict point (Z-jittered) ─────────────────────────────
        float zOffset = float.IsNaN(conflictZOffset)
            ? Random.Range(-conflictZJitter, conflictZJitter)
            : conflictZOffset;
        Vector3 effectiveConflict = conflictPoint.position + new Vector3(0f, 0f, zOffset);

        // Update StopGo conflict reference on all pool agents
        foreach (var a in _pool)
            a.conflictPoint = conflictPoint;   // keep original; offset applied to spawn calc

        // ── Crossing direction ────────────────────────────────────────────────
        float dirSign = crossDirSign != 0f
            ? Mathf.Sign(crossDirSign)
            : (randomiseCrossDirection && Random.value < 0.5f ? -1f : 1f);

        // ── Layout: how many agents and which modes ───────────────────────────
        int nActive;
        TrajectoryMode[] modes;
        ResolveLayout(difficulty, _pool.Count, out nActive, out modes);

        float egoTtc = Mathf.Max(0f,
            (effectiveConflict.z - aircraftTransform.position.z) / Mathf.Max(1f, aircraftSpeed));

        for (int i = 0; i < nActive; i++)
        {
            var agent = _pool[i];
            agent.gameObject.SetActive(true);

            // Stagger Δt so agents arrive at different times
            float dtOffset = baseDt + i * 1.5f;

            // Slight speed variation per agent
            float spd = ambulanceSpeed * (1f + i * speedVariation);

            // Alternate crossing direction per agent (agent 0 goes dirSign, 1 goes -dirSign, …)
            float agentDirSign = (i % 2 == 0) ? dirSign : -dirSign;

            // Base crossing direction from the agent's own crossDirection, flipped by sign
            Vector3 baseDir = agent.CrossDirectionNormalized * agentDirSign;

            // Lateral spread: offset spawn perpendicular to crossing so agents don't stack
            Vector3 lateralAxis = Vector3.Cross(baseDir, Vector3.up).normalized;
            float   lateralOff  = (i - (nActive - 1) * 0.5f) * lateralSpread;

            Vector3 spawnBase = effectiveConflict - baseDir * spd * (egoTtc + dtOffset);
            Vector3 start     = spawnBase + lateralAxis * lateralOff;

            agent.trajectoryMode = modes[i];

            // Override crossDirection on the controller so it matches agentDirSign
            // (the controller normalises it internally)
            agent.crossDirection = baseDir;

            agent.ResetCrossing(start, spd);
            _active.Add(agent);
        }

        Debug.Log($"[TaxiScenarioManager] reset: diff={difficulty:F2} n={nActive} " +
                  $"modes=[{string.Join(",", modes)}] " +
                  $"zOff={zOffset:+.1f} dir={dirSign:+0}");
    }

    // ── difficulty → layout ────────────────────────────────────────────────────

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

        nActive = Mathf.Min(nActive, poolSize);
        if (modes.Length > nActive)
        {
            var trimmed = new TrajectoryMode[nActive];
            System.Array.Copy(modes, trimmed, nActive);
            modes = trimmed;
        }
    }

    // ── Gizmos ────────────────────────────────────────────────────────────────

    void OnDrawGizmosSelected()
    {
        if (conflictPoint == null) return;
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(conflictPoint.position, 6f);
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(conflictPoint.position - Vector3.right * 40f,
                        conflictPoint.position + Vector3.right * 40f);
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(conflictPoint.position - Vector3.forward * conflictZJitter,
                        conflictPoint.position + Vector3.forward * conflictZJitter);
    }
}
