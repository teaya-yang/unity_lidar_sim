using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Scenario types pushed by Python via "scenario_type" side channel.
/// </summary>
public enum ScenarioType
{
    Standard    = 0,   // difficulty-based layout, perpendicular crossing
    Stationary  = 1,   // one agent placed on the taxiway, never moves
    HeadOn      = 2,   // agents travel along taxiway axis (±Z) toward airplane
    HighSpeed   = 3,   // standard layout, airplane desired_speed pushed separately
    Accelerating= 4,   // agents start slow and accelerate into the conflict zone
}

/// <summary>
/// Centralized per-episode scenario configurator.
///
/// SETUP:
///   - Assign one incursionPrefab (IncursionAgentController + Rigidbody + Collider).
///   - Assign one or more conflictPoints (empty GameObjects at taxiway intersections).
///   - Set maxAgents (copies pre-instantiated at Awake).
///
/// Python pushes "scenario_type" (int cast to float) each episode to select the
/// active scenario. Within each type, "difficulty" further controls agent count/modes.
/// </summary>
public class TaxiScenarioManager : MonoBehaviour
{
    [Header("Prefab")]
    public GameObject incursionPrefab;
    public int        maxAgents = 3;

    [Header("Conflict points — one per taxiway intersection")]
    public List<Transform> conflictPoints = new List<Transform>();

    [Header("Defaults")]
    public float defaultDifficulty     = 0.0f;
    public float defaultIncursionDt    = 0.0f;
    public float defaultAmbulanceSpeed = 5.0f;

    [Header("Spawn randomisation")]
    public float conflictZJitter         = 10f;
    public bool  randomiseCrossDirection = true;
    public float lateralSpread           = 3f;
    public float speedVariation          = 0.05f;

    // ── public read-only ───────────────────────────────────────────────────────
    public IReadOnlyList<IncursionAgentController> ActiveAgents => _active;

    // ── internal ───────────────────────────────────────────────────────────────
    readonly List<IncursionAgentController> _pool   = new List<IncursionAgentController>();
    readonly List<IncursionAgentController> _active = new List<IncursionAgentController>();

    // ── Awake ──────────────────────────────────────────────────────────────────

    void Awake()
    {
        if (incursionPrefab == null || conflictPoints == null || conflictPoints.Count == 0)
        {
            Debug.LogError("[TaxiScenarioManager] incursionPrefab or conflictPoints not assigned.", this);
            return;
        }
        for (int i = 0; i < maxAgents; i++)
        {
            var go   = Instantiate(incursionPrefab, Vector3.zero, Quaternion.identity, transform);
            go.name  = $"IncursionAgent_{i}";
            go.SetActive(false);
            var ctrl = go.GetComponent<IncursionAgentController>();
            if (ctrl == null) { Destroy(go); continue; }
            _pool.Add(ctrl);
        }
        Debug.Log($"[TaxiScenarioManager] {_pool.Count} agents, {conflictPoints.Count} conflict point(s).");
    }

    // ── per-episode reset ──────────────────────────────────────────────────────

    public void ResetEpisode(float difficulty,
                             float aircraftSpeed,
                             Transform aircraftTransform,
                             float baseDt,
                             float ambulanceSpeed,
                             float conflictZOffset  = float.NaN,
                             float crossDirSign     = 0f,
                             int   scenarioType     = 0,
                             float headOnProb       = 0f)
    {
        _active.Clear();
        foreach (var a in _pool) a.gameObject.SetActive(false);

        if (_pool.Count == 0 || conflictPoints == null || conflictPoints.Count == 0) return;

        // ── Z jitter ──────────────────────────────────────────────────────────
        float zOffset = float.IsNaN(conflictZOffset)
            ? Random.Range(-conflictZJitter, conflictZJitter)
            : conflictZOffset;

        // ── Crossing direction ─────────────────────────────────────────────────
        float dirSign = crossDirSign != 0f
            ? Mathf.Sign(crossDirSign)
            : (randomiseCrossDirection && Random.value < 0.5f ? -1f : 1f);

        // ── Layout from scenario type ──────────────────────────────────────────
        int              nActive;
        TrajectoryMode[] modes;
        ResolveLayout((ScenarioType)scenarioType, difficulty, _pool.Count, out nActive, out modes);

        for (int i = 0; i < nActive; i++)
        {
            var agent = _pool[i];
            agent.gameObject.SetActive(true);

            // Assign conflict point round-robin
            Transform cp          = conflictPoints[i % conflictPoints.Count];
            Vector3   effectiveCP = cp.position + new Vector3(0f, 0f, zOffset);
            agent.conflictPoint   = cp;

            float egoTtc = Mathf.Max(0f,
                (effectiveCP.z - aircraftTransform.position.z) / Mathf.Max(1f, aircraftSpeed));

            float dtOffset = baseDt + i * 1.5f;
            float spd      = ambulanceSpeed * (1f + i * speedVariation);

            // ── Head-on scenario: some agents travel along taxiway axis (±Z) ──
            Vector3 baseDir;
            bool isHeadOn = (ScenarioType)scenarioType == ScenarioType.HeadOn
                            || Random.value < headOnProb;

            if (isHeadOn)
            {
                // Head-on: travel along -Z (toward airplane) or +Z (away)
                baseDir = dirSign < 0f ? -Vector3.forward : Vector3.forward;
            }
            else
            {
                float agentSign = (i % 2 == 0) ? dirSign : -dirSign;
                baseDir = agent.CrossDirectionNormalized * agentSign;
            }

            // ── Stationary: place directly on taxiway centreline ───────────────
            Vector3 start;
            if (modes[i] == TrajectoryMode.Stationary)
            {
                // Place the stationary obstacle at the effective conflict point
                start = effectiveCP;
            }
            else
            {
                Vector3 lateralAxis = Vector3.Cross(baseDir, Vector3.up).normalized;
                float   lateralOff  = (i - (nActive - 1) * 0.5f) * lateralSpread;
                start = effectiveCP - baseDir * spd * (egoTtc + dtOffset)
                      + lateralAxis * lateralOff;
            }

            agent.crossDirection = baseDir;
            agent.trajectoryMode = modes[i];
            agent.ResetCrossing(start, spd);
            _active.Add(agent);

            Debug.Log($"[TaxiScenarioManager] Agent {i}: {modes[i]} cp='{cp.name}' " +
                      $"headOn={isHeadOn} spd={spd:F1} dtOff={dtOffset:+.1f}");
        }

        Debug.Log($"[TaxiScenarioManager] reset type={(ScenarioType)scenarioType} " +
                  $"diff={difficulty:F2} n={nActive} zOff={zOffset:+.1f}");
    }

    // ── layout table ──────────────────────────────────────────────────────────

    static void ResolveLayout(ScenarioType type, float d, int poolSize,
                              out int nActive, out TrajectoryMode[] modes)
    {
        switch (type)
        {
            case ScenarioType.Stationary:
                nActive = 1;
                modes   = new[] { TrajectoryMode.Stationary };
                break;

            case ScenarioType.HeadOn:
                // Head-on: always Straight mode, direction handled in ResetEpisode
                nActive = 1;
                modes   = new[] { TrajectoryMode.Straight };
                break;

            case ScenarioType.Accelerating:
                nActive = d < 0.5f ? 1 : 2;
                modes   = nActive == 1
                    ? new[] { TrajectoryMode.Accelerating }
                    : new[] { TrajectoryMode.Accelerating, TrajectoryMode.Straight };
                break;

            case ScenarioType.HighSpeed:
            case ScenarioType.Standard:
            default:
                // Standard difficulty ramp
                if (d < 0.33f)      { nActive = 1; modes = new[] { TrajectoryMode.Straight }; }
                else if (d < 0.55f) { nActive = 1; modes = new[] { TrajectoryMode.StopGo }; }
                else if (d < 0.70f) { nActive = 2; modes = new[] { TrajectoryMode.Straight, TrajectoryMode.StopGo }; }
                else if (d < 0.85f) { nActive = 2; modes = new[] { TrajectoryMode.Straight, TrajectoryMode.Curved }; }
                else                { nActive = 3; modes = new[] { TrajectoryMode.Straight, TrajectoryMode.Curved, TrajectoryMode.Erratic }; }
                break;
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
