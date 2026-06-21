using System.Collections.Generic;
using UnityEngine;

// Top-level traffic orchestrator — replaces ScenarioManager + EpisodeSweepRunner.
// Attach one instance to a scene GameObject. Wire ego vehicles, then populate
// randomSims / routeSims / navMeshSims in the Inspector.
public class TrafficManager : MonoBehaviour
{
    [Header("Ego")]
    [Tooltip("Ego aircraft transforms. Passed to every NPC for startle/reaction wiring. " +
             "Also used to keep NPCs from spawning too close.")]
    public Transform[] egoVehicles;

    [Header("Population")]
    [SerializeField, Tooltip("RNG seed — same seed reproduces the same traffic pattern.")]
    int _seed = 0;

    [SerializeField, Tooltip("Hard cap on simultaneously active NPCs across all simulators.")]
    int _maxNpcCount = 40;

    [SerializeField, Tooltip("Don't spawn NPCs closer than this to the nearest ego vehicle (m).")]
    float _spawnDistanceToEgo = 50f;

    [Header("Culling")]
    [SerializeField, Tooltip("NPCs farther than this from the nearest ego are disabled (m). " +
                             "They re-enable automatically when the ego approaches again.")]
    float _cullingDistance = 100f;

    [SerializeField, Tooltip("How many times per second to run the cull check. " +
                             "Lower values cost less CPU; 2 Hz is sufficient for most scenes.")]
    float _cullingHz = 2f;

    [Header("Simulators")]
    [SerializeField] RandomTrafficSimulatorConfig[]  _randomSims   = new RandomTrafficSimulatorConfig[0];
    [SerializeField] RouteTrafficSimulatorConfig[]   _routeSims    = new RouteTrafficSimulatorConfig[0];
    [SerializeField] NavMeshTrafficSimulatorConfig[] _navMeshSims  = new NavMeshTrafficSimulatorConfig[0];

    // Runtime state
    readonly List<ITrafficSimulator> _simulators = new();
    readonly List<GameObject>      _activeNpcs = new();
    Transform                      _npcRoot;       // scene-hierarchy parent for spawned NPCs
    float                          _cullTimer;

    // Expose for GroundTruthPublisher or external tooling.
    public IReadOnlyList<GameObject> ActiveNpcs => _activeNpcs;
    public int Seed => _seed;

    // ── Unity messages ────────────────────────────────────────────────────────

    void Start()
    {
        _npcRoot = new GameObject("NPCs").transform;
        _npcRoot.SetParent(transform, worldPositionStays: false);
        Initialize(_seed);
    }

    void FixedUpdate()
    {
        ManageSpawning();
        Despawn();
    }

    void Update()
    {
        _cullTimer -= Time.deltaTime;
        if (_cullTimer > 0f) return;
        _cullTimer = 1f / Mathf.Max(_cullingHz, 0.1f);
        Cull();
    }

    // ── Public API ────────────────────────────────────────────────────────────

    // Build all simulators and seed the RNG.
    // Call Restart() to re-run with a new seed without reloading the scene.
    public void Initialize(int seed)
    {
        _seed = seed;
        Random.InitState(seed);
        _simulators.Clear();

        foreach (var cfg in _randomSims)
        {
            if (cfg == null) continue;
            if (!cfg.enabled)                         { Debug.Log("[TrafficManager] RandomSim skipped: disabled."); continue; }
            if (cfg.prefabs == null || cfg.prefabs.Length == 0) { Debug.LogWarning("[TrafficManager] RandomSim skipped: no prefabs assigned."); continue; }
            if (cfg.spawnableLanes == null || cfg.spawnableLanes.Length == 0) { Debug.LogWarning("[TrafficManager] RandomSim skipped: no spawnableLanes assigned. Drag TaxiwayLane objects into the list."); continue; }
            _simulators.Add(new RandomTrafficSimulator(cfg.prefabs, cfg.spawnableLanes, cfg.spawnCountLimit, cfg.spawnClearance));
        }

        foreach (var cfg in _routeSims)
        {
            if (cfg == null) continue;
            if (!cfg.enabled)                    { Debug.Log("[TrafficManager] RouteSim skipped: disabled."); continue; }
            if (cfg.prefabs == null || cfg.prefabs.Length == 0) { Debug.LogWarning("[TrafficManager] RouteSim skipped: no prefabs assigned."); continue; }
            if (cfg.route == null || cfg.route.Length == 0) { Debug.LogWarning("[TrafficManager] RouteSim skipped: no route assigned."); continue; }
            _simulators.Add(new RouteTrafficSimulator(cfg.prefabs, cfg.route, cfg.spawnCountLimit, cfg.egoExclusionRadius, cfg.spawnClearance));
        }

        foreach (var cfg in _navMeshSims)
        {
            if (cfg == null) continue;
            if (!cfg.enabled)                    { Debug.Log("[TrafficManager] NavMeshSim skipped: disabled."); continue; }
            if (cfg.prefabs == null || cfg.prefabs.Length == 0) { Debug.LogWarning("[TrafficManager] NavMeshSim skipped: no prefabs assigned."); continue; }
            _simulators.Add(new NavMeshTrafficSimulator(cfg.prefabs, cfg.spawnCenter, cfg.spawnRadius, cfg.spawnCountLimit, cfg.patrolLane));
        }

        if (_simulators.Count == 0)
            Debug.LogWarning("[TrafficManager] No simulators built — nothing will spawn. " +
                             "Assign prefabs and TaxiwayLanes to at least one entry in Random Sims, Route Sims, or Nav Mesh Sims.", this);

        Debug.Log($"[TrafficManager] Initialized seed={seed} | simulators={_simulators.Count} | maxNpc={_maxNpcCount}");
    }

    // Tear down all NPCs and re-initialize with a new seed.
    // Equivalent to EpisodeSweepRunner.ResetEpisode() but without the multi-config sweep.
    public void Restart(int newSeed = 0)
    {
        ClearAllNpcs();
        Initialize(newSeed);
        Debug.Log($"[TrafficManager] Restarted with seed={newSeed}");
    }

    // ── Spawn / despawn / cull ────────────────────────────────────────────────

    void ManageSpawning()
    {
        if (_activeNpcs.Count >= _maxNpcCount) return;

        foreach (var sim in _simulators)
        {
            if (!sim.IsEnabled()) continue;
            if (_activeNpcs.Count >= _maxNpcCount) break;

            if (sim.TrySpawn(egoVehicles, _activeNpcs.Count, _maxNpcCount, _npcRoot, out var npc))
            {
                _activeNpcs.Add(npc);
                Debug.Log($"[TrafficManager] Spawned '{npc.name}' | total={_activeNpcs.Count}");
            }
        }
    }

    void Despawn()
    {
        for (int i = _activeNpcs.Count - 1; i >= 0; i--)
        {
            var go = _activeNpcs[i];
            if (go == null) { _activeNpcs.RemoveAt(i); continue; }

            var npc = go.GetComponent<INpc>();
            if (npc != null && npc.ShouldDespawn)
            {
                Destroy(go);
                _activeNpcs.RemoveAt(i);
            }
        }
    }

    void Cull()
    {
        Vector3 egoPos = NearestEgoPosition();
        foreach (var go in _activeNpcs)
        {
            if (go == null) continue;
            go.SetActive(Vector3.Distance(go.transform.position, egoPos) <= _cullingDistance);
        }
    }

    void ClearAllNpcs()
    {
        foreach (var go in _activeNpcs)
            if (go != null) Destroy(go);
        _activeNpcs.Clear();
    }

    Vector3 NearestEgoPosition()
    {
        if (egoVehicles == null || egoVehicles.Length == 0) return Vector3.zero;
        return egoVehicles[0] != null ? egoVehicles[0].position : Vector3.zero;
    }

    // ── Editor ────────────────────────────────────────────────────────────────

    void OnDrawGizmos()
    {
        if (egoVehicles == null) return;
        Gizmos.color = new Color(0f, 1f, 0f, 0.15f);
        foreach (var ego in egoVehicles)
            if (ego != null)
                Gizmos.DrawWireSphere(ego.position, _spawnDistanceToEgo);
    }
}

