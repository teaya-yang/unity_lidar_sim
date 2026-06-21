using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

// Scatters static, UNLABELED clutter props (cones, crates, luggage, barriers, …) each episode.
// Purpose is purely GEOMETRIC: props add occlusion (rays blocked before reaching agents -> fewer
// num_lidar_pts, truncated/split boxes) and distractor returns (point clusters that belong to no
// annotation). Both are realism gaps a clean sim otherwise lacks. Appearance is irrelevant —
// Physics.Raycast only sees colliders, so props MUST have a (mesh) collider to register at all.
//
// Props are deliberately NOT Agent/Vehicle, so GroundTruthPublisher emits no boxes
// for them — their points are unlabeled background by design. Make sure your training setup treats
// unlabeled points as background rather than assuming every point belongs to an annotated object.
//
// Order in the pipeline: run AFTER AgentPlacementRandomizer if you want the ego-exclusion radius to
// apply (it uses manager.minEgoDistance); placement of props is otherwise independent of agents.
public class PropScatterRandomizer : EpisodeRandomizer
{
    [Header("Prop prefabs")]
    [Tooltip("Pool of clutter prefabs — each scattered prop picks one at random. Each prefab should " +
             "have a mesh collider (no collider = invisible to the LiDAR) and must NOT be an " +
             "Agent/Vehicle (those would get labeled as annotation targets).")]
    public GameObject[] propPrefabs;

    [Header("Count")]
    public int minProps = 5;
    public int maxProps = 20;

    [Header("Placement")]
    [Tooltip("How far from the NavMesh a candidate can land and still snap (metres).")]
    public float navMeshSnapRadius = 3f;

    [Tooltip("Attempts to find a valid (NavMesh + ego-clear + non-overlapping) position before " +
             "giving up on a prop. Raise it if you scatter many props in a small radius.")]
    public int maxPlacementAttempts = 15;

    [Tooltip("Props won't spawn closer than this (centre-to-centre, metres) to another prop OR to a " +
             "spawned agent, so nothing stacks. 0 disables the check. Set ~ the prop footprint.")]
    public float minSeparation = 1.5f;

    [Header("Orientation & scale")]
    [Tooltip("Give each prop a random yaw about the up axis.")]
    public bool randomYaw = true;

    [Tooltip("Per-prop uniform scale is drawn from [minScale, maxScale]. Set both to 1 to disable.")]
    public float minScale = 1f;
    public float maxScale = 1f;

    [Header("References")]
    public ScenarioManager manager;

    // Props scattered this episode — destroyed on the next Randomize call (and on teardown).
    readonly List<GameObject> m_Spawned = new();

    void Reset() => manager = GetComponent<ScenarioManager>();

    public override void Randomize(int episodeSeed, int randomizerIndex)
    {
        if (manager == null) manager = GetComponent<ScenarioManager>();
        if (manager == null)
        {
            Debug.LogError("[PropScatterRandomizer] No ScenarioManager assigned.", this);
            return;
        }

        SeedStream(episodeSeed, randomizerIndex);
        ClearProps();

        if (propPrefabs == null || propPrefabs.Length == 0)
        {
            Debug.LogWarning("[PropScatterRandomizer] No propPrefabs assigned — nothing to scatter.", this);
            return;
        }

        int requested = Random.Range(minProps, maxProps + 1);
        int placed = 0;

        // Occupied points to keep clear of: seed with the agents already spawned this episode, then
        // grow as each prop lands, so props avoid both agents and each other.
        var occupied = new List<Vector3>();
        foreach (GameObject agentGO in manager.SpawnedAgents)
            if (agentGO != null) occupied.Add(agentGO.transform.position);

        for (int i = 0; i < requested; i++)
        {
            GameObject prefab = propPrefabs[Random.Range(0, propPrefabs.Length)];
            if (prefab == null) continue;

            WarnIfMislabeledOrInvisible(prefab);

            Vector3 pos = SampleScatterPosition(occupied);
            // NOTE: test the sentinel with float.IsPositiveInfinity, NOT `== Vector3.positiveInfinity`.
            // Vector3's == compares (a-b).sqrMagnitude < eps; for infinity that subtraction is NaN and
            // the check is always false, so a failed placement would be instantiated at (∞,∞,∞).
            if (float.IsPositiveInfinity(pos.x)) continue;   // couldn't place this one — skip it
            occupied.Add(pos);

            Quaternion rot = randomYaw
                ? Quaternion.Euler(0f, Random.Range(0f, 360f), 0f)
                : Quaternion.identity;

            GameObject go = Object.Instantiate(prefab, pos, rot);
            go.name = $"Prop_{i}";

            if (maxScale != 1f || minScale != 1f)
                go.transform.localScale *= Random.Range(minScale, maxScale);

            m_Spawned.Add(go);
            placed++;
        }

        Debug.Log($"[PropScatterRandomizer] Scattered {placed}/{requested} props " +
                  $"(requested {minProps}-{maxProps}).");
    }

    // Sample a NavMesh-snapped position near spawnCenter that clears every ego vehicle by
    // manager.minEgoDistance. Mirrors ScenarioManager's agent placement so props share the same
    // play area and never spawn under the wings. Returns positiveInfinity if no spot is found.
    Vector3 SampleScatterPosition(List<Vector3> occupied)
    {
        ScenarioConfig config = manager.config;
        if (config == null) return Vector3.positiveInfinity;

        for (int attempt = 0; attempt < maxPlacementAttempts; attempt++)
        {
            Vector3 candidate = config.spawnCenter + new Vector3(
                Random.Range(-config.spawnRadius, config.spawnRadius), 0f,
                Random.Range(-config.spawnRadius, config.spawnRadius));

            if (!NavMesh.SamplePosition(candidate, out NavMeshHit hit, navMeshSnapRadius, NavMesh.AllAreas))
                continue;

            if (IsTooCloseToEgo(hit.position))
                continue;

            // Reject stacks: NavMesh snapping can collapse candidates onto the same point, and
            // nothing separates overlapping props at spawn (no Rigidbody sim).
            if (IsTooClose(hit.position, occupied))
                continue;

            return hit.position;
        }
        return Vector3.positiveInfinity;
    }

    bool IsTooClose(Vector3 pos, List<Vector3> occupied)
    {
        if (minSeparation <= 0f) return false;
        foreach (Vector3 o in occupied)
            if (Vector3.Distance(pos, o) < minSeparation)
                return true;
        return false;
    }

    bool IsTooCloseToEgo(Vector3 pos)
    {
        if (manager.egoVehicles == null) return false;
        foreach (Transform ego in manager.egoVehicles)
        {
            if (ego == null) continue;
            if (Vector3.Distance(pos, ego.position) < manager.minEgoDistance)
                return true;
        }
        return false;
    }

    // One-time-ish sanity check per scatter: a collider-less prop is invisible to the raycast LiDAR,
    // and an Agent/Vehicle prop would be picked up by GroundTruthPublisher as a label.
    static void WarnIfMislabeledOrInvisible(GameObject prefab)
    {
        if (prefab.GetComponentInChildren<Collider>() == null)
            Debug.LogWarning($"[PropScatterRandomizer] Prop '{prefab.name}' has no Collider — the LiDAR " +
                             "raycast cannot see it. Add a mesh collider or it contributes nothing.", prefab);

        if (prefab.GetComponentInChildren<Agent>() != null ||
            prefab.GetComponentInChildren<Vehicle>() != null)
            Debug.LogWarning($"[PropScatterRandomizer] Prop '{prefab.name}' is an Agent/Vehicle — " +
                             "it will be emitted as a labeled annotation, not unlabeled clutter.", prefab);
    }

    void ClearProps()
    {
        foreach (GameObject go in m_Spawned)
            if (go != null) Destroy(go);
        m_Spawned.Clear();
    }

    void OnDestroy() => ClearProps();
}
