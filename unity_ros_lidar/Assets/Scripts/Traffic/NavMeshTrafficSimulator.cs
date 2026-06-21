using System;
using UnityEngine;
using UnityEngine.AI;

// Inspector-serializable config block — drag into TrafficManager.navMeshSims[].
[Serializable]
public class NavMeshTrafficSimulatorConfig
{
    [Tooltip("Disable without removing from the manager's list.")]
    public bool enabled = true;

    [Tooltip("Pool of NPC prefabs. Must have Agent + INpc.")]
    public GameObject[] prefabs;

    [Tooltip("Centre of the area where pedestrians/animals may spawn.")]
    public Vector3 spawnCenter;

    [Tooltip("Radius around spawnCenter to sample spawn positions on the NavMesh.")]
    [Min(1f)] public float spawnRadius = 20f;

    [Tooltip("Maximum NPCs this simulator will ever spawn. 0 = unlimited.")]
    [Min(0)] public int spawnCountLimit;

    [Tooltip("Optional: if set, spawned agents patrol this lane's waypoints instead of wandering freely.")]
    public TaxiwayLane patrolLane;
}

// Spawns NavMesh-based pedestrian/animal agents (Agent) by sampling random
// positions on the baked NavMesh near a spawn centre.
// If patrolLane is provided, agents patrol that lane's waypoints instead of wandering freely.
public class NavMeshTrafficSimulator : ITrafficSimulator
{
    readonly GameObject[] _prefabs;
    readonly Vector3      _center;
    readonly float        _radius;
    readonly int          _limit;
    readonly TaxiwayLane  _patrolLane;
    int                   _spawnCount;

    const int k_MaxSampleAttempts = 10;

    public NavMeshTrafficSimulator(
        GameObject[] prefabs,
        Vector3      spawnCenter,
        float        spawnRadius,
        int          limit = 0,
        TaxiwayLane  patrolLane = null)
    {
        _prefabs    = prefabs;
        _center     = spawnCenter;
        _radius     = spawnRadius;
        _limit      = limit;
        _patrolLane = patrolLane;
    }

    public bool IsEnabled() =>
        _prefabs != null && _prefabs.Length > 0 &&
        (_limit == 0 || _spawnCount < _limit);

    public bool TrySpawn(
        Transform[]    egoVehicles,
        int            currentCount,
        int            maxCount,
        Transform      parent,
        out GameObject spawnedNpc)
    {
        spawnedNpc = null;
        if (!IsEnabled() || currentCount >= maxCount) return false;

        Vector3 pos;
        if (!SampleNavMeshPosition(egoVehicles, out pos)) return false;

        GameObject prefab = _prefabs[UnityEngine.Random.Range(0, _prefabs.Length)];
        spawnedNpc = NpcSpawner.SpawnAtPosition(prefab, pos, Quaternion.identity, parent);
        spawnedNpc.GetComponent<INpc>()?.OnNpcInitialize(_patrolLane, egoVehicles);

        _spawnCount++;
        return true;
    }

    bool SampleNavMeshPosition(Transform[] egoVehicles, out Vector3 result)
    {
        for (int i = 0; i < k_MaxSampleAttempts; i++)
        {
            Vector2 disk   = UnityEngine.Random.insideUnitCircle * _radius;
            Vector3 candidate = _center + new Vector3(disk.x, 0f, disk.y);

            if (!NavMesh.SamplePosition(candidate, out NavMeshHit hit, _radius, NavMesh.AllAreas))
                continue;

            if (IsTooCloseToEgo(hit.position, egoVehicles)) continue;

            result = hit.position;
            return true;
        }

        result = Vector3.zero;
        return false;
    }

    static bool IsTooCloseToEgo(Vector3 pos, Transform[] egos)
    {
        if (egos == null) return false;
        foreach (var ego in egos)
            if (ego != null && Vector3.Distance(pos, ego.position) < 15f) return true;
        return false;
    }
}
