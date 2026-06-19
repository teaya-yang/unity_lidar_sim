using System;
using UnityEngine;

// Inspector-serializable config block — drag into ApronTrafficManager.randomSims[].
[Serializable]
public class RandomApronSimulatorConfig
{
    [Tooltip("Disable without removing from the manager's list.")]
    public bool enabled = true;

    [Tooltip("Pool of NPC prefabs. Must have ErraticVehicle + IApronNpc. One is picked at random per spawn.")]
    public GameObject[] prefabs;

    [Tooltip("Lanes where NPCs may spawn. Vehicles are placed at the lane's first waypoint.")]
    public TaxiwayLane[] spawnableLanes;

    [Tooltip("Maximum NPCs this simulator will ever spawn. 0 = unlimited.")]
    [Min(0)] public int spawnCountLimit;
}

// Spawns ground vehicles on random TaxiwayLanes. After spawning, ErraticVehicle
// traverses the lane graph autonomously (picking random NextLanes at each junction).
// Mirrors AWSIM's RandomTrafficSimulator without the Lanelet2 / traffic-light dependencies.
public class RandomApronSimulator : IApronSimulator
{
    readonly GameObject[]   _prefabs;
    readonly TaxiwayLane[]  _lanes;
    readonly int            _limit;
    int                     _spawnCount;

    // Prefab is locked between TryGetSpawnInfo and Spawn to avoid the size-race
    // condition described in AWSIM's RandomTrafficSimulator (smaller vehicles always
    // winning the bounds check over larger ones on the same lane).
    GameObject _pendingPrefab;

    public RandomApronSimulator(GameObject[] prefabs, TaxiwayLane[] spawnableLanes, int limit = 0)
    {
        _prefabs = prefabs;
        _lanes   = spawnableLanes;
        _limit   = limit;
    }

    public bool IsEnabled() =>
        _prefabs != null && _prefabs.Length > 0 &&
        _lanes   != null && _lanes.Length   > 0 &&
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

        // Lock a prefab for this attempt if we don't have one yet.
        _pendingPrefab ??= _prefabs[UnityEngine.Random.Range(0, _prefabs.Length)];

        // Pick a random spawnable lane.
        TaxiwayLane lane = _lanes[UnityEngine.Random.Range(0, _lanes.Length)];
        if (lane == null || lane.Waypoints == null || lane.Waypoints.Length == 0)
        {
            UnityEngine.Debug.LogWarning("[RandomApronSimulator] Picked a lane with no waypoints — assign waypoints in the TaxiwayLane Inspector.");
            return false;
        }

        // Reject if too close to any ego vehicle (distance passed from ApronTrafficManager).
        if (IsTooCloseToEgo(lane.Waypoints[0], egoVehicles))
        {
            UnityEngine.Debug.Log("[RandomApronSimulator] Spawn blocked: lane waypoint is within ego exclusion radius.");
            return false;
        }

        // NOTE: Physics.CheckBox is intentionally skipped here.
        // On an open airport apron the ground mesh collider causes every spawn to fail.
        // ErraticVehicle's separation force resolves any brief overlap at spawn time.

        spawnedNpc = ApronSpawner.SpawnOnLane(_pendingPrefab, lane, parent);
        spawnedNpc.GetComponent<IApronNpc>()?.OnApronInitialize(lane, egoVehicles);

        _pendingPrefab = null;
        _spawnCount++;
        return true;
    }

    static bool IsTooCloseToEgo(Vector3 pos, Transform[] egos)
    {
        if (egos == null) return false;
        foreach (var ego in egos)
            if (ego != null && Vector3.Distance(pos, ego.position) < 20f) return true;
        return false;
    }
}
