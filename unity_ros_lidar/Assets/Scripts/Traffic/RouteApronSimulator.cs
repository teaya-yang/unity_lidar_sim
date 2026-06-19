using System;
using UnityEngine;

// Inspector-serializable config block — drag into ApronTrafficManager.routeSims[].
[Serializable]
public class RouteApronSimulatorConfig
{
    [Tooltip("Disable without removing from the manager's list.")]
    public bool enabled = true;

    [Tooltip("Pool of NPC prefabs. Must have ErraticVehicle + IApronNpc.")]
    public GameObject[] prefabs;

    [Tooltip("Ordered lane sequence. The first lane is the spawn point. " +
             "Vehicle follows the route then continues randomly from the last lane's NextLanes.")]
    public TaxiwayLane[] route;

    [Tooltip("Maximum NPCs this simulator will ever spawn. 0 = unlimited.")]
    [Min(0)] public int spawnCountLimit;
}

// Spawns vehicles that follow a fixed ordered lane sequence, then continue randomly
// once the route ends. Mirrors AWSIM's RouteTrafficSimulator.
// Use this for deterministic paths: ambulance route, fuel truck circuit, follow-me car.
public class RouteApronSimulator : IApronSimulator
{
    readonly GameObject[]  _prefabs;
    readonly TaxiwayLane[] _route;
    readonly int           _limit;
    int                    _spawnCount;

    public RouteApronSimulator(GameObject[] prefabs, TaxiwayLane[] route, int limit = 0)
    {
        _prefabs = prefabs;
        _route   = route;
        _limit   = limit;
    }

    public bool IsEnabled() =>
        _prefabs != null && _prefabs.Length > 0 &&
        _route   != null && _route.Length   > 0 &&
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

        TaxiwayLane spawnLane = _route[0];
        if (spawnLane == null || spawnLane.Waypoints == null || spawnLane.Waypoints.Length == 0)
            return false;

        if (IsTooCloseToEgo(spawnLane.Waypoints[0], egoVehicles)) return false;

        GameObject prefab = _prefabs[UnityEngine.Random.Range(0, _prefabs.Length)];
        var bounds      = ApronSpawner.GetRendererBounds(prefab);
        var localBounds = new Bounds(Vector3.zero, bounds.size);
        if (!ApronSpawner.IsSpawnable(localBounds, spawnLane.Waypoints[0], spawnLane.ForwardAtWaypoint(0)))
            return false;

        spawnedNpc = ApronSpawner.SpawnOnLane(prefab, spawnLane, parent);

        // Wire the fixed route so ErraticVehicle follows it before going random.
        var vehicle = spawnedNpc.GetComponent<ErraticVehicle>();
        if (vehicle != null) vehicle.fixedRoute = _route;

        spawnedNpc.GetComponent<IApronNpc>()?.OnApronInitialize(spawnLane, egoVehicles);

        _spawnCount++;
        return true;
    }

    static bool IsTooCloseToEgo(Vector3 pos, Transform[] egos)
    {
        if (egos == null) return false;
        foreach (var ego in egos)
            if (ego != null && Vector3.Distance(pos, ego.position) < 50f) return true;
        return false;
    }
}
