using System;
using UnityEngine;

// Inspector-serializable config block — drag into TrafficManager.routeSims[].
[Serializable]
public class RouteTrafficSimulatorConfig
{
    [Tooltip("Disable without removing from the manager's list.")]
    public bool enabled = true;

    [Tooltip("Pool of NPC prefabs. Must have ErraticVehicle + INpc.")]
    public GameObject[] prefabs;

    [Tooltip("Ordered lane sequence. The first lane is the spawn point. " +
             "Vehicle follows the route then continues randomly from the last lane's NextLanes.")]
    public TaxiwayLane[] route;

    [Tooltip("Maximum NPCs this simulator will ever spawn. 0 = unlimited.")]
    [Min(0)] public int spawnCountLimit;

    [Tooltip("Don't spawn if the route's first waypoint is within this distance of an ego vehicle (m). " +
             "Set to 0 to allow spawning right next to the ego — useful for close-range occlusion scenarios.")]
    [Min(0f)] public float egoExclusionRadius = 50f;

    [Tooltip("Don't spawn if another vehicle is already within this distance of the entry waypoint (m). " +
             "Prevents vehicles stacking at the spawn point and gridlocking car-following. " +
             "Set roughly to the prefab's stopDistance or a bit more.")]
    [Min(0f)] public float spawnClearance = 6f;
}

// Spawns vehicles that follow a fixed ordered lane sequence, then continue randomly
// once the route ends. Mirrors AWSIM's RouteTrafficSimulator.
// Use this for deterministic paths: ambulance route, fuel truck circuit, follow-me car.
public class RouteTrafficSimulator : ITrafficSimulator
{
    readonly GameObject[]  _prefabs;
    readonly TaxiwayLane[] _route;
    readonly int           _limit;
    readonly float         _egoExclusionRadius;
    readonly float         _spawnClearance;
    int                    _spawnCount;

    public RouteTrafficSimulator(GameObject[] prefabs, TaxiwayLane[] route, int limit = 0,
                                 float egoExclusionRadius = 50f, float spawnClearance = 6f)
    {
        _prefabs            = prefabs;
        _route              = route;
        _limit              = limit;
        _egoExclusionRadius = egoExclusionRadius;
        _spawnClearance     = spawnClearance;
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
        {
            UnityEngine.Debug.LogWarning("[RouteTrafficSimulator] Spawn blocked: route[0] is null or has no waypoints. " +
                                         "Assign a TaxiwayLane (with waypoints) as the first element of the route.");
            return false;
        }

        if (IsTooCloseToEgo(spawnLane.Waypoints[0], egoVehicles))
        {
            UnityEngine.Debug.Log($"[RouteTrafficSimulator] Spawn blocked: route[0] first waypoint is within " +
                                  $"{_egoExclusionRadius} m of an ego vehicle. Lower 'Ego Exclusion Radius' on " +
                                  "this Route Sim config (0 = spawn right next to the ego).");
            return false;
        }

        // Hold the spawn until the entry waypoint is clear — otherwise a new vehicle stacks
        // on top of the previous one and the car-following logic gridlocks them both.
        if (_spawnClearance > 0f && ErraticVehicle.AnyVehicleWithin(spawnLane.Waypoints[0], _spawnClearance))
            return false;

        // NOTE: Physics.CheckBox is intentionally skipped here — same reason as RandomTrafficSimulator.
        // The airport ground mesh collider causes IsSpawnable() to reject every position.
        // ErraticVehicle's separation force resolves any brief overlap at spawn time.
        GameObject prefab = _prefabs[UnityEngine.Random.Range(0, _prefabs.Length)];
        spawnedNpc = NpcSpawner.SpawnOnLane(prefab, spawnLane, parent);

        // Wire the fixed route so ErraticVehicle follows it before going random.
        var vehicle = spawnedNpc.GetComponent<ErraticVehicle>();
        if (vehicle != null) vehicle.fixedRoute = _route;

        spawnedNpc.GetComponent<INpc>()?.OnNpcInitialize(spawnLane, egoVehicles);

        _spawnCount++;
        return true;
    }

    bool IsTooCloseToEgo(Vector3 pos, Transform[] egos)
    {
        if (egos == null || _egoExclusionRadius <= 0f) return false;
        foreach (var ego in egos)
            if (ego != null && Vector3.Distance(pos, ego.position) < _egoExclusionRadius) return true;
        return false;
    }
}
