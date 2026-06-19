using UnityEngine;

// Implemented by RandomApronSimulator, RouteApronSimulator, and NavMeshApronSimulator.
// ApronTrafficManager holds a list of these and calls TrySpawn() each FixedUpdate.
//
// Each simulator owns its full spawn logic (prefab selection, position sampling,
// bounds checking, route wiring) so the manager stays thin and simulators are
// independently testable and swappable.
public interface IApronSimulator
{
    // False → manager skips this simulator entirely (disabled flag or spawn limit reached).
    bool IsEnabled();

    // Attempt to instantiate one NPC. Returns true and sets spawnedNpc on success.
    // egoVehicles: passed through to IApronNpc.OnApronInitialize after spawn.
    // currentCount / maxCount: manager's global budget enforcement.
    // parent: transform under which spawned objects are parented for scene hygiene.
    bool TrySpawn(
        Transform[]   egoVehicles,
        int           currentCount,
        int           maxCount,
        Transform     parent,
        out GameObject spawnedNpc);
}
