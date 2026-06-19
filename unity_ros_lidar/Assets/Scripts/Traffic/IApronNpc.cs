using UnityEngine;

// Implemented by every NPC that ApronTrafficManager can spawn, cull, and despawn.
// Concrete implementations: ErraticVehicle (lane-based), ErraticAgent (NavMesh-based).
//
// The manager calls OnApronInitialize() once immediately after Instantiate(), before
// the first Update() fires, so implementations can treat it as a late constructor.
public interface IApronNpc
{
    // True → ApronTrafficManager will Destroy this NPC on the next despawn pass.
    // Set in concrete class when the vehicle leaves the map or completes a limited route.
    bool ShouldDespawn { get; }

    // World-space AABB used by ApronSpawner.IsSpawnable() to reject overlapping spawns.
    Bounds Bounds { get; }

    // Called by ApronTrafficManager after Instantiate().
    // startLane: initial lane for vehicle NPCs; null for NavMesh pedestrian NPCs.
    // egoVehicles: ego aircraft transforms to wire into startle/reaction logic.
    void OnApronInitialize(TaxiwayLane startLane, Transform[] egoVehicles);
}
