using UnityEngine;

// Shared spawn utilities used by all IApronSimulator implementations.
// Mirrors AWSIM's NpcVehicleSpawner.IsSpawnable / Spawn pattern but
// decoupled from any specific simulator type.
public static class ApronSpawner
{
    // Returns true if localBounds (in the prefab's local space) placed at position
    // with the given forward direction does not overlap any existing colliders.
    // Raises the test box 1 m above ground (same offset as AWSIM) to avoid false
    // positives from the ground plane.
    public static bool IsSpawnable(Bounds localBounds, Vector3 position, Vector3 forward)
    {
        if (forward == Vector3.zero) forward = Vector3.forward;
        var rot    = Quaternion.LookRotation(forward);
        var center = rot * localBounds.center + position + Vector3.up;
        return !Physics.CheckBox(center, localBounds.extents, rot);
    }

    // Instantiate prefab at the first waypoint of lane, oriented along lane direction.
    // Returns the spawned GameObject, parented under parent (keeps scene hierarchy clean).
    public static GameObject SpawnOnLane(GameObject prefab, TaxiwayLane lane, Transform parent)
    {
        var pos = lane.Waypoints[0];
        var fwd = lane.ForwardAtWaypoint(0);
        var go  = Object.Instantiate(prefab, pos, Quaternion.LookRotation(fwd));
        if (parent != null) go.transform.SetParent(parent, worldPositionStays: true);
        return go;
    }

    // Instantiate prefab at an arbitrary world position with given rotation.
    // Used by NavMeshApronSimulator which samples positions, not lanes.
    public static GameObject SpawnAtPosition(GameObject prefab, Vector3 pos, Quaternion rot, Transform parent)
    {
        var go = Object.Instantiate(prefab, pos, rot);
        if (parent != null) go.transform.SetParent(parent, worldPositionStays: true);
        return go;
    }

    // Returns the combined world-space Bounds of all Renderers on go and its children,
    // or a 1x1x1 unit cube at go's position when no renderers are found.
    public static Bounds GetRendererBounds(GameObject go)
    {
        Renderer[] rs = go.GetComponentsInChildren<Renderer>();
        if (rs.Length == 0) return new Bounds(go.transform.position, Vector3.one);
        Bounds b = rs[0].bounds;
        for (int i = 1; i < rs.Length; i++) b.Encapsulate(rs[i].bounds);
        return b;
    }
}
