using System;
using UnityEngine;

/// <summary>
/// Spawns a handful of prefabs at random positions around the ego vehicle on Start.
/// No NavMesh, no ROS, no MAPF required.
/// </summary>
public class SimpleVehicleSpawner : MonoBehaviour
{
    [Header("Ego")]
    public Transform egoVehicle;        // drag Airplane (1) here

    [Header("Prefabs")]
    public GameObject[] prefabs;        // drag any prefab(s) here

    [Header("Spawn settings")]
    public int   count  = 6;
    public float radius = 60f;          // metres around ego
    public int   seed   = 42;

    void Start()
    {
        if (prefabs == null || prefabs.Length == 0) 
        {
            Debug.LogWarning("[SimpleVehicleSpawner] No prefabs assigned.");
            return;
        }

        Vector3 centre = egoVehicle != null ? egoVehicle.position : Vector3.zero;
        var rng = new System.Random(seed);

        for (int i = 0; i < count; i++)
        {
            var prefab = prefabs[rng.Next(prefabs.Length)];
            if (prefab == null) continue;

            float angle = (float)(rng.NextDouble() * 2 * Math.PI);
            float dist  = (float)(rng.NextDouble() * radius);
            Vector3 pos = centre + new Vector3(
                Mathf.Cos(angle) * dist, 0f, Mathf.Sin(angle) * dist);

            float yRot = (float)(rng.NextDouble() * 360f);
            Instantiate(prefab, pos, Quaternion.Euler(0f, yRot, 0f));
        }
    }
}
