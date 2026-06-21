using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(Vehicle))]
public class VehicleEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        var v = (Vehicle)target;
        if (!Application.isPlaying) return;

        EditorGUILayout.Space(8);
        EditorGUILayout.LabelField("── Runtime State ──", EditorStyles.boldLabel);

        // Routing mode with colour coding
        Color prev = GUI.color;
        GUI.color = v.CurrentRoutingMode switch
        {
            Vehicle.RoutingMode.FixedRoute  => Color.cyan,
            Vehicle.RoutingMode.RandomLane  => Color.yellow,
            _                                       => Color.white,
        };
        EditorGUILayout.LabelField("Routing Mode", v.CurrentRoutingMode.ToString());
        GUI.color = prev;

        EditorGUILayout.LabelField("Current Lane",    v.CurrentLaneName);
        EditorGUILayout.LabelField("Waypoint Index",  v.WaypointIndex.ToString());
        EditorGUILayout.LabelField("Stopped",         v.IsStopped.ToString());
        EditorGUILayout.LabelField("Yielding",        v.IsYielding.ToString());

        // Force repaint so values update every frame while selected
        if (Application.isPlaying) Repaint();
    }
}
