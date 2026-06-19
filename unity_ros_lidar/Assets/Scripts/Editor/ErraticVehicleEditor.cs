using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(ErraticVehicle))]
public class ErraticVehicleEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        var v = (ErraticVehicle)target;
        if (!Application.isPlaying) return;

        EditorGUILayout.Space(8);
        EditorGUILayout.LabelField("── Runtime State ──", EditorStyles.boldLabel);

        // Routing mode with colour coding
        Color prev = GUI.color;
        GUI.color = v.CurrentRoutingMode switch
        {
            ErraticVehicle.RoutingMode.FixedRoute  => Color.cyan,
            ErraticVehicle.RoutingMode.RandomLane  => Color.yellow,
            _                                       => Color.white,
        };
        EditorGUILayout.LabelField("Routing Mode", v.CurrentRoutingMode.ToString());
        GUI.color = prev;

        EditorGUILayout.LabelField("Current Lane",    v.CurrentLaneName);
        EditorGUILayout.LabelField("Waypoint Index",  v.WaypointIndex.ToString());
        EditorGUILayout.LabelField("Stopped",         v.IsStopped.ToString());
        EditorGUILayout.LabelField("Reacting to Ego", v.IsReacting.ToString());

        // Force repaint so values update every frame while selected
        if (Application.isPlaying) Repaint();
    }
}
