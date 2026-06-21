using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(EgoRouteFollower))]
public class EgoRouteFollowerEditor : Editor
{
    SerializedProperty _routeProp;
    SerializedProperty _loopProp;
    SerializedProperty _speedProp;
    SerializedProperty _rotationSpeedProp;
    SerializedProperty _waypointDistProp;
    SerializedProperty _showGizmosProp;

    int _selectedLaneIndex = -1;

    static readonly Color[] k_LaneColors =
    {
        new Color(0.3f, 0.7f, 1f),   // blue
        new Color(0.3f, 1f, 0.5f),   // green
        new Color(1f, 0.8f, 0.2f),   // yellow
        new Color(1f, 0.4f, 0.8f),   // pink
        new Color(0.6f, 0.4f, 1f),   // purple
        new Color(1f, 0.55f, 0.2f),  // orange
    };

    void OnEnable()
    {
        _routeProp        = serializedObject.FindProperty("route");
        _loopProp         = serializedObject.FindProperty("loop");
        _speedProp        = serializedObject.FindProperty("speed");
        _rotationSpeedProp= serializedObject.FindProperty("rotationSpeed");
        _waypointDistProp = serializedObject.FindProperty("waypointReachedDistance");
        _showGizmosProp   = serializedObject.FindProperty("showGizmos");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        EditorGUILayout.PropertyField(_speedProp);
        EditorGUILayout.PropertyField(_rotationSpeedProp);
        EditorGUILayout.PropertyField(_waypointDistProp);
        EditorGUILayout.PropertyField(_loopProp);
        EditorGUILayout.PropertyField(_showGizmosProp);

        EditorGUILayout.Space(8);
        EditorGUILayout.LabelField("Route", EditorStyles.boldLabel);

        int newSelected = _selectedLaneIndex;

        for (int i = 0; i < _routeProp.arraySize; i++)
        {
            SerializedProperty elem = _routeProp.GetArrayElementAtIndex(i);

            bool isSelected = (i == _selectedLaneIndex);
            Color bg = GUI.backgroundColor;
            if (isSelected) GUI.backgroundColor = new Color(0.5f, 0.8f, 1f);

            EditorGUILayout.BeginHorizontal();

            // Click the index button to select / deselect the lane for highlighting.
            GUIStyle indexStyle = new GUIStyle(GUI.skin.button)
            {
                fontStyle = isSelected ? FontStyle.Bold : FontStyle.Normal,
                normal    = { textColor = LaneColor(i) }
            };
            if (GUILayout.Button($"[{i}]", indexStyle, GUILayout.Width(32)))
                newSelected = (isSelected ? -1 : i);

            EditorGUILayout.PropertyField(elem, GUIContent.none);

            if (GUILayout.Button("✕", GUILayout.Width(22)))
            {
                _routeProp.DeleteArrayElementAtIndex(i);
                if (_selectedLaneIndex >= _routeProp.arraySize) newSelected = -1;
                GUI.backgroundColor = bg;
                EditorGUILayout.EndHorizontal();
                break;
            }

            GUI.backgroundColor = bg;
            EditorGUILayout.EndHorizontal();
        }

        _selectedLaneIndex = newSelected;

        EditorGUILayout.Space(2);
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Add Lane"))
        {
            int idx = _routeProp.arraySize;
            _routeProp.InsertArrayElementAtIndex(idx);
            _routeProp.GetArrayElementAtIndex(idx).objectReferenceValue = null;
            _selectedLaneIndex = idx;
        }
        GUI.enabled = _routeProp.arraySize > 0;
        if (GUILayout.Button("Remove Last"))
        {
            _routeProp.DeleteArrayElementAtIndex(_routeProp.arraySize - 1);
            if (_selectedLaneIndex >= _routeProp.arraySize) _selectedLaneIndex = -1;
        }
        GUI.enabled = true;
        EditorGUILayout.EndHorizontal();

        serializedObject.ApplyModifiedProperties();

        // Repaint the scene whenever inspector state changes so highlights update instantly.
        SceneView.RepaintAll();
    }

    void OnSceneGUI()
    {
        var follower = (EgoRouteFollower)target;
        if (follower.route == null || follower.route.Length == 0) return;

        for (int i = 0; i < follower.route.Length; i++)
        {
            TaxiwayLane lane = follower.route[i];
            if (lane == null || lane.Waypoints == null || lane.Waypoints.Length == 0) continue;

            bool highlighted = (i == _selectedLaneIndex);
            Color col = LaneColor(i);
            col.a = highlighted ? 1f : 0.5f;
            Handles.color = col;

            float thickness = highlighted ? 4f : 2f;

            // Draw waypoint segments.
            for (int w = 0; w < lane.Waypoints.Length - 1; w++)
                Handles.DrawLine(lane.Waypoints[w], lane.Waypoints[w + 1], thickness);

            // Spheres at each waypoint.
            float sphereSize = highlighted ? 0.6f : 0.35f;
            foreach (Vector3 wp in lane.Waypoints)
                Handles.SphereHandleCap(0, wp, Quaternion.identity, sphereSize, EventType.Repaint);

            // Label above first waypoint — always show lane index; show name when highlighted.
            string label = highlighted
                ? $"[{i}] {lane.name}"
                : $"[{i}]";

            GUIStyle style = new GUIStyle
            {
                normal    = { textColor = highlighted ? Color.white : col },
                fontStyle = highlighted ? FontStyle.Bold : FontStyle.Normal,
                fontSize  = highlighted ? 13 : 11,
            };
            Handles.Label(lane.Waypoints[0] + Vector3.up * (highlighted ? 2f : 1.2f), label, style);

            // Arrow from end of lane[i] to start of lane[i+1] showing the sequence order.
            if (i < follower.route.Length - 1)
            {
                TaxiwayLane next = follower.route[i + 1];
                if (next != null && next.Waypoints != null && next.Waypoints.Length > 0)
                {
                    Handles.color = new Color(col.r, col.g, col.b, 0.4f);
                    Vector3 from = lane.Waypoints[^1];
                    Vector3 to   = next.Waypoints[0];
                    Handles.DrawDottedLine(from, to, 4f);
                }
            }
        }
    }

    static Color LaneColor(int index) => k_LaneColors[index % k_LaneColors.Length];
}
