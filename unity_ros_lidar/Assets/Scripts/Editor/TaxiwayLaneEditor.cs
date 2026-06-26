using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

[CustomEditor(typeof(TaxiwayLane))]
public class TaxiwayLaneEditor : Editor
{
    // Serialized properties
    SerializedProperty _waypointsProp;
    SerializedProperty _nextLanesProp;
    SerializedProperty _prevLanesProp;
    SerializedProperty _speedLimitProp;
    SerializedProperty _holdingProp;
    SerializedProperty _yieldToLanesProp;

    // Index of the waypoint currently being dragged (-1 = none)
    int _selectedIndex = -1;

    // Index of the yield-to lane currently selected in the Inspector list (-1 = none)
    int _selectedYieldIndex = -1;

    void OnEnable()
    {
        _waypointsProp  = serializedObject.FindProperty("_waypoints");
        _nextLanesProp  = serializedObject.FindProperty("_nextLanes");
        _prevLanesProp  = serializedObject.FindProperty("_prevLanes");
        _speedLimitProp   = serializedObject.FindProperty("_speedLimit");
        _holdingProp      = serializedObject.FindProperty("_isHoldingPosition");
        _yieldToLanesProp = serializedObject.FindProperty("_yieldToLanes");
    }

    // ── Inspector ─────────────────────────────────────────────────────────────

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        var lane = (TaxiwayLane)target;

        EditorGUILayout.PropertyField(_speedLimitProp);
        EditorGUILayout.PropertyField(_holdingProp);

        EditorGUILayout.Space(6);
        EditorGUILayout.LabelField("Waypoints", EditorStyles.boldLabel);

        // Waypoint list with per-entry Remove button
        for (int i = 0; i < _waypointsProp.arraySize; i++)
        {
            EditorGUILayout.BeginHorizontal();

            SerializedProperty elem = _waypointsProp.GetArrayElementAtIndex(i);
            bool isSelected = (i == _selectedIndex);

            GUI.color = isSelected ? Color.yellow : Color.white;
            EditorGUILayout.PropertyField(elem, new GUIContent($"  [{i}]"));
            GUI.color = Color.white;

            if (GUILayout.Button("✕", GUILayout.Width(22)))
            {
                _waypointsProp.DeleteArrayElementAtIndex(i);
                if (_selectedIndex >= _waypointsProp.arraySize) _selectedIndex = -1;
                break;
            }

            EditorGUILayout.EndHorizontal();
        }

        EditorGUILayout.Space(2);

        EditorGUILayout.BeginHorizontal();

        if (GUILayout.Button("Add Waypoint"))
        {
            // Insert at end, initialise to last waypoint + 5 m forward, or origin if empty.
            int idx = _waypointsProp.arraySize;
            _waypointsProp.InsertArrayElementAtIndex(idx);
            SerializedProperty newElem = _waypointsProp.GetArrayElementAtIndex(idx);

            Vector3 newPos;
            if (idx > 0)
            {
                SerializedProperty prev = _waypointsProp.GetArrayElementAtIndex(idx - 1);
                newPos = prev.vector3Value + lane.transform.forward * 5f;
            }
            else
            {
                newPos = lane.transform.position;
            }
            newElem.vector3Value = newPos;
            _selectedIndex = idx;
        }

        GUI.enabled = _waypointsProp.arraySize > 0;
        if (GUILayout.Button("Remove Last"))
        {
            int last = _waypointsProp.arraySize - 1;
            _waypointsProp.DeleteArrayElementAtIndex(last);
            if (_selectedIndex >= _waypointsProp.arraySize) _selectedIndex = -1;
        }
        GUI.enabled = true;

        EditorGUILayout.EndHorizontal();

        EditorGUILayout.Space(6);
        EditorGUILayout.PropertyField(_nextLanesProp, includeChildren: true);
        EditorGUILayout.PropertyField(_prevLanesProp, includeChildren: true);

        EditorGUILayout.Space(6);
        EditorGUILayout.LabelField("Intersection right-of-way", EditorStyles.boldLabel);

        // Draw yield-to list manually so we can track which entry is selected
        // and highlight that lane larger in the Scene view.
        int yieldCount = _yieldToLanesProp.arraySize;
        for (int i = 0; i < yieldCount; i++)
        {
            EditorGUILayout.BeginHorizontal();

            bool isYieldSelected = (i == _selectedYieldIndex);
            GUI.color = isYieldSelected ? new Color(1f, 0.55f, 0f) : Color.white;
            SerializedProperty elem = _yieldToLanesProp.GetArrayElementAtIndex(i);
            EditorGUILayout.PropertyField(elem, new GUIContent($"  [{i}]"));
            GUI.color = Color.white;

            if (GUILayout.Button(isYieldSelected ? "★" : "☆", GUILayout.Width(26)))
            {
                _selectedYieldIndex = isYieldSelected ? -1 : i;
                SceneView.RepaintAll();
            }

            if (GUILayout.Button("✕", GUILayout.Width(22)))
            {
                _yieldToLanesProp.DeleteArrayElementAtIndex(i);
                if (_selectedYieldIndex >= _yieldToLanesProp.arraySize) _selectedYieldIndex = -1;
                SceneView.RepaintAll();
                break;
            }

            EditorGUILayout.EndHorizontal();
        }

        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Add Yield Lane"))
        {
            _yieldToLanesProp.InsertArrayElementAtIndex(yieldCount);
            _yieldToLanesProp.GetArrayElementAtIndex(yieldCount).objectReferenceValue = null;
        }
        GUI.enabled = yieldCount > 0;
        if (GUILayout.Button("Remove Last"))
        {
            _yieldToLanesProp.DeleteArrayElementAtIndex(yieldCount - 1);
            if (_selectedYieldIndex >= _yieldToLanesProp.arraySize) _selectedYieldIndex = -1;
            SceneView.RepaintAll();
        }
        GUI.enabled = true;
        EditorGUILayout.EndHorizontal();

        serializedObject.ApplyModifiedProperties();
    }

    // ── Scene view ────────────────────────────────────────────────────────────

    void OnSceneGUI()
    {
        var lane = (TaxiwayLane)target;
        if (lane.Waypoints == null || lane.Waypoints.Length == 0) return;

        serializedObject.Update();

        Handles.color = lane.IsHoldingPosition ? Color.red : Color.yellow;

        for (int i = 0; i < lane.Waypoints.Length; i++)
        {
            Vector3 wp = lane.Waypoints[i];

            // Selection disc — click to select a waypoint.
            float discSize = HandleUtility.GetHandleSize(wp) * 0.12f;
            if (Handles.Button(wp, Quaternion.identity, discSize, discSize * 1.5f, Handles.SphereHandleCap))
                _selectedIndex = (i == _selectedIndex) ? -1 : i;

            // Draggable position handle for the selected waypoint.
            if (i == _selectedIndex)
            {
                EditorGUI.BeginChangeCheck();
                Vector3 moved = Handles.PositionHandle(wp, Quaternion.identity);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(lane, "Move TaxiwayLane Waypoint");
                    _waypointsProp.GetArrayElementAtIndex(i).vector3Value = moved;
                    serializedObject.ApplyModifiedProperties();
                }
            }

            // Label
            Handles.Label(wp + Vector3.up * 0.5f,
                $"{lane.name}[{i}]",
                new GUIStyle { normal = { textColor = Color.white }, fontSize = 10 });

            // Segment line to next waypoint
            if (i < lane.Waypoints.Length - 1)
            {
                Handles.color = lane.IsHoldingPosition ? new Color(1f, 0.3f, 0.3f) : Color.yellow;
                DrawArrow(wp, lane.Waypoints[i + 1]);
            }
        }

        // Highlighted yield-to lane (selected via ★ button in the Inspector)
        if (_selectedYieldIndex >= 0 && _selectedYieldIndex < _yieldToLanesProp.arraySize)
        {
            var yieldRef = _yieldToLanesProp.GetArrayElementAtIndex(_selectedYieldIndex).objectReferenceValue as TaxiwayLane;
            if (yieldRef != null && yieldRef.Waypoints != null && yieldRef.Waypoints.Length > 0)
            {
                Color highlight = new Color(1f, 0.55f, 0f);
                Handles.color = highlight;

                for (int i = 0; i < yieldRef.Waypoints.Length; i++)
                {
                    float size = HandleUtility.GetHandleSize(yieldRef.Waypoints[i]) * 0.35f;
                    Handles.SphereHandleCap(0, yieldRef.Waypoints[i], Quaternion.identity, size, EventType.Repaint);

                    if (i < yieldRef.Waypoints.Length - 1)
                    {
                        Handles.DrawLine(yieldRef.Waypoints[i], yieldRef.Waypoints[i + 1], 4f);
                    }

                    Handles.Label(yieldRef.Waypoints[i] + Vector3.up * 1.2f,
                        $"YIELD→ {yieldRef.name}[{i}]",
                        new GUIStyle { normal = { textColor = highlight }, fontSize = 11, fontStyle = FontStyle.Bold });
                }
            }
        }

        // Green arrows to connected NextLanes
        if (lane.NextLanes != null)
        {
            Handles.color = Color.green;
            Vector3 lastWp = lane.Waypoints[^1];
            foreach (var next in lane.NextLanes)
            {
                if (next != null && next.Waypoints != null && next.Waypoints.Length > 0)
                    DrawArrow(lastWp, next.Waypoints[0]);
            }
        }

        serializedObject.ApplyModifiedProperties();
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    static void DrawArrow(Vector3 from, Vector3 to)
    {
        Handles.DrawLine(from, to);

        Vector3 dir  = (to - from).normalized;
        Vector3 mid  = Vector3.Lerp(from, to, 0.7f);
        float   size = HandleUtility.GetHandleSize(mid) * 0.15f;
        Handles.ConeHandleCap(0, mid, Quaternion.LookRotation(dir), size, EventType.Repaint);
    }
}
