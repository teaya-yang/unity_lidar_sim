using System.Collections.Generic;
using UnityEngine;

// Directed lane segment on the airport apron / taxiway network.
// Mirrors AWSIM's TrafficLane structure (waypoints + next/prev graph + speed limit)
// without road-specific fields (traffic lights, right-of-way, intersection flag).
//
// Wire NextLanes and PrevLanes in the Inspector to form the taxiway graph.
// RandomTrafficSimulator traverses NextLanes randomly; RouteTrafficSimulator follows
// a fixed ordered sequence supplied at spawn time.
public class TaxiwayLane : MonoBehaviour
{
    public Vector3[] Waypoints    => _waypoints;
    public List<TaxiwayLane> NextLanes => _nextLanes;
    public List<TaxiwayLane> PrevLanes => _prevLanes;
    public float SpeedLimit       => _speedLimit;

    // When true, vehicles entering this lane stop before proceeding until the
    // ego aircraft has cleared the area (runway crossing, apron give-way point).
    public bool IsHoldingPosition => _isHoldingPosition;

    // Lanes that have priority over this one at a shared junction (AWSIM's RightOfWayLanes).
    // A vehicle on this lane stops at its final waypoint until these lanes are clear of nearby
    // traffic. Empty = a priority lane that never yields.
    public List<TaxiwayLane> YieldToLanes => _yieldToLanes;
    public bool HasYieldRule => _yieldToLanes != null && _yieldToLanes.Count > 0;

    [Tooltip("World-space waypoints through this lane segment, in traversal order.")]
    [SerializeField] Vector3[] _waypoints;

    [Tooltip("Outgoing connected lanes. Vehicle picks one at random (or follows fixedRoute).")]
    [SerializeField] List<TaxiwayLane> _nextLanes = new();

    [Tooltip("Incoming connected lanes. Used for graph validation only — not read at runtime.")]
    [SerializeField] List<TaxiwayLane> _prevLanes = new();

    [Tooltip("Maximum speed on this segment (m/s). Vehicles clamp their speed to this value on entry.")]
    [SerializeField] float _speedLimit = 5f;

    [Tooltip("If true, vehicles stop at the last waypoint until the ego aircraft moves away. " +
             "Use for runway crossings and apron give-way points.")]
    [SerializeField] bool _isHoldingPosition;

    [Tooltip("Lanes that have priority over this one at a junction. A vehicle on this lane holds " +
             "at its final waypoint until these lanes are clear of nearby traffic. " +
             "Leave empty for priority lanes that never yield.")]
    [SerializeField] List<TaxiwayLane> _yieldToLanes = new();

    // Programmatic construction used by tooling / tests.
    public static TaxiwayLane Create(Vector3[] waypoints, float speedLimit = 5f, string laneNameOverride = "")
    {
        string n = string.IsNullOrEmpty(laneNameOverride) ? "TaxiwayLane" : laneNameOverride;
        var go = new GameObject(n, typeof(TaxiwayLane));
        go.transform.position = waypoints[0];
        var lane = go.GetComponent<TaxiwayLane>();
        lane._waypoints  = waypoints;
        lane._speedLimit = speedLimit;
        return lane;
    }

    // Returns the world-space forward direction at waypoint index i.
    public Vector3 ForwardAtWaypoint(int i)
    {
        if (_waypoints == null || _waypoints.Length < 2) return Vector3.forward;
        if (i < _waypoints.Length - 1) return (_waypoints[i + 1] - _waypoints[i]).normalized;
        return (_waypoints[i] - _waypoints[i - 1]).normalized;
    }

    void OnDrawGizmosSelected()
    {
        if (_waypoints == null || _waypoints.Length == 0) return;

        Gizmos.color = _isHoldingPosition ? Color.red : Color.yellow;

        for (int i = 0; i < _waypoints.Length; i++)
        {
            Gizmos.DrawSphere(_waypoints[i], 0.3f);
            if (i < _waypoints.Length - 1)
                Gizmos.DrawLine(_waypoints[i], _waypoints[i + 1]);
        }

        Gizmos.color = Color.green;
        foreach (var next in _nextLanes)
        {
            if (next != null && next.Waypoints != null && next.Waypoints.Length > 0)
                Gizmos.DrawLine(_waypoints[^1], next.Waypoints[0]);
        }

        // Yield-to gizmos: stop-bar sphere at this lane's hold point + orange lines to priority lanes.
        if (_yieldToLanes != null && _yieldToLanes.Count > 0)
        {
            Color orange = new Color(1f, 0.55f, 0f);
            Vector3 stopPoint = _waypoints[^1];

            // Filled stop-bar sphere at the hold waypoint.
            Gizmos.color = orange;
            Gizmos.DrawSphere(stopPoint, 0.55f);

            // Orange wire-sphere showing the conflict-detection radius (matches ErraticVehicle default).
            Gizmos.color = new Color(1f, 0.55f, 0f, 0.25f);
            Gizmos.DrawWireSphere(stopPoint, 20f);

            // Lines to each priority lane.
            Gizmos.color = orange;
            foreach (var yieldTo in _yieldToLanes)
            {
                if (yieldTo == null || yieldTo.Waypoints == null || yieldTo.Waypoints.Length == 0) continue;
                Gizmos.DrawLine(stopPoint, yieldTo.Waypoints[^1]);
                // Small sphere at the far end to mark the priority lane's last waypoint.
                Gizmos.DrawWireSphere(yieldTo.Waypoints[^1], 0.4f);
            }
        }
    }
}
