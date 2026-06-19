using System.Collections.Generic;
using UnityEngine;

// Directed lane segment on the airport apron / taxiway network.
// Mirrors AWSIM's TrafficLane structure (waypoints + next/prev graph + speed limit)
// without road-specific fields (traffic lights, right-of-way, intersection flag).
//
// Wire NextLanes and PrevLanes in the Inspector to form the taxiway graph.
// RandomApronSimulator traverses NextLanes randomly; RouteApronSimulator follows
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
    }
}
