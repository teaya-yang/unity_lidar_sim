using System.Collections.Generic;
using UnityEngine;

// Place this on an empty GameObject with a BoxCollider covering the sidewalk/curb.
// Assign crossTarget to a Transform on the opposite side of the street.
// Detection is by POSITION (not physics overlap): every scanInterval the zone finds all
// ErraticAgents whose position lies inside the box and rolls them to cross. This works
// regardless of whether agents have colliders/Rigidbodies, and catches agents already
// standing inside the zone at spawn.
[RequireComponent(typeof(BoxCollider))]
public class StreetCrossingZone : MonoBehaviour
{
    [Tooltip("Point on the far side of the street agents will walk toward")]
    public Transform crossTarget;

    [Range(0f, 1f)]
    [Tooltip("Probability that an agent found in the zone will actually cross")]
    public float crossProbability = 0.55f;

    [Tooltip("Seconds between scans for agents inside the zone")]
    public float scanInterval = 0.5f;

    [Tooltip("Extra vertical tolerance (m) added to the box height when testing agents")]
    public float heightTolerance = 2f;

    BoxCollider m_Box;
    float m_NextScan;
    // remember who we've already handled so each agent only rolls once per visit
    readonly HashSet<ErraticAgent> m_Seen = new();

    void Awake()
    {
        m_Box = GetComponent<BoxCollider>();
        m_Box.isTrigger = true;
    }

    void Update()
    {
        if (crossTarget == null) return;
        if (Time.time < m_NextScan) return;
        m_NextScan = Time.time + scanInterval;
        ScanZone();
    }

    void ScanZone()
    {
        ErraticAgent[] agents = Object.FindObjectsByType<ErraticAgent>(FindObjectsSortMode.None);

        int insideCount = 0;
        var stillInside = new HashSet<ErraticAgent>();
        foreach (ErraticAgent agent in agents)
        {
            if (!IsInside(agent.transform.position)) continue;
            insideCount++;

            stillInside.Add(agent);
            if (m_Seen.Contains(agent)) continue;   // already rolled this visit
            m_Seen.Add(agent);

            if (Random.value > crossProbability) continue;

            Debug.Log($"{agent} is goint to the target position");
            agent.TriggerCrossing(crossTarget.position);
        }

        // TEMP diagnostic — how many agents exist and how many are inside the box
        Debug.Log($"[StreetCrossingZone '{name}'] {agents.Length} agents in scene, " +
                  $"{insideCount} inside the zone.", this);

        // TEMP diagnostic — for the first agent, show its local pos vs the box half-extents.
        // |local| must be <= half on every axis (Y gets +heightTolerance) to count as inside.
        if (insideCount == 0 && agents.Length > 0)
        {
            Vector3 local = transform.InverseTransformPoint(agents[0].transform.position) - m_Box.center;
            Vector3 half  = m_Box.size * 0.5f;
            Debug.Log($"  '{agents[0].name}' local={local} | box half={half} " +
                      $"(Y limit {half.y + heightTolerance}). " +
                      $"X ok:{Mathf.Abs(local.x) <= half.x} " +
                      $"Y ok:{Mathf.Abs(local.y) <= half.y + heightTolerance} " +
                      $"Z ok:{Mathf.Abs(local.z) <= half.z}", this);
        }

        // forget agents that have left so they can roll again next time they return
        m_Seen.IntersectWith(stillInside);
    }

    // Point-in-box test in the box's local space, with vertical tolerance so an agent
    // whose pivot is at its feet still counts against a thin ground-level box.
    bool IsInside(Vector3 worldPos)
    {
        Vector3 local = transform.InverseTransformPoint(worldPos) - m_Box.center;
        Vector3 half = m_Box.size * 0.5f;
        return Mathf.Abs(local.x) <= half.x
            && Mathf.Abs(local.z) <= half.z
            && Mathf.Abs(local.y) <= half.y + heightTolerance;
    }

#if UNITY_EDITOR
    void OnDrawGizmos()
    {
        var box = GetComponent<BoxCollider>();
        if (box != null)
        {
            Gizmos.color = new Color(0f, 0.6f, 1f, 0.25f);
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.DrawCube(box.center, box.size);
            Gizmos.matrix = Matrix4x4.identity;
        }
        if (crossTarget == null) return;
        Gizmos.color = new Color(1f, 0.4f, 0f, 0.5f);
        Gizmos.DrawLine(transform.position, crossTarget.position);
        Gizmos.DrawSphere(crossTarget.position, 0.4f);
    }
#endif
}
