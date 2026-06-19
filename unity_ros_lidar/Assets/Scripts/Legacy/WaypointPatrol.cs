using UnityEngine;

public class WaypointPatrol : MonoBehaviour
{
    public Transform[] waypoints;
    public float speed = 6f;
    public float waypointReachedDistance = 1f;
    public float rotationSpeed = 2f;

    public enum LoopMode { Loop, PingPong, Once }
    public LoopMode loopMode = LoopMode.Loop;

    int m_Index;
    int m_Direction = 1;
    bool m_Done;

    void Start()
    {
        if (waypoints == null || waypoints.Length == 0) return;
        float best = float.MaxValue;
        for (int i = 0; i < waypoints.Length; i++)
        {
            if (waypoints[i] == null) continue;
            float d = Vector3.Distance(transform.position, waypoints[i].position);
            if (d < best) { best = d; m_Index = i; }
        }
    }

    void Update()
    {
        if (m_Done || waypoints == null || waypoints.Length == 0) return;

        Transform target = waypoints[m_Index];
        if (target == null) { Advance(); return; }

        transform.position = Vector3.MoveTowards(
            transform.position, target.position, speed * Time.deltaTime);

        Vector3 dir = target.position - transform.position;
        if (dir.sqrMagnitude > 0.001f)
        {
            Quaternion targetRot = Quaternion.LookRotation(dir);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, rotationSpeed * Time.deltaTime);
        }

        if (Vector3.Distance(transform.position, target.position) < waypointReachedDistance)
            Advance();
    }

    void Advance()
    {
        switch (loopMode)
        {
            case LoopMode.Loop:
                m_Index = (m_Index + 1) % waypoints.Length;
                break;

            case LoopMode.PingPong:
                m_Index += m_Direction;
                if (m_Index >= waypoints.Length - 1 || m_Index <= 0)
                    m_Direction *= -1;
                break;

            case LoopMode.Once:
                m_Index++;
                if (m_Index >= waypoints.Length) m_Done = true;
                break;
        }
    }
}
