using UnityEngine;

public class WaypointPatrol : MonoBehaviour
{
    public Transform[] waypoints;
    public float speed = 6f;
    public float waypointReachedDistance = 1f;

    public enum LoopMode { Loop, PingPong, Once }
    public LoopMode loopMode = LoopMode.Loop;

    int m_Index;
    int m_Direction = 1;
    bool m_Done;

    void Update()
    {
        if (m_Done || waypoints == null || waypoints.Length == 0) return;

        Transform target = waypoints[m_Index];
        if (target == null) { Advance(); return; }

        transform.position = Vector3.MoveTowards(
            transform.position, target.position, speed * Time.deltaTime);

        Vector3 dir = target.position - transform.position;
        if (dir.sqrMagnitude > 0.001f)
            transform.rotation = Quaternion.LookRotation(dir);

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
