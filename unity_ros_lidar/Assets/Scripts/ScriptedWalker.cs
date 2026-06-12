using UnityEngine;

// Moves a GameObject from its start position toward a target at a fixed speed,
// after an optional delay. Assign a target Transform in the Inspector.
public class ScriptedWalker : MonoBehaviour
{
    public Transform target;
    public float speed = 1.4f;
    public float startDelay = 0f;

    float m_Timer;
    bool m_Moving;
    Vector3 m_Start;

    void Start()
    {
        m_Timer = startDelay;
        m_Start = transform.position;
    }

    void Update()
    {
        if (!m_Moving)
        {
            m_Timer -= Time.deltaTime;
            if (m_Timer <= 0f) m_Moving = true;
            return;
        }

        if (target == null) return;

        transform.position = Vector3.MoveTowards(
            transform.position, target.position, speed * Time.deltaTime);

        Vector3 dir = target.position - transform.position;
        if (dir.sqrMagnitude > 0.001f)
            transform.rotation = Quaternion.LookRotation(dir);
    }
}
