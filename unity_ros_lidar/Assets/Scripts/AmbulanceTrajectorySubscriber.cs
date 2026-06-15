using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

// Drives the ambulance from a ROS2-generated trajectory (see scripts/trajectory_publisher.py).
// Attach to ambulance_root. The incoming PoseStamped is treated as a displacement from the
// ambulance's pose when the first message arrives, so it moves from wherever it is in the scene.
public class AmbulanceTrajectorySubscriber : MonoBehaviour
{
    public string trajectory_topic = "/ambulance/trajectory";

    Vector3 m_InitialPosition;
    Quaternion m_InitialRotation;
    Vector3 m_PrevWorldPos;
    bool m_Initialized = false;
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PoseStampedMsg>(trajectory_topic, OnTrajectoryPose);
    }

    void OnTrajectoryPose(PoseStampedMsg msg)
    {
        if (!m_Initialized)
        {
            m_InitialPosition = transform.position;
            m_InitialRotation = transform.rotation;
            m_PrevWorldPos = transform.position;
            m_Initialized = true;
        }

        // convert position from FLU coordinate system use in ROS
        // ex: (x=3, y=0, z=0) FLU -> (x=0,y=0,z=3) 
        Vector3 offset = msg.pose.position.From<FLU>();
        // rigid transformation
        Vector3 newPos = m_InitialPosition + m_InitialRotation * offset;

        // Derive heading from velocity — avoids the handedness error From<FLU>() introduces
        // on non-zero yaw quaternions (which breaks circle but not straight/accel_decel).
        Vector3 velocity = newPos - m_PrevWorldPos;
        if (velocity.sqrMagnitude > 0.0001f)
            transform.rotation = Quaternion.LookRotation(velocity, Vector3.up);

        transform.position = newPos;
        m_PrevWorldPos = newPos;
    }
}
