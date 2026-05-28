using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

// Drives the ambulance from a ROS2-generated trajectory (see scripts/trajectory_publisher.py).
// Attach to the ambulance GameObject in place of MoveAlongAxis. The incoming PoseStamped is
// treated as a displacement from the ambulance's pose at Play time, so it moves forward from
// its current scene position instead of teleporting to ROS world coordinates.
public class AmbulanceTrajectorySubscriber : MonoBehaviour
{
    public string trajectory_topic = "/ambulance/trajectory";

    Vector3 m_InitialPosition;
    Quaternion m_InitialRotation;
    ROSConnection ros;

    void Start()
    {
        m_InitialPosition = transform.position;
        m_InitialRotation = transform.rotation;

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PoseStampedMsg>(trajectory_topic, OnTrajectoryPose);
    }

    void OnTrajectoryPose(PoseStampedMsg msg)
    {
        // ROS (FLU) displacement -> Unity displacement, then apply in the ambulance's local
        // frame so "+x forward" follows its facing direction. (Drop the m_InitialRotation
        // factor for pure world-axis motion.)
        Vector3 offset = msg.pose.position.From<FLU>();
        transform.position = m_InitialPosition + m_InitialRotation * offset;

        // Apply the trajectory yaw on top of the ambulance's starting orientation.
        transform.rotation = m_InitialRotation * msg.pose.orientation.From<FLU>();
    }
}
