using UnityEngine;

using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;

using Unity.Robotics.ROSTCPConnector;

public class PointCloudPublisher : MonoBehaviour
{
    public GameObject laser_sensor_link;
    public string point_cloud_topic = "/point_cloud";
    public string pose_topic = "/laser_scan_pose";

    public float RangeMetersMin = 0;
    public float RangeMetersMax = 1000;

    public float fov_horizontal = 360;
    public float fov_vertical = 45;

    public float angularResolution_vertical = 1;
    public float angularResolution_horizontal = 1;

    public bool publishMaxRangeOnNoHit = false;
    public string frameIdOverride = "";

    public double m_PublishRateHz = 10.0;

    ROSConnection ros;
    LaserSensor3D laser_sensor_3d;

    double m_LastPublishTimeSeconds;
    double PublishPeriodSeconds => 1.0 / m_PublishRateHz;

    bool ShouldPublishMessage =>
        Time.timeAsDouble >= m_LastPublishTimeSeconds + PublishPeriodSeconds;

    void Start()
    {
        if (m_PublishRateHz <= 0.0)
        {
            Debug.LogWarning($"{nameof(PointCloudPublisher)} publish rate must be > 0. Using 10 Hz.");
            m_PublishRateHz = 10.0;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointCloud2Msg>(point_cloud_topic);
        ros.RegisterPublisher<PoseMsg>(pose_topic);

        laser_sensor_3d = new LaserSensor3D(
            laser_sensor_link,
            RangeMetersMin,
            RangeMetersMax,
            fov_horizontal,
            fov_vertical,
            angularResolution_vertical,
            angularResolution_horizontal,
            publishMaxRangeOnNoHit,
            frameIdOverride
        );

        // Publish immediately on the first eligible Update.
        m_LastPublishTimeSeconds = Time.timeAsDouble - PublishPeriodSeconds;
    }

    void Update()
    {
        if (!ShouldPublishMessage)
            return;

        // Debug.Log("Publishing point cloud");

        if (laser_sensor_3d == null)
        {
            Debug.LogError("[PointCloudPublisher] laser_sensor_3d is null — Start() failed. Check laser_sensor_link assignment and Console for earlier errors.");
            enabled = false;
            return;
        }
        PointCloud2Msg point_cloud_msg = laser_sensor_3d.getScanMsg();

        PoseMsg pose_msg = new PoseMsg
        {
            position = new PointMsg(
                laser_sensor_link.transform.position.x,
                laser_sensor_link.transform.position.y,
                laser_sensor_link.transform.position.z
            ),
            orientation = new QuaternionMsg(
                laser_sensor_link.transform.rotation.x,
                laser_sensor_link.transform.rotation.y,
                laser_sensor_link.transform.rotation.z,
                laser_sensor_link.transform.rotation.w
            ),
        };

        ros.Publish(point_cloud_topic, point_cloud_msg);
        ros.Publish(pose_topic, pose_msg);

        m_LastPublishTimeSeconds = Time.timeAsDouble;
    }
}