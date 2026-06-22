using RGLUnityPlugin;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[RequireComponent(typeof(LidarSensor))]
public class RglTcpPointCloudPublisher : MonoBehaviour
{
    public string topic = "/point_cloud";
    public string frameId = "lidar_link";
    public string intensityFieldName = "i";

    const int PointStep = 16;
    ROSConnection ros;
    LidarSensor lidar;
    RGLNodeSequence outputGraph;
    byte[] data = System.Array.Empty<byte>();

    static readonly RGLField[] Fields =
    {
        RGLField.XYZ_VEC3_F32,
        RGLField.INTENSITY_F32
    };

    void Awake()
    {
        outputGraph = new RGLNodeSequence()
            .AddNodePointsTransform("UNITY_TO_ROS", UnityToRosMatrix())
            .AddNodePointsFormat("FORMAT", Fields);
    }

    void Start()
    {
        if (outputGraph == null)
        {
            Debug.LogError($"{name}: failed to initialize RGL TCP point cloud output graph. Check earlier console errors from Awake().");
            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointCloud2Msg>(topic);

        lidar = GetComponent<LidarSensor>();
        if (lidar == null)
        {
            Debug.LogError($"{name}: RglTcpPointCloudPublisher requires a LidarSensor on the same GameObject.");
            enabled = false;
            return;
        }

        lidar.ConnectToLidarFrame(outputGraph);
        lidar.onNewData += Publish;
    }

    void Publish()
    {
        int count = outputGraph.GetResultDataRaw(ref data, PointStep);
        int dataLength = count * PointStep;
        var msgData = new byte[dataLength];
        System.Array.Copy(data, msgData, dataLength);
        var stamp = new TimeStamp(Clock.time);

        var msg = new PointCloud2Msg
        {
            header = new HeaderMsg
            {
                frame_id = frameId,
                stamp = new TimeMsg(stamp.Seconds, stamp.NanoSeconds)
            },
            height = 1,
            width = (uint)count,
            fields = new[]
            {
                new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg(intensityFieldName, 12, PointFieldMsg.FLOAT32, 1)
            },
            is_bigendian = false,
            point_step = PointStep,
            row_step = (uint)dataLength,
            data = msgData,
            is_dense = false
        };

        ros.Publish(topic, msg);
    }

    static Matrix4x4 UnityToRosMatrix()
    {
        return new Matrix4x4(
            new Vector4(0f, 0f, 1f, 0f),
            new Vector4(-1f, 0f, 0f, 0f),
            new Vector4(0f, 1f, 0f, 0f),
            new Vector4(0f, 0f, 0f, 1f)
        ).transpose;
    }

    void OnDestroy()
    {
        if (lidar != null) lidar.onNewData -= Publish;
        outputGraph?.Clear();
    }
}
