using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public static class TransformExtensions
{
    static HeaderMsg CreateHeader(double timeStamp, string frameId)
    {
#if ROS2
        return new HeaderMsg(new TimeStamp(timeStamp), frameId);
#else
        return new HeaderMsg(0, new TimeStamp(timeStamp), frameId);
#endif
    }

    public static TransformMsg ToROSTransform(this Transform tfUnity)
    {
        return new TransformMsg(
            // Using vector/quaternion To<>() because Transform.To<>() doesn't use localPosition/localRotation
            tfUnity.localPosition.To<FLU>(),
            tfUnity.localRotation.To<FLU>());
    }

    public static TransformStampedMsg ToROSTransformStamped(this Transform tfUnity, double timeStamp)
    {
        return new TransformStampedMsg(
            CreateHeader(timeStamp, tfUnity.parent.gameObject.name),
            tfUnity.gameObject.name,
            tfUnity.ToROSTransform());
    }
}
