using System.Text;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.Core;
using RosMessageTypes.Std;

public class GroundTruthPublisher : MonoBehaviour
{
    [Header("ROS")]
    public string topic = "/ground_truth/agents";
    public double publishRateHz = 10.0;

    [Header("Ego vehicle")]
    [Tooltip("The ego sensor platform (Airplane). Its pose is published per frame as the " +
             "nuScenes-style ego_pose. Leave null to auto-find the first ScenarioManager ego.")]
    public Transform egoVehicle;

    [Header("Episode context (set by EpisodeSweepRunner or manually)")]
    public int currentEpisode;
    public string currentConfig;
    public int currentSeed;

    int m_Frame;

    ROSConnection m_Ros;
    double m_LastPublishTime;
    double PublishPeriod => 1.0 / publishRateHz;
    bool ShouldPublish => Time.timeAsDouble >= m_LastPublishTime + PublishPeriod;

    public void ResetFrame() => m_Frame = 0;

    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<StringMsg>(topic);
        m_LastPublishTime = Time.timeAsDouble - PublishPeriod;

        if (egoVehicle == null)
        {
            ScenarioManager sm = Object.FindFirstObjectByType<ScenarioManager>();
            if (sm != null && sm.egoVehicles != null && sm.egoVehicles.Length > 0)
                egoVehicle = sm.egoVehicles[0];
        }

        if (egoVehicle == null)
            Debug.LogWarning("[GroundTruthPublisher] No egoVehicle assigned and none found on a " +
                             "ScenarioManager — ego_pose will be published as null.", this);
    }

    void Update()
    {
        if (!ShouldPublish) return;

        TimeStamp stamp = new TimeStamp(Clock.time);
        ErraticAgent[]  agents   = Object.FindObjectsByType<ErraticAgent>(FindObjectsSortMode.None);
        ErraticVehicle[] vehicles = Object.FindObjectsByType<ErraticVehicle>(FindObjectsSortMode.None);

        var sb = new StringBuilder(1024);
        sb.Append("{");
        sb.Append($"\"stamp_sec\":{stamp.Seconds},");
        sb.Append($"\"stamp_nsec\":{stamp.NanoSeconds},");
        sb.Append($"\"episode\":{currentEpisode},");
        sb.Append($"\"frame\":{m_Frame++},");
        sb.Append($"\"config\":\"{currentConfig}\",");
        sb.Append($"\"seed\":{currentSeed},");

        AppendEgoPose(sb);

        sb.Append("\"agents\":[");
        for (int i = 0; i < agents.Length; i++)
        {
            ErraticAgent a = agents[i];
            if (i > 0) sb.Append(",");
            // GetInstanceID is stable for the object's lifetime (the episode), so the same
            // physical agent keeps the same id across frames — required for instance/tracking.
            AppendPose(sb, a.gameObject.GetInstanceID(), a.agentType.ToString(), GetState(a), a.transform);
            AppendBbox(sb, a.gameObject);
            sb.Append("}");
        }
        sb.Append("],");

        sb.Append("\"vehicles\":[");
        for (int i = 0; i < vehicles.Length; i++)
        {
            ErraticVehicle v = vehicles[i];
            if (i > 0) sb.Append(",");
            AppendPose(sb, v.gameObject.GetInstanceID(), "Vehicle", "Moving", v.transform);
            AppendBbox(sb, v.gameObject);
            sb.Append("}");
        }
        sb.Append("]}");

        m_Ros.Publish(topic, new StringMsg(sb.ToString()));
        m_LastPublishTime = Time.timeAsDouble;
    }

    // Appends the nuScenes-style ego_pose: global-frame translation + full orientation quaternion.
    // Same global ROS frame as sample_annotation translations, so boxes and ego live in one frame.
    void AppendEgoPose(StringBuilder sb)
    {
        if (egoVehicle == null)
        {
            sb.Append("\"ego\":null,");
            return;
        }

        // Translation: Unity → ROS axis swap (same convention as objects).
        Vector3 p = egoVehicle.position;
        float tx =  p.z;
        float ty = -p.x;
        float tz =  p.y;

        // Orientation: convert the full Unity quaternion into the ROS frame.
        // ROS basis is (x=Unity z, y=-Unity x, z=Unity y); applying that change of basis to a
        // quaternion (qx,qy,qz,qw) maps the vector part the same way the axis swap maps points.
        Quaternion q = egoVehicle.rotation;
        float qx =  q.z;
        float qy = -q.x;
        float qz =  q.y;
        float qw =  q.w;

        // Yaw kept too (heading about up axis) for convenience / consistency with objects.
        float yaw = -egoVehicle.eulerAngles.y * Mathf.Deg2Rad;

        sb.Append("\"ego\":{");
        sb.Append($"\"tx\":{tx:F3},\"ty\":{ty:F3},\"tz\":{tz:F3},");
        sb.Append($"\"qw\":{qw:F4},\"qx\":{qx:F4},\"qy\":{qy:F4},\"qz\":{qz:F4},");
        sb.Append($"\"yaw\":{yaw:F4}");
        sb.Append("},");
    }

    // Writes opening of a JSON object with pose fields (no closing brace — bbox is appended next).
    static void AppendPose(StringBuilder sb, int id, string type, string state, Transform t)
    {
        float rx =  t.position.z;
        float ry = -t.position.x;
        float rz =  t.position.y;
        float yaw = -t.eulerAngles.y * Mathf.Deg2Rad;

        sb.Append("{");
        sb.Append($"\"id\":{id},");
        sb.Append($"\"type\":\"{type}\",");
        sb.Append($"\"state\":\"{state}\",");
        sb.Append($"\"rx\":{rx:F3},\"ry\":{ry:F3},\"rz\":{rz:F3},");
        sb.Append($"\"yaw\":{yaw:F4},");
    }

    // Appends "bbox":{...} from the world-space AABB of all Renderers on the GameObject.
    static void AppendBbox(StringBuilder sb, GameObject go)
    {
        Bounds b = GetCombinedBounds(go);

        // Center: Unity → ROS axis swap
        float cx =  b.center.z;
        float cy = -b.center.x;
        float cz =  b.center.y;

        // Size: axes swap the same way (extents are always positive)
        float sx = b.size.z;
        float sy = b.size.x;
        float sz = b.size.y;

        sb.Append($"\"bbox\":{{\"cx\":{cx:F3},\"cy\":{cy:F3},\"cz\":{cz:F3},\"sx\":{sx:F3},\"sy\":{sy:F3},\"sz\":{sz:F3}}}");
    }

    // Returns world-space AABB enclosing all Renderers on the GameObject and its children.
    // Falls back to a unit cube at the object's position if no renderers are found.
    static Bounds GetCombinedBounds(GameObject go)
    {
        Renderer[] renderers = go.GetComponentsInChildren<Renderer>();
        if (renderers.Length == 0)
            return new Bounds(go.transform.position, Vector3.one);

        Bounds b = renderers[0].bounds;
        for (int i = 1; i < renderers.Length; i++)
            b.Encapsulate(renderers[i].bounds);
        return b;
    }

    static string GetState(ErraticAgent a)
    {
        var nav = a.GetComponent<UnityEngine.AI.NavMeshAgent>();
        if (nav == null) return "Unknown";
        if (nav.isStopped)                        return "Paused";
        if (nav.speed > a.maxSpeed * 1.4f)        return "Reacting";
        if (a.patrolWaypoints != null
            && a.patrolWaypoints.Length > 0
            && nav.remainingDistance > 0.5f)      return "Patrolling";
        return "Wandering";
    }
}
