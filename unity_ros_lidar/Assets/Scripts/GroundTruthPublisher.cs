using System.Text;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.Core;
using RosMessageTypes.Std;

// Publishes per-frame ground-truth agent states as a JSON-encoded StringMsg.
// Attach to the same "ROS Publishers" GameObject as PointCloudPublisher.
// Must run at the same rate as PointCloudPublisher so timestamps correlate.
//
// Topic: /ground_truth/agents  (std_msgs/String, JSON payload)
//
// JSON schema per message:
// {
//   "stamp_sec":  int,
//   "stamp_nsec": int,
//   "episode":    int,
//   "config":     string,
//   "seed":       int,
//   "agents": [
//     { "id": int, "type": string, "state": string,
//       "rx": float, "ry": float, "rz": float,   // ROS frame (x=fwd, y=left, z=up)
//       "yaw": float }                            // radians, ROS convention
//   ]
// }
//
// Coordinate convention (from CLAUDE.md):
//   ROS x = Unity z,  ROS y = -Unity x,  ROS z = Unity y
public class GroundTruthPublisher : MonoBehaviour
{
    [Header("ROS")]
    public string topic = "/ground_truth/agents";
    public double publishRateHz = 10.0;

    [Header("Episode context (set by EpisodeSweepRunner or manually)")]
    public int currentEpisode;
    public string currentConfig;
    public int currentSeed;

    ROSConnection m_Ros;
    double m_LastPublishTime;
    double PublishPeriod => 1.0 / publishRateHz;
    bool ShouldPublish => Time.timeAsDouble >= m_LastPublishTime + PublishPeriod;

    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<StringMsg>(topic);
        m_LastPublishTime = Time.timeAsDouble - PublishPeriod;
    }

    void Update()
    {
        if (!ShouldPublish) return;

        TimeStamp stamp = new TimeStamp(Clock.time);
        ErraticAgent[] agents = Object.FindObjectsByType<ErraticAgent>(FindObjectsSortMode.None);

        var sb = new StringBuilder(512);
        sb.Append("{");
        sb.Append($"\"stamp_sec\":{stamp.Seconds},");
        sb.Append($"\"stamp_nsec\":{stamp.NanoSeconds},");
        sb.Append($"\"episode\":{currentEpisode},");
        sb.Append($"\"config\":\"{currentConfig}\",");
        sb.Append($"\"seed\":{currentSeed},");
        sb.Append("\"agents\":[");

        for (int i = 0; i < agents.Length; i++)
        {
            ErraticAgent a = agents[i];
            Vector3 up = a.transform.position;

            // Unity → ROS axis swap
            float rx = up.z;
            float ry = -up.x;
            float rz = up.y;

            // Yaw: Unity rotates around Y, ROS around Z. Negate for handedness.
            float yaw = -a.transform.eulerAngles.y * Mathf.Deg2Rad;

            if (i > 0) sb.Append(",");
            sb.Append("{");
            sb.Append($"\"id\":{i},");
            sb.Append($"\"type\":\"{a.agentType}\",");
            sb.Append($"\"state\":\"{GetState(a)}\",");
            sb.Append($"\"rx\":{rx:F3},\"ry\":{ry:F3},\"rz\":{rz:F3},");
            sb.Append($"\"yaw\":{yaw:F4}");
            sb.Append("}");
        }

        sb.Append("]}");

        m_Ros.Publish(topic, new StringMsg(sb.ToString()));
        m_LastPublishTime = Time.timeAsDouble;
    }

    // Reflect the private State enum out via the agent's public behaviour.
    // ErraticAgent doesn't expose State publicly, so we infer it from NavMeshAgent.
    static string GetState(ErraticAgent a)
    {
        var nav = a.GetComponent<UnityEngine.AI.NavMeshAgent>();
        if (nav == null) return "Unknown";
        if (nav.isStopped)                          return "Paused";
        if (nav.speed > a.maxSpeed * 1.4f)          return "Reacting";
        if (a.patrolWaypoints != null
            && a.patrolWaypoints.Length > 0
            && nav.remainingDistance > 0.5f)        return "Patrolling";
        return "Wandering";
    }
}
