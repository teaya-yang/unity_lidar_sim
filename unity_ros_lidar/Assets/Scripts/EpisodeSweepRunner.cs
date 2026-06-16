using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

// Cycles through every (ScenarioConfig, seed) pair automatically.
// After episodeDuration seconds it calls ScenarioManager.ResetEpisode() with the next seed,
// then advances to the next config when all seeds are exhausted.
// A JSON sidecar is written to Application.persistentDataPath/sweep_log.json so every
// episode's parameters are stored alongside the rosbag recording.
public class EpisodeSweepRunner : MonoBehaviour
{
    [Header("References")]
    public ScenarioManager scenarioManager;
    public GroundTruthPublisher groundTruthPublisher;

    [Header("Configs to sweep (drag assets here in order)")]
    public ScenarioConfig[] configs;

    [Header("Sweep parameters")]
    [Tooltip("Seeds 0 .. seedCount-1 are run for every config.")]
    public int seedCount = 10;
    [Tooltip("How long each episode runs before the next reset (seconds).")]
    public float episodeDuration = 30f;
    [Tooltip("Pause this many seconds after reset before the timer starts (let agents settle).")]
    public float settleTime = 2f;

    int m_ConfigIndex;
    int m_SeedIndex;
    float m_Timer;
    bool m_Settling;
    bool m_Done;

    readonly List<EpisodeRecord> m_Log = new();

    void Start()
    {
        if (scenarioManager == null)
        {
            Debug.LogError("[EpisodeSweepRunner] scenarioManager not assigned.", this);
            enabled = false;
            return;
        }
        if (configs == null || configs.Length == 0)
        {
            Debug.LogError("[EpisodeSweepRunner] No configs assigned.", this);
            enabled = false;
            return;
        }

        RunEpisode();
    }

    void Update()
    {
        if (m_Done) return;

        m_Timer -= Time.deltaTime;
        if (m_Timer > 0f) return;

        if (m_Settling)
        {
            m_Settling = false;
            m_Timer = episodeDuration;
            return;
        }

        Advance();
    }

    void RunEpisode()
    {
        ScenarioConfig cfg = configs[m_ConfigIndex];
        int seed = m_SeedIndex;

        scenarioManager.config = cfg;
        scenarioManager.ResetEpisode(seed);

        int total = configs.Length * seedCount;
        int current = m_ConfigIndex * seedCount + m_SeedIndex + 1;

        if (groundTruthPublisher != null)
        {
            groundTruthPublisher.currentEpisode = current;
            groundTruthPublisher.currentConfig  = cfg.name;
            groundTruthPublisher.currentSeed    = seed;
            groundTruthPublisher.ResetFrame();
        }
        int agentTotal = cfg.spawnEntries != null
            ? System.Linq.Enumerable.Sum(cfg.spawnEntries, e => e.resolvedCount)
            : 0;
        Debug.Log($"[SWEEP] episode {current}/{total} | config={cfg.name} | seed={seed} | " +
                  $"agents={agentTotal} spawnRadius={cfg.spawnRadius}");

        m_Log.Add(new EpisodeRecord
        {
            episode     = current,
            config      = cfg.name,
            seed        = seed,
            agents      = agentTotal,
            spawnRadius = cfg.spawnRadius,
            timestamp   = System.DateTimeOffset.UtcNow.ToUnixTimeSeconds()
        });

        m_Settling = true;
        m_Timer = settleTime;
    }

    void Advance()
    {
        m_SeedIndex++;
        if (m_SeedIndex >= seedCount)
        {
            m_SeedIndex = 0;
            m_ConfigIndex++;
        }

        if (m_ConfigIndex >= configs.Length)
        {
            m_Done = true;
            WriteLog();
            Debug.Log("[SWEEP] All episodes complete.");
            return;
        }

        RunEpisode();
    }

    void WriteLog()
    {
        string path = Path.Combine(Application.persistentDataPath, "sweep_log.json");
        string json = JsonUtility.ToJson(new EpisodeLogWrapper { episodes = m_Log.ToArray() }, prettyPrint: true);
        File.WriteAllText(path, json);
        Debug.Log($"[SWEEP] Log written to {path}");
    }

    void OnApplicationQuit() => WriteLog();

    [System.Serializable]
    struct EpisodeRecord
    {
        public int    episode;
        public string config;
        public int    seed;
        public int    agents;
        public float  spawnRadius;
        public long   timestamp;
    }

    [System.Serializable]
    class EpisodeLogWrapper { public EpisodeRecord[] episodes; }
}
