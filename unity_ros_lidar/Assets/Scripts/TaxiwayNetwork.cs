using System.Collections.Generic;
using System.IO;
using UnityEngine;
using Newtonsoft.Json.Linq;

/// <summary>
/// Runtime state of the ego aircraft relative to a TaxiwayPath (Frenet frame).
///   s         — arc-length from path start [m]
///   d         — signed cross-track error [m]  (+ = left of path direction)
///   thetaError — heading error vs path tangent [rad]
///   tangent    — world-space unit vector along the path at the closest point
/// </summary>
public struct PathState
{
    public float   s;
    public float   d;
    public float   thetaError;
    public Vector3 tangent;
}

/// <summary>
/// A runtime-only taxiway path loaded from GeoJSON (or built programmatically).
/// Not a MonoBehaviour — lives purely in memory, held by TaxiwayNetwork.
/// </summary>
public class TaxiwayPath
{
    public readonly List<Vector3> Waypoints = new List<Vector3>();
    public float[] CumulativeLength { get; private set; }
    public float   TotalLength      => CumulativeLength != null && CumulativeLength.Length > 0
                                        ? CumulativeLength[CumulativeLength.Length - 1] : 0f;

    // Sharpest turn between consecutive segments [deg]. Used to skip un-navigable
    // paths (corners tighter than the aircraft's steering can follow).
    public float MaxTurnDeg { get; private set; }

    public void Precompute()
    {
        CumulativeLength = new float[Waypoints.Count];
        CumulativeLength[0] = 0f;
        for (int i = 1; i < Waypoints.Count; i++)
            CumulativeLength[i] = CumulativeLength[i - 1]
                                 + Vector3.Distance(Waypoints[i - 1], Waypoints[i]);

        MaxTurnDeg = 0f;
        for (int i = 1; i < Waypoints.Count - 1; i++)
        {
            Vector3 a = (Waypoints[i]     - Waypoints[i - 1]); a.y = 0f;
            Vector3 b = (Waypoints[i + 1] - Waypoints[i]);     b.y = 0f;
            if (a.sqrMagnitude < 1e-6f || b.sqrMagnitude < 1e-6f) continue;
            float ang = Vector3.Angle(a, b);
            if (ang > MaxTurnDeg) MaxTurnDeg = ang;
        }
    }
}

/// <summary>
/// Parses a GeoJSON file from StreamingAssets and exposes:
///   • A list of TaxiwayPaths (runtime waypoint chains).
///   • GetRelativeState() — Frenet-frame position/heading error for the ego.
///   • TryFindIntersection() — closest crossing point between two paths.
///
/// Coordinate mapping: GeoJSON lon/lat → flat-earth Mercator → Unity (X=East, Z=North).
/// Set originLat / originLon to the map's geographic centre so local coords stay small.
///
/// If no GeoJSON file is present the network starts empty; callers must null-check
/// or use the fallback straight-line path in TaxiAgent.
/// </summary>
public class TaxiwayNetwork : MonoBehaviour
{
    [Header("GeoJSON source")]
    [Tooltip("File name relative to Application.streamingAssetsPath (e.g. 'taxiway.geojson').")]
    public string geoJsonFileName = "taxiway.geojson";

    [Header("Geographic origin (map centre)")]
    [Tooltip("Latitude of the coordinate origin. All positions are relative to this.")]
    public double originLat = 0.0;
    [Tooltip("Longitude of the coordinate origin.")]
    public double originLon = 0.0;

    [Header("Intersection detection")]
    [Tooltip("Two path segments are considered intersecting when their closest approach is within this distance [m].")]
    public float intersectionThreshold = 5f;

    public IReadOnlyList<TaxiwayPath> Paths => _paths;
    readonly List<TaxiwayPath> _paths = new List<TaxiwayPath>();

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    void Awake() => LoadMapData();

    // ── Public API ────────────────────────────────────────────────────────────

    public void LoadMapData()
    {
        _paths.Clear();
        string filePath = Path.Combine(Application.streamingAssetsPath, geoJsonFileName);

        if (!File.Exists(filePath))
        {
            Debug.LogWarning(
                $"[TaxiwayNetwork] '{filePath}' not found — network is empty. " +
                "Place a GeoJSON file in Assets/StreamingAssets/ and re-enter Play.", this);
            return;
        }

        JObject root;
        try { root = JObject.Parse(File.ReadAllText(filePath)); }
        catch (System.Exception ex)
        {
            Debug.LogError($"[TaxiwayNetwork] Failed to parse GeoJSON: {ex.Message}", this);
            return;
        }

        var features = root["features"] as JArray;
        if (features == null) { Debug.LogError("[TaxiwayNetwork] No 'features' array found.", this); return; }

        foreach (var feature in features)
        {
            var geom = feature?["geometry"];
            if (geom == null) continue;

            string geomType = (string)geom["type"];
            JArray rawCoords = null;

            if (geomType == "LineString")
                rawCoords = geom["coordinates"] as JArray;
            else if (geomType == "Polygon")
                rawCoords = (geom["coordinates"] as JArray)?[0] as JArray; // outer ring only

            var path = BuildPath(rawCoords);
            if (path != null) _paths.Add(path);
        }

        Debug.Log($"[TaxiwayNetwork] Loaded {_paths.Count} path(s) from '{geoJsonFileName}'.", this);
    }

    /// <summary>
    /// Returns the Frenet-frame state of the agent relative to the given path.
    /// agentForward should be the world-space forward direction of the agent (Y ignored).
    /// </summary>
    public PathState GetRelativeState(Vector3 agentPos, Vector3 agentForward, TaxiwayPath path)
    {
        var wps = path.Waypoints;

        // Find closest point on any segment
        int   bestSeg  = 0;
        float bestT    = 0f;
        float bestDist2 = float.MaxValue;
        Vector3 bestProj = wps[0];

        for (int i = 0; i < wps.Count - 1; i++)
        {
            Vector3 a = wps[i], b = wps[i + 1];
            Vector3 ab = b - a;
            float len2 = ab.sqrMagnitude;
            float t = len2 > 1e-6f
                ? Mathf.Clamp01(Vector3.Dot(agentPos - a, ab) / len2)
                : 0f;
            Vector3 proj = a + ab * t;
            float dist2  = (agentPos - proj).sqrMagnitude;
            if (dist2 < bestDist2)
            {
                bestDist2 = dist2; bestSeg = i; bestT = t; bestProj = proj;
            }
        }

        Vector3 tangent = (wps[bestSeg + 1] - wps[bestSeg]).normalized;

        // Arc-length s
        float segLen = Vector3.Distance(wps[bestSeg], wps[bestSeg + 1]);
        float s = path.CumulativeLength[bestSeg] + bestT * segLen;

        // Signed cross-track error: positive = left of path direction
        Vector3 toAgent = agentPos - bestProj;
        toAgent.y = 0f;
        float dSign = Mathf.Sign(Vector3.Dot(Vector3.Cross(tangent, toAgent), Vector3.up));
        float d = dSign * Mathf.Sqrt(bestDist2);

        // Signed heading error
        Vector3 fwd = agentForward;
        fwd.y = 0f;
        if (fwd.sqrMagnitude > 1e-6f) fwd.Normalize();
        float dot   = Mathf.Clamp(Vector3.Dot(fwd, tangent), -1f, 1f);
        float cross = Vector3.Dot(Vector3.Cross(fwd, tangent), Vector3.up);
        float thetaErr = Mathf.Atan2(cross, dot);

        return new PathState { s = s, d = d, thetaError = thetaErr, tangent = tangent };
    }

    /// <summary>
    /// Arc-length remaining along path from agentPos to the end (clamped to 0).
    /// </summary>
    public float RemainingArcLength(Vector3 agentPos, TaxiwayPath path)
    {
        PathState ps = GetRelativeState(agentPos, Vector3.forward, path);
        return Mathf.Max(0f, path.TotalLength - ps.s);
    }

    /// <summary>
    /// Finds the closest world-space intersection point between pathA and pathB.
    /// Returns false if no segment pair comes within intersectionThreshold metres.
    /// </summary>
    public bool TryFindIntersection(TaxiwayPath pathA, TaxiwayPath pathB,
                                    out Vector3 intersection)
    {
        intersection = Vector3.zero;
        float best = intersectionThreshold;
        bool found = false;

        var wa = pathA.Waypoints;
        var wb = pathB.Waypoints;

        for (int i = 0; i < wa.Count - 1; i++)
        {
            for (int j = 0; j < wb.Count - 1; j++)
            {
                SegmentsClosestPoints(wa[i], wa[i+1], wb[j], wb[j+1],
                                      out Vector3 pA, out Vector3 pB);
                float d = Vector3.Distance(pA, pB);
                if (d < best) { best = d; intersection = (pA + pB) * 0.5f; found = true; }
            }
        }
        return found;
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    TaxiwayPath BuildPath(JArray rawCoords)
    {
        if (rawCoords == null || rawCoords.Count < 2) return null;

        var path = new TaxiwayPath();
        foreach (var pt in rawCoords)
        {
            var arr = pt as JArray;
            if (arr == null || arr.Count < 2) continue;
            double lon = (double)arr[0];
            double lat = (double)arr[1];
            path.Waypoints.Add(LatLonToUnity(lat, lon));
        }

        if (path.Waypoints.Count < 2) return null;
        path.Precompute();
        return path;
    }

    Vector3 LatLonToUnity(double lat, double lon)
    {
        const double R      = 6378137.0;
        const double DEG2RAD = System.Math.PI / 180.0;
        double dLat  = (lat - originLat) * DEG2RAD;
        double dLon  = (lon - originLon) * DEG2RAD;
        double cosLat = System.Math.Cos(originLat * DEG2RAD);
        float  z = (float)(dLat * R);           // north → Unity +Z
        float  x = (float)(dLon * R * cosLat);  // east  → Unity +X
        return new Vector3(x, 0f, z);
    }

    // 3-D segment–segment closest-point (Ericson, Real-Time Collision Detection §5.1.9)
    static void SegmentsClosestPoints(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4,
                                      out Vector3 c1, out Vector3 c2)
    {
        Vector3 d1 = p2 - p1, d2 = p4 - p3, r = p1 - p3;
        float a = Vector3.Dot(d1, d1), e = Vector3.Dot(d2, d2), f = Vector3.Dot(d2, r);
        float s, t;

        if (a < 1e-6f && e < 1e-6f) { s = t = 0f; }
        else if (a < 1e-6f) { s = 0f; t = Mathf.Clamp01(f / e); }
        else
        {
            float c = Vector3.Dot(d1, r);
            if (e < 1e-6f)
            {
                t = 0f; s = Mathf.Clamp01(-c / a);
            }
            else
            {
                float b     = Vector3.Dot(d1, d2);
                float denom = a * e - b * b;
                s = denom > 1e-6f ? Mathf.Clamp01((b * f - c * e) / denom) : 0f;
                t = (f + b * s) / e;
                if      (t < 0f) { t = 0f; s = Mathf.Clamp01(-c / a); }
                else if (t > 1f) { t = 1f; s = Mathf.Clamp01((b - c) / a); }
            }
        }
        c1 = p1 + d1 * s;
        c2 = p3 + d2 * t;
    }

    // ── Gizmos ────────────────────────────────────────────────────────────────

    void OnDrawGizmosSelected()
    {
        if (_paths == null) return;
        for (int pi = 0; pi < _paths.Count; pi++)
        {
            Gizmos.color = Color.HSVToRGB(pi / (float)Mathf.Max(_paths.Count, 1), 0.8f, 1f);
            var wps = _paths[pi].Waypoints;
            for (int i = 0; i < wps.Count - 1; i++)
            {
                Gizmos.DrawLine(wps[i], wps[i + 1]);
                Gizmos.DrawSphere(wps[i], 0.5f);
            }
            if (wps.Count > 0) Gizmos.DrawSphere(wps[wps.Count - 1], 0.5f);
        }
    }
}
