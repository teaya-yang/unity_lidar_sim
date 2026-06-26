using UnityEngine;

/// <summary>
/// Deterministic constant-velocity incursion crosser for parameterized scenario
/// generation. Replaces free-running Vehicle.cs / AmbulanceTrajectorySubscriber
/// movement during ML-Agents episodes so each encounter is reproducible.
///
/// TaxiAgent.OnEpisodeBegin computes a start position (from the per-episode
/// time-to-conflict Δt) and calls ResetCrossing(startPos). This component then
/// moves the agent in a straight line at a fixed speed each FixedUpdate.
///
/// Disable any Vehicle / AmbulanceTrajectorySubscriber on the same GameObject so
/// they don't fight this controller for the Transform.
/// </summary>
public class IncursionAgentController : MonoBehaviour
{
    [Header("Crossing motion")]
    [Tooltip("World-space direction the agent travels across the taxiway. " +
             "Will be normalised. Typically perpendicular to the taxiway (±X).")]
    public Vector3 crossDirection = Vector3.right;

    [Tooltip("Crossing speed [m/s]. Overridden per-episode by TaxiAgent if a " +
             "side-channel 'ambulance_speed' parameter is provided.")]
    public float crossSpeed = 5.0f;

    [Tooltip("If true, face the travel direction. Untick to keep authored rotation.")]
    public bool faceTravelDirection = true;

    [Tooltip("Tick if the model's visual front points down -Z (like the ambulance/NPC cars). " +
             "Untick for models whose front is +Z.")]
    public bool frontIsNegativeZ = true;

    bool    _moving;
    Vector3 _dir;
    float   _speed;

    public Vector3 CrossDirectionNormalized =>
        crossDirection.sqrMagnitude > 1e-6f ? crossDirection.normalized : Vector3.right;

    /// <summary>Current world velocity (zero when idle). Read by callers if needed.</summary>
    public Vector3 Velocity => _moving ? _dir * _speed : Vector3.zero;

    /// <summary>
    /// Teleport to startPos and begin crossing at the given speed (or crossSpeed
    /// if speed ≤ 0). Called by TaxiAgent at the start of each episode.
    /// </summary>
    public void ResetCrossing(Vector3 startPos, float speed = -1f)
    {
        transform.position = startPos;
        _dir   = CrossDirectionNormalized;
        _speed = speed > 0f ? speed : crossSpeed;
        _moving = true;

        if (faceTravelDirection && _dir.sqrMagnitude > 1e-6f)
        {
            Vector3 faceDir = frontIsNegativeZ ? -_dir : _dir;
            transform.rotation = Quaternion.LookRotation(faceDir, Vector3.up);
        }
    }

    public void StopCrossing() => _moving = false;

    void FixedUpdate()
    {
        if (!_moving) return;
        transform.position += _dir * (_speed * Time.fixedDeltaTime);
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.magenta;
        Vector3 d = CrossDirectionNormalized;
        Gizmos.DrawLine(transform.position - d * 20f, transform.position + d * 20f);
        Gizmos.DrawWireSphere(transform.position, 1f);
    }
}
