using UnityEngine;

// Place this on an empty GameObject with a BoxCollider covering the sidewalk/curb.
// Assign crossTarget to a Transform on the opposite side of the street.
// This force unity to add a BoxCollider 
// if an Erratic agents enters the zone it is redirected to cross it if probability is smaller than 0.55 (tune this) it walks trought the zone otherwise keeps it target position
[RequireComponent(typeof(BoxCollider))]
public class StreetCrossingZone : MonoBehaviour
{
    [Tooltip("Point on the far side of the street agents will walk toward")]
    public Transform crossTarget;

    [Range(0f, 1f)]
    [Tooltip("Probability that an entering agent will actually cross")]
    public float crossProbability = 0.55f;

    void Awake()
    {
        GetComponent<BoxCollider>().isTrigger = true;
    }

    void OnTriggerEnter(Collider other)
    {
        if (crossTarget == null) return;
        if (Random.value > crossProbability) return;

        ErraticAgent agent = other.GetComponent<ErraticAgent>();
        agent?.TriggerCrossing(crossTarget.position);
    }

#if UNITY_EDITOR
    void OnDrawGizmos()
    {
        if (crossTarget == null) return;
        Gizmos.color = new Color(1f, 0.4f, 0f, 0.5f);
        Gizmos.DrawLine(transform.position, crossTarget.position);
        Gizmos.DrawSphere(crossTarget.position, 0.4f);
    }
#endif
}
