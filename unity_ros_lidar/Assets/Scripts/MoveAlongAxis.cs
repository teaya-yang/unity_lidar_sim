using UnityEngine;

public class MoveAlongAxis : MonoBehaviour
{
    public enum Axis
    {
        X,
        Y,
        Z
    }

    [Header("Movement Settings")]
    public Axis moveAxis = Axis.X;
    public float speed = 1.0f;

    void Update()
    {
        Vector3 direction = Vector3.zero;

        switch (moveAxis)
        {
            case Axis.X:
                direction = Vector3.right;
                break;

            case Axis.Y:
                direction = Vector3.up;
                break;

            case Axis.Z:
                direction = Vector3.forward;
                break;
        }

        transform.position += direction * speed * Time.deltaTime;
    }
}