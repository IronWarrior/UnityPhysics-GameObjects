using UnityEngine;

public class PhysicsJoint : MonoBehaviour
{
    public PhysicsBody Target;
    public Vector3 Anchor;
    public Vector3 AngularAnchor;

    public PhysicsBody Body { get; private set; }

    private void Awake()
    {
        Body = GetComponent<PhysicsBody>();

        Debug.Assert(Body != null, "Joint attached to GameObject without PhysicsBody.");
    }

    private void OnDrawGizmosSelected()
    {
        Vector3 anchorWorldSpace = transform.TransformPoint(Anchor);

        var matrix = Gizmos.matrix;
        Gizmos.color = new Color(0.55f, 0.95f, 0.55f, 0.75f);
        Gizmos.DrawLine(transform.position, anchorWorldSpace);
        Gizmos.DrawWireCube(anchorWorldSpace, Vector3.one * 0.5f);
        Gizmos.matrix = matrix;
    }
}