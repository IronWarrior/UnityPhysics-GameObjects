using Unity.Entities;
using UnityEngine;

public class PhysicsBody : MonoBehaviour
{
    [SerializeField]
    Vector3 boxColliderDimensions = Vector3.one;

    public enum MotionType { Dynamic, Static }

    public MotionType Motion;
    public Vector3 BoxColliderSize => new Vector3(
        boxColliderDimensions.x * transform.localScale.x,
        boxColliderDimensions.y * transform.localScale.y,
        boxColliderDimensions.z * transform.localScale.z);

    public float Mass = 1;

    public Vector3 Velocity;
    public Vector3 AngularVelocity;

    public int Entity;
    public BlobAssetReference<Unity.Physics.Collider> BoxCollider;

    private void OnDrawGizmosSelected()
    {
        var matrix = Gizmos.matrix;
        Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
        Gizmos.color = new Color(0.55f, 0.95f, 0.55f, 0.75f);
        Gizmos.DrawWireCube(Vector3.zero, BoxColliderSize);
        Gizmos.matrix = matrix;
    }
}
