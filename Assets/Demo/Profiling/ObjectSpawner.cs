using UnityEngine;

public class ObjectSpawner : MonoBehaviour
{
    public GameObject uPhysicsPrefab, PhysXPrefab;

    public float spacing = 1.1f;
    public int rows = 5, columns = 5, stacks = 5;
    public bool randomizeRotation;

    private void Awake()
    {
        PhysicsRunner.Mode mode = FindObjectOfType<PhysicsRunner>().currentMode;
        GameObject prefab = mode == PhysicsRunner.Mode.UPhysics ? uPhysicsPrefab : PhysXPrefab;

        // Keep the rotations consistent between runs.
        Random.InitState(0);

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < columns; j++)
            {
                for (int k = 0; k < stacks; k++)
                {
                    Vector3 position = new Vector3(i, j, k) * spacing + Vector3.up * spacing;

                    Quaternion rotation = randomizeRotation ? Random.rotation : Quaternion.identity;

                    Instantiate(prefab, position, rotation);
                }
            }
        }
    }
}
