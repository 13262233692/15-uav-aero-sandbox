using UnityEngine;

public class DroneCameraFollow : MonoBehaviour
{
    public Transform droneTarget;
    public float distance = 8f;
    public float height = 4f;
    public float followSmoothness = 5f;
    public float lookSmoothness = 5f;
    public bool autoFindDrone = true;

    private Vector3 _velocityOffset;

    private void Start()
    {
        if (autoFindDrone && droneTarget == null)
        {
            QuadcopterSimulator sim = FindObjectOfType<QuadcopterSimulator>();
            if (sim != null)
                droneTarget = sim.transform;
        }
    }

    private void LateUpdate()
    {
        if (droneTarget == null) return;

        Vector3 targetPos = droneTarget.position - droneTarget.forward * distance + Vector3.up * height;
        transform.position = Vector3.Lerp(transform.position, targetPos, followSmoothness * Time.deltaTime);

        Vector3 lookTarget = droneTarget.position + Vector3.up * 1f;
        Quaternion targetRot = Quaternion.LookRotation(lookTarget - transform.position);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, lookSmoothness * Time.deltaTime);
    }
}
