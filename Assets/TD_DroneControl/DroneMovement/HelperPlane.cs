using UnityEngine;

namespace DroneMovement
{
    public class HelperPlane : MonoBehaviour
    {
        [SerializeField] private float maxTiltDegree = 30f;

        public void Tilt(Vector2 d)
        {
            float xRot = Mathf.Clamp(
                maxTiltDegree * d.y,
                -maxTiltDegree,
                maxTiltDegree
            );
            float zRot = Mathf.Clamp(
                maxTiltDegree * d.x,
                -maxTiltDegree,
                maxTiltDegree
            );
            transform.localRotation = Quaternion.Euler(xRot, 0, -zRot);
        }
    }
}