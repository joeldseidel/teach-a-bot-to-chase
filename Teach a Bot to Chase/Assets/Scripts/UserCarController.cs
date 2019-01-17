using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UserCarController : MonoBehaviour
{
    public List<AxleInfo> axleInfos;
    public float maxMotorTorque;
    public float maxSteeringAngle;
    public List<Light> lights;

    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if(collider.transform.childCount == 0)
        {
            return;
        }
        Transform visualWheel = collider.transform.GetChild(0);
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);
        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }

    private bool inReverse = false;

	public void FixedUpdate () {
        float motor = maxMotorTorque * Input.GetAxis("Vertical") * -1;
        float steering = maxSteeringAngle * Input.GetAxis("Horizontal");
        foreach (AxleInfo axleInfo in axleInfos)
        {
            if(axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (axleInfo.motor)
            {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }
            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }
        foreach(Light light in lights)
        {
            //Turn the lights on if the car is braking and not in reverse
            light.enabled = System.Convert.ToBoolean(Input.GetAxis("Vertical") < 0) && !inReverse;
        }
    }
}


[System.Serializable]
public class AxleInfo
{
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor;
    public bool steering;
}