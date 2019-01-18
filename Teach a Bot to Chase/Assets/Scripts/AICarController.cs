using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AICarController : MonoBehaviour {
    public List<AxleInfo> axleInfos;
    public float maxSteeringAngle;
    public float maxMotorTorque;

    public void FixedUpdate()
    {
        
    }

    public void updateDriving()
    {
        //Calculate motor force from input
        //float motor = getMotorInput();
        float motor = 0;
        //Calculate steering force from input
        float steering = maxSteeringAngle * Input.acceleration.x;
        //Apply steering and motor force to axles
        updateAxles(motor, steering);
    }

    void updateAxles(float motor, float steering)
    {
        //Apply steering / motor force to defined axles
        foreach (AxleInfo axleInfo in axleInfos)
        {
            //Apply steering force to steering wheels
            if (axleInfo.steering)
            {
                //Apply steering force to both wheels on axle
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (axleInfo.motor)
            {
                //Apply motor force to both wheels on axle
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }
            //Update visual objects to reflect added force
            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }
    }

    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0)
        {
            return;
        }
        Transform visualWheel = collider.transform.GetChild(0);
        Vector3 position;
        Quaternion rotation;
        //Update visuals to reflect added force
        collider.GetWorldPose(out position, out rotation);
        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }
}
