using System;
using System.Collections.Generic;
using UnityEngine;

public class UserCarController : MonoBehaviour
{
    public List<AxleInfo> axleInfos;
    public float maxMotorTorque;
    public float maxSteeringAngle;
    public List<Light> lights;
    private Rigidbody carRigidbody;

    public void FixedUpdate()
    {
        updateDriving();
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            Application.Quit();
        }
    }

    public void updateDriving()
    {
        float motor = getMotorInput();
        float steering = maxSteeringAngle * Input.acceleration.x;
        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.steering)
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
    }

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

    public float getMotorInput()
    {
        //FIXME remove when I know that this works
        //ORIGINAL MOTOR CALCULATION
        ///float motor = maxMotorTorque * -1 * Input.GetAxis("Fire1") * (Input.acceleration.z + 0.25f) * 2;
        //ORIGINAL MOTOR CALCULATION

        float motor = 0;

        //Get is the accelerator down
        if (Convert.ToBoolean(Input.GetAxis("Fire1")))
        {
            //Get the acclerometer input
            if((Input.acceleration.z + 0.3f) > 0)
            {
                setLightsEnabled(false);
                //Go forward
                //Remove the drag from the most recent brake
                carRigidbody.drag = 0;
                //Add the force to the motor
                motor = maxMotorTorque * -1 * (Input.acceleration.z + 0.3f) * 3;
                //FIXME speed limit
                    //Quick responding acceleration to a clearly defined vehicle speed limit
            }
            else
            {
                //Brake
                //Increase the drag on the car
                carRigidbody.drag = 0.1f;
                setLightsEnabled(true);
            }
        }
        return motor;
    }

    void setLightsEnabled(bool lightsEnabled)
    {
        foreach (Light light in lights)
        {
            //Turn the lights on if the car is braking
            light.enabled = lightsEnabled;
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