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

    public void Start()
    {
        //Assign the car's rigidbody
        carRigidbody = GetComponent<Rigidbody>();
        //Init car with lights off
        setLightsEnabled(false);
    }

    public void FixedUpdate()
    {
        //Update the driving input and perform logic
        updateDriving();
        //Allow user to exit the application using the little X in VR
        if (Input.GetKeyDown(KeyCode.Escape)) { Application.Quit(); }
    }

    /// <summary>
    /// Manage the vehicle controls and apply input to the axles
    /// </summary>
    public void updateDriving()
    {
        //Calculate motor force from input
        float motor = getMotorInput();
        //Calculate steering force from input
        float steering = maxSteeringAngle * Input.acceleration.x;
        //Apply steering and motor force to axles
        updateAxles(motor, steering);
    }

    /// <summary>
    /// Send determined acceleration to the axles
    /// </summary>
    /// <param name="motor">motor magnitude</param>
    /// <param name="steering">steering magnitude</param>
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

    /// <summary>
    /// Apply the movement from the collider to the visuals
    /// </summary>
    /// <param name="collider">wheel collider to sync game object with</param>
    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if(collider.transform.childCount == 0)
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

    /// <summary>
    /// Get mobile input for increaing the magnitude on the accelerator
    /// </summary>
    /// <returns>motor magnitude</returns>
    public float getMotorInput()
    {
        float motor = 0;
        
        //Get is the accelerator down
        if (Convert.ToBoolean(Input.GetAxis("Fire1")))
        {
            //Get the acclerometer input
            if((Input.acceleration.z + 0.3f) > 0 && carRigidbody.velocity.magnitude < 30)
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
                carRigidbody.drag = 0.5f;
                setLightsEnabled(true);
            }
        }
        return motor;
    }

    /// <summary>
    /// Enable the brakelights when they should be occuring
    /// </summary>
    /// <param name="lightsEnabled">should the lights be on?</param>
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