using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AICarController : MonoBehaviour {
    public List<AxleInfo> axleInfos;
    public GameObject forwardRay, backRay;
    public float maxSteeringAngle;
    public float maxMotorTorque;

    public float turnSensitivity;
    public float verticalSensitivity;

    private List<CarDistanceProfile> movementProfiles = new List<CarDistanceProfile>();

    public void Start()
    {
        PlayerPrefs.SetFloat("turnSens", 1.0f);
        PlayerPrefs.SetFloat("vertSens", 1.0f);
        turnSensitivity = PlayerPrefs.GetFloat("turnSens");
        verticalSensitivity = PlayerPrefs.GetFloat("vertSens");
    }

    public void FixedUpdate()
    {
        CarDistanceProfile distanceFromEdges = getRelativeCarPosition();
        CarAggregateProfile aggregateProfile = createAggregateProfile(distanceFromEdges);
        bool shouldTurn = shouldITurn(aggregateProfile);
        if (shouldTurn)
        {
            TurnProfile turnProfile = generateTurnProfile(aggregateProfile);
        }
        float motor = updateAccelerator();
        float steering = updateSteering();
        updateAxles(motor, steering);
    }

    float updateAccelerator()
    {
        return -1 * maxMotorTorque;
    }

    float updateSteering()
    {
        //TODO: enact turn profile
        return 0.0f;
    }

    TurnProfile generateTurnProfile(CarAggregateProfile cap)
    {
        //TODO finish
        return new TurnProfile();
    }

    /// <summary>
    /// Determine whether the car should be turning or not
    /// </summary>
    /// <param name="aggregateProfile">car profile containing whether the marginal rates of approach</param>
    /// <returns>boolean to invoke turning</returns>
    bool shouldITurn(CarAggregateProfile aggregateProfile)
    {
        if(aggregateProfile.frontRate < 0)
        {
            //Moving towards something in front of the car
            //Generate a probability of the car making a turn
            //This increases as the distance decreases - also corrected for car length
            //Turn sensitivity is the heuristics which determines this
            float turnProbability = (1 / (aggregateProfile.cdp.front + 5)) * turnSensitivity;
            if (turnProbability > 1)
            {
                //Turn probability is greater than 100%, just do it
                return true;
            }
            else
            {
                //Generate a random 0.0 -> 1.0 float
                float turnRandom = Random.Range(0.0f, 1.0f);
                if (turnRandom > turnProbability)
                {
                    //The number was above the probability, so no turn this 0.2s
                    return false;
                }
                else
                {
                    //The number was within the probability, invoke a turn
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>
    /// Create marginal profile which encapsulates rate of distance profile changing
    /// </summary>
    /// <param name="mostRecent">Most recent profile</param>
    /// <returns>Aggregate car profile containing most recent marginal rates</returns>
    CarAggregateProfile createAggregateProfile(CarDistanceProfile mostRecent)
    {
        //Add most recent profile to the list
        movementProfiles.Add(mostRecent);
        //Get the last cdp in the list
        CarDistanceProfile lastCdp = movementProfiles[movementProfiles.Count- 2];
        //Create aggregate profile
        CarAggregateProfile aggregateMarginalProfile = new CarAggregateProfile
        {
            frontRate = (mostRecent.front - lastCdp.front) / Time.deltaTime,
            frontRightRate = (mostRecent.frontRight - lastCdp.frontRight) / Time.deltaTime,
            frontLeftRate = (mostRecent.frontLeft - lastCdp.frontLeft) / Time.deltaTime
        };
        //debugging
        Debug.Log("Front: " + (aggregateMarginalProfile.frontRate < 0).ToString() + " at " + aggregateMarginalProfile.frontRate / 2 + ".  Right: " + (aggregateMarginalProfile.frontRightRate < 0).ToString() + " at " + aggregateMarginalProfile.frontRightRate / 2 + ".  Left:" + (aggregateMarginalProfile.frontLeftRate < 0).ToString() + " at " + aggregateMarginalProfile.frontLeftRate / 2);
        //Remove the oldest profile if the count is more than 5
        if (movementProfiles.Count > 5)
        {
            //Remove oldest profile
            movementProfiles.RemoveAt(0);
        }
        return aggregateMarginalProfile;
    }

    /// <summary>
    /// Perform raycasts and determine distance from sides of the road
    /// </summary>
    /// <returns>device profile containing the distances to sides of road</returns>
    CarDistanceProfile getRelativeCarPosition()
    {
        //FIXME adjust front and back wheels to be local vector scale
        CarDistanceProfile thisDistanceProfile = new CarDistanceProfile();
        RaycastHit colliderHit;
        //Ray cast from front wheel and debug
        Physics.Raycast(axleInfos[0].leftWheel.transform.position, axleInfos[0].leftWheel.transform.TransformDirection(Vector3.right), out colliderHit, Mathf.Infinity);
        Debug.DrawRay(axleInfos[0].leftWheel.transform.position, axleInfos[0].leftWheel.transform.TransformDirection(Vector3.right) * colliderHit.distance, Color.red);
        thisDistanceProfile.frontLeft = colliderHit.distance;
        Physics.Raycast(axleInfos[0].rightWheel.transform.position, axleInfos[0].rightWheel.transform.TransformDirection(Vector3.left), out colliderHit, Mathf.Infinity);
        Debug.DrawRay(axleInfos[0].rightWheel.transform.position, axleInfos[0].rightWheel.transform.TransformDirection(Vector3.left) * colliderHit.distance, Color.red);
        thisDistanceProfile.frontRight = colliderHit.distance;
        //Ray case from back wheels and debug
        Physics.Raycast(axleInfos[1].leftWheel.transform.position, Vector3.forward, out colliderHit, Mathf.Infinity);
        Debug.DrawRay(axleInfos[1].leftWheel.transform.position, Vector3.forward * colliderHit.distance, Color.green);
        thisDistanceProfile.backLeft = colliderHit.distance;
        Physics.Raycast(axleInfos[1].rightWheel.transform.position, Vector3.back, out colliderHit, Mathf.Infinity);
        Debug.DrawRay(axleInfos[1].rightWheel.transform.position, Vector3.back * colliderHit.distance, Color.green);
        thisDistanceProfile.backRight = colliderHit.distance;
        //Ray cast forward and debug
        Physics.Raycast(forwardRay.transform.position, Vector3.right, out colliderHit, Mathf.Infinity);
        Debug.DrawRay(forwardRay.transform.position, Vector3.right * colliderHit.distance, Color.green);
        thisDistanceProfile.front = colliderHit.distance;
        //Ray cast from backward
        Physics.Raycast(backRay.transform.position, Vector3.left, out colliderHit, Mathf.Infinity);
        Debug.DrawRay(backRay.transform.position, Vector3.left * colliderHit.distance, Color.green);
        thisDistanceProfile.back = colliderHit.distance;
        return thisDistanceProfile;
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


[System.Serializable]
public class CarDistanceProfile
{
    public float frontLeft, backLeft, frontRight, backRight, front, back;
}

[System.Serializable]
public class CarAggregateProfile
{
    public float frontLeftRate, frontRightRate, frontRate;
    public CarDistanceProfile cdp;
}

[System.Serializable]
public class TurnProfile
{
    public float torquemagnitude;
    public TURN_DIRECTION direction;
}

public enum TURN_DIRECTION
{
    localright, localleft
}