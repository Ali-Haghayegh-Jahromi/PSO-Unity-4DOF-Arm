using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FourLinkRobot : MonoBehaviour
{
    public Panel panel;

    public GameObject Base; // The base of the robot
    public GameObject Link1; // First link connecting Base and Joint1
    public GameObject Joint1; // Joint 1 
    public GameObject Link2; // Second link connecting Joint1 and Joint2
    public GameObject Joint2; // Joint 2 
    public GameObject Link3; // Third link connecting Joint2 and Joint3
    public GameObject Joint3; // Joint 3
    public GameObject Link4; // Fourth link connecting Joint3 and EndEffector
    public GameObject EndEffector; // The end of the robot arm

    public Vector3 Angles1; // Rotation angles for Base
    public Vector3 Angles2; // Rotation angles for Joint1
    public Vector3 Angles3; // Rotation angles for Joint2
    public Vector3 Angles4; // Rotation angles for Joint3

    private float linkLength = 1.0f;

    void Update()
    {
        // Calculate kinematics
        CalculateKinematics();

        if (panel.Timer == 0f)
        {
            // Log positions
            //LogPositions();
        }
    }

    public void CalculateKinematics()
    {
        if (Base != null)
        {
            // Base rotation
            Base.transform.rotation = Quaternion.Euler(Angles1);

            // Joint1 rotation and position
            Quaternion joint1Rotation = Base.transform.rotation * Quaternion.Euler(Angles2);
            Joint1.transform.rotation = joint1Rotation;
            Joint1.transform.position = Base.transform.position + Base.transform.up * linkLength;

            // Link1 position and rotation
            Link1.transform.position = (Base.transform.position + Joint1.transform.position) / 2;
            Link1.transform.rotation = Base.transform.rotation;

            // Joint2 rotation and position
            Quaternion joint2Rotation = Joint1.transform.rotation * Quaternion.Euler(Angles3);
            Joint2.transform.rotation = joint2Rotation;
            Joint2.transform.position = Joint1.transform.position + Joint1.transform.up * linkLength;

            // Link2 position and rotation
            Link2.transform.position = (Joint1.transform.position + Joint2.transform.position) / 2;
            Link2.transform.rotation = Joint1.transform.rotation;

            // Joint3 rotation and position
            Quaternion joint3Rotation = Joint2.transform.rotation * Quaternion.Euler(Angles4);
            Joint3.transform.rotation = joint3Rotation;
            Joint3.transform.position = Joint2.transform.position + Joint2.transform.up * linkLength;

            // Link3 position and rotation
            Link3.transform.position = (Joint2.transform.position + Joint3.transform.position) / 2;
            Link3.transform.rotation = Joint2.transform.rotation;

            // EndEffector position and rotation
            EndEffector.transform.position = Joint3.transform.position + Joint3.transform.up * linkLength;
            EndEffector.transform.rotation = Joint3.transform.rotation;

            // Link4 position and rotation
            Link4.transform.position = (Joint3.transform.position + EndEffector.transform.position) / 2;
            Link4.transform.rotation = Joint3.transform.rotation;
        }
    }

    void LogPositions()
    {
        if (EndEffector != null)
        {
            Debug.Log("End Effector Position: " + EndEffector.transform.position);
            Debug.Log("End Effector Rotation: " + EndEffector.transform.rotation);
        }
    }
}