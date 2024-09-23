using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PSOManager : MonoBehaviour
{
    public Vector3 targetPosition;              // Desired end effector position
    public Vector3 targetEulerAngles;           // Desired end effector rotation in Euler angles
    public float maxReach = 4.0f;               // Maximum reach of the robot arm

    public RA_PSO pso;                          // Reference to the RA_PSO instance
    public FourLinkRobot robot;                 // Reference to the robot script for controlling joint angles
    public GameObject obstacle;                 // Reference to the obstacle GameObject

    private bool psoCalculated = false;         // Flag to check if PSO calculations have finished
    private bool visualizationStarted = false;  // Flag to check if visualization has started
    private float visualizationSpeed = 0.05f;   // Speed of the visualization
    private int currentIteration = 0;           // Current step in the visualization process
    private float timer = 0.0f;                 // Timer for the interpolation
    private List<Vector3[]> allBestPositions;   // Positions to visualize

    void Start()
    {
        if (ValidateTarget())
        {
            pso = new RA_PSO();
            pso.targetPosition = targetPosition;
            pso.targetEulerAngles = targetEulerAngles;
            pso.obstacle = obstacle;            // Set the obstacle in the PSO instance

            pso.Initialize();
            Debug.Log("PSO instance initialized successfully.");
        }
        else
        {
            Debug.LogError("Invalid target position or rotation. PSO not initialized.");
        }
    }

    void Update()
    {
        // Handle the first space bar press to run the PSO calculations
        if (Input.GetKeyDown(KeyCode.Space) && !psoCalculated)
        {
            pso.Run();
            psoCalculated = true; // Mark PSO as calculated
            allBestPositions = pso.allBestPositions; // Store the best positions
            Debug.Log("PSO calculation finished. Press Space again to start the visualization.");
        }
        // Handle the second space bar press to start the visualization
        else if (Input.GetKeyDown(KeyCode.Space) && psoCalculated && !visualizationStarted)
        {
            visualizationStarted = true; // Start visualization
            Debug.Log("Visualization started.");
        }

        // If visualization has started, interpolate the robot movements
        if (visualizationStarted && allBestPositions != null && currentIteration < allBestPositions.Count)
        {
            timer += Time.deltaTime;
            if (timer >= visualizationSpeed)
            {
                timer = 0.0f; // Reset the timer
                SetRobotAngles(allBestPositions[currentIteration]); // Set the robot angles
                currentIteration++; // Move to the next position
            }
        }
    }

    void SetRobotAngles(Vector3[] jointAngles)
    {
        if (robot != null)
        {
            robot.Angles1 = jointAngles[0];
            robot.Angles2 = jointAngles[1];
            robot.Angles3 = jointAngles[2];
            robot.Angles4 = jointAngles[3];
            robot.CalculateKinematics();
        }
        else
        {
            Debug.LogError("Robot reference is not set.");
        }
    }

    private bool ValidateTarget()
    {
        if (targetPosition.magnitude > maxReach)
        {
            Debug.LogError($"Target position {targetPosition} is out of reach. Max reach is {maxReach}.");
            return false;
        }

        if (targetEulerAngles.x < -180 || targetEulerAngles.x > 180 ||
            targetEulerAngles.y < -180 || targetEulerAngles.y > 180 ||
            targetEulerAngles.z < -180 || targetEulerAngles.z > 180)
        {
            Debug.LogError("Target Euler angles are out of valid range. Must be within [-180, 180] degrees.");
            return false;
        }

        return true;
    }
}
