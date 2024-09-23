using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotVisualization : MonoBehaviour
{
    public PSOManager psoManager; // Reference to the PSOManager script
    public FourLinkRobot robot; // Reference to the robot script for controlling joint angles
    public float visualizationSpeed = 100.0f; // Speed of the visualization

    private int currentIteration = 0;
    private float timer = 0.0f;
    private List<Vector3[]> allBestPositions;
    private bool isVisualizing = false; // Flag to check if visualization has started

    void Start()
    {
        // Ensure PSOManager is set
        if (psoManager == null)
        {
            Debug.LogError("PSOManager is not set in RobotVisualization.");
            return;
        }

        // Ensure PSOManager has initialized the PSO instance
        if (psoManager.pso == null)
        {
            Debug.LogError("PSO instance in PSOManager is not set.");
            return;
        }

        // Ensure PSO has run and positions are available
        allBestPositions = psoManager.pso.allBestPositions;
        if (allBestPositions == null || allBestPositions.Count == 0)
        {
            Debug.LogError("PSO has not run or no positions are available.");
            return;
        }

        Debug.Log("PSOManager and its PSO instance are set. Number of positions: " + allBestPositions.Count);

        // Proceed with visualization setup
    }

    void Update()
    {
        if (isVisualizing && allBestPositions != null && currentIteration < allBestPositions.Count)
        {
            // Update timer
            timer += Time.deltaTime * visualizationSpeed;

            // If timer exceeds a threshold (e.g., 1 second), move to the next set of angles
            if (timer > 1.0f)
            {
                timer = 0.0f;        
                SetRobotAngles(allBestPositions[currentIteration]);
                currentIteration++;
            }
        }
    }

    public void StartVisualization()
    {
        isVisualizing = true;
        currentIteration = 0;
        timer = 0.0f;
        Debug.Log("Starting visualization of robot movement.");
    }

    void SetRobotAngles(Vector3[] jointAngles)
    {
        if (robot != null)
        {
            // Update the robot's joint angles
            robot.Angles1 = jointAngles[0];
            robot.Angles2 = jointAngles[1];
            robot.Angles3 = jointAngles[2];
            robot.Angles4 = jointAngles[3];

            // Log the applied angles for debugging
            //Debug.Log("Setting joint angles: " + string.Join(", ", jointAngles));

            // Manually trigger the kinematics calculation
            robot.CalculateKinematics();
        }
        else
        {
            Debug.LogError("Robot reference is not set.");
        }
    }
}
