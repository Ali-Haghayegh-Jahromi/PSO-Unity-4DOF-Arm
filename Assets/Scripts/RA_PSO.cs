using System.Collections.Generic;
using UnityEngine;

public class RA_PSO
{
    public int numParticles = 1000;
    public int numIterations = 100000; // Reduced iterations for hardware limitations
    public float fitnessThreshold = 1.0f; // Threshold for stopping early

    public float w = 0.4f;  // Starting Inertia weight
    public float c1 = 2.75f; // Starting Cognitive weight
    public float c2 = 2.75f; // Starting Social weight

    // Penalty Weights
    public float positionPenaltyWeight = 1.0f; // Normalized Position Penalty
    public float rotationPenaltyWeight = 5.0f; // Normalized Rotation Penalty
    public float overfittingPenaltyWeight = 1.0f;  // Normalized Over-fitting Penalty
    public float movementPenaltyWeight = 1.0f;  // Normalized Movement Penalty
    public float timePenaltyWeight = 3.0f; // Normalized Time Penalty
    public float collisionPenalty = 1000000.0f; // Huge penalty for collisions

    public Vector3[] globalBestPosition; // Best joint angles found
    public float globalBestFitness = float.MaxValue;
    public List<Vector3[]> allBestPositions; // Store best positions for each iteration
    public Vector3 finalPosition;
    public Quaternion finalRotation;
    public int bestIteration = 0;  // To store the iteration of the best fitness

    private Particle[] particles;
    public Vector3 targetPosition; // Desired end effector position
    public Vector3 targetEulerAngles; // Desired end effector rotation in Euler angles
    public GameObject obstacle;  // Reference to the obstacle GameObject
    public GameObject[] robotLinks; // Array of GameObjects representing robot links

    private int numJoints = 4; // Number of joints in the robot
    private float linkLength = 1.0f; // Fixed length for each link

    public class Particle
    {
        public Vector3[] Position;   // The current position (joint angles)
        public Vector3[] Velocity;   // The current velocity
        public Vector3[] BestPosition; // The best position (joint angles) found so far
        public float Fitness;        // The current fitness value
        public float BestFitness;    // The best fitness value found so far
        public float TotalTransitionTime; // Accumulated transition time
    }

    public void Initialize()
    {
        allBestPositions = new List<Vector3[]>();
        particles = new Particle[numParticles];
        globalBestPosition = new Vector3[numJoints]; // Initialize the global best position

        for (int i = 0; i < numParticles; i++)
        {
            particles[i] = new Particle
            {
                Position = new Vector3[numJoints], // Initialize joint angles
                Velocity = new Vector3[numJoints], // Initialize velocities
                BestPosition = new Vector3[numJoints],
                Fitness = float.MaxValue,
                BestFitness = float.MaxValue,
                TotalTransitionTime = 0.0f
            };

            for (int j = 0; j < numJoints; j++)
            {
                // Narrower range for initialization to improve convergence
                particles[i].Position[j] = new Vector3(
                    Random.Range(-45, 45), 
                    Random.Range(-45, 45), 
                    Random.Range(-45, 45)
                );
                particles[i].Velocity[j] = new Vector3(Random.Range(-1, 1), Random.Range(-1, 1), Random.Range(-1, 1));
            }
        }
    }

    public void Run()
    {
        float fitness = 99.0f;
        for (int iteration = 0; iteration < numIterations; iteration++)
        {
            // Adaptive parameters
            float currentW  = Mathf.Lerp(0.9f, 0.4f,  iteration / (float)numIterations);
            float currentC1 = Mathf.Lerp(1.5f, 2.75f, iteration / (float)numIterations);
            float currentC2 = Mathf.Lerp(1.5f, 2.75f, iteration / (float)numIterations);

            foreach (var particle in particles)
            {
                // Calculate fitness with the updated comprehensive function
                fitness = FitnessFunction(targetPosition, targetEulerAngles, particle.Position, iteration, ref particle.TotalTransitionTime);

                // Update personal best
                if (fitness < particle.BestFitness)
                {
                    particle.BestFitness = fitness;
                    particle.BestPosition = (Vector3[])particle.Position.Clone();
                }

                // Update global best
                if (fitness < globalBestFitness)
                {
                    globalBestFitness = fitness;
                    globalBestPosition = (Vector3[])particle.Position.Clone();
                    bestIteration = iteration; // Store the iteration with the best fitness
                }
            }

            // Stop early if the overall fitness is within the defined threshold of fitness
            if (globalBestFitness <= fitnessThreshold)
            {
                Debug.Log($"Stopping early at iteration {iteration}");
                break;
            }

            // Store the best position of this iteration
            allBestPositions.Add((Vector3[])globalBestPosition.Clone());

            // Update velocities and positions
            foreach (var particle in particles)
            {
                for (int j = 0; j < numJoints; j++)
                {
                    float r1 = Random.Range(0.0f, 1.0f);
                    float r2 = Random.Range(0.0f, 1.0f);
                    particle.Velocity[j] = currentW * particle.Velocity[j] +
                                           currentC1 * r1 * (particle.BestPosition[j] - particle.Position[j]) +
                                           currentC2 * r2 * (globalBestPosition[j] - particle.Position[j]);

                    // Update position and calculate transition time
                    Vector3 previousPosition = particle.Position[j];
                    particle.Position[j] += particle.Velocity[j];
                    particle.TotalTransitionTime += Vector3.Distance(previousPosition, particle.Position[j]);

                    // Ensure the updated position is within valid joint limits
                    particle.Position[j] = new Vector3(
                        Mathf.Clamp(particle.Position[j].x, -180, 180),
                        Mathf.Clamp(particle.Position[j].y, -180, 180),
                        Mathf.Clamp(particle.Position[j].z, -180, 180));
                }
            }
        }

        // After completing the PSO run, log the final expected position, rotation, and best fitness
        LogFinalResults();
    }

    private float FitnessFunction(Vector3 targetPos, Vector3 targetEulerAngles, Vector3[] jointAngles, int currentIterationNumber, ref float totalTransitionTime)
    {
        Vector3 position;
        Quaternion rotation;
        CalculateForwardKinematics(jointAngles, out position, out rotation);

        // Normalized penalties
        float positionPenalty = positionPenaltyWeight * 30.0f * Vector3.Distance(position, targetPos);
        float rotationPenalty = rotationPenaltyWeight * 15.0f * Quaternion.Angle(rotation, Quaternion.Euler(targetEulerAngles));
        float overfittingPenalty = overfittingPenaltyWeight * 2.0f * CalculateJointMovementPenalty(jointAngles);
        float movementPenalty = movementPenaltyWeight * 0.2f * CalculateSimplicityPenalty(jointAngles);
        
        // Time penalty: scaled down to prevent large numbers
        float timePenalty = timePenaltyWeight * 0.001f * totalTransitionTime;

        // Collision penalty
        float collisionPenaltyValue = CheckForIntermediateCollisions(jointAngles) ? collisionPenalty : 0.0f;

        // Log the total transition time for debugging
        Debug.Log($"Total Transition Time: {totalTransitionTime}");

        return positionPenalty + rotationPenalty + overfittingPenalty + movementPenalty + timePenalty + collisionPenaltyValue;
    }

    private float CalculateJointMovementPenalty(Vector3[] jointAngles)
    {
        float penalty = 0.0f;

        for (int i = 1; i < jointAngles.Length; i++)
        {
            penalty += Vector3.Distance(jointAngles[i], jointAngles[i - 1]);
        }

        return penalty * 0.01f; // Adjust the coefficient as needed
    }

    private float CalculateSimplicityPenalty(Vector3[] jointAngles)
    {
        float penalty = 0.0f;

        for (int i = 1; i < jointAngles.Length; i++)
        {
            penalty += Mathf.Abs(jointAngles[i-1].magnitude); // Penalize large absolute values of joint angles
            penalty += 10.0f * Vector3.Distance(jointAngles[i], jointAngles[i - 1]);
        }

        return penalty * 0.05f; // Adjust the coefficient as needed
    }

    private bool CheckForIntermediateCollisions(Vector3[] jointAngles)
    {
        int steps = 10; // Number of steps to interpolate between joint configurations
        for (int step = 0; step <= steps; step++)
        {
            float t = step / (float)steps;
            Vector3[] interpolatedAngles = InterpolateJointAngles(globalBestPosition, jointAngles, t);

            if (CheckForCollision(interpolatedAngles))
            {
                return true; // Collision detected at this intermediate position
            }
        }

        return false; // No collision detected
    }

    private Vector3[] InterpolateJointAngles(Vector3[] startAngles, Vector3[] endAngles, float t)
    {
        Vector3[] interpolatedAngles = new Vector3[numJoints];
        for (int i = 0; i < numJoints; i++)
        {
            interpolatedAngles[i] = Vector3.Lerp(startAngles[i], endAngles[i], t);
        }
        return interpolatedAngles;
    }

    private bool CheckForCollision(Vector3[] jointAngles)
    {
        CalculateForwardKinematics(jointAngles, out Vector3 position, out _);

        if (obstacle == null || robotLinks == null)
            return false;

        if (IsColliding(obstacle, position))
            return true;

        foreach (var link in robotLinks)
        {
            if (IsColliding(obstacle, link.transform.position))
                return true;
        }

        return false;
    }

    private bool IsColliding(GameObject obstacle, Vector3 position)
    {
        Bounds obstacleBounds = obstacle.GetComponent<Collider>().bounds;
        return obstacleBounds.Contains(position);
    }

    private void CalculateForwardKinematics(Vector3[] jointAngles, out Vector3 position, out Quaternion rotation)
    {
        position = Vector3.zero;
        rotation = Quaternion.identity;

        for (int i = 0; i < jointAngles.Length; i++)
        {
            Quaternion jointRotation = Quaternion.Euler(jointAngles[i].x, jointAngles[i].y, jointAngles[i].z);
            rotation *= jointRotation; // Apply joint rotation

            position += rotation * Vector3.up * linkLength;
        }
    }

    private void LogFinalResults()
    {
        CalculateForwardKinematics(globalBestPosition, out finalPosition, out finalRotation);

        Debug.Log($"Final Expected Position: {finalPosition}");
        Debug.Log($"Final Expected Rotation: {finalRotation.eulerAngles}");

        // Find the particle corresponding to the globalBestPosition
        float totalTransitionTime = 0.0f;
        foreach (var particle in particles)
        {
            if (System.Linq.Enumerable.SequenceEqual(particle.BestPosition, globalBestPosition))
            {
                totalTransitionTime = particle.TotalTransitionTime;
                break;
            }
        }

        Debug.Log($"Total Transition Time: {totalTransitionTime}");

        // Log individual penalty contributions
        Vector3 bestPosition;
        Quaternion bestRotation;
        CalculateForwardKinematics(globalBestPosition, out bestPosition, out bestRotation);

        float positionPenalty = positionPenaltyWeight * 30.0f * Vector3.Distance(bestPosition, targetPosition);
        float rotationPenalty = rotationPenaltyWeight * 15.0f * Quaternion.Angle(bestRotation, Quaternion.Euler(targetEulerAngles));
        float overfittingPenalty = overfittingPenaltyWeight * 2.0f * CalculateJointMovementPenalty(globalBestPosition);
        float movementPenalty = movementPenaltyWeight * 0.2f * CalculateSimplicityPenalty(globalBestPosition);
        float timePenalty = timePenaltyWeight * 0.001f * totalTransitionTime;
        float collisionPenaltyValue = CheckForCollision(globalBestPosition) ? collisionPenalty : 0.0f;

        Debug.Log($"{positionPenalty} + {rotationPenalty} + {overfittingPenalty} + {movementPenalty} + {timePenalty} + {collisionPenaltyValue} = {globalBestFitness}");
    }

}