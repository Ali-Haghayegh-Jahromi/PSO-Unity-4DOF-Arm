
# RL-PSO-Unity-4DOF-Arm

**RL-PSO-Unity-4DOF-Arm** is a simulation project that integrates Reinforcement Learning (RL) and Particle Swarm Optimization (PSO) for trajectory generation of a 4 Degrees of Freedom (4DOF) robotic arm in the Unity 2023 environment. This project demonstrates advanced AI-based motion planning and control techniques, including obstacle avoidance, minimum effort and time penalties, and precision penalties for efficient and accurate performance.

## Features

- **RL/PSO Approach**: Reinforcement Learning and Particle Swarm Optimization for effective trajectory generation.
- **4-DOF Robotic Arm**: Simulated robotic arm with four degrees of freedom, providing complex movement capabilities.
- **Obstacle Avoidance**: Ensures the arm navigates around obstacles while reaching the target.
- **Performance Penalties**: Includes minimum effort, time, and precision penalties to optimize trajectories.
- **Real-Time Visualization**: Visual representation of the arm's movements and trajectory in Unity.

## Prerequisites

- Unity 2023 or later
- Git
- Basic understanding of Reinforcement Learning and Particle Swarm Optimization

## Installation

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/Al-Hagh-J/RL-PSO-Unity-4DOF-Arm.git
   ```
2. **Open in Unity:**
   - Open Unity Hub.
   - Click on `Add` and select the cloned project directory.
3. **Install Required Packages:**
   - Make sure to install any required Unity packages for physics and machine learning.

## Usage

1. **Run the Simulation:**
   - Open the `MainScene` in the Unity Editor.
   - Press the `Play` button to start the simulation.
2. **Modify Parameters:**
   - Adjust the RL/PSO parameters in the `PSOManager` script to see how different settings impact the arm's performance.
3. **View Results:**
   - Observe the robotic arm's trajectory and performance in the Unity scene.

## Configuration

The following parameters can be adjusted in the `PSOManager` script:

- **`numParticles`**: Number of particles in the swarm.
- **`numIterations`**: Maximum number of iterations for the PSO algorithm.
- **`fitnessThreshold`**: Threshold value for fitness to enable early stopping.
- **`penaltyWeights`**: Weights for various penalties (position, rotation, movement, etc.).

## Classes and Methods


### Class: `FourLinkRobot`
This class represents a 4 Degrees of Freedom (4DOF) robotic arm. It is responsible for managing the arm's kinematics and updating its position and orientation in the simulation environment.

#### Methods:
- `Update()`
    - This method is called every frame to update the state of the robotic arm. It calculates the current joint angles and updates the kinematic properties of each joint and link.

- `CalculateKinematics()`
    - This method computes the forward kinematics of the robotic arm. It calculates the position and orientation of each link based on the joint angles using transformation matrices. The positions and orientations are updated sequentially, starting from the base and propagating through each joint.

--------------------------------------------------

### Class: `PSOManager`
This class manages the Particle Swarm Optimization (PSO) process for optimizing the robotic arm's trajectory. It initializes the PSO parameters and coordinates the optimization loop.

#### Methods:
- `Start()`
    - Initializes the PSO process by setting up initial parameters and validating the target position. It creates an instance of the RA_PSO class and prepares it for optimization.

- `Run()`
    - Executes the main PSO loop, evaluating particles based on their fitness. It updates the positions and velocities of the particles according to the PSO algorithm. The method iterates through the particles, updating their velocities and positions based on personal and global bests.

--------------------------------------------------

### Class: `RA_PSO`
This class implements the Particle Swarm Optimization (PSO) algorithm. It is responsible for evaluating the fitness of each particle and updating the global best solution.

#### Methods:
- `FitnessFunction()`
    - Calculates the fitness of a given set of joint angles for the robotic arm. The fitness function incorporates several penalties, including position error, rotation error, overfitting, movement smoothness, and collision. It returns a fitness value that is used to evaluate and compare different particle solutions.

--------------------------------------------------

## Properties:
- ### position
    - The current set of joint angles representing the particle's position in the solution space.

- ### velocity
    - The change in the particle's position, which is updated during each iteration based on the particle's and swarm's best known positions.

- ### p_best
    - The best position (set of joint angles) the particle has achieved so far.

- ### fitness
    - The fitness value associated with the particle's current position.

--------------------------------------------------

## Additional Methods:

- `CheckForCollisions()` (in FourLinkRobot)
    - Interpolates between joint configurations to check for any collisions with obstacles. This method ensures that the robotic arm's path is free from collisions throughout its movement.

- `RobotVisualization()` (in FourLinkRobot or a related class)
    - Handles the visualization of the robot's joint angles in real-time. It interpolates between different joint angles to ensure smooth visual transitions during movement.
    - 
## Images
![Screenshot (2)](https://github.com/user-attachments/assets/2681968a-f18a-4094-b022-2fe24ed3a7b3)
![Screenshot (3)](https://github.com/user-attachments/assets/7d316068-9ad8-4fb2-a677-0ff6b2c05fc1)


## Contributing

Contributions are welcome! If you have ideas for improvements or new features, feel free to open an issue or submit a pull request.

## License

This project is licensed under the GNU Affero General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

- Unity for providing a versatile simulation environment.
- Open-source communities for sharing knowledge and resources on RL and PSO.
