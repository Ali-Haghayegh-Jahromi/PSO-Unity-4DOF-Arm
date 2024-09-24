
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

### `FourLinkRobot`
- **Method: `Update`**: Updates the kinematics of the robotic arm every frame.
- **Method: `CalculateKinematics`**: Calculates the position and orientation of each link using forward kinematics.

### `PSOManager`
- **Method: `Start`**: Initializes the PSO process.
- **Method: `Run`**: Manages the main PSO loop and updates particles based on fitness.

### `RA_PSO`
- **Method: `FitnessFunction`**: Evaluates the fitness of each set of joint angles based on penalties.

## Contributing

Contributions are welcome! If you have ideas for improvements or new features, feel free to open an issue or submit a pull request.

## License

This project is licensed under the GNU Affero General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

- Unity for providing a versatile simulation environment.
- Open-source communities for sharing knowledge and resources on RL and PSO.
