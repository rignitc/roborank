# RoboRank Solutions

Welcome to the **RoboRank Solutions** repository! This repo contains the solutions for the robotics control and algorithm challenges available at [roborank.dev](https://roborank.dev).

## General Overview

The repository is structured around the various tasks provided by RoboRank, located in the `challenges/` directory. For each challenge, you will find a `robot_policy.py` file which contains the implemented `RobotPolicy` in Python to solve that specific task. These solutions have been / have to be submitted at the site for each solutions.

### Solved Challenges

The repository currently includes **6** solutions for the following challenges:
- **`diff_drive_odometry`**: Estimating the pose of a differential drive robot using encoder and gyro data.
- **`imu_collision_detection`**: Detecting collisions using IMU sensor data.
- **`inverse_kinematics_1` & `inverse_kinematics_2`**: Solving inverse kinematics for robotic arms.
- **`motor_torque_scale_control`**: Controlling motor torque.
- **`trapezoidal_motion_profile`**: Generating and following a trapezoidal motion profile.

More will be added as they are solved and submitted with verified results.

Each challenge is self contained and has its own README.md file with a helpful description of the approach used and the score obtained with that approach.

## Getting Started

### Prerequisites

You will need Python installed on your system. The project uses `uv` for dependency management and execution, and relies on the `roborank` package and `rerun-sdk` for evaluating the solutions.

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/rignitc/roborank.git
   cd roborank
   ```

2. Install the required dependencies using `pip` or `uv`:
   ```bash
   uv venv
   source .venv/bin/activate
   uv pip install -r requirements.txt
   ```

### How to use this repository

1. Clone and install dependencies.
2. Check the constraints, given utilities and classes in the boilerplate from the site. Edit `challenges/<challenge-ID>/robot_policy.py` accordingly to implement the solution.
3. run `uv run roborank eval run <challenge-ID> --policy-source robot_policy.py --out ../../logs/<challenge-ID> --json` for each challenge in `challenges` folder.
4. Check the results locally, and once satisfied with results / score, submit at the site.

### Running a Solution

Each challenge has a certain challenge ID assigned to it, which can be viewed at the site for each problem statement under the **Environment** section. The ID is mentioned in the problem description itself as `roborank/<challenge-ID>`.

Each challenge validation has to be run using the cli tool `roborank`, in the format:

```bash
uv run roborank eval run <challenge-ID> --policy-source robot_policy.py --out ../../logs/<challenge-ID> --json
```

where <challenge-ID> is the ID of the challenge you want to run.

### Running a Solution - An Example

Each challenge directory contains a `local_run.sh` script to help you execute the policy locally and verify the solution.

To run a solution via the script:

```bash
cd challenges/diff_drive_odometry
chmod +x local_run.sh # required only once for each script in each folder!
./local_run.sh
```


To run a solution manually using `uv`, you can use the `roborank eval run` command. For example, to evaluate the `diff_drive_odometry` solution:

```bash
cd challenges/diff_drive_odometry
uv run roborank eval run diff_drive_odometry --policy-source robot_policy.py --out ../../logs/ddo --json
```

This will run the solution script (`robot_policy.py`) against the `diff_drive_odometry` challenge using the RoboRank CLI, and output the results to the specified log directory.


## Contributing


1. Create a new directory in the `challenges/` folder with the name of the challenge.
2. Copy the boilerplate `robot_policy.py` file to the new directory.
3. Implement the solution in the `robot_policy.py` file. 
4. Open a pull request with the title "feat: Add solution for challenge-ID", e.g. "feat: Add solution for diff_drive_odometry".
5. Verify that the solution passes all the tests in the CI pipeline.
6. Once the tests pass, the solution will be merged into the main branch.