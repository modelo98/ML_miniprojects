# Machine Learning Applied to Robotics

This repository contains a progressive personal learning project focused on the connection between mobile robotics, mathematical modeling, sensor-based navigation, and Machine Learning.

The main idea of the project is to build a complete learning path from basic 2D geometry and robot motion to supervised learning methods that can imitate or approximate robot control behavior.

The project starts with simple vector operations and gradually introduces robot orientation, coordinate transformations, trajectory generation, obstacle detection, reactive control, noisy sensors, filtering, artificial potential fields, dataset generation, and a first Machine Learning model.

## Project Goal

The goal of this project is to understand how classical robotics concepts can be transformed into data-driven learning problems.

The general progression is:

```text
geometry → robot motion → sensors → obstacle avoidance → data generation → machine learning
```

Instead of starting directly with neural networks, the project first builds the mathematical and robotic foundations needed to understand what the model should learn.

## Current Progress

The project has currently been completed up to:

```text
exp_23_linear_regression_speed.py
```

At this point, the project has already reached the first Machine Learning stage.

A supervised dataset has been generated from an expert controller, and a linear regression model has been implemented to learn the relationship between the robot's distance to the goal and its linear velocity.

## Repository Structure

```text
Robotica/
├── data/
│   └── navigation_dataset_30.csv
│
├── experiments/
│   ├── exp_01_vectors.py
│   ├── exp_02_multiple_goals.py
│   ├── exp_03_angles.py
│   ├── ...
│   ├── exp_22_dataset_generation.py
│   └── exp_23_linear_regression_speed.py
│
├── src/
│   ├── __init__.py
│   └── math_utils.py
│
|
│
├── requirements.txt
└── README.md
```
## Paper Folders

Some project folders include a `paper/` directory containing the LaTeX files, figures, and material used to create the final PDF. The experiments and mini-projects are explained mathematically in the final document, so for reading purposes it is only necessary to open **<u>`main.pdf`</u>** inside each `paper/` folder.

## Main Topics Covered

So far, the project covers the following areas:

- 2D vectors and Euclidean distance
- Direction vectors and normalization
- Robot heading and orientation
- Dot product and spatial classification
- Angle computation with `atan2`
- Angle normalization
- Local and global coordinate transformations
- 2D robot visualization with Matplotlib
- Circular obstacle representation
- Clearance and distance to obstacles
- Robot trajectory generation
- Discrete robot motion
- Unicycle motion model
- Go-to-goal control
- Speed control based on distance
- Waypoint navigation
- Sensor rays mounted on the robot
- Ray-based obstacle detection
- Reactive obstacle avoidance
- Combination of goal-seeking and obstacle avoidance
- Sensor noise simulation
- Filtering of noisy sensor data
- Artificial potential fields
- Gradient-based navigation
- Supervised dataset generation
- Linear regression for robot speed prediction

## Experiments Completed

### exp_01_vectors.py

Introduces basic 2D vector operations. The robot and goal are represented as points in a plane. The experiment computes the vector from the robot to the goal, the Euclidean distance, and the normalized direction vector.

### exp_02_multiple_goals.py

Extends the first experiment to multiple goals. The robot evaluates several target positions and uses the dot product with its heading direction to classify whether each goal is in front of it, behind it, or to the side.

### exp_03_angles.py

Introduces angle computation using `atan2`. The robot computes the target angle toward a goal, the angular error with respect to its current orientation, and the normalized angular error.

### exp_04_rotation_heading.py

Introduces 2D rotation matrices and heading vectors. The robot orientation is converted into forward and lateral direction vectors.

### exp_05_local_global_transform.py

Implements transformations between the robot's local coordinate frame and the global/world coordinate frame.

### exp_06_basic_2d_visualization.py

Creates the first basic 2D visualization using Matplotlib. The robot, goal, heading, and direction vector are drawn.

### exp_07_obstacles_visualization.py

Adds circular obstacles to the environment. The experiment computes distance to obstacle centers and clearance from obstacle boundaries.

### exp_08_robot_trajectory.py

Introduces robot trajectories as sequences of positions. The experiment compares the total path length with the direct distance to the goal.

### exp_09_simple_motion.py

Implements simple forward motion using the robot heading, velocity, and time step.

### exp_10_unicycle_motion.py

Introduces the unicycle motion model with linear velocity, angular velocity, and orientation update.

### exp_11_go_to_goal_controller.py

Implements a proportional go-to-goal controller. The robot computes the angular error to the target and turns toward it.

### exp_12_go_to_goal_with_speed_control.py

Adds speed control based on the distance to the goal. The robot slows down as it approaches the target and stops within a tolerance region.

### exp_13_waypoint_navigation.py

Extends go-to-goal navigation to a sequence of waypoints. The robot switches to the next waypoint when it reaches the current one.

### exp_14_sensor_rays.py

Introduces simple range sensors represented as rays mounted on the robot. The sensors are defined in the robot's local frame and transformed into the global frame.

### exp_15_obstacle_detection.py

Implements obstacle detection using ray-circle geometry. The robot checks whether a sensor ray intersects circular obstacles and stores the closest detection.

### exp_16_reactive_obstacle_avoidance.py

Uses sensor readings to create a reactive obstacle avoidance behavior. The robot turns away from obstacles using simple sensor-action rules.

### exp_17_goal_seeking_with_obstacles.py

Combines go-to-goal navigation with reactive obstacle avoidance. The robot tries to reach a goal while avoiding obstacles using behavior arbitration.

### exp_18_sensor_noise.py

Adds Gaussian noise to range sensor readings. The experiment compares true distance, measured distance, and measurement error.

### exp_19_filtering_sensor_data.py

Implements filtering methods for noisy sensor data, including moving average and exponential filtering. The experiment studies the trade-off between smoothing and response delay.

### exp_20_potential_fields.py

Implements artificial potential fields for robot navigation. The goal generates an attractive force, while obstacles generate repulsive forces. The robot moves according to the normalized total force.

### exp_21_gradient_navigation.py

Connects potential fields with gradient-based navigation. The robot moves in the direction of decreasing potential, linking robot navigation with optimization concepts.

### exp_22_dataset_generation.py

Generates a supervised dataset from an expert controller. Each row stores a robot state, sensor readings, goal-related information, and the expert action.

The dataset has the structure:

```text
robot_x
robot_y
robot_theta
goal_x
goal_y
distance_to_goal
angle_error
front_sensor
left_sensor
right_sensor
v
omega
```

This transforms the robot control problem into a supervised learning problem:

```text
robot state + sensors + goal → expert action
```

### exp_23_linear_regression_speed.py

Implements a first Machine Learning model using linear regression.

The model learns the relationship:

```text
distance_to_goal → linear velocity
```

The learned model has the form:

```text
v_pred = w * distance_to_goal + b
```

The experiment computes the regression parameters, makes predictions, evaluates the error using MSE and MAE, and visualizes the real expert data together with the learned regression line.

## Machine Learning Stage

The project has now entered the Machine Learning phase.

The first learning experiment is intentionally simple. It uses only one input feature:

```text
distance_to_goal
```

to predict one output:

```text
v
```

This makes it possible to understand the basic supervised learning workflow:

```text
dataset → feature selection → model fitting → prediction → error evaluation
```

The linear regression model does not perfectly imitate the expert controller because the expert behavior also depends on sensors and obstacle avoidance rules. Therefore, two samples with the same distance to the goal may require different velocities depending on whether an obstacle is detected.

This is an important result: it shows why more features and more expressive models are needed for realistic robot behavior learning.

## Key Learning Ideas

This project shows how classical robotics and Machine Learning are connected.

Classical robotics part:

```text
geometry
motion models
sensors
controllers
obstacle avoidance
potential fields
```

Machine Learning part:

```text
dataset generation
features and labels
supervised learning
regression
prediction error
model limitations
```

The most important conceptual transition is:

```text
programmed controller → expert data → learned model
```

## How to Run Experiments

From the root folder of the project:

```bash
source .venv/bin/activate
python3 -m experiments.exp_23_linear_regression_speed
```

Other experiments can be run in the same way:

```bash
python3 -m experiments.exp_01_vectors
python3 -m experiments.exp_10_unicycle_motion
python3 -m experiments.exp_22_dataset_generation
```

## Dataset

The dataset generated in `exp_22_dataset_generation.py` is stored in:

```text
data/navigation_dataset_30.csv
```

It contains random valid robot states, sensor readings, goal information, and expert actions.

The dataset is used in `exp_23_linear_regression_speed.py` to train and evaluate the first regression model.

## Current Limitations

The current linear regression model is very limited because it only uses the distance to the goal as input.

However, the expert controller also depends on:

```text
angle_error
front_sensor
left_sensor
right_sensor
obstacle detections
```

Therefore, a single linear model using only distance cannot fully reproduce the expert behavior.

This limitation is intentional and useful, because it motivates the next steps.

## Next Steps

The next planned stages are:

```text
exp_24_safe_danger_classifier.py
→ classify situations as safe or dangerous using sensor readings

exp_25_neural_policy.py
→ train a small neural network to predict robot actions

exp_26_behavior_cloning.py
→ imitate the expert controller using supervised learning

exp_27_train_test_evaluation.py
→ evaluate generalization using train/test splits

exp_28_decision_boundary_visualization.py
→ visualize classification boundaries for safe/danger situations
```

## Project Vision

The long-term goal is to build a complete learning-based robot navigation laboratory.

The final direction is to develop a system where a robot can perceive its environment, avoid obstacles, move toward goals, generate data from expert behavior, and train Machine Learning models to approximate or improve control decisions.

This project is a step-by-step bridge between:

```text
robotics fundamentals
mathematical modeling
sensor-based control
supervised learning
robot learning
```
