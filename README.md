# Kinematic Control for TurtleBot3

A ROS 2 package implementing kinematic control algorithms for TurtleBot3 mobile robots. This package provides implementations of odometry handling, velocity control, and feedback control systems for autonomous navigation using Gazebo simulation.

## Overview

This project is designed to control TurtleBot3 robots using kinematic models and feedback control laws. It includes utilities for monitoring odometry data, publishing velocity commands, and implementing the Siegwart feedback control algorithm for goal-directed navigation. All development and testing is done in the Gazebo simulator with TurtleBot3.

## Features

- **Odometry Monitoring**: Utilities to wait for and subscribe to odometry data from TurtleBot3
- **Velocity Control**: Publish velocity commands to control robot motion
- **Siegwart Controller**: Implements a feedback control law for point-to-point navigation using polar error coordinates
- **Gazebo Simulation**: Integrated launch files for testing with TurtleBot3 in Gazebo
- **PlotJuggler Integration**: Real-time visualization of robot pose and controller performance
- **ROS 2 Integration**: Full compatibility with ROS 2 middleware
- **tf_transformations**: Includes quaternion to Euler angle conversion for pose estimation

## Requirements

- **ROS 2** (tested with standard ROS 2 distributions)
- **Python 3.6+**
- **TurtleBot3 packages**:
  - `turtlebot3_gazebo` - Gazebo simulation environment
  - `turtlebot3_bringup` - Hardware/simulation bringup
- **Optional**: PlotJuggler for visualization

### Dependencies

- `rclpy` - ROS 2 Python client library
- `geometry_msgs` - Standard ROS 2 geometry message types
- `nav_msgs` - Navigation message types (Odometry)
- `turtlesim` - Turtle simulator for basic testing
- `tf_transformations` - Quaternion and transformation utilities

## Installation

### 1. Install TurtleBot3 Packages
```bash
sudo apt-get install ros-<distro>-turtlebot3-gazebo ros-<distro>-turtlebot3-bringup
```
Replace `<distro>` with your ROS 2 distribution (e.g., `humble`, `iron`).

### 2. Clone the Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/SciNoLimits/Kinematic-Control-TurtleBot3.git
cd ..
```

### 3. Build the Package
```bash
colcon build --packages-select kinematic_control_turtle_bot_3
source install/setup.bash
```

## Available Executables

### 1. `wait_for_odom`
Waits for the first odometry message from the robot. Useful for ensuring the simulation/hardware is ready before starting the controller.

```bash
ros2 run kinematic_control_turtle_bot_3 wait_for_odom
```

**Usage**: Automatically called by the launch file before starting the controller.

### 2. `minimal_velocity_publisher`
Simple velocity publisher that commands the robot to move forward at 0.1 m/s.

```bash
ros2 run kinematic_control_turtle_bot_3 minimal_velocity_publisher
```

**Example Output**:
```
[INFO] Minimal Velocity Publisher has been started
```

### 3. `minimal_odometry_subscriber`
Subscribes to odometry and logs the robot's pose (x, y, yaw).

```bash
ros2 run kinematic_control_turtle_bot_3 minimal_odometry_subscriber
```

**Example Output**:
```
[INFO] x=0.123, y=0.456, yaw=1.234
```

### 4. `tb3_siegwart_controller`
Implements the Siegwart feedback control algorithm for autonomous navigation to a goal pose.

```bash
ros2 run kinematic_control_turtle_bot_3 tb3_siegwart_controller
```

**Features**:
- Navigates to a specified goal position and orientation
- Uses polar error coordinates (ρ, α, β) for feedback control
- Respects hardware velocity limits with proportional saturation
- 10 Hz control loop
- Automatic stopping when goal is reached

## Configuration

Controller parameters are configurable via `config/controller_params.yaml`:

```yaml
tb3_siegwart_controller:
    ros__parameters:
        # Goal Pose (in meters and radians)
        x_goal: 1.0         # Target x position
        y_goal: 1.0         # Target y position
        theta_goal: 0.0     # Target orientation

        # Controller Gains
        k_rho: 0.4          # Forward velocity gain
        k_alpha: 0.8        # Heading angle gain
        k_beta: -0.15       # Robot orientation gain

        # Goal Tolerance
        rho_tol: 0.05       # Stop within 5 cm of goal

        # Hardware Limits (TurtleBot3 Burger)
        v_max: 0.22         # Maximum linear velocity (m/s)
        w_max: 2.84         # Maximum angular velocity (rad/s)
```

### Tuning Control Gains
- **k_rho**: Increases forward velocity toward the goal. Higher values = faster approach
- **k_alpha**: Adjusts heading angle error. Controls how quickly the robot turns toward the goal direction
- **k_beta**: Adjusts final robot orientation error. Negative values ensure smooth approach angle

## Launch Files

### tb3_siegwart_controller.launch.py
Complete launch file that orchestrates the entire system:

```bash
ros2 launch kinematic_control_turtle_bot_3 tb3_siegwart_controller.launch.py
```

**What it does**:
1. Launches Gazebo with the empty world and TurtleBot3
2. Waits for odometry to be available
3. Starts the Siegwart controller
4. Launches PlotJuggler for real-time visualization (commented out by default)

**Launch Flow**:
```
Gazebo (TurtleBot3) → Wait for Odom → Siegwart Controller + PlotJuggler
```

## Usage Examples

### Basic Test with Minimal Velocity Publisher
```bash
# Terminal 1: Launch Gazebo with TurtleBot3
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Terminal 2: Subscribe to odometry
ros2 run kinematic_control_turtle_bot_3 minimal_odometry_subscriber

# Terminal 3: Publish velocity commands
ros2 run kinematic_control_turtle_bot_3 minimal_velocity_publisher
```

### Full Autonomous Navigation (Recommended)
```bash
# Single command launches everything:
ros2 launch kinematic_control_turtle_bot_3 tb3_siegwart_controller.launch.py
```

This will:
- Start Gazebo simulator
- Spawn TurtleBot3 in an empty world
- Wait for odometry
- Start the controller to navigate to the goal pose specified in `config/controller_params.yaml`
- Display control logs and status

### Enable PlotJuggler Visualization
Edit `launch/tb3_siegwart_controller.launch.py` and uncomment line 74:
```python
ld.add_action(action=plotjuggler)
```

Then rebuild and run:
```bash
colcon build --packages-select kinematic_control_turtle_bot_3
ros2 launch kinematic_control_turtle_bot_3 tb3_siegwart_controller.launch.py
```

## Topics

### Published
- `/cmd_vel_stamped` (TwistStamped): Velocity commands with timestamp (Siegwart controller)
- `/cmd_vel` (Twist): Basic velocity commands (minimal publisher)

### Subscribed
- `/odom` (Odometry): Robot odometry data from Gazebo/TurtleBot3

## Architecture

### Node Structure
```
WaitForOdom
├── Waits for first /odom message
└── Exits to trigger next nodes

SiegwartController
├── Subscribes: /odom
├── Publishes: /cmd_vel_stamped
├── Control Loop: 10 Hz
└── Uses polar error feedback

MinimalVelocityPublisher
├── Publishes: /cmd_vel
└── Test publisher for basic motion

MinimalOdometrySubscriber
└── Logs: Robot pose (x, y, yaw)
```

### Control Algorithm
The Siegwart controller uses a two-stage feedback control approach:

1. **Polar Error Computation**:
   - ρ (rho): Distance to goal
   - α (alpha): Angle from robot heading to goal direction
   - β (beta): Difference between goal orientation and direction to goal

2. **Control Law**:
   ```
   v = k_rho * ρ                    (linear velocity)
   w = k_alpha * α + k_beta * β     (angular velocity)
   ```

3. **Saturation**: Commands are scaled proportionally to respect hardware limits:
   ```
   scale = min(1.0, v_max/|v|, w_max/|w|)
   ```

4. **Goal Detection**: When ρ < rho_tol, the controller stops and exits

## Testing

### Unit Tests
```bash
cd ~/ros2_ws
colcon test --packages-select kinematic_control_turtle_bot_3
```

### Manual Testing Checklist
- [ ] Gazebo launches successfully
- [ ] Robot appears in simulation
- [ ] `/odom` topic publishes data
- [ ] Controller starts after odometry is available
- [ ] Robot moves toward goal
- [ ] Robot stops at goal (within tolerance)
- [ ] Control parameters are logged at ~1 Hz

## Expected Behavior

When you launch the full system:

1. **Initialization** (~5 seconds):
   ```
   [INFO] Checking for Odometry...
   [INFO] Odometry data received.
   ```

2. **Control Loop** (continuous):
   ```
   [INFO] rho =1.234 alpha =0.567 beta =-0.123 v =0.300 w =0.500
   [INFO] rho =1.100 alpha =0.450 beta =-0.100 v =0.330 w =0.480
   ...
   ```

3. **Goal Reached**:
   ```
   [INFO] Goal reached. rho = 0.0234
   [INFO] Robot Stopped.
   ```

## File Structure

```
kinematic_control_turtle_bot_3/
├── launch/
│   └── tb3_siegwart_controller.launch.py    # Main launch file
├── config/
│   └── controller_params.yaml               # Controller configuration
├── plotjuggler_layout/
│   └── xy_pose_layout.xml                   # Visualization layout
├── kinematic_control_turtle_bot_3/          # Python package
│   ├── __init__.py
│   ├── wait_for_odom.py
│   ├── minimal_velocity_publisher.py
│   ├── minimal_odometry_subscriber.py
│   └── tb3_siegwart_controller.py
├── resource/
│   └── kinematic_control_turtle_bot_3
├── test/
├── package.xml
└── setup.py
```

## Troubleshooting

### "Gazebo fails to launch"
```bash
# Ensure TurtleBot3 Gazebo package is installed
sudo apt-get install ros-<distro>-turtlebot3-gazebo

# Set the model
export TURTLEBOT3_MODEL=burger
# or
export TURTLEBOT3_MODEL=waffle
```

### "Odometry data not received"
- Check that Gazebo is running and TurtleBot3 is spawned
- Verify `/odom` topic is publishing:
  ```bash
  ros2 topic echo /odom --once
  ```
- Check Gazebo plugins are loaded (check Gazebo terminal output)

### "Robot doesn't move or moves in wrong direction"
- Verify `/cmd_vel_stamped` is being published:
  ```bash
  ros2 topic echo /cmd_vel_stamped --once
  ```
- Check controller gains in `config/controller_params.yaml`
- Verify robot can respond to manual velocity commands:
  ```bash
  ros2 topic pub /cmd_vel_stamped geometry_msgs/msg/TwistStamped \
    "{header: {stamp: now}, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"
  ```

### "Controller exits immediately"
- Check if goal is very close to starting position
- Increase `rho_tol` in configuration if goal is hard to reach precisely
- Verify initial pose in Gazebo matches odometry

### "High computational load / lag in Gazebo"
- Reduce simulation quality settings in Gazebo
- Close PlotJuggler if not needed
- Check system resources with `top` command

## License

MIT License

## Authors

- **Prajwal Dutta (SciNoLimits)** - Initial development and implementation
- Email: prazwaldutta7@gmail.com

## References

- [ROS 2 Documentation](https://docs.ros.org/)
- [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [Gazebo Documentation](https://gazebosim.org/)
- [PlotJuggler](https://github.com/facontidavide/PlotJuggler)
- Siegwart et al. - Introduction to Autonomous Mobile Robots (Feedback control law)

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.
