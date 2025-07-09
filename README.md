# Bimanual_Control

This repository contains the ROS 2 implementation of a **bimanual control system** using two Interbotix RX200 robotic arms in a **leader–follower configuration**. The system is designed for safe, synchronized manipulation of shared objects using real‑time motion coordination and mutual validity checks.

---

## Project Overview

- **Leader Arm**  
  - Receives Cartesian pose goals (e.g., via teleoperation or predefined trajectory).  
  - Computes inverse kinematics (IK) to generate joint commands.

- **Follower (Mimic) Arm**  
  - Mirrors the leader’s end‑effector pose in its own base frame.  
  - Computes corresponding joint states using IK.

- **Mutual Validity Logic**  
  - Motion execution is only triggered when both robots produce valid joint solutions.  
  - Ensures coordinated and fail‑safe execution.

---

## Features

- Modular ROS 2 (`rclpy`) architecture  
- Real‑time synchronization between two robot arms  
- Safety‑critical mutual gating using Boolean flags  
- Support for Interbotix RX‑series robots (tested on RX200)  
- TF2‑based frame transformations for pose mirroring  
- Logging and dry‑run modes for testing and debugging  

---

## Repository Structure

```
bimanual_control/
├── leader/
│   ├── leader_node.py           # IK computation & joint-state publisher for leader arm
│   └── ik_utils.py              # IK service helper functions
├── mimic/
│   ├── mimic_node.py            # Pose mirroring & IK for mimic arm
│   └── tf_utils.py              # TF2 utilities for frame transforms
├── interfaces/
│   └── custom_interfaces/       # Shared ROS 2 message types and control flags
├── launch/
│   └── bimanual.launch.py       # Launch file for both arms and control nodes
├── config/
│   └── robot_params.yaml        # Robot-specific parameters
└── README.md                    # This documentation
```

---

## Dependencies

- **ROS 2 Humble** (or compatible distribution)  
- `interbotix_ros_core` & `interbotix_ros_toolbox`  
- `rclpy`, `tf2_ros`, `geometry_msgs`, `sensor_msgs`, `std_msgs`

### Installation

```bash
# Clone and build workspace
cd ~/ros2_ws/src
git clone https://github.com/Drunk-ed/Bimanual_Control.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---


### How to Run the System

Before running the control nodes, launch the Interbotix drivers for both robots and set up the necessary TF transform.

```bash
# Terminal 1: Launch the Leader Robot (rx200_1)
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=rx200 robot_name:=rx200_1 use_sim:=true

# Terminal 2: Launch the Mimic Robot (mimic)
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=rx200 robot_name:=mimic use_sim:=true use_rviz:=false

# Terminal 3: Start Omni Device Listener (if required)
ros2 run omni_common omni_state

# Terminal 4: Publish Static Transform Between the Two Arms
ros2 run tf2_ros static_transform_publisher 0.58 0 0 -3.14 0 0 rx200_1/base_link mimic/base_link

# Terminal 5: Run Leader Node
ros2 run bimanual_control leader_node

# Terminal 6: Run Mimic Node
ros2 run bimanual_control mimic_node
```

> Make sure the `dry_run` flag inside the nodes is set appropriately depending on whether you're using real hardware or testing in simulation.


## System Flow

1. **Leader Node**  
   - Subscribes to a target `PoseStamped`.  
   - Computes IK via `interbotix_ros_core` service.  
   - Publishes `sensor_msgs/JointState` and a Boolean validity flag.

2. **Mimic Node**  
   - Listens to the leader’s end‑effector pose and transforms it into the mimic’s frame.  
   - Computes its own IK solution.  
   - Publishes its `JointState` only if **both** validity flags are `true`.

3. **Execution Logic**  
   - Both robots move only when both IK solutions are valid.  
   - Prevents unsafe or partial motions that could drop or collide shared objects.

---

## Visualization

Use RViz for debugging TF frames and joint states:

```bash
ros2 launch interbotix_xsarm_descriptions rviz.launch.py robot_model:=rx200
```

