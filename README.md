#  Heart-Shaped Trajectory with Franka Robot

This project demonstrates how to execute a **pre-defined heart-shaped trajectory** created in MATLAB, using the **Franka Emika Panda** robot with **MoveIt** framework in the **Gazebo** simulation environment. The trajectory is recorded using rosbags and analyzed using MATLAB.


## Folder Structure

- **`ir_project/`**

  Ros package that contains the ROS node to execute the generated trajectory.

- **`Data Creation & Analysis/`**

   - MATLAB scripts for creating, reading and analyzing the data
  - `.bag` file with the executed trajectory data  
  - `.csv` files for the data of the designed trajectory and the robot positions and orientations after execution 


## Prerequisites

- [ROS Noetic]
- [MoveIt configured for Franka Emika Panda]
- Gazebo simulator
- MATLAB


## How to Execute the Code

### Clone and Build the Package

```bash
cd ~/ws_moveit/src
git clone https://github.com/OzanOezel/Franka-Trajectory/tree/main/ir_project  # Clone the ir_project package
cd ~/ws_moveit
source ~/ws_moveit/devel/setup.bash
catkin build ir_project
```



### Launch Simulation (Gazebo + RViz)

```bash
roslaunch panda_moveit_config demo_gazebo.launch
```

This will launch both **Gazebo** and **RViz** with the Franka Panda robot.



### Run the Trajectory Controller

In a second terminal:

```bash
cd ~/ws_moveit
source ~/ws_moveit/devel/setup.bash
rosrun ir_project trajectory_controller
```

This will execute the heart-shaped trajectory.



### Step 4: Record Rosbag Data

In a third terminal:

```bash
rosbag record -o heart_trajectory_data.bag /tf /tf_static /joint_states
```

This records the transformation frames and joint states during trajectory execution.



## MATLAB Analysis

The MATLAB code can be executed independently. ROS is not needed since the `.bag` file is available inside.

