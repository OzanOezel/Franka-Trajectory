# Heart Shaped Franka Robot Trajectory
In this project, a pre-defined trajectory is executed using moveit and the franka robot in the Gazebo environment. The resulting data is then analyzed.

cd ~/ws_moveit
source ~/ws_moveit/devel/setup.bash
roslaunch panda_moveit_config demo_gazebo.launch 
rosrun ir_project trajectory_controller 
rosbag record -o name.bag /tf /tf_static /joint_states
