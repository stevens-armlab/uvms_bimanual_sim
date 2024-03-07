# Haptic Bimanual Teleoperation Simulation

## Prerequisites
- Install [uuv_simulator](https://uuvsimulator.github.io/installation/)
- Install [haptic_control](https://github.com/stevens-armlab/haptic_control)
- Install [teleop_core](https://github.com/stevens-armlab/teleop_core)
- Install [gazebo_grasp_fix](https://github.com/JenniferBuehler/gazebo-pkgs)

## Spawning and Controlling Dual-Arm Rexrov
- NOTE: If using haptic devices via ethernet, run the following when beginning each new terminal:
  - `export ROS_IP=192.168.1.XX`
  - `export ROS_MASTER_URI=http://192.168.1.XX:11311/`
  - XX is the IP address of the local machine (running gazebo)
- Launch gazebo using **one** of the following commands:
  - `roslaunch uuv_gazebo_worlds empty_underwater_world.launch`
  - `roslaunch uuv_gazebo_worlds ocean_waves.launch`
  - Or launch any other world in uuv_gazebo_worlds
- Spawn vehicle: `roslaunch rexrov_dual_oberon_description upload_dual_rexrov_oberon7.launch`
- Launch vehicle velocity PID + orientation stabilization controller using **one** of the following commands:
  - If using gamepad controller: `roslaunch rexrov_dual_oberon_control joy_velocity.launch`
  - If using haptic controller to generate vehicle commands: `roslaunch rexrov_dual_oberon_control haptic_control.launch`
- Launch manipulataor joint position PID controller: `roslaunch rexrov_dual_oberon_control manipulator_joint_control.launch`

## Generaing Manipulator Commands
- Run joint position publisher node (converts redundancy resolution commands to manipulator_joint_control commands): `rosrun rexrov_dual_oberon_control manipulator_publisher.py`
- Run redundancy resolution node: `rosrun haptic_control redundancy_controller_oberon.py `
- (Optional) Launch manipulator visualization node using **one** of the following commands:
  - Just the left manipulator: `roslaunch rexrov_dual_oberon_description display.launch`
  - Both manipulators: `roslaunch rexrov_dual_oberon_description dual_display.launch`
- Run teleop node: `rosrun teleop_core teleop_cmd.py`
- See [teleop_core](https://github.com/stevens-armlab/teleop_core) for more info about using haptic devices
- gazebo_grasp_plugin allows manipulators to grasp objects by rigidly fixing them to the gripper links when opposing contact forces are detected.

TIP: Closing Gazebo using CTRL-C takes a very long time, use `killall -9 gzserver gzclient`