## swarm_sync_sim

### Introduction
A synchronized (lock-stepped) numerical simulation platform for multi-robot swarm system based on ROS. It offers a **lightweight, scalable and fast** (10x) simulation engine to verify your motion planning and control algorithms for various kinds of robots in seconds before real experiments. Quadrotors, unmannded ground vehicles (UGV), fixed-wing UAVs, and customized models are supported.


### Installation

```bash
# Dependencies
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-robot-state-publisher -y

# Build
git clone https://gitee.com/bhswift/swarm_sync_sim.git
cd swarm_sync_sim/
catkin_make
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
```

### Usage

#### PX4 Rotor Simulation

Launch px4 rotor simulation nodes (mavros + px4 stil + quadrotor dynamics + visualization):

```bash
### 1. Launch sim clock
roslaunch sss_sim_env sim_clock.launch max_speed_ratio:=1 auto_start:=true # You can also specify max_simulation_rate and auto_start in the launch file

### 2. Launch multiple mavros-px4-rotor sim nodes
roslaunch px4_rotor_sim multi_px4_rotor_sim.launch # Specify initial positions in the launch file
```
![image_name](pictures/multi-px4-rotor-sim.png)

To control the simulation clock:

```bash
# @param proceed: true(go on), false(stop)
# @param max_sim_speed: 0.0(keep unchanged)
rosservice call /sss_clock_control "proceed: true
max_sim_speed: 0.0"
```

Then you can launch your control algorithm nodes to communicate with the mavros topics and services.

Some useful commands:

```bash
### Arm UAV1
rosservice call /uav1/mavros/cmd/arming "value: true" 

### Send position setpoints to UAV1
rostopic pub /uav1/mavros/setpoint_raw/local mavros_msgs/PositionTarget "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
coordinate_frame: 1
type_mask: 0b110111111000
position: {x: 0.0, y: 1.0, z: 2.0}
velocity: {x: 0.0, y: 0.0, z: 0.0}
acceleration_or_force: {x: 0.0, y: 0.0, z: 0.0}
yaw: 0.0
yaw_rate: 0.0" -r 10

### Switch UAV1 into offboard mode:
rosservice call /uav1/mavros/set_mode "base_mode: 0 
custom_mode: 'OFFBOARD'"
```





### Advanced

1.  xxxx
2.  xxxx
3.  xxxx


### Contributions

This project is developed by xxx in 2023.5.12

### Picture

Attach pictures here (upload images to pictures/ folder first):

![image_name](pictures/img1.png)
