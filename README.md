# ltl_automaton_nexus
Allow to use Nexus 4WD mecanum robot as LTL agent of ltl_automaton_core package.

## Installation

### Dependencies
- [ltl_automaton_core](https://github.com/KTH-SML/ltl_automaton_core). Core ROS package for LTL.

- [sml_nexus_simulator](https://github.com/KTH-SML/sml_nexus_simulator). Nexus robot simulation package, only needed for running gazebo simulation

- [sml_nexus](https://github.com/KTH-SML/sml_nexus). Nexus robot package, only needed for running gazebo simulation

- [motion_capture_simulator](https://github.com/KTH-SML/motion_capture_simulator.git). Simulate the motion capture system in gazebo, only needed for running gazebo simulation.

- [PyYAML](https://pyyaml.org/). Should be integrated with ROS but it's better to check if version is up-to-date.
	- For Python2 (ROS Kinetic & Melodic):
	`pip install pyyaml`
	- For Python3 (ROS Morenia):
	`pip3 install pyyaml`
  
### Building
To build the package, clone the current repository in your catkin workspace and build it.
```
cd catkin_ws/src
git clone https://github.com/KTH-SML/ltl_automaton_nexus.git
```
Build your workspace with either *catkin_make* or *catkin build*
```
cd ...
catkin_make
```

## Usage
The package provides the agent-level code needed for interacting with the Nexus robot.

To launch the planner and LTL nexus node, simply run the following command. Please notice that a move_base node needs to be running for the nexus robot to receive the velocity commands.

```
roslaunch ltl_automaton_nexus ltl_nexus.launch
```
This will run the planner using the task specification in `config/nexus_ltl_formula.yaml` and transition system in `config/nexus_ts.yaml` and the obstacle list `["nexus2", "nexus3"]`.

To run the aformentioned program with a simulation of the nexus robot in Gazebo, run the following:
```
roslaunch ltl_automaton_nexus ltl_nexus_gazebo.launch
```

### Transition system and actions
The robot transition system needs to be of the following type: `[2d_pose_region, nexus_load]`. The robot can carry the following actions:
- `goto_<region>` (from *2d_pose_region*): Go to the defined regions. The action needs to be part of the transition system textfile with the following attributes
  
  ```Python
  attr:
      region: r6                   # Region name
      pose: [[1,1.2,0], [0,0,0,1]] # Pose of center of region:
                                   # [origin [x,y,0], orientation quaternion [x,y,z,w]]
  ```
  
- `pick_box`  (from *nexus_load*): Perform pick up action. The nexus robot doesn't do anything per se but wait for confirmation of placed box.

- `pick_assembly`  (from *nexus_load*): Perform pick up action of full assembly. The nexus robot doesn't do anything per se but wait for confirmation of placed assembly.

- `deliver_assembly`  (from *nexus_load*): Perform deliver action. The nexus robot doesn't do anything per se but wait for confirmation that assembly has been taken off.

### Obstacle avoidance
A very limited "obstacle avoidance" feature is included. In addition of the low-level path planner avoidance, the robot will not move into an occupied region even when given the order to. The `goto_<region>` action will not output a goal pose command if the region it is supposed to go to is occupied. Occupation is checked by simply looking at the `current_region` topic of all agent included in the `obstacle_names` list parameters. All agents in `obstacle_names` must use a region_2d_pose pose monitor and the same `2d_pose_region` state model in their transition system.

## Config files

## Launch files

## Nodes
