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
A very limited "obstacle avoidance" feature is included. In addition of the low-level path planner avoidance, the robot will not move into an occupied region even when given the order to. The `goto_<region>` action will not output a goal pose command if the region it is supposed to go to is occupied. Occupation is checked by simply looking at the `current_region` topic of all agent included in the `obstacle_names` list parameter. All agents in `obstacle_names` must use a region_2d_pose_monitor and the same `2d_pose_region` state model in their transition system.

## Config files
- **nexus_ltl_formula.yaml** Example of LTL formula with both hard and soft task.

- **nexus_ts.yaml** Example of LTL transition system definition.

## Launch files

- **ltl_nexus.launch**: Example of the LTL planner implementation with the nexus as agent. Run the planner node and nexus node with an example TS (Transition System) and example LTL formula.
    - `initial_ts_state_from_agent` If false, get initial TS (Transition System) state from the TS definition text parameter. If true, get initial TS state from agent topic. Default: `true`.
    - `agent_name` Agent name. Default: `nexus`.

-  **ltl_nexus_gazebo.launch**: Gazebo simulation of the LTL planner implementation with the nexus as agent. Run the planner node and nexus node with an example TS (Transition System) and example LTL formula.
    - `initial_ts_state_from_agent` If false, get initial TS (Transition System) state from the TS definition text parameter. If true, get initial TS state from agent topic. Default: `true`.
    - `agent_name` Agent name. Default: `nexus`.

-  **ltl_nexus_hil.launch**: Gazebo simulation with HIL (Human-in-The-Loop) and IRL (Inverse Reinforcement Learning) features. Run the planner node and nexus node with an example TS (Transition System) and example LTL formula.
    - `initial_ts_state_from_agent` If false, get initial TS (Transition System) state from the TS definition text parameter. If true, get initial TS state from agent topic. Default: `true`.
    - `agent_name` Agent name. Default: `nexus`.
    
## Nodes
### ltl_automaton_nexus_node.py
LTL Nexus node, execute the action sent by the LTL planner and returns the aggregated TS state from the state monitors. The nexus load state monitor is integrated within the nexus node and switching state in automatically done after completed the relevant action.

#### Actions
*Action published topics*
- `move_base/goal` ([move_base_msgs/MoveBaseActionGoal](http://docs.ros.org/en/api/move_base_msgs/html/msg/MoveBaseActionGoal.html))
    
    Send pose command to reach region when executing the action `goto_<region>`.

#### Subscribed Topics
- `next_move_cmd` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

    Next move from the output word (action sequence) to be carried out by the agent in order to satisfy the plan.
    
- `current_region` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

    Agent region from the transition system state model `2d_pose_region`.
  
- `placed_box_ack` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

    Feedback for the action `pick_box`. The action is considered completed when an acknowledgement message is received on this topic.

- `placed_assembly_ack` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

    Feedback for the action `pick_assembly`. The action is considered completed when an acknowledgement message is received on this topic.
  
- `delivered_assembly_ack` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

    Feedback for the action `deliver_assembly`. The action is considered completed when an acknowledgement message is received on this topic.

- `<obstacle_name>/current_region` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))
    
    For each obstacle in the `obstacle_names` list parameter, get the current region and mark it s occupied.
    
#### Published Topics
- `ts_state` ([ltl_automaton_msgs/TransitionSystemStateStamped](/ltl_automaton_msgs/msg/TransitionSystemStateStamped.msg))

    Agent TS state topic. The agent TS state is composed of a list of states from the different state models composing the action model. The Nexus node aggretates the `2d_pose_region` state from a region_2d_pose_monitor with the internal load state.
    
#### Parameters
- `agent_name` (string, default: "agent")

    Agent name. NOT IN USE, SHOULD BE REMOVED?
    
- `transition_system_textfile` (string)

    Action model transition system definition.
    
- `obstacle_names` (string[])

    List of obstacle to be tracked to check if a region is occupied. All tracked agents must use the region_2d_pose_monitor and the same `2d_pose_region` state model in their transition system.
