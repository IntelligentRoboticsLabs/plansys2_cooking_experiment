# PlanSys2 Cooking Experiment

[![](https://img.youtube.com/vi/2yCuZLhCzKk/0.jpg)](https://www.youtube.com/watch?v=2yCuZLhCzKk&feature=youtu.be "Click to play on You Tube")


## Description

This experiment corresponds to the paper "PlanSys2: A Planning System Framework for ROS2" being evaluated for IROS. In the experiments, a scenario is proposed that emulates a kitchen where the robot must continuously prepare dishes. This repository contains the code and instructions to reproduce the experiment, both in the real robot Tiago and simulated with three robots at the same time.

## Requisitos

* Install ROS2 and Nav2 following the instructions [here](https://navigation.ros.org/build_instructions/index.html#quickstart-build-main)
  Before compile the workspace, change the ros1_bridge for [this version](https://github.com/fmrico/ros1_bridge/tree/dedicated_bridges).
    - Is important to compile this workspace, compile first without the ros1_bridge:
        ```
        colcon build --symlink-install --packages-skip ros1_bridge
        ```
    - If the compilation conclude and everything is OK do:
        ```
        source /opt/ros/noetic/setup.bash
        colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

* (Install PlanSys2)(https://intelligentroboticslab.gsyc.urjc.es/ros2_planning_system.github.io/build_instructions/index.html)
* Clone this repo in your workspace and compile it.


## Running in Tiago

You would need to put this in .bashrc:

```
export ROS_IP=10.68.0.129
export ROS_MASTER_URI=http://tiago-81c:11311
```

In terminal 1 run the bridges:

```
source [WHEREVER_YOU_CREATED]/ros2_ws/install/setup.bash
source /opt/ros/noetic/setup.bash
ros2 launch ros1_bridge dedicated_bridges_launch.py 
```

In terminal 2 start the Nav2 system:

```
source [WHEREVER_YOU_CREATED]/nav2_ws/install/setup.bash
ros2 launch plansys2_cooking_example nav2_tiago_cooking_launch.py
```

In terminal 3 start PlanSy2 with all the action nodes of the experiment:

```
source [WHEREVER_YOU_CREATED]/plansys2_ws/install/setup.bash
ros2 launch plansys2_cooking_example plansys2_cooking_example_launch.p
```

In terminal 4 start the experiment:

```
source [WHEREVER_YOU_CREATED]/plansys2_ws/install/setup.bash
ros2 run plansys2_cooking_example cooking_controller_node
```

## Run the simulation

In terminal 1 start PlanSy2 with all the action nodes of the experiment:

```
source [WHEREVER_YOU_CREATED]/plansys2_ws/install/setup.bas
ros2 launch plansys2_cooking_example plansys2_cooking_example_launch_three_robots_sim.py
```

In terminal 2 start the experiment:

```
source [WHEREVER_YOU_CREATED]/nav2_ws/install/setup.bash
ros2 run plansys2_cooking_example cooking_controller_node_three_robots_sim
```

You can check the temporary file in /tmp (/tmp/domain.pddl, /tmp/problem.pddl, /tmp/plan, and /tmp/bt.xml)

[![](https://img.youtube.com/vi/rFFjTr4s0zs/0.jpg)](https://www.youtube.com/watch?v=rFFjTr4s0zs&feature=youtu.be "Click to play on You Tube")
