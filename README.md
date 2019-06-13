# Path Planning

## Overview

This package contains an implementation of path planning within an OctoMap. It uses the OMPL Library to create safe obstacle-free and smooth trajectories that reach a specific goal.

## Usage

Run the main node with

	roslaunch path_planning path_planning.launch


## Launch files

* **path_planning.launch:**  Run the main node

## Nodes

### path_planning

Create obstacle-free paths in an OctoMap that are suitable for the drone.

#### Subscribed Topics

* **`/octomap_binary`** [octomap_msgs/Octomap]

	The map that is used for path planning.

* **`/amcl_pose`** [geometry_msgs/PoseStamped]

	The real-time position estimation of the drone.

* **`/next_goal`** [geometry_msgs/TransformStamped]

	The next goal that the drone must reach and a path towards it must be created.

#### Published Topics

* **`/waypoints`** [trajectory_msgs/MultiDOFJointTrajectory]

	The set of points that the drone must navigate through in order to achieve a smooth and obstacle-free navigation.

* **`/waypoints_smooth`** [trajectory_msgs/MultiDOFJointTrajectory]

	A smoother version of the trajectory, after smoothing it using B-spline functions.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/kosmastsk/thesis/issues).
