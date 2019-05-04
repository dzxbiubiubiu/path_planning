/*
* Copyright 2017 Ayush Gaud
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include "path_planning/path_planning.h"

namespace path_planning
{
Planner::Planner()
{
  // Initialize some variables
  _path_smooth = NULL;
  _replan_flag = false;
  _set_start = false;

  _octree_sub = _nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &Planner::octomapCallback, this);
  _odom_sub = _nh.subscribe<nav_msgs::Odometry>("/odom", 1, &Planner::odometryCallback, this);
  _goal_sub = _nh.subscribe<geometry_msgs::TransformStamped>("/next_goal", 1, &Planner::goalCallback, this);

  _vis_pub = _nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  _traj_pub = _nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/waypoints", 1);
  _smooth_traj_pub = _nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/waypoints_smooth", 3);

  ROS_INFO("OMPL version %s\n", OMPL_VERSION);

  // Add some more width to the drone box, to keep a safe distance from obstacles
  _quadrotor = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(1.5, 1.5, 0.2));
  fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.05)));
  _tree = std::shared_ptr<fcl::CollisionGeometry>(tree);

  _space = ompl::base::StateSpacePtr(new ompl::base::SE3StateSpace());
}

Planner::~Planner()
{
}

void Planner::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
  // Load octomap msg
  octomap::AbstractOcTree* abstract = octomap_msgs::msgToMap(*msg);
  octomap::ColorOcTree* tree_coloct = dynamic_cast<octomap::ColorOcTree*>(abstract);

  // convert ColorOcTree to OcTree
  octomap::OcTree* tree_oct = reinterpret_cast<octomap::OcTree*>(tree_coloct);

  // convert octree to collision object
  fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
  ROS_INFO("Octomap loaded\n");

  double min_bounds[3];
  double max_bounds[3];
  tree_oct->getMetricMin(min_bounds[0], min_bounds[1], min_bounds[2]);
  tree_oct->getMetricMax(max_bounds[0], max_bounds[1], max_bounds[2]);

  initializations(min_bounds, max_bounds);

  // Update the octree used for collision checking
  updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
  replan();
}

void Planner::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  setStart(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
           msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
           msg->pose.pose.orientation.w);
  initStart();
}

void Planner::goalCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  setGoal(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z,
          msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
}

void Planner::initializations(double min_bounds[3], double max_bounds[3])
{
  // create a start state
  ompl::base::ScopedState<ompl::base::SE3StateSpace> start(_space);

  // create a goal state
  ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(_space);

  // set the bounds for the R^3 part of SE(3)
  ompl::base::RealVectorBounds bounds(3);

  bounds.setLow(0, min_bounds[0]);   // x min
  bounds.setHigh(0, max_bounds[0]);  // x max
  bounds.setLow(1, min_bounds[1]);   // y min
  bounds.setHigh(1, max_bounds[1]);  // y max
  bounds.setLow(2, min_bounds[2]);   // z min
  bounds.setHigh(2, max_bounds[2]);  // z max

  _space->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

  // construct an instance of _space information from this state _space
  _si = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(_space));

  goal->setXYZ(0, 0, 0);
  _prev_goal[0] = 0;
  _prev_goal[1] = 0;
  _prev_goal[2] = 0;
  _prev_goal[3] = 0;
  _prev_goal[4] = 0;
  _prev_goal[5] = 0;
  _prev_goal[6] = 1;
  goal->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(0, 0, 0, 1);
  // goal.random();

  // set state validity checking for this _space
  _si->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1));

  // create a problem instance
  _pdef = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(_si));

  // set the start and goal states
  _pdef->setStartAndGoalStates(start, goal);

  // set Optimizattion objective
  _pdef->setOptimizationObjective(Planner::getPathLengthObjWithCostToGo(_si));
}

bool Planner::isStateValid(const ompl::base::State* state)
{
  // cast the abstract state type to the type we expect
  const ompl::base::SE3StateSpace::StateType* se3state = state->as<ompl::base::SE3StateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const ompl::base::RealVectorStateSpace::StateType* pos = se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

  // extract the second component of the state and cast it to what we expect
  const ompl::base::SO3StateSpace::StateType* rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);

  fcl::CollisionObject treeObj((_tree));
  fcl::CollisionObject aircraftObject(_quadrotor);

  // check validity of state defined by pos & rot
  fcl::Vec3f translation(pos->values[0], pos->values[1], pos->values[2]);
  fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
  aircraftObject.setTransform(rotation, translation);
  fcl::CollisionRequest requestType(1, false, 1, false);
  fcl::CollisionResult collisionResult;
  fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

  return (!collisionResult.isCollision());
}

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration _space of computed
// paths.
ompl::base::OptimizationObjectivePtr Planner::getThresholdPathLengthObj(const ompl::base::SpaceInformationPtr& si)
{
  ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
  // obj->setCostThreshold(ompl::base::Cost(1.51));
  return obj;
}

ompl::base::OptimizationObjectivePtr Planner::getPathLengthObjWithCostToGo(const ompl::base::SpaceInformationPtr& si)
{
  ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
  obj->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
  return obj;
}

void Planner::initStart()
{
  if (!_set_start)
    ROS_INFO("Planner Initialized\n");
  _set_start = true;
}

void Planner::setStart(double x, double y, double z, double ax, double ay, double az, double a)
{
  ompl::base::ScopedState<ompl::base::SE3StateSpace> start(_space);
  start->setXYZ(x, y, z);
  start->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(ax, ay, az, a);
  _pdef->clearStartStates();
  _pdef->addStartState(start);
}

void Planner::setGoal(double x, double y, double z, double ax, double ay, double az, double a)
{
  if (_prev_goal[0] != x || _prev_goal[1] != y || _prev_goal[2] != z)
  {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(_space);
    goal->setXYZ(x, y, z);
    _prev_goal[0] = x;
    _prev_goal[1] = y;
    _prev_goal[2] = z;
    _prev_goal[3] = ax;
    _prev_goal[4] = ay;
    _prev_goal[5] = az;
    _prev_goal[6] = a;

    goal->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(ax, ay, az, a);
    _pdef->clearGoal();
    _pdef->setGoalState(goal);
    ROS_INFO("Goal point set to : %f %f %f, %f %f %f %f \n", x, y, z, ax, ay, az, a);
    if (_set_start)
      plan();
  }
}

void Planner::updateMap(std::shared_ptr<fcl::CollisionGeometry> map)
{
  _tree = map;
}

void Planner::replan()
{
  if (_path_smooth != NULL && _set_start)
  {
    ROS_INFO("Total Points : %zu\n", _path_smooth->getStateCount());
    if (_path_smooth->getStateCount() <= 2)
      plan();
    else
    {
      for (std::size_t idx = 0; idx < _path_smooth->getStateCount(); idx++)
      {
        if (!_replan_flag)
          _replan_flag = !isStateValid(_path_smooth->getState(idx));
        else
          break;
      }
      if (_replan_flag)
        plan();
      else
        ROS_INFO("Replanning not required\n");
    }
  }
}

void Planner::plan()
{
  // create a Planner for the defined _space
  ompl::base::PlannerPtr plan(new ompl::geometric::InformedRRTstar(_si));

  // set the problem we are trying to solve for the Planner
  plan->setProblemDefinition(_pdef);

  // perform setup steps for the Planner
  plan->setup();

  // print the settings for this _space
  _si->printSettings(std::cout);

  // print the problem settings
  _pdef->print(std::cout);

  // Get the distance to the desired goal for the top solution
  _pdef->getSolutionDifference();

  // attempt to solve the problem within one second of planning time
  ompl::base::PlannerStatus solved = plan->solve(2);

  if (solved)
  {
    // get the goal representation from the problem definition (not the same as the goal state)
    // and inquire about the found path
    ROS_INFO("Found solution:\n");
    ompl::base::PathPtr path = _pdef->getSolutionPath();
    ompl::geometric::PathGeometric* pth = _pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
    pth->printAsMatrix(std::cout);
    // print the path to screen
    // path->print(std::cout);
    trajectory_msgs::MultiDOFJointTrajectory msg;
    trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.joint_names.clear();
    msg.points.clear();
    msg.joint_names.push_back("quadrotor");

    for (std::size_t path_idx = 0; path_idx < pth->getStateCount(); path_idx++)
    {
      const ompl::base::SE3StateSpace::StateType* se3state =
          pth->getState(path_idx)->as<ompl::base::SE3StateSpace::StateType>();

      // extract the first component of the state and cast it to what we expect
      const ompl::base::RealVectorStateSpace::StateType* pos =
          se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

      // extract the second component of the state and cast it to what we expect
      const ompl::base::SO3StateSpace::StateType* rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);

      point_msg.time_from_start.fromSec(ros::Time::now().toSec());
      point_msg.transforms.resize(1);

      point_msg.transforms[0].translation.x = pos->values[0];
      point_msg.transforms[0].translation.y = pos->values[1];
      point_msg.transforms[0].translation.z = pos->values[2];

      point_msg.transforms[0].rotation.x = rot->x;
      point_msg.transforms[0].rotation.y = rot->y;
      point_msg.transforms[0].rotation.z = rot->z;
      point_msg.transforms[0].rotation.w = rot->w;

      msg.points.push_back(point_msg);
    }
    _traj_pub.publish(msg);

    // Path smoothing using bspline
    ompl::geometric::PathSimplifier* pathBSpline = new ompl::geometric::PathSimplifier(_si);
    _path_smooth = new ompl::geometric::PathGeometric(
        dynamic_cast<const ompl::geometric::PathGeometric&>(*_pdef->getSolutionPath()));

    // TODO Find a way to calculate the smoothness I want
    ROS_WARN("Path smoothness : %f\n", _path_smooth->smoothness());
    // Using 5, as is the default value of the function
    // If the path is not smooth, the value of smoothness() will be closer to 1
    int bspline_steps = ceil(5 * _path_smooth->smoothness());

    pathBSpline->smoothBSpline(*_path_smooth, bspline_steps);
    ROS_INFO("Smoothed Path\n");
    _path_smooth->print(std::cout);

    trajectory_msgs::MultiDOFJointTrajectory msg_smooth;
    trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg_smooth;

    msg_smooth.header.stamp = ros::Time::now();
    msg_smooth.header.frame_id = "world";
    msg_smooth.joint_names.clear();
    msg_smooth.points.clear();
    msg_smooth.joint_names.push_back("quadrotor");

    for (std::size_t path_idx = 0; path_idx < _path_smooth->getStateCount(); path_idx++)
    {
      const ompl::base::SE3StateSpace::StateType* se3state =
          _path_smooth->getState(path_idx)->as<ompl::base::SE3StateSpace::StateType>();

      // extract the first component of the state and cast it to what we expect
      const ompl::base::RealVectorStateSpace::StateType* pos =
          se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

      // extract the second component of the state and cast it to what we expect
      const ompl::base::SO3StateSpace::StateType* rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);

      point_msg_smooth.time_from_start.fromSec(ros::Time::now().toSec());
      point_msg_smooth.transforms.resize(1);

      point_msg_smooth.transforms[0].translation.x = pos->values[0];
      point_msg_smooth.transforms[0].translation.y = pos->values[1];
      point_msg_smooth.transforms[0].translation.z = pos->values[2];

      point_msg_smooth.transforms[0].rotation.x = rot->x;
      point_msg_smooth.transforms[0].rotation.y = rot->y;
      point_msg_smooth.transforms[0].rotation.z = rot->z;
      point_msg_smooth.transforms[0].rotation.w = rot->w;

      msg_smooth.points.push_back(point_msg_smooth);
    }
    _smooth_traj_pub.publish(msg_smooth);

    // Publish path as markers
    visualization_msgs::Marker marker;
    // marker.action = visualization_msgs::Marker::DELETEALL;
    //_vis_pub.publish(marker);

    for (std::size_t idx = 0; idx < _path_smooth->getStateCount(); idx++)
    {
      // cast the abstract state type to the type we expect
      const ompl::base::SE3StateSpace::StateType* se3state =
          _path_smooth->getState(idx)->as<ompl::base::SE3StateSpace::StateType>();

      // extract the first component of the state and cast it to what we expect
      const ompl::base::RealVectorStateSpace::StateType* pos =
          se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

      // extract the second component of the state and cast it to what we expect
      const ompl::base::SO3StateSpace::StateType* rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);

      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = "path";
      marker.id = idx;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = pos->values[0];
      marker.pose.position.y = pos->values[1];
      marker.pose.position.z = pos->values[2];
      marker.pose.orientation.x = rot->x;
      marker.pose.orientation.y = rot->y;
      marker.pose.orientation.z = rot->z;
      marker.pose.orientation.w = rot->w;
      marker.scale.x = 0.2;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.a = 1.0;
      marker.color.r = idx * 0.20;
      marker.color.g = idx * 0.20;
      marker.color.b = idx * 0.20;
      _vis_pub.publish(marker);
      ros::Duration(0.1).sleep();
      ROS_INFO("Published marker %zu\n", idx);
    }

    // Clear memory
    _pdef->clearSolutionPaths();
    _replan_flag = false;
  }
  else
    ROS_INFO("No solution found\n");
}

}  // namespace path_planning

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planning_node");

  path_planning::Planner planner;

  ros::spin();
  return 0;
}
