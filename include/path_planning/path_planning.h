#ifndef PATH_PLANNING_HEADER
#define PATH_PLANNING_HEADER

#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace path_planning
{
class Planner
{
private:
  // ROS related
  ros::NodeHandle _nh;

  ros::Subscriber _octree_sub;
  ros::Subscriber _pose_sub;
  ros::Subscriber _goal_sub;

  ros::Publisher _vis_pub;
  ros::Publisher _traj_pub;
  ros::Publisher _smooth_traj_pub;

  // goal state
  double _prev_goal[7];

  bool _replan_flag;
  // Flag for initialization
  bool _set_start;
  bool isStateValid(const ompl::base::State* state);

  // FCL
  std::shared_ptr<fcl::CollisionGeometry> _quadrotor;
  std::shared_ptr<fcl::CollisionGeometry> _tree;

  // OMPL
  // construct the state space we are planning in
  ompl::base::StateSpacePtr _space;
  // construct an instance of  space information from this state space
  ompl::base::SpaceInformationPtr _si;
  // create a problem instance
  ompl::base::ProblemDefinitionPtr _pdef;
  ompl::geometric::PathGeometric* _path_smooth;

  ompl::base::OptimizationObjectivePtr getThresholdPathLengthObj(const ompl::base::SpaceInformationPtr& si);
  ompl::base::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ompl::base::SpaceInformationPtr& si);

  // Callbacks
  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void goalCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);

public:
  Planner();
  ~Planner();

  void initializations(double min_bounds[3], double max_bounds[3]);
  void initStart();
  void setStart(double x, double y, double z);

  void setGoal(double x, double y, double z, double ax, double ay, double az, double a);

  void updateMap(std::shared_ptr<fcl::CollisionGeometry> map);

  void replan();
  void plan();
};
}  // namespace path_planning

#endif
