/*
 * NavigationDemo.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 *
 */

#include "grid_map_cdt/Challenge.hpp"
#include <tf_conversions/tf_eigen.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>
#include <highgui.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen_conversions/eigen_msg.h>

#define PI 22.0/7.0

using namespace grid_map;
using namespace std::chrono;


namespace grid_map_demos {


void quat_to_euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}

NavigationDemo::NavigationDemo(ros::NodeHandle& nodeHandle, bool& success)
    : nodeHandle_(nodeHandle),
      filterChain_("grid_map::GridMap"),
      demoMode_(false)
{
  if (!readParameters()) {
    success = false;
    return;
  }

  subscriber_ = nodeHandle_.subscribe(inputTopic_, 1, &NavigationDemo::callback, this);
  listener_ = new tf::TransformListener();

  outputGridmapPub_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/filtered_map", 1, true);
  if (demoMode_) {
    footstepPlanRequestPub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/footstep_plan_request", 10);
  }  
  ray1pub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ray1", 10);
  ray2pub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ray2", 10);

  actionPub_ = nodeHandle_.advertise<std_msgs::Int16>("/action_cmd", 10);

  drivingRvizSub_  = nodeHandle_.subscribe(std::string("/move_base_simple/goal"), 100, &NavigationDemo::newDrivingGoalRvizHandler, this);


  // Setup filter chain.
  if (!filterChain_.configure(filterChainParametersName_, nodeHandle)) {
    ROS_ERROR("Could not configure the filter chain!");
    success = false;
    return;
  }

  success = true;

  verbose_ = false;
  verboseTimer_ = true;
  plannerEnabled_ = true; // start enabled
}


NavigationDemo::~NavigationDemo()
{
}

int cnt = 0;
bool NavigationDemo::readParameters()
{
  if (!nodeHandle_.getParam("input_topic", inputTopic_)) {
    ROS_ERROR("Could not read parameter `input_topic`.");
    return false;
  }
  nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("grid_map_filters"));

  nodeHandle_.param("demo_mode", demoMode_, true);
  if (demoMode_)
    ROS_INFO("In demo mode [%d]. will use a hard coded gridmap bag and robot pose", int(demoMode_) );
  else
    ROS_INFO("In live mode [%d]. will listen for poses continuously", int(demoMode_) );

  return true;
}

void NavigationDemo::tic(){
  lastTime_ = high_resolution_clock::now();
}


std::chrono::duration<double> NavigationDemo::toc(){
  auto nowTime = high_resolution_clock::now();
  duration<double> elapsedTime = duration_cast<milliseconds>(nowTime - lastTime_);
  lastTime_ = nowTime;
  // std::cout << elapsedTime.count() << "ms elapsed" << std::endl;
  return elapsedTime;
}

grid_map_msgs::GridMap map_msg;
void NavigationDemo::newDrivingGoalRvizHandler(const geometry_msgs::PoseStampedConstPtr& msg){

  // The all important position goal - get the robot there
  Position pos_goal(8.5,4.0);

  std::cout << "YOW YOWYOWO " << std::endl;
  Eigen::Isometry3d pose_robot = Eigen::Isometry3d::Identity();
  tf::poseMsgToEigen(msg->pose, pose_robot);

  geometry_msgs::PoseStamped m;
  m.header = map_msg.info.header;
  tf::poseEigenToMsg (pose_robot, m.pose);
  ray1pub_.publish(m);
  
  Eigen::Isometry3d pose_chosen_carrot = Eigen::Isometry3d::Identity();
  bool sendCommand = planCarrot(map_msg, pose_robot, pos_goal, pose_chosen_carrot);

  if(sendCommand){
    // Send the carrot to the position controller
    geometry_msgs::PoseStamped m;
    m.header = map_msg.info.header;
    tf::poseEigenToMsg (pose_chosen_carrot, m.pose);
    footstepPlanRequestPub_.publish(m);
  }
}


void NavigationDemo::callback(const grid_map_msgs::GridMap& message)
{
  if (!plannerEnabled_){
    std::cout << "planner enabled. at the goal? grab a beer!\n";
    return;
  }

  map_msg = message;

  // The all important position goal - get the robot there
  Position pos_goal(8.5,4.0);

  Eigen::Isometry3d pose_robot = Eigen::Isometry3d::Identity();
  if(demoMode_){ // demoMode
    
    Eigen::Vector3d robot_xyz = Eigen::Vector3d(4.0,4.0,0); //rpy
    //Eigen::Vector3d robot_xyz = Eigen::Vector3d(3.0,3.0,0); //rpy
    Eigen::Vector3d robot_rpy = Eigen::Vector3d(0,0,0); //rpy

    pose_robot.setIdentity();
    pose_robot.translation() = robot_xyz;
    Eigen::Quaterniond motion_R = Eigen::AngleAxisd(robot_rpy(2), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(robot_rpy(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(robot_rpy(0), Eigen::Vector3d::UnitX()); // order is ypr

    pose_robot.rotate( motion_R );

    geometry_msgs::PoseStamped m;
    m.header = message.info.header;
    tf::poseEigenToMsg (pose_robot, m.pose);
    ray1pub_.publish(m);


  }else{ // online

    tf::StampedTransform transform;
    try {
        listener_->waitForTransform("/odom", "/base", ros::Time(0), ros::Duration(10.0) );
        listener_->lookupTransform("/odom", "/base", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    tf::transformTFToEigen (transform, pose_robot);    
  }


  Eigen::Isometry3d pose_chosen_carrot = Eigen::Isometry3d::Identity();
  bool sendCommand = planCarrot(message, pose_robot, pos_goal, pose_chosen_carrot);

  if(sendCommand){
    // Send the carrot to the position controller
    geometry_msgs::PoseStamped m;
    m.header = message.info.header;
    tf::poseEigenToMsg (pose_chosen_carrot, m.pose);
    footstepPlanRequestPub_.publish(m);
  }

}

// angle should be in global reference angles
float scanForObstacle(Position robot, float theta, GridMap map, float &x, float &y) {

  float distance = 10;
  float threshold = 0.85;

  Position ray_end(distance*cos(theta), distance*sin(theta));
  ray_end += robot;

  Position ray_start(.4*cos(theta), .4*sin(theta));
  ray_start += robot;

  for (grid_map::LineIterator iterator(map, ray_start, ray_end); !iterator.isPastEnd(); ++iterator) {
    Position position;
    map.getPosition(*iterator, position);
    float val = map.at("traversability_clean_dilated", *iterator);
    x = position.x();
    y = position.y();
    if (val < threshold) {
      //std::cout << start_direction.x() << " " << start_direction.y() << " " << position.x() << " " << position.y() << " : " << map.at("traversability_clean_dilated", *iterator) << std::endl;
      return (position - robot).norm();
    }
  }
}

bool NavigationDemo::planCarrot(const grid_map_msgs::GridMap& message,
  Eigen::Isometry3d pose_robot, Position pos_goal,
  Eigen::Isometry3d& pose_chosen_carrot)
{
  std::cout << "start - carrot planner\n";
  tic();

  // Compute distance to the goal:
  Position pos_robot( pose_robot.translation().head(2) );
  double current_dist_to_goal = (pos_goal - pos_robot).norm();
  std::cout << "current distance to goal: " << current_dist_to_goal << std::endl;

  // If within 1.5m of goal - stop walking
  if (current_dist_to_goal < 1.5){
    // Determine a carrot pose: x and y from the above. z is the robot's height.
    // yaw in the direction of the carrot. roll,pitch zero
    Eigen::Vector4d carrot_relative_pose = pose_robot.matrix().inverse()*Eigen::Vector4d(pos_goal(0), pos_goal(1), 0, 1) ;
    double carrot_relative_theta = atan2(carrot_relative_pose(1),carrot_relative_pose(0));
    if (verbose_) std::cout << carrot_relative_pose.transpose() << " - relative carrot\n";
    if (verbose_) std::cout << carrot_relative_theta << " - relative carrot - theta\n";

    Eigen::Isometry3d pose_chosen_carrot_relative = Eigen::Isometry3d::Identity();
    pose_chosen_carrot_relative.translation() = Eigen::Vector3d( carrot_relative_pose(0),carrot_relative_pose(1),0);
    Eigen::Quaterniond motion_R = Eigen::AngleAxisd(carrot_relative_theta, Eigen::Vector3d::UnitZ()) // yaw
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) // pitch
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()); // roll

    pose_chosen_carrot_relative.rotate( motion_R );
    pose_chosen_carrot = pose_robot * pose_chosen_carrot_relative;
    std::cout << current_dist_to_goal << "m to goal. carrot is goal\n";
    // disable carrot planner
    plannerEnabled_ = false;

    // Send message to position_controller to start free gait action
    std_msgs::Int16 actionMsg;
    actionMsg.data = 1;
    ros::Duration(1.0).sleep();
    actionPub_.publish(actionMsg);

    return true;
  }


  // Convert message to map.
  GridMap inputMap;
  GridMapRosConverter::fromMessage(message, inputMap);
  // Apply filter chain.
  grid_map::GridMap outputMap;
  if (!filterChain_.update(inputMap, outputMap)) {
    ROS_ERROR("Could not update the grid map filter chain!");
    return false;
  }
  if (verboseTimer_) std::cout << toc().count() << "ms: filter chain\n";


  ////// Put your code here ////////////////////////////////////

  const float minValue = outputMap.get("traversability_clean").minCoeffOfFinites();
  const float maxValue = outputMap.get("traversability_clean").maxCoeffOfFinites();

  // Add carrot layer.
  outputMap.add("carrots", Matrix::Zero(outputMap.getSize()(0), outputMap.getSize()(1)));

  // clean traversability_clean around robot
  for (grid_map::CircleIterator iterator(outputMap, pos_robot, 0.9); !iterator.isPastEnd(); ++iterator) {
    outputMap.at("traversability_clean", *iterator) = 1.0f;
  }

  // Convert to OpenCV image, erode, convert back.
  cv::Mat originalImage, erodeImage, thresholdedImage;

  GridMapCvConverter::toImage<unsigned short, 1>(outputMap, "traversability_clean", CV_16UC1, minValue, maxValue, originalImage);
  //cv::imwrite( "originalImage.bmp", originalImage );
  // Specify dilation type.
  int erosion_size = 25;
  cv::Mat erosion_specs = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                      cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ));

  cv::erode(originalImage, erodeImage, erosion_specs);
  //cv::threshold(erodeImage, thresholdedImage, 0.99, 1.0, cv::THRESH_BINARY);

  GridMapCvConverter::addLayerFromImage<unsigned short, 1>(erodeImage, "traversability_clean_dilated", outputMap, minValue, maxValue);

// We have the eroded image. Now go towards the goal.

  // check between here and the goal
  std::cout << "Robot " << pos_robot << std::endl;
  std::cout << "Robot " << pos_goal << std::endl;
  for (grid_map::LineIterator iterator(outputMap, pos_robot, Position(2.0, 2.0)); !iterator.isPastEnd(); ++iterator) {
    Position position;
    outputMap.getPosition(*iterator, position);
    //std::cout << position.x() << " " << position.y() << " : " << outputMap.at("traversability_clean_dilated", *iterator) << std::endl;
  }

/*
  Position difference = pos_goal - pos_robot;
  double difference_value = difference.norm();
  Index pt_index;
  bool placed_carrot = false;
  // Check whether goal is within grid map.
  if ( outputMap.isInside(pos_goal) ){
    // TODO: this is assuming that the number is in meters.
    if (difference_value < 1) {
      // If we're close to the goal, set the carrot there.
      outputMap.getIndex(pos_goal, pt_index );
      Position pt_cell;
      outputMap.getPosition(pt_index, pt_cell);
      outputMap.at("carrots", pt_index) = 1.0;
      placed_carrot = true;
    }
  }

  if (!placed_carrot) {
    Position optimistic_carrot;
    optimistic_carrot = pos_robot + difference/difference_value;

    if ( outputMap.isInside(pos_goal) ){
      // TODO: this is assuming that the number is in meters.
      if (difference_value < 1) {
        // If we're close to the goal, set the carrot there.
        outputMap.getIndex(pos_goal, pt_index );
        Position pt_cell;
        outputMap.getPosition(pt_index, pt_cell);
        outputMap.at("carrots", pt_index) = 1.0;
        placed_carrot = true;
      }
    }
  }
*/

  /*
  // Else, project it somewhere.
  {
    Position difference = pos_goal - pos_robot;
    double difference_value = difference.norm();
    double alpha = 0.1;
    for(int counter = 0; counter < difference_value/alpha; counter++) {;}
  }*/


  //outputMap.at("carrots", pt_index) = 1.0;

  ////// Put your code here ////////////////////////////////////



  // Publish filtered output grid map.
  grid_map_msgs::GridMap outputMessage;
  GridMapRosConverter::toMessage(outputMap, outputMessage);
  outputGridmapPub_.publish(outputMessage);
  if (verbose_) std::cout << "finished processing\n";
  if (verboseTimer_) std::cout << toc().count() << "ms: publish output\n";

  std::cout << "finish - carrot planner\n\n";


  // REMOVE THIS WHEN YOUR ARE DEVELOPING ----------------
  // create a fake carrot - replace with a good carrot

/*
  Eigen::Vector4d carrot_relative_pose = pose_robot.matrix().inverse()*Eigen::Vector4d(pos_goal(0), pos_goal(1), 0, 1) ;
    double carrot_relative_theta = atan2(carrot_relative_pose(1),carrot_relative_pose(0));
    if (verbose_) std::cout << carrot_relative_pose.transpose() << " - relative carrot\n";
    if (verbose_) std::cout << carrot_relative_theta << " - relative carrot - theta\n";
  Eigen::Quaterniond motion_R = Eigen::AngleAxisd(carrot_relative_theta, Eigen::Vector3d::UnitZ()) // yaw
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) // pitch
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()); // roll


*/

  Eigen::Quaterniond q(pose_robot.rotation());
  double robot_roll, robot_pitch, robot_yaw;
  quat_to_euler(q, robot_roll, robot_pitch, robot_yaw);

  // rotates counter-clockwise the pose_robot
  int N = 180;
  float max_distance = 0;
  float max_x, max_y;
  float new_carrot_theta = 0;
  int k = 0;
  for (int i=0; i<N; i++) {
    float angle = i-N/2.0;
    angle = angle * PI / 180;
    float x, y;
    float res = scanForObstacle(pos_robot, robot_yaw -angle, outputMap, x, y);
    if (res > max_distance) {
      max_x = x;
      max_y = y;
      new_carrot_theta = -angle;
      max_distance = res;
    }
  }

  // check also for the goal ray
  std::cout << " ROBOT " << pos_robot.x() << " " << pos_robot.y() << std::endl;
  double goal_yaw = atan2(pos_goal.y()-pos_robot.y(), pos_goal.x()-pos_robot.x());  
  float x, y;
  float res = scanForObstacle(pos_robot,  -goal_yaw, outputMap, x, y);
  std::cout << " TO GOAL " << res << std::endl; 

  // if it looks the either way around, fuck off and turn back
  if (res > 0.84 || res > max_distance) {
    max_x = x;
    max_y = y;
    new_carrot_theta = goal_yaw - robot_yaw;
    max_distance = res;
  }

  float carrot_angle = robot_yaw+new_carrot_theta;

  //Eigen::Quaterniond motion_R = Eigen::AngleAxisd(robot_yaw+new_carrot_theta, Eigen::Vector3d::UnitZ()) // yaw
  Eigen::Quaterniond motion_R = Eigen::AngleAxisd(carrot_angle, Eigen::Vector3d::UnitZ()) // yaw
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) // pitch
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()); // roll

  pose_chosen_carrot.translation() = Eigen::Vector3d(max_x, max_y, 0);
  pose_chosen_carrot.rotate(motion_R);

  // REMOVE THIS -----------------------------------------

  return true;
}

} /* namespace */
