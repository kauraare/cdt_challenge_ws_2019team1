#include "position_controller_cdt/position_controller_cdt.hpp"

PositionController::PositionController(){
  std::cout << "Finished setting up PositionController\n";
}


void PositionController::quat_to_euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}


// constrain angle to be -180:180 in radians
double PositionController::constrainAngle(double x){
    x = fmod(x + M_PI,2.0*M_PI);
    if (x < 0)
        x += 2.0*M_PI;
    return x - M_PI;
}

Eigen::Vector3d PositionController::constrainPosition(Eigen::Vector3d pos){
    //we need more copypaste here, I believe
    double threshold_max = 3.0;
    doubel threshold_min = 0.5;
    if(pos[0] > threshold_max) {
	pos[0] = threshold_max;
    }
    if(pos[1] > threshold_max) {
	pos[1] = threshold_max;
    }
    if(pos[0] < -threshold_max) {
	pos[0] = -threshold_max;
    }
    if(pos[1] < -threshold_max) {
	pos[1] = -threshold_max;
    }
    if(pos[0] < threshold_min && pos[0] > -threshold_min) {
	pos[0] = 0;
    }
    if(pos[1] < threshold_min && pos[1] > -threshold_min) {
	pos[1] = 0;
    }
    return pos;
}

FOLLOWER_OUTPUT PositionController::computeControlCommand(Eigen::Isometry3d current_pose, int64_t current_utime){
  double linear_forward_x = 0;
  double linear_forward_y = 0;
  double angular_velocity = 0;
  // Develop your controller here within the calls

  // EXAMPLE HEADING CONTROLLER CODE - ADD YOUR OWN POSITION + HEADING CONTROLLER HERE
  ///////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////

  Eigen::Quaterniond q(current_pose.rotation());
  double current_roll, current_pitch, current_yaw;
  quat_to_euler(q, current_roll, current_pitch, current_yaw);

  Eigen::Quaterniond q_goal(current_goal_.rotation());
  double goal_roll, goal_pitch, goal_yaw;
  quat_to_euler(q_goal, goal_roll, goal_pitch, goal_yaw);

  // compute the P control output:
  double headingErrorRaw = current_yaw - goal_yaw;
  double headingError = constrainAngle(headingErrorRaw);
  double angular_gain_p_ = 2.5; // TODO find a better parameter manually
  angular_velocity = -headingError * angular_gain_p_;

  std::cout << "current_yaw: " << current_yaw << ", raw error: " << headingErrorRaw
            << ", constrained error: " << headingError << ", des ang vel: " << angular_velocity << std::endl;

  ///////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////

  Eigen::Vector3d positionErrorRaw = current_pose.translation() - current_goal_.translation();
  Eigen::Vector3d positionError = constrainPosition(positionErrorRaw);
  //Eigen::Vector2d oldErrorPos(positionError[0], positionError[1]);

  double linear_gain_x_ = 5.5;
  double linear_gain_y_ = 5.5;

  //linear_forward_x = linear_gain_x_* tanh(-positionError[0]) * exp(-angular_velocity*angular_velocity/3);
  //linear_forward_y = linear_gain_y_* tanh(-positionError[1]) * exp(-angular_velocity*angular_velocity/3);
  linear_forward_x = -positionError[0] * linear_gain_x_; //* tanh(-positionError[0]);
  linear_forward_y = -positionError[1] * linear_gain_y_; //* tanh(-positionError[1]);
  Eigen::Vector3d vel(linear_forward_x, linear_forward_y, 0);

  double threshold = 0.05;
  if(linear_forward_x < threshold) {
    linear_forward_x = 0;
  }
  if(linear_forward_y < threshold) {
    linear_forward_y = 1;
  }

  vel = Eigen::AngleAxisd(-current_yaw, Eigen::Vector3d::UnitZ()) * vel;

  // set outputs
  output_linear_velocity_ = Eigen::Vector3d(vel[0], vel[1], 0);
  output_angular_velocity_ = Eigen::Vector3d(0,0, angular_velocity) ;
  return SEND_COMMAND;
}
