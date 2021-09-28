#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocites
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
  ball_chaser::DriveToTarget::Response& res) {
  
  float requested_lx = (float)req.linear_x;
  float requested_az = (float)req.angular_z;

  ROS_INFO("DriveToTargetRequest received - lx: %1.2f, az: %1.2f", requested_lx, requested_az);

  // Create a motor_command object of type geometry_msgs::Twist
  geometry_msgs::Twist motor_command;

  std::vector<float> motor_velocities = { requested_lx, requested_az };

  motor_command.linear.x = motor_velocities[0];
  motor_command.angular.z = motor_velocities[1];

  // Publish angles to drive the robot
  motor_command_publisher.publish(motor_command);

  res.msg_feedback = "motor velocities set - lx: " + std::to_string(motor_velocities[0]) + ", az: " + std::to_string(motor_velocities[1]);
  ROS_INFO_STREAM(res.msg_feedback);

  return true;
}

int main(int argc, char** argv) {
  // Initialize a ROS node
  ros::init(argc, argv, "drive_bot");

  // Create a ROS NodeHandle object
  ros::NodeHandle n;

  // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
  motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
  ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

  // TODO: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values

  // TODO: Handle ROS communication events
  ros::spin();

  return 0;
}
