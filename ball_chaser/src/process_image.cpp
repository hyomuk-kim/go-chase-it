#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z) {
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  if (!client.call(srv))
    ROS_ERROR("Failed to call service command_robot");
}

void process_image_callback(const sensor_msgs::Image img) {
  int white_pixel = 255;
  int subpix_r = 0, subpix_g = 0, subpix_b = 0;
  int num_col_pixel = img.step / 3;

  float req_lin_x = 0;
  float req_ang_z = 0;

  int left_line_pidx = num_col_pixel / 3;
  int right_line_pidx = left_line_pidx * 2;

  int ball_cidx = -1;

  for (int row = 0; row < img.height; row++) {
    for (int pix = 0; pix < num_col_pixel; pix++) {
      subpix_r = pix * 3;
      subpix_g = pix * 3 + 1;
      subpix_b = pix * 3 + 2;
      if (img.data[img.step * row + subpix_r] == white_pixel &&
          img.data[img.step * row + subpix_g] == white_pixel &&
          img.data[img.step * row + subpix_b] == white_pixel) {
        ball_pidx = pix;
        break;
      }
    }
  }

  if (0 <= ball_pidx && ball_pidx <= left_line_pidx) {
    req_lin_x = 0.3;
    req_ang_z = 0.5;
  }
  else if (left_line_pidx < ball_pidx && ball_pidx < right_line_pidx) {
    req_lin_x = 0.3;
    req_ang_z = 0.0;
  }
  else if (right_line_pidx <= ball_pidx && ball_pidx < num_col_pixel) {
    req_lin_x = 0.3;
    req_ang_z = -0.5;
  }
  else {
    req_lin_x = 0.0;
    req_ang_z = 0.0;
  }

  drive_robot(req_lin_x, req_ang_z);
}

int main(int argc, char** argv) {
  // initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin();

  return 0;
}

