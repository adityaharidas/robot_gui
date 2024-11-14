#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <nav_msgs/Odometry.h>
#include <thread>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

class RobotGUI {
public:
  RobotGUI()
      : frame(cv::Mat(600, 800, CV_8UC3)), nh_("~"), x_(0.0), y_(0.0), z_(0.0),
        linear_speed_(0.0), angular_speed_(0.0), speed_increment_(0.05) {
    info_sub_ = nh_.subscribe("/robot_info", 10, &RobotGUI::robotInfoCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 10, &RobotGUI::odomCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    distance_client_ = nh_.serviceClient<std_srvs::Trigger>("/get_distance");
    cvui::init("Robot Info GUI");
    cv::namedWindow("Robot Info GUI");

    ros_thread_ = std::thread(&RobotGUI::rosSpin, this);
  }

  ~RobotGUI() { ros_thread_.join(); }

  void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
    robot_description_ = msg->data_field_01;
    serial_number_ = msg->data_field_02;
    ip_address_ = msg->data_field_03;
    firmware_version_ = msg->data_field_04;
    maximum_payload_ = msg->data_field_05;
    hydraulic_system_data_ = msg->data_field_06;  
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    z_ = msg->pose.pose.position.z;
  }

  void run() {
    while (ros::ok()) {
      frame = cv::Scalar(49, 52, 49); // Background color

      // General Info Area (Top of the screen)
      cvui::text(frame, 20, 30, "General Info Area", 0.6);
      cvui::text(frame, 20, 60, "Robot Description: " + robot_description_);
      cvui::text(frame, 20, 90, "Serial Number: " + serial_number_);
      cvui::text(frame, 20, 120, "IP Address: " + ip_address_);
      cvui::text(frame, 20, 150, "Firmware Version: " + firmware_version_);
      cvui::text(frame, 20, 180, "Maximum Payload: " + maximum_payload_);
      cvui::text(frame, 20, 210, "Hydraulic System Data: " + hydraulic_system_data_);

      // Teleoperation Controls 
      cvui::text(frame, 400, 30, "Teleoperation Control", 0.6);

      // Forward and Backward Buttons
      if (cvui::button(frame, 400, 60, "Forward")) {
        linear_speed_ += speed_increment_;  // Increase speed gradually
      }
      if (cvui::button(frame, 400, 120, "Backward")) {
        linear_speed_ -= speed_increment_;  // Decrease speed gradually
      }

      // Left and Right Buttons
      if (cvui::button(frame, 400, 180, "Left")) {
        angular_speed_ += speed_increment_;  // Increase angular speed
      }
      if (cvui::button(frame, 400, 240, "Right")) {
        angular_speed_ -= speed_increment_;  // Decrease angular speed
      }

      // Stop Button
      if (cvui::button(frame, 400, 300, "Stop")) {
        linear_speed_ = 0.0;
        angular_speed_ = 0.0;
      }

      // Publish velocities continuously
      publishVelocities();

      // Current Velocities
      cvui::text(frame, 400, 350, "Current Velocities", 0.6);
      cvui::text(frame, 400, 380, "Linear Velocity (X): " + std::to_string(linear_speed_));
      cvui::text(frame, 400, 410, "Angular Velocity (Z): " + std::to_string(angular_speed_));

      // Robot Position
      cvui::text(frame, 20, 250, "Robot Position (Odometry)", 0.6);
      cvui::text(frame, 20, 280, "X: " + std::to_string(x_));
      cvui::text(frame, 20, 310, "Y: " + std::to_string(y_));
      cvui::text(frame, 20, 340, "Z: " + std::to_string(z_));

      // Distance Tracker Service Button
      cvui::text(frame, 400, 450, "Distance Tracker", 0.6);
      if (cvui::button(frame, 400, 480, "Get Distance Traveled")) {
        callGetDistanceService();
      }

      // Display Distance Traveled
      cvui::text(frame, 400, 510, "Distance Travelled: " + distance_traveled_);

      cvui::update();
      cv::imshow("Robot Info GUI", frame);
      if (cv::waitKey(20) == 27)
        break; // Exit on ESC key
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber info_sub_, odom_sub_;
  cv::Mat frame;
  std::thread ros_thread_;
  ros::Publisher cmd_vel_pub_;
  float linear_speed_, angular_speed_;
  ros::ServiceClient distance_client_;
  std::string robot_description_, serial_number_, ip_address_,
      firmware_version_, maximum_payload_, hydraulic_system_data_;
  double x_, y_, z_;
  std::string distance_traveled_;
  float speed_increment_;  // Speed change increment

  void publishVelocities() {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = linear_speed_;
    vel_msg.angular.z = angular_speed_;
    cmd_vel_pub_.publish(vel_msg);
  }

  void callGetDistanceService() {
    std_srvs::Trigger srv;
    if (distance_client_.call(srv)) {
      distance_traveled_ = srv.response.message;
    } else {
      ROS_ERROR("Failed to call service /get_distance");
    }
  }

  void rosSpin() { ros::spin(); }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_node");
  RobotGUI gui;
  gui.run();
  return 0;
}
