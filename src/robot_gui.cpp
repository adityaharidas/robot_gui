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
  RobotGUI() : frame(cv::Mat(600, 800, CV_8UC3)), nh_("~"), x_(0.0), y_(0.0), z_(0.0) {    // Initialize the subscriber and the GUI window
    info_sub_ = nh_.subscribe("/robot_info", 10, &RobotGUI::robotInfoCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 10, &RobotGUI::odomCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    distance_client_ = nh_.serviceClient<std_srvs::Trigger>("/get_distance");
    cvui::init("Robot Info GUI");
    cv::namedWindow("Robot Info GUI");

    // Start the ROS thread
    ros_thread_ = std::thread(&RobotGUI::rosSpin, this);
  }

  ~RobotGUI() { ros_thread_.join(); }
  
  void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
    robot_description_ = msg->data_field_01;
    serial_number_ = msg->data_field_02;
    ip_address_ = msg->data_field_03;
    firmware_version_ = msg->data_field_04;
    maximum_payload_ = msg->data_field_05;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    z_ = msg->pose.pose.position.z;
  }

  void run() {
    while (ros::ok()) {
      frame = cv::Scalar(49, 52, 49); // Background color

      // Display General Info
      cvui::text(frame, 20, 30, "General Info Area", 0.6);
      cvui::text(frame, 20, 60, "Robot Description: " + robot_description_);
      cvui::text(frame, 20, 90, "Serial Number: " + serial_number_);
      cvui::text(frame, 20, 120, "IP Address: " + ip_address_);
      cvui::text(frame, 20, 150, "Firmware Version: " + firmware_version_);
      cvui::text(frame, 20, 180, "Maximum Payload: " + maximum_payload_);

      //Display Teleoperation Controls
      cvui::text(frame, 400, 30, "Teleoperation Control");
      if (cvui::button(frame, 400, 60, "Forward")) {
        linear_speed_ = 0.5; // Move forward with positive speed
        angular_speed_ = 0.0;
      publishVelocities();
            }
      if (cvui::button(frame, 400, 100, "Backward")) {
        linear_speed_ = -0.5; // Move backward with negative speed
        angular_speed_ = 0.0;
        publishVelocities();
            }
      if (cvui::button(frame, 400, 140, "Left")) {
        linear_speed_ = 0.0; // Move backward with negative speed
        angular_speed_ = 0.5;
        publishVelocities();
            }
      if (cvui::button(frame, 400, 180, "Right")) {
        linear_speed_ = 0.0; // Move backward with negative speed
        angular_speed_ = -0.5;
        publishVelocities();
            }
      if (cvui::button(frame, 400, 220, "STOP")) {
        linear_speed_ = 0.0;
        angular_speed_ = 0.0;
        publishVelocities();
            }

      //Display current velocities
      cvui::text(frame, 400, 260, "Current Velocities");
      cvui::text(frame, 400, 290, "Linear Velocity (X): " + std::to_string(linear_speed_));
      cvui::text(frame, 400, 320, "Angular Velocity (Z): " + std::to_string(angular_speed_));
    
      //Display Current Position
      cvui::text(frame, 20, 220, "Robot Position (Odometry)");
      cvui::text(frame, 20, 250, "X: " + std::to_string(x_));
      cvui::text(frame, 20, 280, "Y: " + std::to_string(y_));
      cvui::text(frame, 20, 310, "Z: " + std::to_string(z_));

      //Distance Tracker Service Button
      if (cvui::button(frame, 400, 400, "Get Distance Traveled")) {
        callGetDistanceService();
      }

      cvui::text(frame, 400, 430, "Distance Travelled: " + distance_traveled_);

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
      firmware_version_, maximum_payload_;
  double x_, y_, z_;
  std::string distance_traveled_;
  
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
