#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <thread>

class RobotGUI {
public:
  RobotGUI() : frame(cv::Mat(600, 800, CV_8UC3)), nh_("~") {
    // Initialize the subscriber and the GUI window
    info_sub_ =
        nh_.subscribe("/robot_info", 10, &RobotGUI::robotInfoCallback, this);
    cvui::init("Robot Info GUI");
    cv::namedWindow("Robot Info GUI");

    // Start the ROS thread
    ros_thread_ = std::thread(&RobotGUI::rosSpin, this);
  }

  ~RobotGUI() { ros_thread_.join(); }

  void
  robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
    robot_description_ = msg->data_field_01;
    serial_number_ = msg->data_field_02;
    ip_address_ = msg->data_field_03;
    firmware_version_ = msg->data_field_04;
    maximum_payload_ = msg->data_field_05;
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

      cvui::update();
      cv::imshow("Robot Info GUI", frame);
      if (cv::waitKey(20) == 27)
        break; // Exit on ESC key
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber info_sub_;
  cv::Mat frame;
  std::thread ros_thread_;

  std::string robot_description_, serial_number_, ip_address_,
      firmware_version_, maximum_payload_;

  void rosSpin() { ros::spin(); }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_node");
  RobotGUI gui;
  gui.run();
  return 0;
}
