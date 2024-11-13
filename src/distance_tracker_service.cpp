#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <cmath>

class DistanceTracker {
public:
    DistanceTracker() : distance_traveled_(0.0), last_x_(0.0), last_y_(0.0), has_previous_position_(false) {
        odom_sub_ = nh_.subscribe("/odom", 10, &DistanceTracker::odomCallback, this);
        distance_service_ = nh_.advertiseService("/get_distance", &DistanceTracker::getDistanceCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::ServiceServer distance_service_;
    double distance_traveled_;
    double last_x_, last_y_;
    bool has_previous_position_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        if (has_previous_position_) {
            double dx = x - last_x_;
            double dy = y - last_y_;
            distance_traveled_ += std::sqrt(dx * dx + dy * dy);
        }

        last_x_ = x;
        last_y_ = y;
        has_previous_position_ = true;
    }

    bool getDistanceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        res.success = true;
        res.message = "Distance traveled: " + std::to_string(distance_traveled_);
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_tracker_service");
    DistanceTracker distance_tracker;
    ros::spin();
    return 0;
}
