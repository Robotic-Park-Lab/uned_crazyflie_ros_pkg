#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TrajectoryController : public rclcpp::Node
{
  public:
    TrajectoryController() : Node("trajectory_controller"), count_(0)
    {}
    bool initialize();
    bool iterate();

  private:
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ref_pose_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr GT_pose_;
    void gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        GT_pose.position = msg->position;
        GT_pose.orientation = msg->orientation;
        if(!first_pose_received){
            ref_pose.position = msg->position;
            ref_pose.orientation = msg->orientation;
            first_pose_received = true;
            RCLCPP_INFO(this->get_logger(),"Initial Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);
        }
    }
    geometry_msgs::msg::Pose GT_pose, ref_pose;
    bool first_pose_received = false;
    bool new_ref = true;
    bool end_dataset = false;
    size_t count_;
    double start = 0.0;
    double end = 0.0;
    double t = 0.0;
};
