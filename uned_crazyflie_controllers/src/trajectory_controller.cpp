#include "uned_crazyflie_controllers/CrazyflieTrajectoryController.hpp"

using std::placeholders::_1;

bool TrajectoryController::initialize(){
    RCLCPP_INFO(this->get_logger(),"TrajectoryController::inicialize() ok.");
    // Crazyflie Pose Ref
    ref_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("goal_pose", 10);
    // Crazyflie Pose
    GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("cf_pose", 10, std::bind(&TrajectoryController::gtposeCallback, this, _1));
    start = time(NULL);
    end = time(NULL);
    t = end-start;
    while(t<5){
        end = time(NULL);
        t = end-start;
    }
    return true;
}

bool TrajectoryController::iterate(){
    if(first_pose_received && new_ref && !end_dataset){
        RCLCPP_INFO(this->get_logger(),"TrajectoryController::iterate() New Reference.");
        RCLCPP_INFO(this->get_logger(),"Z0: %f.", ref_pose.position.z);
        ref_pose.position.z = ref_pose.position.z + 0.5;
        RCLCPP_INFO(this->get_logger(),"Z1: %f.", ref_pose.position.z);
        auto msg = geometry_msgs::msg::Pose();
        msg = ref_pose;
        // ref_pose_->publish(msg);
        new_ref = false;
        start = time(NULL);
        end = time(NULL);
        t = end-start;
        while(t<4){
            end = time(NULL);
            t = end-start;
        }
        ref_pose_->publish(msg);
        start = time(NULL);
        RCLCPP_INFO(this->get_logger(),"Despegue ...");
    }
    end = time(NULL);
    t = end-start;
    if(t>15 && !new_ref && !end_dataset){
        ref_pose.position.z = ref_pose.position.z - 0.5;
        auto msg = geometry_msgs::msg::Pose();
        msg = ref_pose;
        // ref_pose_->publish(msg);
        end_dataset = true;
        start = time(NULL);
        end = time(NULL);
        t = end-start;
        while(t<2){
            end = time(NULL);
            t = end-start;
        }
        ref_pose_->publish(msg);
        RCLCPP_INFO(this->get_logger(),"Aterrizaje ...");
    }
    return true;
}


int main(int argc, char ** argv){
    try{
        rclcpp::init(argc, argv);
        auto trajectory_controller_node = std::make_shared<TrajectoryController>();
        rclcpp::Rate loop_rate(10);
        trajectory_controller_node->initialize();

        while (rclcpp::ok()){
            trajectory_controller_node->iterate();
            rclcpp::spin_some(trajectory_controller_node);
            loop_rate.sleep();
        }
        return 0;
    }catch (std::exception &e){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s",e.what());
	}
}
