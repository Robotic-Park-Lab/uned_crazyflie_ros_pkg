#include "uned_crazyflie_controllers/CrazyfliePositionController.hpp"

using std::placeholders::_1;

bool PositionController::initialize(){
    RCLCPP_INFO(this->get_logger(),"PositionController::inicialize() ok.");

    // Lectura de parámetros
    RCLCPP_INFO(this->get_logger(),"TO-DO: Read Params.");
    m_controller_type = "PID";
    m_robot_id = "dron_test";
    m_controller_mode = "close loop";
    RCLCPP_INFO(this->get_logger(),"Controller Type: %s, \tRobot id: %s, \tMode: %s", m_controller_type, m_robot_id, m_controller_mode);
    m_x_init = 0.0;
    m_y_init = 0.0;
    m_z_init = 0.0;

    // Z Controller
    init_controller("Z", z_controller, 2.0, 0.5, 0.0, 0.0, 100, 1.0, -1.0);
    // W Controller
    init_controller("W", w_controller, 25.0, 15.0, 0.0, 0.0, 100, 1160.0, -640.0);
    // X Controller
    init_controller("X", x_controller, 2.0, 0.0, 0.0, 0.0, 100, 1.0, -1.0);
    // U Controller
    init_controller("U", u_controller, 25.0, 1.0, 0.0, 0.0, 100, 20.0, -20.0);
    // Y Controller
    init_controller("Y", y_controller, 2.0, 0.0, 0.0, 0.0, 100, 1.0, -1.0);
    // V Controller
    init_controller("V", v_controller, 25.0, 1.0, 0.0, 0.0, 100, 20.0, -20.0);
    // Yaw Controller
    init_controller("Yaw", yaw_controller, 6.0, 1.0, 0.3499, 0.0583, 100, 20.0, -20.0);

    // Publisher:
    // Referencias para los controladores PID Attitude y Rate
    pub_cmd_ = this->create_publisher<uned_crazyflie_config::msg::Cmdsignal>("cf_cmd_control", 10);

    // Subscriber:
    // Crazyflie Pose
    // GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("ground_truth/pose", 10, std::bind(&PositionController::gtposeCallback, this, _1));
    GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("pose", 10, std::bind(&PositionController::gtposeCallback, this, _1));
    // Reference:
    ref_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("pose_ref", 10, std::bind(&PositionController::positionreferenceCallback, this, _1));

    return true;
}


bool PositionController::iterate(){
    RCLCPP_INFO_ONCE(this->get_logger(), "PositionController::iterate(). ok.");
    if (first_pose_received && first_ref_received) {
        // Z Controller
        z_controller.error[0] = ref_pose.position.z - GT_pose.position.z;
        w_ref = pid_controller(z_controller, 0.01);
        // W Controller - TO-DO!!
        w_controller.error[0] = w_ref - 0.0;
        thrust = pid_controller(w_controller, 0.01);

        thrust = thrust * 1000 + 36000;
        RCLCPP_INFO(this->get_logger(), "Altitude Controller. Thrust: \t%.2f \tError:%.2f", thrust, z_controller.error[0]);

        // Convert quaternion to yw
        siny_cosp_ref = 2 * (ref_pose.orientation.w * ref_pose.orientation.z + ref_pose.orientation.x * ref_pose.orientation.y);
        cosy_cosp_ref = 1 - 2 * (ref_pose.orientation.y * ref_pose.orientation.y + ref_pose.orientation.z * ref_pose.orientation.z);
        yaw_ref = std::atan2(siny_cosp_ref, cosy_cosp_ref);
        siny_cosp = 2 * (GT_pose.orientation.w * GT_pose.orientation.z + GT_pose.orientation.x * GT_pose.orientation.y);
        cosy_cosp = 1 - 2 * (GT_pose.orientation.y * GT_pose.orientation.y + GT_pose.orientation.z * GT_pose.orientation.z);
        yaw = std::atan2(siny_cosp, cosy_cosp);

        x_global_error = ref_pose.position.x - GT_pose.position.x;
        y_global_error = ref_pose.position.y - GT_pose.position.y;
        // X Controller
        x_controller.error[0] = x_global_error * cos(yaw) + y_global_error * sin(yaw);
        u_ref = pid_controller(x_controller, 0.01);
        // Y Controller
        y_controller.error[0] = -x_global_error * sin(yaw) + y_global_error * cos(yaw);
        v_ref = pid_controller(y_controller, 0.01);

        // Speed
        u_feedback[1] = u_feedback[0];
        u_feedback[0] = GT_pose.position.x;
        u_signal = (u_feedback[0] - u_feedback[1]) / 0.01;
        v_feedback[1] = v_feedback[0];
        v_feedback[0] = GT_pose.position.y;
        v_signal = (v_feedback[0] - v_feedback[1]) / 0.01;

        // U Controller
        u_controller.error[0] = u_ref - u_signal;
        pitch = pid_controller(u_controller, 0.01);

        // V Controller
        v_controller.error[0] = v_ref - v_signal;
        roll = pid_controller(v_controller, 0.01);

        // Yaw Controller
        yaw_controller.error[0] = (yaw_ref - yaw) * 180 / 3.14159265;
        dyaw = pid_controller(yaw_controller, 0.01);

        // Publish Control CMD
        auto msg_cmd = uned_crazyflie_config::msg::Cmdsignal();
        msg_cmd.thrust = (int)thrust;
        msg_cmd.roll = roll;
        msg_cmd.pitch = pitch;
        msg_cmd.yaw = yaw;
        pub_cmd_->publish(msg_cmd);
    }
    else {
        RCLCPP_INFO_ONCE(this->get_logger(), "PositionController::iterate(). Waiting reference & feedback position");
    }

  return true;
}


int main(int argc, char ** argv){
  try{
    rclcpp::init(argc, argv);
    auto crazyflie_position_controller = std::make_shared<PositionController>();
    rclcpp::Rate loop_rate(100);
    crazyflie_position_controller->initialize();

    while (rclcpp::ok()){
      rclcpp::spin_some(crazyflie_position_controller);
      crazyflie_position_controller->iterate();
      loop_rate.sleep();
    }
    return 0;
  } catch (std::exception &e){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s",e.what());
    }
}
