#include "uned_crazyflie_controllers/CrazyfliePositionController.hpp"

using std::placeholders::_1;

bool PositionController::initialize(){
    RCLCPP_INFO(this->get_logger(),"PositionController::inicialize() ok.");

    // Lectura de parámetros
    RCLCPP_INFO(this->get_logger(),"TO-DO: Read Params.");
    m_controller_type = "PID";
    m_robot_id = "dron01";
    m_controller_mode = "close loop";
    RCLCPP_INFO(this->get_logger(),"Controller Type: %s, \tRobot id: %s, \tMode: %s", m_controller_type.c_str(), m_robot_id.c_str(), m_controller_mode.c_str());
    m_x_init = 0.0;
    m_y_init = 0.0;
    m_z_init = 0.0;

    // Z Controller
    str_id = "Z";
    z_controller = init_controller(str_id.c_str(), 2.0, 0.5, 0.0, 0.0, 100, 1.0, -1.0);
    // W Controller
    str_id = "W";
    w_controller = init_controller(str_id.c_str(), 25.0, 15.0, 0.0, 0.0, 100, 1160.0, -640.0);
    // X Controller
    str_id = "X";
    x_controller = init_controller(str_id.c_str(), 2.0, 0.0, 0.0, 0.0, 100, 1.0, -1.0);
    // U Controller
    str_id = "U";
    u_controller = init_controller(str_id.c_str(), 25.0, 1.0, 0.0, 0.0, 100, 30.0, -30.0);
    // Y Controller
    str_id = "Y";
    y_controller = init_controller(str_id.c_str(), 2.0, 0.0, 0.0, 0.0, 100, 1.0, -1.0);
    // V Controller
    str_id = "V";
    v_controller = init_controller(str_id.c_str(), -25.0, -1.0, 0.0, 0.0, 100, 30.0, -30.0);

    // Publisher:
    // Referencias para los controladores PID Attitude y Rate
    pub_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("onboard_cmd", 10);

    // Subscriber:
    // Crazyflie Pose {Real: /pose; Sim: /ground_truth/pose}
    GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("pose", 10, std::bind(&PositionController::gtposeCallback, this, _1));
    // Reference:
    ref_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("goal_pose", 10, std::bind(&PositionController::positionreferenceCallback, this, _1));

    return true;
}


bool PositionController::iterate(){
    RCLCPP_INFO_ONCE(this->get_logger(), "PositionController::iterate(). ok.");
    if (first_pose_received && first_ref_received) {
        RCLCPP_INFO_ONCE(this->get_logger(), "PositionController::iterate(). Running ...");
        // Z Controller
        z_controller.error[0] = ref_pose.position.z - GT_pose.position.z;
        w_ref = pid_controller(z_controller, dt);

        // W Controller
        w_feedback[1] = w_feedback[0];
        w_feedback[0] = GT_pose.position.z;
        w_signal = (w_feedback[0] - w_feedback[1]) / dt;
        w_controller.error[0] = w_ref - w_signal;
        thrust = pid_controller(w_controller, dt);

        thrust = thrust * 1000 + 36000;

        // Convert quaternion to yw
        rpy_ref = quaternion2euler(ref_pose.orientation);
        rpy_state = quaternion2euler(GT_pose.orientation);

        x_global_error = ref_pose.position.x - GT_pose.position.x;
        y_global_error = ref_pose.position.y - GT_pose.position.y;
        // X Controller
        x_controller.error[0] = x_global_error * cos(rpy_state.yaw) + y_global_error * sin(rpy_state.yaw);
        u_ref = pid_controller(x_controller, dt);
        // Y Controller
        y_controller.error[0] = -x_global_error * sin(rpy_state.yaw) + y_global_error * cos(rpy_state.yaw);
        v_ref = pid_controller(y_controller, dt);

        // Speed
        u_feedback[1] = u_feedback[0];
        u_feedback[0] = GT_pose.position.x;
        u_signal = (u_feedback[0] - u_feedback[1]) / dt;
        v_feedback[1] = v_feedback[0];
        v_feedback[0] = GT_pose.position.y;
        v_signal = (v_feedback[0] - v_feedback[1]) / dt;

        // U Controller
        u_controller.error[0] = u_ref - u_signal;
        pitch = pid_controller(u_controller, dt);

        // V Controller
        v_controller.error[0] = v_ref - v_signal;
        roll = pid_controller(v_controller, dt);

        // RCLCPP_INFO(this->get_logger(), "Thrust: \t%.2f \tRoll:%.2f \tPitch:%.2f \tYaw:%.2f", thrust, roll, pitch, rpy_ref.yaw);


        // Publish Control CMD

        auto msg_cmd = std_msgs::msg::Float64MultiArray();
        msg_cmd.data = { thrust, roll, pitch, rpy_ref.yaw };
        //msg_cmd.data = { thrust, 0.0, 0.0, 0.0 };
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
