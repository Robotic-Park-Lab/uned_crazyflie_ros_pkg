#include "uned_crazyflie_controllers/CrazyflieAttitudeRateController.hpp"

using std::placeholders::_1;

bool AttitudeRateController::initialize(){
    RCLCPP_INFO(this->get_logger(),"AttitudeRateController::inicialize() ok.");

    // Lectura de parámetros
    RCLCPP_INFO(this->get_logger(),"TO-DO: Read Params.");
    m_controller_type = "PID";
    m_robot_id = "dron01";
    m_controller_mode = "close loop";
    RCLCPP_INFO(this->get_logger(),"Controller Type: %s, \tRobot id: %s, \tMode: %s", m_controller_type, m_robot_id, m_controller_mode);

    // Pitch Controller
    init_controller("Pitch", pitch_controller, 6.0, 3.0, 0.0, 0.0, 100, 50.0, -50.0);
    // Roll Controller
    init_controller("Roll", roll_controller, 6.0, 3.0, 0.0, 0.0, 100, 50.0, -50.0);
    // Yaw Controller
    init_controller("Yaw", yaw_controller, 6.0, 1.0, 0.3499, 0.0583, 100, 20.0, -20.0);
    // dPitch Controller
    init_controller("dPitch", dpitch_controller, 250.0, 500.0, 2.5, 0.01, 100, 720.0, -720.0);
    // dRoll Controller
    init_controller("dRoll", droll_controller, 250.0, 500.0, 2.5, 0.01, 100, 720.0, -720.0);
    // dYaw Controller
    init_controller("dYaw", dyaw_controller, 120.0, 16.7, 0.0, 0.0, 100, 400.0, -400.0);

    // Publisher:
    pub_cmd_ = this->create_publisher<uned_crazyflie_config::msg::Actuators>("cmd_control", 10);

    // Subscriber:
    // Crazyflie Pose {Real: /pose; Sim: /ground_truth/pose}
    GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("pose", 10, std::bind(&AttitudeRateController::gtposeCallback, this, _1));
    // Reference from Off-board Controller
    ref_cmd_ = this->create_subscription<uned_crazyflie_config::msg::Cmdsignal>("onboard_cmd", 10, std::bind(&AttitudeRateController::refcmdCallback, this, _1));

    return true;
}


bool AttitudeRateController::iterate(){
    RCLCPP_INFO_ONCE(this->get_logger(), "AttitudeRateController::iterate(). ok.");
    if (first_ref_received && first_pose_received) {
        RCLCPP_INFO_ONCE(this->get_logger(), "AttitudeRateController::iterate(). Running ...");
        // Feedback:
        rpy_state = quaternion2euler(GT_pose.orientation);

        // Attitude Controller
        // Pitch controller
        pitch_controller.error[0] = (ref_cmd.pitch - rpy_state.pitch);
        dpitch_ref = pid_controller(pitch_controller, dt);
        // Roll controller
        roll_controller.error[0] = (ref_cmd.roll - rpy_state.roll);
        droll_ref = pid_controller(roll_controller, dt);
        // Yaw controller
        yaw_controller.error[0] = (ref_cmd.yaw - rpy_state.yaw);
        dyaw_ref = pid_controller(yaw_controller, dt);

        // Rate Controller
        // dPitch controller
        dpitch_feedback[1] = dpitch_feedback[0];
        dpitch_feedback[0] = rpy_state.pitch;
        dpitch_signal = (dpitch_feedback[0] - dpitch_feedback[1]) / dt;
        dpitch_controller.error[0] = dpitch_ref - dpitch_signal;
        delta_pitch = pid_controller(dpitch_controller, dt);
        // dRoll controller
        droll_feedback[1] = droll_feedback[0];
        droll_feedback[0] = rpy_state.roll;
        droll_signal = (droll_feedback[0] - droll_feedback[1]) / dt;
        droll_controller.error[0] = droll_ref - droll_signal;
        delta_roll = pid_controller(droll_controller, dt);
        // dYaw controller
        dyaw_feedback[1] = dyaw_feedback[0];
        dyaw_feedback[0] = rpy_state.yaw;
        dyaw_signal = (dyaw_feedback[0] - dyaw_feedback[1]) / dt;
        dyaw_controller.error[0] = dyaw_ref - dyaw_signal;
        delta_yaw = pid_controller(dyaw_controller, dt);

        // Mixer Controller
        motors[0] = ref_cmd.thrust - 0.5 * delta_pitch - 0.5 * delta_roll - delta_yaw;
        motors[1] = ref_cmd.thrust + 0.5 * delta_pitch - 0.5 * delta_roll + delta_yaw;
        motors[2] = ref_cmd.thrust + 0.5 * delta_pitch + 0.5 * delta_roll - delta_yaw;
        motors[3] = ref_cmd.thrust - 0.5 * delta_pitch + 0.5 * delta_roll + delta_yaw;

        // Publish Control CMD
        auto msg_cmd = uned_crazyflie_config::msg::Actuators();
        for (int i = 0; i < 4; i++)
            msg_cmd.angular_velocities[i] = motors[i];
        pub_cmd_->publish(msg_cmd);
    }
    else {
        RCLCPP_INFO_ONCE(this->get_logger(), "AttitudeRateController::iterate(). Waiting reference & feedback orientation");
    }
    

  return true;
}


int main(int argc, char ** argv){
  try{
    rclcpp::init(argc, argv);
    auto crazyflie_attituderate_controller = std::make_shared<AttitudeRateController>();
    rclcpp::Rate loop_rate(500);
    crazyflie_attituderate_controller->initialize();

    while (rclcpp::ok()){
      rclcpp::spin_some(crazyflie_attituderate_controller);
      crazyflie_attituderate_controller->iterate();
      loop_rate.sleep();
    }
    return 0;
  } catch (std::exception &e){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s",e.what());
    }
}
