#include "uned_crazyflie_controllers/CrazyflieAttitudeRateController.hpp"

using std::placeholders::_1;

bool AttitudeRateController::initialize(){
    RCLCPP_INFO(this->get_logger(),"AttitudeRateController::inicialize() ok.");

    // Lectura de parámetros
    this->get_parameter("ROBOT_ID", robotid);
    this->get_parameter("DEBUG", debug_flag);
    m_controller_type = "PERIODIC PID";
    RCLCPP_INFO(this->get_logger(),"Controller Type: %s, \tRobot id: %s,", m_controller_type.c_str(), robotid.c_str());

    // Pitch Controller
    this->get_parameter("PitchKp", Kp);
    this->get_parameter("PitchKi", Ki);
    this->get_parameter("PitchKd", Kd);
    this->get_parameter("PitchTd", Td);
    pitch_controller = init_controller("Pitch", Kp, Ki, Kd, Td, 100, 50.0, -50.0);
    // Roll Controller
    this->get_parameter("RollKp", Kp);
    this->get_parameter("RollKi", Ki);
    this->get_parameter("RollKd", Kd);
    this->get_parameter("RollTd", Td);
    roll_controller = init_controller("Roll", Kp, Ki, Kd, Td, 100, 50.0, -50.0);
    // Yaw Controller
    this->get_parameter("YawKp", Kp);
    this->get_parameter("YawKi", Ki);
    this->get_parameter("YawKd", Kd);
    this->get_parameter("YawTd", Td);
    yaw_controller = init_controller("Yaw", Kp, Ki, Kd, Td, 100, 20.0, -20.0);
    // dPitch Controller
    this->get_parameter("dPitchKp", Kp);
    this->get_parameter("dPitchKi", Ki);
    this->get_parameter("dPitchKd", Kd);
    this->get_parameter("dPitchTd", Td);
    dpitch_controller = init_controller("dPitch", Kp, Ki, Kd, Td, 100, 720.0, -720.0);
    // dRoll Controller
    this->get_parameter("dRollKp", Kp);
    this->get_parameter("dRollKi", Ki);
    this->get_parameter("dRollKd", Kd);
    this->get_parameter("dRollTd", Td);
    droll_controller = init_controller("dRoll", Kp, Ki, Kd, Td, 100, 720.0, -720.0);
    // dYaw Controller
    this->get_parameter("dYawKp", Kp);
    this->get_parameter("dYawKi", Ki);
    this->get_parameter("dYawKd", Kd);
    this->get_parameter("dYawTd", Td);
    dyaw_controller = init_controller("dYaw", Kp, Ki, Kd, Td, 100, 400.0, -400.0);

    // Publisher:
    pub_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("cmd_control", 10);
    pub_act_ = this->create_publisher<std_msgs::msg::Bool>("attitude_act", 10);

    // Subscriber:
    // Crazyflie Pose {Real: /pose; Sim: /ground_truth/pose}
    GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("pose", 10, std::bind(&AttitudeRateController::gtposeCallback, this, _1));
    // Reference from Off-board Controller
    ref_cmd_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("onboard_cmd", 10, std::bind(&AttitudeRateController::refcmdCallback, this, _1));

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
        auto msg_cmd = std_msgs::msg::Float64MultiArray();
        msg_cmd.data = { motors[0], motors[1], motors[2], motors[3] };
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

euler_angles AttitudeRateController::quaternion2euler(geometry_msgs::msg::Quaternion quat){
    euler_angles rpy;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
    double cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
    rpy.roll = std::atan2(sinr_cosp, cosr_cosp) * (180 / 3.14159265);

    // pitch (y-axis rotation)
    double sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
    if (std::abs(sinp) >= 1)
        rpy.pitch = std::copysign(3.14159265 / 2, sinp) * (180 / 3.14159265);
    else
        rpy.pitch = std::asin(sinp) * (180 / 3.14159265);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    rpy.yaw = std::atan2(siny_cosp, cosy_cosp) * (180 / 3.14159265);

    return rpy;
}

double AttitudeRateController::pid_controller(struct pid_s &controller, double dt){
    double outP = controller.kp * controller.error[0];
    controller.integral = controller.integral + controller.ki * controller.error[1] * dt;
    controller.derivative = (controller.td/(controller.td+controller.nd+dt))*controller.derivative+(controller.kd*controller.nd/(controller.td+controller.nd*dt))*(controller.error[0]-controller.error[1]);
    double out = outP + controller.integral + controller.derivative;

    double out_i = out;

    if (out > controller.upperlimit)
        out = controller.upperlimit;
    if (out < controller.lowerlimit)
        out = controller.lowerlimit;

    controller.integral = controller.integral - (out - out_i) * sqrt(controller.kp / controller.ki);

    controller.error[1] = controller.error[0];

    return out;
}

struct pid_s AttitudeRateController::init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit) {
  struct pid_s controller;

  controller.kp = kp;
  controller.ki = ki;
  controller.kd = kd;
  controller.td = td;
  controller.nd = nd;
  controller.error[0] = 0.0;
  controller.error[1] = 0.0;
  controller.integral = 0.0;
  controller.derivative = 0.0;
  controller.upperlimit = upperlimit;
  controller.lowerlimit = lowerlimit;

  RCLCPP_INFO(this->get_logger(),"%s Controller: kp: %0.2f \tki: %0.2f \tkd: %0.2f", id, controller.kp, controller.ki, controller.kd);
  return controller;
}

void AttitudeRateController::gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    GT_pose.position = msg->position;
    GT_pose.orientation = msg->orientation;
    if (!first_pose_received)
        first_pose_received = true;
}

void AttitudeRateController::refcmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    ref_cmd.thrust = msg->data[0];
    ref_cmd.roll = msg->data[1];
    ref_cmd.pitch = msg->data[2];
    ref_cmd.yaw = msg->data[3];
    if (!first_ref_received){
      first_ref_received = true;
    }
}
