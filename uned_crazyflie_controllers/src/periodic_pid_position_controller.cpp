#include "uned_crazyflie_controllers/CrazyfliePositionController.hpp"

using std::placeholders::_1;

bool PositionController::initialize(){
    RCLCPP_INFO(this->get_logger(),"PositionController::inicialize() ok.");

    // Lectura de parámetros
    this->get_parameter("ROBOT_ID", robotid);
    this->get_parameter("Feedback_topic", feedback_topic);
    this->get_parameter("DEBUG", debug_flag);
    m_controller_type = "PERIODIC PID";
    RCLCPP_INFO(this->get_logger(),"Controller Type: %s, \tRobot id: %s", m_controller_type.c_str(), robotid.c_str());

    // Z Controller
    this->get_parameter("ZKp", Kp);
    this->get_parameter("ZKi", Ki);
    this->get_parameter("ZKd", Kd);
    this->get_parameter("ZTd", Td);
    z_controller = init_controller("Z", Kp, Ki, Kd, Td, 100, 1.0, -1.0);
    // W Controller
    this->get_parameter("WKp", Kp);
    this->get_parameter("WKi", Ki);
    this->get_parameter("WKd", Kd);
    this->get_parameter("WTd", Td);
    w_controller = init_controller("W", Kp, Ki, Kd, Td, 100, 26.0, -16.0);
    // X Controller
    this->get_parameter("XKp", Kp);
    this->get_parameter("XKi", Ki);
    this->get_parameter("XKd", Kd);
    this->get_parameter("XTd", Td);
    x_controller = init_controller("X", Kp, Ki, Kd, Td, 100, 1.0, -1.0);
    // U Controller
    this->get_parameter("UKp", Kp);
    this->get_parameter("UKi", Ki);
    this->get_parameter("UKd", Kd);
    this->get_parameter("UTd", Td);
    u_controller = init_controller("U", Kp, Ki, Kd, Td, 100, 30.0, -30.0);
    // Y Controller
    this->get_parameter("YKp", Kp);
    this->get_parameter("YKi", Ki);
    this->get_parameter("YKd", Kd);
    this->get_parameter("YTd", Td);
    y_controller = init_controller("Y", Kp, Ki, Kd, Td, 100, 1.0, -1.0);
    // V Controller
    this->get_parameter("VKp", Kp);
    this->get_parameter("VKi", Ki);
    this->get_parameter("VKd", Kd);
    this->get_parameter("VTd", Td);
    v_controller = init_controller("V", Kp, Ki, Kd, Td, 100, 30.0, -30.0);

    // Publisher:
    // Referencias para los controladores PID Attitude y Rate
    pub_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("onboard_cmd", 10);

    // Subscriber:
    // Crazyflie Pose {Real: /cf_pose; Sim: /ground_truth/pose}
    GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>(feedback_topic, 10, std::bind(&PositionController::gtposeCallback, this, _1));
    // Reference:
    ref_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("goal_pose", 10, std::bind(&PositionController::positionreferenceCallback, this, _1));

    return true;
}

bool PositionController::iterate(){
    RCLCPP_INFO_ONCE(this->get_logger(), "PositionController::iterate(). ok.");
    if (first_pose_received && first_ref_received && !fail) {
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
        thrust = thrust * 1000 + 38000;

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

        // Debug
        if(debug_flag){
          RCLCPP_INFO(this->get_logger(), "Z: Error: \t%.2f \tSignal:%.2f", z_controller.error[0], w_ref);
          RCLCPP_INFO(this->get_logger(), "W: Error: \t%.2f \tSignal:%.2f", w_controller.error[0], thrust);
          RCLCPP_INFO(this->get_logger(), "X: Error: \t%.2f \tSignal:%.2f", x_controller.error[0], u_ref);
          RCLCPP_INFO(this->get_logger(), "U: Error: \t%.2f \tPitch:%.2f", u_controller.error[0], pitch);
          RCLCPP_INFO(this->get_logger(), "Y: Error: \t%.2f \tSignal:%.2f", y_controller.error[0], v_ref);
          RCLCPP_INFO(this->get_logger(), "V: Error: \t%.2f \tRoll:%.2f", v_controller.error[0], roll);
        }
        // Publish Control CMD
        auto msg_cmd = std_msgs::msg::Float64MultiArray();
        msg_cmd.data = { 0.0, 0.0, 0.0, rpy_ref.yaw };
        // msg_cmd.data = { thrust, roll, pitch, rpy_ref.yaw };
        if (abs(GT_pose.position.x) > 1.2 || abs(GT_pose.position.y) > 1.2)
            fail = true;
        if (!fail)
            msg_cmd.data = { thrust, roll, pitch, rpy_ref.yaw };

        RCLCPP_INFO(this->get_logger(), "Thrust: \t%.2f \tRoll:%.2f \tPitch:%.2f \tYaw:%.2f", msg_cmd.data[0], msg_cmd.data[1], msg_cmd.data[2], msg_cmd.data[3]);
        pub_cmd_->publish(msg_cmd);
    }
    else
        RCLCPP_INFO_ONCE(this->get_logger(), "PositionController::iterate(). Waiting reference & feedback position");

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

euler_angles PositionController::quaternion2euler(geometry_msgs::msg::Quaternion quat) {
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

struct pid_s PositionController::init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit){
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

double PositionController::pid_controller(struct pid_s controller, double dt){
	double outP = controller.kp * controller.error[0];
	controller.integral = controller.integral + controller.ki * controller.error[1] * dt;
	controller.derivative = (controller.td/(controller.td+controller.nd+dt))*controller.derivative+(controller.kd*controller.nd/(controller.td+controller.nd*dt))*(controller.error[0]-controller.error[1]);
	double out = outP + controller.integral + controller.derivative;

	if(controller.upperlimit != 0.0){
		double out_i = out;

		if (out > controller.upperlimit)
			out = controller.upperlimit;
		if (out < controller.lowerlimit)
			out = controller.lowerlimit;

		controller.integral = controller.integral - (out - out_i) * sqrt(controller.kp / controller.ki);
	}

	controller.error[1] = controller.error[0];

	return out;
}

void PositionController::positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
    ref_pose.position = msg->position;
    ref_pose.orientation = msg->orientation;
    if (!first_ref_received)
        first_ref_received = true;
}

void PositionController::gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
    GT_pose.position = msg->position;
    GT_pose.orientation = msg->orientation;
    if(!first_pose_received){
      RCLCPP_INFO_ONCE(this->get_logger(),"Init Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);
      first_pose_received = true;
    }
}
