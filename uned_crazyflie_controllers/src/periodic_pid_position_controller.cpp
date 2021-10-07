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

    Z_q[0] = 915017.5;
    Z_q[1] = -1814982.5;
    Z_q[2] = 900000;
    RCLCPP_INFO(this->get_logger(),"Z PID(Z): \t%.2f \t%.2f \t%.2f", Z_q[0], Z_q[1], Z_q[2]);

    X_q[0] = 1.0;
    X_q[1] = -1.0;
    X_q[2] = 0.0;
    RCLCPP_INFO(this->get_logger(),"X PID(Z): \t%.2f \t%.2f \t%.2f", X_q[0], X_q[1], X_q[2]);

    U_q[0] = 30.0;
    U_q[1] = -29.99;
    U_q[2] = 0.0;
    RCLCPP_INFO(this->get_logger(),"U PID(Z): \t%.2f \t%.2f \t%.2f", U_q[0], U_q[1], U_q[2]);

    Y_q[0] = 1.0;
    Y_q[1] = -1.0;
    Y_q[2] = 0.0;
    RCLCPP_INFO(this->get_logger(),"Y PID(Z): \t%.2f \t%.2f \t%.2f", Y_q[0], Y_q[1], Y_q[2]);

    V_q[0] = -30.0;
    V_q[1] = 29.99;
    V_q[2] = 0.0;
    RCLCPP_INFO(this->get_logger(),"V PID(Z): \t%.2f \t%.2f \t%.2f", V_q[0], V_q[1], V_q[2]);

    Yaw_q[0] = 3.0;
    Yaw_q[1] = -3.0;
    Yaw_q[2] = 0.0;
    RCLCPP_INFO(this->get_logger(),"Yaw PID(Z): \t%.2f \t%.2f \t%.2f", Yaw_q[0], Yaw_q[1], Yaw_q[2]);

    // Publisher:
    // Referencias para los controladores PID Attitude y Rate
    pub_cmd_ = this->create_publisher<uned_crazyflie_config::msg::Cmdsignal>("cf_cmd_control", 10);

    // Subscriber:
    // Crazyflie Pose
    // GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("ground_truth/pose", 10, std::bind(&PositionController::gtposeCallback, this, _1));
    GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("dron01/pose", 10, std::bind(&PositionController::gtposeCallback, this, _1));
    // Reference:
    ref_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("pose_ref", 10, std::bind(&PositionController::positionreferenceCallback, this, _1));

    return true;
}

bool PositionController::iterate(){

    if(first_pose_received && first_ref_received)
    {
        // Altitude Controller
        {
            // Update error vector
            z_error_signal[2] = z_error_signal[1];
            z_error_signal[1] = z_error_signal[0];
            z_error_signal[0] = ref_pose.position.z - GT_pose.position.z;

            // Update signal vector
            delta_omega[1] = delta_omega[0];
            delta_omega[0] = delta_omega[1] + Z_q[0]*z_error_signal[0] + Z_q[1]*z_error_signal[1] + Z_q[2]*z_error_signal[2];

            // Saturation
            if(delta_omega[0]>15000)
                delta_omega[0] = 15000;
            if(delta_omega[0]<-2000)
                delta_omega[0] = -20000;

            // Output signal
            omega = delta_omega[0]+(we-4070.3)/0.2685;
        }

        // Convert quaternion to yw
        double siny_cosp_ref = 2 * (ref_pose.orientation.w*ref_pose.orientation.z+ref_pose.orientation.x*ref_pose.orientation.y);
        double cosy_cosp_ref = 1 - 2 * (ref_pose.orientation.y*ref_pose.orientation.y + ref_pose.orientation.z*ref_pose.orientation.z);
        double yaw_ref = std::atan2(siny_cosp_ref,cosy_cosp_ref);
        double siny_cosp = 2 * (GT_pose.orientation.w*GT_pose.orientation.z+GT_pose.orientation.x*GT_pose.orientation.y);
        double cosy_cosp = 1 - 2 * (GT_pose.orientation.y*GT_pose.orientation.y + GT_pose.orientation.z*GT_pose.orientation.z);
        double yaw = std::atan2(siny_cosp,cosy_cosp);
        // X-Y Controller
        {
            // Position
            // Update local error
            x_error_signal[2] = x_error_signal[1];
            x_error_signal[1] = x_error_signal[0];
            x_error_signal[0] = (ref_pose.position.x - GT_pose.position.x)*cos(yaw)+(ref_pose.position.y - GT_pose.position.y)*sin(yaw);
            y_error_signal[2] = y_error_signal[1];
            y_error_signal[1] = y_error_signal[0];
            y_error_signal[0] = -(ref_pose.position.x - GT_pose.position.x)*sin(yaw)+(ref_pose.position.y - GT_pose.position.y)*cos(yaw);
            // Update signal vector
            uc[1] = uc[0];
            uc[0] = uc[1] + X_q[0]*x_error_signal[0] + X_q[1]*x_error_signal[1] + X_q[2]*x_error_signal[2];
            vc[1] = vc[0];
            vc[0] = vc[1] + Y_q[0]*y_error_signal[0] + Y_q[1]*y_error_signal[1] + Y_q[2]*y_error_signal[2];
            // Speed
            u_feedback[1] = u_feedback[0];
            u_feedback[0] = GT_pose.position.x;
            double u_signal = (u_feedback[0]-u_feedback[1])/0.01;
            v_feedback[1] = v_feedback[0];
            v_feedback[0] = GT_pose.position.y;
            double v_signal = (v_feedback[0]-v_feedback[1])/0.01;

            // Update error
            u_error_signal[2] = u_error_signal[1];
            u_error_signal[1] = u_error_signal[0];
            u_error_signal[0] = uc[0]-u_signal;
            v_error_signal[2] = v_error_signal[1];
            v_error_signal[1] = v_error_signal[0];
            v_error_signal[0] = vc[0]-v_signal;
            // Update signal vector
            pitch_ref[1] = pitch_ref[0];
            pitch_ref[0] = pitch_ref[1] + U_q[0]*u_error_signal[0] + U_q[1]*u_error_signal[1] + U_q[2]*u_error_signal[2];
            roll_ref[1] = roll_ref[0];
            roll_ref[0] = roll_ref[1] + V_q[0]*v_error_signal[0] + V_q[1]*v_error_signal[1] + V_q[2]*v_error_signal[2];
            // Saturation
            if(pitch_ref[0]>30)
                pitch_ref[0] = 30;
            if(pitch_ref[0]<-30)
                pitch_ref[0] = -30;
            if(roll_ref[0]>30)
                roll_ref[0] = 30;
            if(roll_ref[0]<-30)
                roll_ref[0] = -30;
        }

        // Yaw Controller
        {
            // Update error
            yaw_error_signal[2] = yaw_error_signal[1];
            yaw_error_signal[1] = yaw_error_signal[0];
            yaw_error_signal[0] = (yaw_ref-yaw)*180/3.14159265;

            // Update signal vector
            dyaw_ref[1] = dyaw_ref[0];
            dyaw_ref[0] = dyaw_ref[1] + Yaw_q[0]*yaw_error_signal[0] + Yaw_q[1]*yaw_error_signal[1] + Yaw_q[2]*yaw_error_signal[2];
        }
        // Publish Control CMD
        auto msg_cmd = uned_crazyflie_config::msg::Cmdsignal();
        msg_cmd.thrust = (int)omega;
        msg_cmd.roll = roll_ref[0];
        msg_cmd.pitch = pitch_ref[0];
        msg_cmd.yaw = 0.0;
        pub_cmd_->publish(msg_cmd);
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
