#include "uned_crazyflie_controllers/CrazyflieTrajectoryController.hpp"

using std::placeholders::_1;

bool TrajectoryController::initialize(){
    RCLCPP_INFO(this->get_logger(),"TrajectoryController::inicialize() ok.");

  	// Lectura de parámetros
  	this->get_parameter("ROBOT_ID", robotid);
    this->get_parameter("DEBUG", debug_flag);

    readFile("example.txt");

    // Crazyflie Pose Ref
    ref_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("goal_pose", 10);
    // Crazyflie Pose
    GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("cf_pose", 10, std::bind(&TrajectoryController::gtposeCallback, this, _1));

    return true;
}

bool TrajectoryController::iterate(){
//      if(first_pose_received && new_ref){
      // Leer horizonte de referencias
      auto first_pose = trayectory.begin();
      auto aux_pose = first_pose[0];
      double aux = aux_pose[0];

      RCLCPP_INFO(this->get_logger(),"Time %f", aux);
      auto msg = geometry_msgs::msg::Pose();
      msg.position.x = aux_pose[1];
      msg.position.y = aux_pose[2];
      msg.position.z = aux_pose[3];

      ref_pose_->publish(msg);
      
      trayectory.erase(first_pose);
//    }

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

bool TrajectoryController::readFile(std::string name){
	std::string line;
  std::ifstream myfile (name);
  if (myfile.is_open()){
    while ( getline (myfile,line) ){
			// RCLCPP_INFO(this->get_logger(),"Pose: %s", line.c_str());
			std::string delimiter = ",";
			size_t pos = 0;
			while ((pos = line.find(delimiter)) != std::string::npos) {
				file_pose.push_back(std::stod(line.substr(0, pos).c_str()));
    		line.erase(0, pos + delimiter.length());
			}
			file_pose.push_back(std::stod(line.substr(0, pos)));
			trayectory.push_back(file_pose);
			file_pose.clear();
    }
    myfile.close();
  }
  else
		RCLCPP_INFO(this->get_logger(),"Unable to open file");

	return true;
}
