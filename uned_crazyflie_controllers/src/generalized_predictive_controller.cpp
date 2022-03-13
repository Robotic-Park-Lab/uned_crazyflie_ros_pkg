#include "uned_crazyflie_controllers/CrazyfliePositionController.hpp"

using std::placeholders::_1;

bool PositionController::initialize(){
	RCLCPP_INFO(this->get_logger(),"PositionController::inicialize() ok.");
	dt = 0.01;
	// Lectura de parámetros
	this->get_parameter("ROBOT_ID", robotid);
	this->get_parameter("Feedback_pose_topic", feedback_pose_topic);
	this->get_parameter("Feedback_twist_topic", feedback_twist_topic);
	this->get_parameter("DEBUG", debug_flag);
	m_controller_type = "Generalized Predictive Controller";
	RCLCPP_INFO(this->get_logger(),"Controller Type: %s, \tRobot id: %s", m_controller_type.c_str(), robotid.c_str());

	readFile("example.txt");
	/*
	// Displaying the 2D vector
	for (int i = 0; i < gpc_trayectory.size(); i++) {
			for (int j = 0; j < gpc_trayectory[i].size(); j++)
					std::cout << gpc_trayectory[i][j] << " ";
			std::cout << std::endl;
	}
	*/
	// Publisher:
	// Referencias para los controladores PID Attitude y Rate
	pub_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("onboard_cmd", 10);

	return true;
}

bool PositionController::iterate(){
	RCLCPP_INFO_ONCE(this->get_logger(), "PositionController::iterate(). ok.");
	// Leer horizonte de referencias
	auto first = gpc_trayectory.begin();
	for (int i = 0; i < N; i++) {
		w1.push_back(gpc_trayectory[i]);
	}
	gpc_trayectory.erase(first);
	auto first_t = w1[0];
	double aux = first_t[0];
	RCLCPP_INFO(this->get_logger(),"Time %f", aux);

	// Respuesta libre; f = Gp*du + Fp*yaux; // yaux = feedback
	for(int i = 0; i<10; i++)
		f[i] = Gp[i]*du;

	for(int i = 0; i<10; i++)
		for(int k = 0; k<3; k++){
			f[i] += Fp[i][k]*yaux[k];
		}

	// Señal de control futuras; du = K * (w1 - f);
	du = 0;
	for(int i = 0; i<10; i++){
		auto first_w = w1[i];
		aux = first_w[3];
		du += K[i] * (aux - f[i]);
	}

	// Señal de control
	u += du;
	RCLCPP_INFO(this->get_logger(),"U: %f", u);
	// Publicar señal

	w1.clear();
	return true;
}

int main(int argc, char ** argv){
	try{
		rclcpp::init(argc, argv);
		auto crazyflie_position_controller = std::make_shared<PositionController>();
		rclcpp::Rate loop_rate(10);
		crazyflie_position_controller->initialize();

		while (rclcpp::ok()){
			crazyflie_position_controller->iterate();
			rclcpp::spin_some(crazyflie_position_controller);
			loop_rate.sleep();
		}
		return 0;
	}
	catch (std::exception &e){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s",e.what());
	}
}

bool PositionController::readFile(std::string name){
	std::string line;
  std::ifstream myfile (name);
  if (myfile.is_open()){
    while ( getline (myfile,line) ){
			// RCLCPP_INFO(this->get_logger(),"Pose: %s", line.c_str());
			std::string delimiter = ",";
			size_t pos = 0;
			while ((pos = line.find(delimiter)) != std::string::npos) {
				ref_gpc_pose.push_back(std::stod(line.substr(0, pos).c_str()));
    		line.erase(0, pos + delimiter.length());
			}
			ref_gpc_pose.push_back(std::stod(line.substr(0, pos)));
			gpc_trayectory.push_back(ref_gpc_pose);
			ref_gpc_pose.clear();
    }
    myfile.close();
  }
  else
		RCLCPP_INFO(this->get_logger(),"Unable to open file");

	return true;
}
