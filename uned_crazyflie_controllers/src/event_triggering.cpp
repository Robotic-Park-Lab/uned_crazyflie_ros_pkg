#include "uned_crazyflie_controllers/CrazyflieEventTriggering.hpp"
using std::placeholders::_1;

bool EventTriggering::initialize(){
  RCLCPP_INFO(this->get_logger(),"EventTriggering::inicialize() ok.");

  return true;
}

bool EventTriggering::iterate(){
  RCLCPP_INFO(this->get_logger(),"EventTriggering::iterate() ok.");

  return true;
}

int main(int argc, char ** argv){
  try{
    rclcpp::init(argc, argv);
    auto event_triggering_node = std::make_shared<EventTriggering>();
    rclcpp::Rate loop_rate(10);
    event_triggering_node->initialize();

    while (rclcpp::ok()){
      event_triggering_node->iterate();
      rclcpp::spin_some(event_triggering_node);
      loop_rate.sleep();
    }

    return 0;
  } catch (std::exception &e){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s",e.what());
	}
}
