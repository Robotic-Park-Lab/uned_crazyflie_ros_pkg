#include <uned_crazyflie_controllers/CrazyfliePositionController.h>
#include <ros/console.h>


int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, "crazyflie_position_controller");

		CrazyfliePositionController crazyflie_position_controller;
		crazyflie_position_controller.initialize();

		ros::Rate loop_rate(100); // f : 100Hz => T = 10 ms
		while (ros::ok())
		{
			ros::spinOnce();
			crazyflie_position_controller.iterate();
			loop_rate.sleep();
		}

		return 0;

	} catch (std::exception &e)
	{
		ROS_ERROR("Exception: %s",e.what());
	}
}
