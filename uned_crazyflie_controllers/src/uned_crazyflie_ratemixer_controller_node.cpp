#include <uned_crazyflie_controllers/CrazyflieRateMixerController.h>
#include <ros/console.h>


int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, "crazyflie_ratemixer_controller");

		CrazyflieRateMixerController  crazyflie_ratemixer_controller;
		crazyflie_ratemixer_controller.initialize();

		ros::Rate loop_rate(500); // f : 500Hz => T = 2 ms
		while (ros::ok())
		{
			ros::spinOnce();
			crazyflie_ratemixer_controller.iterate();
			loop_rate.sleep();
		}

		return 0;

	} catch (std::exception &e)
	{
		ROS_ERROR("Exception: %s",e.what());
	}
}
