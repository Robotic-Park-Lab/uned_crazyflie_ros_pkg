#include <uned_crazyflie_controllers/CrazyflieAttitudeController.h>
#include <ros/console.h>


int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, "crazyflie_attituderate_controller");

		CrazyflieAttitudeController  crazyflie_attitude_controller;
		crazyflie_attitude_controller.initialize();

		ros::Rate loop_rate(500); // f : 500Hz => T = 2 ms
		while (ros::ok())
		{
			ros::spinOnce();
			crazyflie_attitude_controller.iterate();
			loop_rate.sleep();
		}

		return 0;

	} catch (std::exception &e)
	{
		ROS_ERROR("Exception: %s",e.what());
	}
}
