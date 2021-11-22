#include <uned_crazyflie_driver/CrazyflieDriver.h>
#include <ros/console.h>


int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, "uned_crazyflie_driver");

		CrazyflieDriverSim uned_crazyflie_driver;
		uned_crazyflie_driver.initialize();

		ros::Rate loop_rate(100); // f : 100Hz => T = 10 ms

		while (ros::ok())
		{
			ros::spinOnce();
			uned_crazyflie_driver.iterate();
			loop_rate.sleep();
		}
		return 0;

	} catch (std::exception &e)
	{
		ROS_ERROR("Exception: %s",e.what());
	}
}
