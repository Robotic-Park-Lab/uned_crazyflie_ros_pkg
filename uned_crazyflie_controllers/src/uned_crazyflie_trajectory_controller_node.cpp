#include <uned_crazyflie_controllers/CrazyflieTrajectoryController.h>
#include <ros/console.h>


int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, "crazyflie_trajectory_controller");

		CrazyflieTrajectoryController crazyflie_trajectory_controller;
		crazyflie_trajectory_controller.initialize();

		ros::Rate loop_rate(100); // f : 100Hz => T = 10 ms

		ros::spinOnce();
		crazyflie_trajectory_controller.iterate();
		loop_rate.sleep();
		
		/*
		while (ros::ok())
		{
			ros::spinOnce();
			crazyflie_trajectory_controller.iterate();
			loop_rate.sleep();
		}
		*/
		return 0;

	} catch (std::exception &e)
	{
		ROS_ERROR("Exception: %s",e.what());
	}
}
