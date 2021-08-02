#include <uned_crazyflie_controllers/CrazyflieTrajectoryController.h>

bool CrazyflieTrajectoryController::initialize()
{
	ROS_INFO("CrazyflieController::inicialize() ok.");

	// Publisher:
    m_pub_trayectory = m_nh.advertise<geometry_msgs::Pose>("position_reference", 10);

	return true;
}

bool CrazyflieTrajectoryController::iterate()
{
	// Housekeeping -------
	// TO-DO:
	ROS_INFO_THROTTLE(1, "In progress ...");
	Eigen::Vector3d p0(0.0, 0.0, 0.0);
	double desired_yaw = 0.0;
	trajectoryCallback(p0, desired_yaw);
	ros::Duration(5.0).sleep();

	Eigen::Vector3d p1(0.0, 0.0, 1.0);
	trajectoryCallback(p1, desired_yaw);

	ros::Duration(10.0).sleep();
	Eigen::Vector3d p2(0.0, 1.0, 1.0);
	trajectoryCallback(p2, desired_yaw);

    ros::Duration(10.0).sleep();
    Eigen::Vector3d p3(1.0, 1.0, 1.0);
	trajectoryCallback(p3, desired_yaw);

    ros::Duration(10.0).sleep();
    Eigen::Vector3d p4(1.0, 0.0, 1.0);
    trajectoryCallback(p4, desired_yaw);

    ros::Duration(10.0).sleep();
    trajectoryCallback(p1, desired_yaw);

		ros::Duration(100.0).sleep();

    return true;
}

void CrazyflieTrajectoryController::trajectoryCallback(const Eigen::Vector3d position, double yaw){
    /*
		trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(position, yaw, &trajectory_msg);
    ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
             m_nh.getNamespace().c_str(), position.x(),
             position.y(), position.z());
		*/
		geometry_msgs::Pose trajectory_msg;
		trajectory_msg.position.x = position[0];
		trajectory_msg.position.y = position[1];
		trajectory_msg.position.z = position[2];
		trajectory_msg.orientation.x = 0;
		trajectory_msg.orientation.y = 0;
		trajectory_msg.orientation.z = 0;
		trajectory_msg.orientation.w = 1;
    m_pub_trayectory.publish(trajectory_msg);
}

void CrazyflieTrajectoryController::gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	m_GT_pose.position = msg->position;
	m_GT_pose.orientation = msg->orientation;
}
