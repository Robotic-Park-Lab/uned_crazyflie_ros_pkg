#include <uned_crazyflie_controllers/CrazyflieTrajectoryController.h>

bool CrazyflieTrajectoryController::initialize()
{
	ROS_INFO("CrazyflieController::inicialize() ok.");

	// Publisher:
    m_pub_trayectory = m_nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);

	// Subscriber:
	// Crazyflie Pose
	m_sub_GT_pose = m_nh.subscribe( "ground_truth/pose", 10, &CrazyflieTrajectoryController::gtposeCallback, this);

	return true;
}

bool CrazyflieTrajectoryController::iterate()
{
	// Housekeeping -------
	// TO-DO:
	ROS_INFO_THROTTLE(1, "In progress ...");
	ros::Duration(5.0).sleep();

	Eigen::Vector3d p1(1.0, 1.0, 1.0);
	double desired_yaw = 0.0;
	trajectoryCallback(p1, desired_yaw);

	ros::Duration(30.0).sleep();
	Eigen::Vector3d p2(0.0, 1.0, 1.0);
	trajectoryCallback(p2, desired_yaw);

    ros::Duration(30.0).sleep();
    Eigen::Vector3d p3(0.0, 0.0, 1.0);
	trajectoryCallback(p3, desired_yaw);

    ros::Duration(30.0).sleep();
    Eigen::Vector3d p4(1.0, 0.0, 1.0);
    trajectoryCallback(p4, desired_yaw);

    ros::Duration(30.0).sleep();
    trajectoryCallback(p1, desired_yaw);

    return true;
}

void CrazyflieTrajectoryController::trajectoryCallback(const Eigen::Vector3d position, double yaw){
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(position, yaw, &trajectory_msg);
    ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
             m_nh.getNamespace().c_str(), position.x(),
             position.y(), position.z());
    m_pub_trayectory.publish(trajectory_msg);
}

void CrazyflieTrajectoryController::gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	m_GT_pose.position = msg->position;
	m_GT_pose.orientation = msg->orientation;
}
