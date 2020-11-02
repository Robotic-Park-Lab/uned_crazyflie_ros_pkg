#include<ros/ros.h>
#include<ros/console.h>
#include<std_msgs/String.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Float64.h>
#include<std_msgs/UInt8.h>

class CrazyflieTest
{
public:
    ros::NodeHandle n;

    ros::Subscriber n_sub_value;
    ros::Publisher m_pub_variable;

    bool initialize();

    bool iterate();

};
