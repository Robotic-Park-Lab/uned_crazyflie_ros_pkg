#include<CrazyflieTest.h>

using namespace std;

bool CrazyflieTest::initialize()
{
    ROS_INFO("CrazyflieTest::initialize() ok.");
    ros::Subsriber sub = n.subscribe("chatter",1000,chatterCallback);
}

bool CrazyflieTest::iterate()
{
    ROS_INFO("I heard: [%s]", mensaje);
}
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  mensaje = msg->data.c_str();
}
