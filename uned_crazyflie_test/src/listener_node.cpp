#include<CrazyflieTest.h>
#include<ros/console.h>

int main(int argc, char **argv)
{
  try
  {
        ros::init(argc, argv, "listener_node");

        CrazyflieTest node;
        node.initialize();

        ros::Rate loop_rate(20);
        while (ros::ok())
        {
            ros::spinOnce();
            node.iterate();
            loop_rate.sleep();
        }

        return 0;
  } catch(std::exception &e)
  {
        ROS_ERROR("Exception: %s", e.what());
  }
}
