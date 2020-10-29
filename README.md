# uned_crazyflie_ros_pkg
Simulador Hardware-in-the-Loop del dron crazyflie 2.1 en ROS, Gazebo y Matlab

## Instalación ##
- ROS -> http://wiki.ros.org/noetic/Installation
- Rosbridge -> sudo apt-get install ros-<rosdistro>-rosbridge-suite
  
  Para ejecutar en distintas máquinas:
  export ROS_MASTER_URI=http://xxx.xxx.x.xx:11311
  export ROS_HOSTNAME=http://xxx.xxx.x.xx
  
  lanzar:
  roslaunch rosbridge_server rosbridge_websocket.launch
  
  
## Espacio de trabajo ##
mkdir -p crazyflie_ws/src
cd crazyflie/src
git clone https://github.com/FranciscoJManasAlvarez/uned_crazyflie_ros_pkg
cd ../..
catkin build
source devel/setup.bash
