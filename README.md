# uned_crazyflie_ros_pkg
Repositorio con los paquetes de ROS y ficheros de configuración para la teleoperación y simulación del dron crazyflie 2.1 en ROS, Gazebo y Matlab. La finalidad es obtener una herramienta Hardware-in-the-Loop que sea facilmente escalable y mantenible.

#### Estructura 
- **doc**. Contiene un fichero _.tex_ que aborda más en detalle toda la información relacionada con el repositorio: esquemas de ROS, búsquedas bibliográficas, enlaces de interés, etc.
- **scripts**. Contiene aquellos ficheros auxiliares que no forman parte de ningún paquete de ROS. Por ejemplo, ficheros _.sh_ para automatizar procesos repetitivos como la conversión de los ficheros _.bag_ a txt o los scripts de Matlab para representar datasets.
- **submodules**. En este directorio están vinculados otros repositorios que se reutilizan, o se toman de base, para tareas ya abordadas por otros usuarios.
- **uned_crazyflie_config**. Paquete de ROS. Contiene aquellos elementos auxiliares para la configuración del entorno, así como los _.launch_ para la ejecución en bloque de las diferentes estructuras del sistema.
- **uned_crazyflie_drone**. Paquete de ROS. Comprende los nodos propios desarrollados para la integración de drones en el sistema. Incluye toda la información asociada para su correcta puesta en marcha.
- **uned_crazyflie_test**. Paquete de ROS. Paquete en el que se incluyen todos los elementos destinados a realizar comprobaciones en el sistema de forma rápida. Por ejemplo los nodos _talker_ y _listener_ que se desarrollan al empezar a usar ROS, que en este caso se usan para comprobar la correcta comunicación entre máquinas en el sistema distribuido.

## Instalación ##
- [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- Rosbridge -> `sudo apt-get install ros-<rosdistro>-rosbridge-suite`
  
  Para ejecutar en distintas máquinas:
  ```
  vim ~/.bashrc
  ...
  export ROS_MASTER_URI=http://xxx.xxx.x.xx:11311
  export ROS_HOSTNAME=http://xxx.xxx.x.xx
  ```
  lanzar:
  roslaunch rosbridge_server rosbridge_websocket.launch
 
 - `sudo apt-get install ros-<rosdistro>-octomap`
 - install xacro
## Espacio de trabajo ##
```
mkdir -p crazyflie_ws/src
cd crazyflie/src
git clone https://github.com/FranciscoJManasAlvarez/uned_crazyflie_ros_pkg
cd uned_crazyflie_ros_pkg
git submodules init
git submodules update
cd ..
git clone https://github.com/ethz-asl/mav_comm.git
cd ../..
catkin build
echo "source devel/setup.bash" >> ~/.bashrc
```
mav_comm??


## Configurar los permisos udev
```
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER
sudo touch /etc/udev/rules.d/99-crazyradio.rules
```

Añadir al archivo /etc/udev/rules.d/99-crazyradio.rules:
```
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
```
Para conectar el Crazyflie 2.0 por usb:
```
sudo touch /etc/udev/rules.d/99-crazyradio.rules
```
```
# Crazyflie 2.0
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
```
Recargar udev-rules:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Proyecto anterior
En Ubuntu 18.04 y con ROS melodic, funciona perfectamente [crazyflie_ros](https://github.com/whoenig/crazyflie_ros). Tiene interfaz de rviz (lo que no es demasiado importante ahora. Para la ejecución:
```
roslaunch crazyflie_demo teleop_ps3.launch uri:=radio://0/80/2M joy_dev:=/dev/input/js2
```
Lista de topics que genera la ejecución:
```
Published topics:
 * /crazyflie/imu [sensor_msgs/Imu] 1 publisher
 * /move_base_simple/goal [geometry_msgs/PoseStamped] 1 publisher
 * /crazyflie/joy [sensor_msgs/Joy] 1 publisher
 * /rosout [rosgraph_msgs/Log] 7 publishers
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /clicked_point [geometry_msgs/PointStamped] 1 publisher
 * /initialpose [geometry_msgs/PoseWithCovarianceStamped] 1 publisher
 * /crazyflie/temperature [sensor_msgs/Temperature] 1 publisher
 * /crazyflie/cmd_vel [geometry_msgs/Twist] 1 publisher
 * /crazyflie/magnetic_field [sensor_msgs/MagneticField] 1 publisher
 * /crazyflie/packets [crazyflie_driver/crtpPacket] 1 publisher
 * /diagnostics [diagnostic_msgs/DiagnosticArray] 1 publisher
 * /crazyflie/battery [std_msgs/Float32] 1 publisher
 * /crazyflie/pressure [std_msgs/Float32] 1 publisher
 * /crazyflie/rssi [std_msgs/Float32] 1 publisher

Subscribed topics:
 * /crazyflie/cmd_full_state [crazyflie_driver/FullState] 1 subscriber
 * /crazyflie/joy/set_feedback [sensor_msgs/JoyFeedbackArray] 1 subscriber
 * /crazyflie/cmd_velocity_world [crazyflie_driver/VelocityWorld] 1 subscriber
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /crazyflie/joy [sensor_msgs/Joy] 2 subscribers
 * /crazyflie/external_pose [geometry_msgs/PoseStamped] 1 subscriber
 * /crazyflie/cmd_stop [std_msgs/Empty] 1 subscriber
 * /tf [tf2_msgs/TFMessage] 1 subscriber
 * /tf_static [tf2_msgs/TFMessage] 1 subscriber
 * /crazyflie/cmd_position [crazyflie_driver/Position] 1 subscriber
 * /crazyflie/cmd_vel [geometry_msgs/Twist] 1 subscriber
 * /crazyflie/imu [sensor_msgs/Imu] 1 subscriber
 * /crazyflie/cmd_hover [crazyflie_driver/Hover] 1 subscriber
 * /crazyflie/external_position [geometry_msgs/PointStamped] 1 subscriber
 * /crazyflie/battery [std_msgs/Float32] 1 subscriber
 * /crazyflie/rssi [std_msgs/Float32] 1 subscriber
```
En ROS noetic, falla a los pocos segundos y no es capaz de lanzar el joy. Se debe instalar el paquete de ros correspondiente.
```
sudo apt-get install ros-noetic-joystick-drivers
```
