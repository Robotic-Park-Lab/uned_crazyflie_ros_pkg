# uned_crazyflie_ros_pkg
Repositorio con los paquetes de ROS y ficheros de configuración para la teleoperación y simulación del dron crazyflie 2.1 en ROS, Gazebo y Matlab. La finalidad es obtener una herramienta Hardware-in-the-Loop que sea facilmente escalable y mantenible.

#### Estructura 
- **doc**. Contiene un fichero _.tex_ que aborda más en detalle toda la información relacionada con el repositorio: esquemas de ROS, búsquedas bibliográficas, enlaces de interés, etc.
- **scripts**. Contiene aquellos ficheros auxiliares que no forman parte de ningún paquete de ROS. Por ejemplo, ficheros _.sh_ para automatizar procesos repetitivos como la conversión de los ficheros _.bag_ a txt o los scripts de Matlab para representar datasets.
- **submodules**. En este directorio están vinculados otros repositorios que se reutilizan, o se toman de base, para tareas ya abordadas por otros usuarios.
- **uned_crazyflie_config**. Paquete de ROS. Contiene aquellos elementos auxiliares para la configuración del entorno, así como los _.launch_ para la ejecución en bloque de las diferentes estructuras del sistema.
- **uned_crazyflie_drone**. Paquete de ROS. Comprende los nodos propios desarrollados para la integración de drones en el sistema. Incluye toda la información asociada para su correcta puesta en marcha.
- **uned_crazyflie_test**. Paquete de ROS. Paquete en el que se incluyen todos los elementos destinados a realizar comprobaciones en el sistema de forma rápida. Por ejemplo los nodos _talker_ y _listener_ que se desarrollan al empezar a usar ROS, que en este caso se usan para comprobar la correcta comunicación entre máquinas en el sistema distribuido.

## Instalación
El objetivo es implementar todo el sistema en [ROS Noetic Ninjemys](https://http://wiki.ros.org/noetic) y [Ubuntu 20.04 LTS (Focal Fossa)](https://releases.ubuntu.com/20.04/)  a fin de prolongar el mantenimiento y vigencia de la plataforma. No obstante, inicialmente, se plantea la reutilización de gran parte del material ya disponible en la web, para lo que habrá que trabajar con parte del sistema en la configuración comentada y otra parte en la versión anterior ([ROS Melodic Morenia](http://wiki.ros.org/melodic) y [Ubuntu 18.04 LTS (Bionic Beaver)](https://releases.ubuntu.com/18.04/))

### Pre-requisitos 📋
##### ROS
Lo primero debe ser tener instalada la correspondiente versión de ROS para el sistema operativo del dispositivo ([Noetic](https://http://wiki.ros.org/noetic/Installation), [Melodic](https://http://wiki.ros.org/melodic/Installation)). La máquina donde se ejecuten los paquetes reutilizados debe trabajar con ROS Melodic. No hay problema de compatibilidad en la interconexión de distintas máquinas siempre que los topics no presenten incompatibilidades entre versiones. 

##### rosbridge_suite
La conexión común de todos los componentes de la red de ROS se realiza a través del paquete [rosbridge_suite](http://wiki.ros.org/rosbridge_suite), instalado mediante el comando `sudo apt-get install ros-<rosdistro>-rosbridge-suite` (debe estar instalado previamente ROS en el dispositivo). Para la correcta identificación y conexión de cada máquina, se debe configurar en cada una los parámetros _ROS_MASTER_URI_ y _ROS_HOSTNAME_, dados por la ip de cada dispositivo. 
  ```
  sudo nano ~/.bashrc
  ...
  export ROS_MASTER_URI=http://xxx.xxx.x.xx:11311
  export ROS_HOSTNAME=xxx.xxx.x.xx
  ```
  Para el lanzamiento del paquete, se emplea el comando `roslaunch rosbridge_server rosbridge_websocket.launch`

##### Dependencias
 - **Octomap**. TO-DO: Especificar los paquetes que dependen
 ```
 sudo apt-get install ros-<rosdistro>-octomap
 ```
  - **Xacro**. TO-DO: Especificar los paquetes que dependen
 ```
 sudo apt-get install ros-<rosdistro>-xacro 
 ```
 - **[mav_comm](https://github.com/ethz-asl/mav_comm)**. TO-DO. Este paquete se emplea como complemento a gran parte de los paquetes ya desarrollados de Crazyflie y es compatible con ambas versiones de ROS y Ubuntu por lo que se puede alojar en el espacio de trabajo del dispositivo y compilarlo como cualquier otro paquete. 

##### - Ubuntu 18.04 - ROS Melodic Morenia
A continuación se detalla la instalación en el entorno de trabajo de ROS para el paquete [CrazyS](https://github.com/gsilano/CrazyS), de donde se reutiliza gran parte de la arquitectura de simulación.
```
mkdir -p catkin_ws/src
cd crazyflie/src
catkin_init_workspace
cd ..
catkin init
cd src
git clone https://github.com/gsilano/CrazyS.git
git clone https://github.com/gsilano/mav_comm.git

rosdep install --from-paths src -i
sudo apt install ros-melodic-rqt-rotors ros-melodic-rotors-comm ros-melodic-mav-msgs ros-melodic-rotors-control
sudo apt install ros-melodic-rotors-gazebo ros-melodic-rotors-evaluation ros-melodic-rotors-joy-interface
sudo apt install ros-melodic-rotors-gazebo-plugins ros-melodic-mav-planning-msgs ros-melodic-rotors-description ros-melodic-rotors-hil-interface
rosdep update
catkin build

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
En [link](https://github.com/gsilano/CrazyS#installation-instructions---ubuntu-1804-with-ros-melodic-and-gazebo-9) se detalla est proceso y posibles soluciones en caso de fallos con gazebo (como que no se inicie la simulación).
##### - Ubuntu 20.04 - ROS Noetic Ninjemys
La configuración del entorno de trabajo para el paquete desarrollado se muestra a continuación.
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
Este paquete compila correctamente en ambas versiones de ROS y Ubuntu. 

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
