# uned_crazyflie_ros_pkg
Repositorio con los paquetes de ROS2 y ficheros de configuración para la teleoperación y simulación del dron crazyflie 2.1 en ROS, Gazebo y Matlab. La finalidad es obtener una herramienta Hardware-in-the-Loop que sea facilmente escalable y mantenible.

#### Estructura 
- **doc**. Contiene un fichero _.tex_ que aborda más en detalle toda la información relacionada con el repositorio: esquemas de ROS, búsquedas bibliográficas, enlaces de interés, etc.
- **scripts**. Contiene aquellos ficheros auxiliares que no forman parte de ningún paquete de ROS. Por ejemplo, ficheros _.sh_ para automatizar procesos repetitivos como la conversión de los ficheros _.bag_ a txt o los scripts de Matlab para representar datasets.
- **submodules**. En este directorio están vinculados otros repositorios que se reutilizan, o se toman de base, para tareas ya abordadas por otros usuarios.
- **uned_crazyflie_config**. Paquete de ROS. Contiene aquellos elementos auxiliares para la configuración del entorno, así como los _.launch.py_ para la ejecución en bloque de las diferentes estructuras del sistema.
- **uned_crazyflie_test_ros2**. Paquete de ROS. Paquete en el que se incluyen todos los elementos destinados a realizar comprobaciones en el sistema de forma rápida. Por ejemplo los nodos _talker_ y _listener_ que se desarrollan al empezar a usar ROS, que en este caso se usan para comprobar la correcta comunicación entre máquinas en el sistema distribuido.

## Instalación :book:
El objetivo es implementar todo el sistema en [ROS2 Galactic Geochelone](https://docs.ros.org/en/galactic/index.html) y [Ubuntu 20.04 LTS (Focal Fossa)](https://releases.ubuntu.com/20.04/)  a fin de prolongar el mantenimiento y vigencia de la plataforma.

### Pre-requisitos 📋
##### ROS
Lo primero debe ser tener instalada la correspondiente versión de ROS para el sistema operativo del dispositivo ([Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)). 

##### Matlab


##### Dependencias
 - **Octomap**. TO-DO: Especificar los paquetes que dependen
 ```
 sudo apt-get install ros-<rosdistro>-octomap
 ```
  - **Xacro**. TO-DO: Especificar los paquetes que dependen
 ```
 sudo apt-get install ros-<rosdistro>-xacro 
 ```
 - **Joy**. Paquete para realizar la lectura del joystick para teleoperación.
 ```
sudo apt-get install ros-<rosdistro>-joystick-drivers
```


### - Ubuntu 20.04 - ROS Galactic Gochelone
La configuración del entorno de trabajo para el paquete desarrollado se muestra a continuación.
```
mkdir -p crazyflie_ws/src
cd crazyflie/src
git clone -b ros2-galactic https://github.com/FranciscoJManasAlvarez/uned_crazyflie_ros_pkg 
cd uned_crazyflie_ros_pkg
git submodules init
git submodules update
cd ..
colcon build
echo "source install/setup.bash" >> ~/.bashrc
```

## Uso 🔧
### Simulador
Las simulaciones se hacen sobre Gazebo. El modelo sdf está operativo en el paquete uned_crazyflie_config pero los plugins para los actuadores y sensores aún no se han actualizado. 
#### Exclusivo en ROS
TO-DO

#### Controlador en Matlab
TO-DO

### Hardware-in-the-Loop
TO-DO: Micro-ROS

## Autores ✒️
* **[Francisco José Mañas Álvarez](https://github.com/FranciscoJManasAlvarez)** :envelope: fjmanas@dia.uned.es

## Publicaciones asociadas :paperclip:
- F.J. Mañas-Álvarez, M. Guinaldo, R. Dormido, R. Socas, S. Dormido, "Control basado en eventos mediante umbral relativo aplicado alcontrol de altitud de cuadric ́opteros Crazyflie 2.1", presentado en 42º Jornadas de Automática, Castellón, España, 2021
