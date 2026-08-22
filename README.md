# uned_crazyflie_ros_pkg

> 📖 Para entender las ramas de este repo y la guía de contribución, consulta la rama [`doc`](https://github.com/Robotic-Park-Lab/uned_crazyflie_ros_pkg/tree/doc).
Repositorio con los paquetes de ROS2 y ficheros de configuración para la teleoperación y simulación del dron crazyflie 2.1 en ROS2, Gazebo y Matlab. La finalidad es obtener una herramienta Hardware-in-the-Loop que sea facilmente escalable y mantenible.

#### Estructura 
- **doc**. Contiene un fichero _.tex_ que aborda más en detalle toda la información relacionada con el repositorio: esquemas de ROS, búsquedas bibliográficas, enlaces de interés, etc.
- **scripts**. Contiene aquellos ficheros auxiliares que no forman parte de ningún paquete de ROS. Por ejemplo, ficheros _.sh_ para automatizar procesos repetitivos como la conversión de los ficheros _.bag_ a txt o los scripts de Matlab para representar datasets.
- **[uned_crazyflie_config](uned_crazyflie_config/README.md)**. Paquete de ROS2. Contiene aquellos elementos auxiliares para la configuración del entorno, así como los _.launch.py_ para la ejecución en bloque de las diferentes estructuras del sistema.
- **[uned_crazyflie_controllers](uned_crazyflie_controllers/README.md)**. Paquete de ROS2. Contiene los nodos de control en función de las diferentes arquitecturas de control: PID Periódico (posición y actitud/velocidad angular), PID Basado en Eventos y predictivo generalizado (GPC), pensado como base docente para que trabajos de alumnos añadan nuevas técnicas.
- **[uned_crazyflie_driver](uned_crazyflie_driver/README.md)**. Paquete de ROS2. Contiene los nodos para la comunicación con los crazyflies (reales y, vía módulos compartidos, virtuales en Webots) a través de la librería cflib: _swarm_driver_. También aloja el código Python compartido entre este paquete, _uned_crazyflie_webots_ y _uned_crazyflie_missions_ (controlador PID, aplicación de parámetros PID del Crazyflie) — antes en un paquete `uned_crazyflie_common` aparte, absorbido aquí por no compensar como paquete independiente.
- **[uned_crazyflie_gui](uned_crazyflie_gui/README.md)**. Paquete de ROS2. Contiene la interfaz gráfica PyQt para el manejo del robot individual, más una perspectiva RQT y un fichero RViz genéricos.
- **[uned_crazyflie_missions](uned_crazyflie_missions/README.md)**. Paquete de ROS2 (antes `uned_crazyflie_task`). Nodos de misión/tarea de alto nivel resolubles por uno o varios Crazyflie, indiferentes a si son físicos o virtuales: formaciones (`leader_follower`, `shape_based_formation_control`, `formation_control_webots`) y recorrido de waypoints tipo TSP (`tsp_waypoints`).
- **[uned_crazyflie_webots](uned_crazyflie_webots/README.md)**. Paquete de ROS2. Contiene los dos drivers virtuales del crazyflie 2.1 en Webots (controlador propio y firmware real/gemelo digital), más los mundos y modelos de simulación.

## Instalación :book:
El objetivo es implementar una versión del sistema en [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) y [Windows 10](https://www.microsoft.com/es-es/windows/features?activetab=NewPopular) para poder ejecutar todo el sistema sobre el PC (Windows) en el que está instalado el sistema de posicionamiento de [Vicon](TO-DO:enlace).

### Pre-requisitos 📋
##### ROS2
Lo primero debe ser tener instalada la correspondiente versión de ROS2 para el sistema operativo del dispositivo. Se recomienda seguir las instrucciones disponibles en la documentación oficial ([Humble](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)). 

##### Matlab
TO-DO.

##### Dependencias
- Crazyflie python library: [Robotic-Park-Lab branch](https://github.com/Robotic-Park-Lab/crazyflie-lib-python)


### - Windows 10
La configuración del entorno de trabajo para el paquete desarrollado se muestra a continuación.
```
md \dev_ws\src
cd \dev_ws\src
git clone -b humble-dev https://github.com/Robotic-Park-Lab/uned_crazyflie_ros_pkg.git
cd ..
colcon build --merge-install
```

### - Ubuntu 22.04
La configuración del entorno de trabajo para el paquete desarrollado se muestra a continuación.
```
cd \path_ws\src
git clone -b humble-dev https://github.com/Robotic-Park-Lab/uned_crazyflie_ros_pkg.git
cd ..
colcon build --symlink-install
```

## Uso 🔧
### Variables
https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#pm

#### ROS2
```
cd \dev_ws
colcon build --merge-install --packages-select uned_crazyflie_driver
ros2 run uned_crazyflie_driver crazyflie_driver
```
Actualmente (2021-09-28), solo está implementada la función de despegue y aterrizaje mediante un topic. Las instrucciones para ello son:
```
ros2 topic pub /cf_order std_msgs/String "{data: 'take_off'}"
```
```
ros2 topic pub /cf_order std_msgs/String "{data: 'land'}"
```

```
ros2 launch uned_crazyflie_config test.launch.py
```

![Alt text](doc/figs/rosgraph_ROS2.png?raw=true "rqt_graph")
#### Controlador en Matlab
TO-DO

### Hardware-in-the-Loop
TO-DO

## Autores ✒️
* **[Francisco José Mañas Álvarez](https://github.com/FranciscoJManasAlvarez)** :envelope: fjmanas@dia.uned.es

## Publicaciones asociadas :paperclip:
- Mañas-Álvarez, F.J., Guinaldo, M., Dormido, R., Socas, R., Dormido, S. Control basado en eventos mediante umbral relativo aplicado al control de altitud de cuadricópteros Crazyflie 2.1. En XLII Jornadas de Automática: libro de actas. Castelló, 1-3 de septiembre de 2021 (pp. 341-348). DOI capítulo: https://doi.org/10.17979/spudc.9788497498043.341 DOI libro: https://doi.org/10.17979/spudc.9788497498043
