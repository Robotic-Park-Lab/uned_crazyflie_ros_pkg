# uned_crazyflie_ros_pkg

> 📖 Para entender las ramas de este repositorio y su guía de contribución, consulta la rama [`doc`](https://github.com/Robotic-Park-Lab/uned_crazyflie_ros_pkg/tree/doc).

Paquetes ROS 2 y archivos de configuración para teleoperar y simular el nano-dron Crazyflie 2.1 en ROS 2, Webots y Matlab. El objetivo es una herramienta Hardware-in-the-Loop fácil de escalar y mantener, utilizable de forma independiente por cualquiera que solo quiera trabajar con Crazyflies y ROS 2 — no requiere el resto de la instalación de [Robotic Park Lab](https://robotic-park-lab.github.io), solo las dependencias externas listadas abajo.

#### Estructura
- **doc**. Un archivo `.tex` que detalla el repositorio: diagramas ROS, bibliografía, enlaces de interés, etc.
- **scripts**. Archivos auxiliares que no forman parte de ningún paquete ROS: post-procesado de `ros2 bag` (conversión a CSV, gráficas) y modelos/scripts de identificación de Matlab/Simulink. Ver [scripts/README_es.md](scripts/README_es.md).
- **[uned_crazyflie_config](uned_crazyflie_config/README_es.md)**. Paquete ROS 2. Configuración del entorno: mensajes personalizados, modelos 3D, el archivo de lanzamiento unificado `experience.launch.py` y recursos de RViz/RQT.
- **[uned_crazyflie_controllers](uned_crazyflie_controllers/README_es.md)**. Paquete ROS 2. Nodos de control para distintas arquitecturas de control: PID periódico (posición y actitud/velocidad angular), PID basado en eventos y Control Predictivo Generalizado (GPC) — pensado como base docente para que el trabajo de los estudiantes añada nuevas técnicas.
- **[uned_crazyflie_driver](uned_crazyflie_driver/README_es.md)**. Paquete ROS 2. Dos puntos de entrada de nodo: `swarm_driver` (Crazyflies físicos, vía `cflib`) y `webots_driver` (Crazyflies virtuales, cargado por Webots como un plugin controlador) — consulta el README de ese paquete para conocer el estado actual de la unificación del código compartido entre ambos. Anteriormente existían un paquete independiente `uned_crazyflie_webots` y otro paquete separado `uned_crazyflie_common`; ninguno de los dos merecía mantenerse aparte, así que ambos se absorbieron aquí.
- **[uned_crazyflie_gui](uned_crazyflie_gui/README_es.md)**. Paquete ROS 2. Interfaz gráfica PyQt para manejar un único robot, más una perspectiva RQT genérica y un archivo de RViz.
- **[uned_crazyflie_missions](uned_crazyflie_missions/README_es.md)**. Paquete ROS 2 (antes `uned_crazyflie_task`). Nodos de misión de alto nivel que actúan únicamente sobre topics, independientes de cualquier driver de robot: `formation` (se suscribe a las poses de los robots, publica `goal_pose`), `waypoints` (guía a un robot a través de una secuencia de puntos leída de un archivo de configuración) y `sequencer` (un publicador genérico de comandos por topic, guionizado).

## Instalación :book:

El objetivo es [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) en **Ubuntu 22.04**. También se admite una instalación en Windows 10 para ejecutar el sistema en el PC que aloja el sistema de posicionamiento [Vicon](https://www.vicon.com/) del laboratorio — el soporte de Windows se limita a los paquetes ROS 2 que no necesitan Webots (`uned_crazyflie_driver`, `uned_crazyflie_controllers`, `uned_crazyflie_gui`), ya que la simulación en Webots solo se ejecuta en Ubuntu en este laboratorio.

### Requisitos previos 📋

##### ROS 2
Instala primero ROS 2 Humble para tu sistema operativo, siguiendo la documentación oficial ([Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) / [Windows](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)).

##### Webots (solo Ubuntu, necesario para el `webots_driver` de `uned_crazyflie_driver`)
```
sudo apt install ros-humble-webots-ros2-driver
```
Esto instala [Webots](https://cyberbotics.com/) como dependencia. Ver [`uned_crazyflie_driver/README_es.md`](uned_crazyflie_driver/README_es.md) para el driver simulado y lo que necesita.

##### Teleoperación (opcional, para un joystick)
```
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
```
Ver [Teleoperación con joystick](#teleoperación-con-joystick-) más abajo.

##### Matlab
PENDIENTE — todavía no se ha fijado una versión concreta de Matlab/Simulink ni una lista de toolboxes para `scripts/Matlab/`. Ver [scripts/README_es.md](scripts/README_es.md) para saber qué hace cada script; verifica manualmente los que vayas a usar.

##### Dependencias de otros repositorios del laboratorio
No se declaran con una clave de rosdep (no proceden de un índice público de rosdep) — clónalos y compílalos en el mismo workspace, junto a este repositorio:
- **Crazyflie Python library**: [Robotic-Park-Lab/crazyflie-lib-python](https://github.com/Robotic-Park-Lab/crazyflie-lib-python) (rama `master`), el fork del laboratorio de `cflib`, requerido por `uned_crazyflie_driver`.
- **`multi_agent_pkg`**: de [Robotic-Park-Lab/RoboticPark](https://github.com/Robotic-Park-Lab/RoboticPark), requerido por `uned_crazyflie_driver` para las matemáticas de formación multi-agente (multiplicadores de Lagrange para geometrías de esfera/cono/elipsoide).
- **`vicon_receiver`** (opcional, solo si dispones realmente de hardware Vicon): de [Robotic-Park-Lab/ros2-vicon-receiver](https://github.com/Robotic-Park-Lab/ros2-vicon-receiver).

##### Dependencias de Python
Se instalan automáticamente vía `rosdep` (ver abajo), pero se listan aquí como referencia: `numpy`, `PyYAML`, `matplotlib`, `PyQt5`.

### Compilar el workspace

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone -b humble-dev https://github.com/Robotic-Park-Lab/uned_crazyflie_ros_pkg.git
git clone -b master https://github.com/Robotic-Park-Lab/crazyflie-lib-python.git
git clone -b humble-dev https://github.com/Robotic-Park-Lab/RoboticPark.git   # para multi_agent_pkg
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

En Windows, usa `md \dev_ws\src` / `cd \dev_ws\src` en su lugar, y `colcon build --merge-install` (las instalaciones por symlink no se soportan igual en Windows).

## Uso 🔧

### Lanzar una experiencia

Tras la reestructuración modular (ver `AUDIT.md` en la rama `doc`), existe un **único archivo de lanzamiento parametrizado**, `uned_crazyflie_config/launch/experience.launch.py`, en lugar de un `.launch.py` por demo. Cada experiencia es un archivo `.yaml` en `uned_crazyflie_config/resources/`:

```
ros2 launch uned_crazyflie_config experience.launch.py config_file:=<experiencia>.yaml
```

| `config_file` | Descripción |
|---|---|
| `demo_individual_teleop_webots.yaml` | 1 Crazyflie virtual en Webots, teleoperado, con la interfaz genérica RQT + RViz. |
| `demo_individual_teleop_vicon.yaml` | 1 Crazyflie virtual guiado con datos reales de posicionamiento Vicon, misma interfaz. |
| `demo_individual_waypoints_webots.yaml` | 1 Crazyflie virtual en Webots ejecutando los nodos de misión `sequencer` + `waypoints` sobre una ruta configurada. |

Se irán añadiendo más experiencias a medida que se preparen (Francisco lo hace de forma incremental) — cada una es simplemente un nuevo `.yaml` en `resources/`, siguiendo el esquema documentado al principio de `experience.launch.py` (`Operation` / `Robots` / `Interface` / `Data_Logging` / `Missions`), sin necesidad de un nuevo archivo de lanzamiento. Los antiguos `.launch.py` por demo y el paquete independiente `uned_crazyflie_webots` ya se han eliminado.

### Teleoperación con joystick 🎮

[`teleop_twist_joy`](https://github.com/ros2/teleop_twist_joy) publica un `geometry_msgs/Twist` en `/cmd_vel` a partir de un joystick. El `webots_driver` de `uned_crazyflie_driver` ya está suscrito a `<robot_id>/cmd_vel`, así que solo hace falta un remapeo para un Crazyflie **simulado**:

```
ros2 run joy joy_node
ros2 run teleop_twist_joy teleop_node --ros-args -r cmd_vel:=/dron01/cmd_vel
```

Ejecuta esto junto a un `experience.launch.py` que tenga `dron01` como robot `virtual`/`digital_twin`. Ajusta los parámetros propios de `teleop_twist_joy` (mapeo de ejes, escala) para tu mando — ver su [documentación](https://index.ros.org/p/teleop_twist_joy/).

**Limitación conocida**: el driver de hardware físico (`Crazyflie_ROS2` en `uned_crazyflie_driver/crazyflie_agent.py`) **no** está suscrito hoy a un topic `cmd_vel` — la teleoperación por joystick solo funciona ahora mismo contra un Crazyflie simulado en Webots, no contra hardware real. Conectarlo para drones físicos (vía los setpoints offboard tipo `Twist` de `cflib`) es trabajo abierto, aún sin hacer.

### Órdenes al Crazyflie

Independientemente de la misión/tarea en ejecución, el despegue y el aterrizaje se ordenan con un `std_msgs/String` en `<robot_id>/order`:
```
ros2 topic pub /dron01/order std_msgs/String "{data: 'take_off'}"
ros2 topic pub /dron01/order std_msgs/String "{data: 'land'}"
```

### Variables de log del firmware

https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#pm

![Alt text](doc/figs/rosgraph_ROS2.png?raw=true "rqt_graph")

> Esta captura de `rqt_graph` es anterior a la reestructuración modular (los nombres de topics/nodos han cambiado desde entonces) — se mantiene como ilustración hasta que se capture una nueva.

### Controlador Matlab
PENDIENTE

### Hardware-in-the-Loop
PENDIENTE

## Autores ✒️
* **[Francisco José Mañas Álvarez](https://github.com/FranciscoJManasAlvarez)** :envelope: fjmanas@dia.uned.es

## Publicaciones relacionadas :paperclip:
- Mañas-Álvarez, F.J., Guinaldo, M., Dormido, R., Socas, R., Dormido, S. Control basado en eventos mediante umbral relativo aplicado al control de altitud de cuadricópteros Crazyflie 2.1. In XLII Jornadas de Automática: libro de actas. Castelló, September 1-3, 2021 (pp. 341-348). Chapter DOI: https://doi.org/10.17979/spudc.9788497498043.341 Book DOI: https://doi.org/10.17979/spudc.9788497498043
