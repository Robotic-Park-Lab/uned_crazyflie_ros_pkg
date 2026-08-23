# uned_crazyflie_config

Paquete `ament_cmake` con todos los elementos de configuración compartidos por el resto de paquetes de este repositorio: mensajes personalizados, modelos 3D, archivos de lanzamiento y configuración de herramientas (RViz, RQT).

## Estructura

- **`msg/`**: mensajes ROS 2 personalizados.
  - `Pidcontroller.msg`: parámetros del controlador PID por eje (`id`, `kp`/`ki`/`kd`/`td`/`nd`, `co`/`ai` para el modo basado en eventos, límites).
  - `StateEstimate.msg`: estado estimado del Crazyflie (posición, actitud, empuje).
  - `Cmdsignal.msg`: señal de comando de bajo nivel (`thrust`, `roll`, `pitch`, `yaw`, `vbat`).
  - `Triggering.msg`, `Actuators.msg`: disparo de eventos y actuadores (este último heredado de `mav_msgs`, para compatibilidad con el resto del ecosistema MAV).
- **`model/`**: modelos 3D del Crazyflie (URDF/mallas), usados por RViz.
- **`resources/crazyflie.urdf`**: la descripción del robot para Webots de un Crazyflie virtual, que carga el plugin `webots_driver` de `uned_crazyflie_driver`. Los placeholders (`CameraAlwayOn`/`CameraEnable`/`CameraUpdateRate`, `name_id_value`, `config_file_path`) los sustituye `experience.launch.py` en el momento del lanzamiento.
- **`worlds/`**: los mundos de simulación de Webots (`RoboticPark_N01.wbt`...`N05.wbt`) referenciados por el campo `Operation.world` de los archivos `.yaml` de experiencia.
- **`launch/experience.launch.py`**: único archivo de lanzamiento parametrizado por experiencia — un `ros2 launch uned_crazyflie_config experience.launch.py config_file:=<experiencia>.yaml` en lugar de un `.launch.py` por demo. Lee las secciones `Operation`/`Robots`/`Interface`/`Data_Logging`/`Missions` de un `.yaml` en `resources/`; ver la cabecera del propio archivo para el esquema completo. Es ahora el **único** archivo de lanzamiento del repositorio — los antiguos `.launch.py` por demo, y el paquete independiente `uned_crazyflie_webots` en el que vivían, se han eliminado. Francisco va añadiendo más archivos `.yaml` de experiencia de ejemplo de forma incremental.
- **`resources/`**: los archivos `.yaml` de experiencia actuales (`demo_individual_teleop_webots.yaml`, `demo_individual_teleop_vicon.yaml`, `demo_individual_waypoints_webots.yaml`) más los archivos de configuración que referencian (`demo_individual_waypoints.yaml`, `demo_individual_waypoints_topics.yaml` para `uned_crazyflie_missions`; `crazyflie_parameters.yaml`, `crazyflie_distances.yaml`, `LightHouseV2_Geometry.yaml`).
- **`rviz/demo_individual.rviz`**: la configuración de RViz usada por los archivos de experiencia actuales (`rviz/demo_formation_N20.rviz` es un resto de una demo antigua, pendiente de revisión). Para un `.rviz` genérico que funcione con cualquier Crazyflie, ver `uned_crazyflie_gui/rviz/crazyflie.rviz`.
- **`rqt/`**: perspectivas RQT de demos antiguas (`Robotic Park Lab.perspective`, `Swarm_teleop_one.perspective`), no referenciadas por ningún archivo `.yaml` de experiencia actual — pendiente de revisión. Para una perspectiva RQT genérica, ver `uned_crazyflie_gui/rqt/crazyflie.perspective`.

## Tests

Este paquete no tiene nodos propios (solo mensajes, recursos y archivos de lanzamiento), así que no hay mucho que testear unitariamente más allá de lo que `rosidl` ya comprueba en tiempo de compilación (fallo de build si un `.msg` está mal formado). Lo único que realmente necesitaba un test de verdad era `experience.launch.py`, ya que contiene lógica de ramificación real (qué nodos lanzar según `Operation`/`Robots`/`Interface`/`Data_Logging`/`Missions`) que un fallo de compilación nunca detectaría:

- `test/test_experience_launch.py` (vía `ament_cmake_pytest`): carga el `experience.launch.py` **instalado** y ejecuta su `get_ros2_nodes()` con un `LaunchContext` real contra los 3 archivos de experiencia actuales, comprobando que las acciones que genera son las correctas — por ejemplo, que `demo_individual_teleop_webots.yaml` añade un `WebotsLauncher` + un `WebotsController` + `rviz2` y ningún nodo de `uned_crazyflie_driver` (no hay robot físico en ella), que `demo_individual_teleop_vicon.yaml` añade en su lugar el nodo `vicon_receiver`, y que `demo_individual_waypoints_webots.yaml` lanza los dos nodos de misión de `uned_crazyflie_missions` que configura. También comprueba que los recursos de los que depende (la perspectiva RQT y el archivo RViz genéricos de `uned_crazyflie_gui`, los propios `.yaml` de ejemplo y `crazyflie.urdf`) realmente se instalan donde el archivo de lanzamiento los espera.
