# uned_crazyflie_config

Paquete `ament_cmake` con todos los elementos de configuración compartidos por el resto de paquetes de este repo: mensajes propios, modelos 3D, ficheros de lanzamiento, y configuración de herramientas (RViz, RQT).

## Estructura

- **`msg/`**: mensajes ROS 2 propios.
  - `Pidcontroller.msg`: parámetros de un controlador PID por eje (`id`, `kp`/`ki`/`kd`/`td`/`nd`, `co`/`ai` para el modo basado en eventos, límites).
  - `StateEstimate.msg`: estado estimado por el Crazyflie (posición, actitud, empuje).
  - `Cmdsignal.msg`: señal de mando de bajo nivel (`thrust`, `roll`, `pitch`, `yaw`, `vbat`).
  - `Triggering.msg`, `Actuators.msg`: disparo de eventos y actuadores (este último, heredado de `mav_msgs`, para compatibilidad con el resto del ecosistema MAV).
- **`model/`**: modelos 3D del Crazyflie (URDF/Xacro/SDF/mallas `.dae`), usados tanto en RViz como en Webots. `model/convert/` conserva los ficheros intermedios de la conversión Gazebo→Webots.
- **`launch/experience.launch.py`** (nuevo): launch único parametrizado por experiencia — una única `ros2 launch uned_crazyflie_config experience.launch.py config_file:=<experiencia>.yaml` en vez de un `.launch.py` por demo. Lee de un `.yaml` en `resources/` las secciones `Simulation`/`Robots`/`Interface`/`Data_Logging`/`Missions`; ver la cabecera del propio fichero para el esquema completo, y `resources/experience_swarm_teleop.yaml` (teleoperación, 2 drones virtuales) / `resources/experience_tsp_digital_twin.yaml` (1 dron en modo gemelo digital con el driver de firmware real, ejecutando la misión `tsp_waypoints`) como ejemplos reales, verificados ejecutando de verdad la lógica del launch (no solo leyéndola).
- **`launch/*.launch.py`** (los 14 anteriores): **pendientes de revisión manual por Francisco** — decidir, demo a demo, cuál se retira ya migrada a un `.yaml` de experiencia y cuál se mantiene. Ver `AUDIT.md` (rama `doc`), punto 8 de la Fase 2.
- **`resources/`**: un `.yaml` de configuración por demo (parámetros PID por defecto, geometría de formaciones, waypoints, geometría del Lighthouse V2 para Vicon...), más los `.yaml` de experiencia nuevos (`experience_*.yaml`) para `experience.launch.py`.
- **`rviz/`**: ficheros `.rviz` ligados a demos concretas (`demo_swarm_formation.rviz`, `demo_swarm_teleop.rviz`, `sphere.rviz`, `test.rviz`). Para un `.rviz` genérico que sirva para cualquier Crazyflie, ver `uned_crazyflie_gui/rviz/crazyflie.rviz`.
- **`rqt/`**, **`Webots_rviz.perspective`**: perspectivas RQT ligadas a demos concretas. Para una perspectiva RQT genérica, ver `uned_crazyflie_gui/rqt/crazyflie.perspective`.

## Dependencia con otros repos del laboratorio

Algunos `.launch.py` de este paquete referencian una IP fija (`10.196.92.136`, el PC del sistema Vicon) — pendiente a propósito de evaluar en el laboratorio real antes de parametrizarla (ver `AUDIT.md`).
