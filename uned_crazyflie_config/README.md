# uned_crazyflie_config

Paquete `ament_cmake` con todos los elementos de configuración compartidos por el resto de paquetes de este repo: mensajes propios, modelos 3D, ficheros de lanzamiento, y configuración de herramientas (RViz, RQT).

## Estructura

- **`msg/`**: mensajes ROS 2 propios.
  - `Pidcontroller.msg`: parámetros de un controlador PID por eje (`id`, `kp`/`ki`/`kd`/`td`/`nd`, `co`/`ai` para el modo basado en eventos, límites).
  - `StateEstimate.msg`: estado estimado por el Crazyflie (posición, actitud, empuje).
  - `Cmdsignal.msg`: señal de mando de bajo nivel (`thrust`, `roll`, `pitch`, `yaw`, `vbat`).
  - `Triggering.msg`, `Actuators.msg`: disparo de eventos y actuadores (este último, heredado de `mav_msgs`, para compatibilidad con el resto del ecosistema MAV).
- **`model/`**: modelos 3D del Crazyflie (URDF/Xacro/SDF/mallas `.dae`), usados tanto en RViz como en Webots. `model/convert/` conserva los ficheros intermedios de la conversión Gazebo→Webots.
- **`launch/`**: 14 ficheros `.launch.py`, uno por combinación de demo (teleoperación, formación por distancia/posición/esfera, con/sin Vicon, número de drones). **Serán sustituidos por un único launch parametrizado por experiencia** — ver el punto 8 de la Fase 2 en `AUDIT.md` (rama `doc`).
- **`resources/`**: un `.yaml` de configuración por demo (parámetros PID por defecto, geometría de formaciones, waypoints, geometría del Lighthouse V2 para Vicon...).
- **`rviz/`**: ficheros `.rviz` ligados a demos concretas (`demo_swarm_formation.rviz`, `demo_swarm_teleop.rviz`, `sphere.rviz`, `test.rviz`). Para un `.rviz` genérico que sirva para cualquier Crazyflie, ver `uned_crazyflie_gui/rviz/crazyflie.rviz`.
- **`rqt/`**, **`Webots_rviz.perspective`**: perspectivas RQT ligadas a demos concretas. Para una perspectiva RQT genérica, ver `uned_crazyflie_gui/rqt/crazyflie.perspective`.

## Dependencia con otros repos del laboratorio

Algunos `.launch.py` de este paquete referencian una IP fija (`10.196.92.136`, el PC del sistema Vicon) — pendiente a propósito de evaluar en el laboratorio real antes de parametrizarla (ver `AUDIT.md`).
