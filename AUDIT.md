# Auditoría — uned_crazyflie_ros_pkg (2026-08-21)

Checklist de hallazgos de la auditoría de `humble-dev`, de lo más simple a lo más complejo. Verificado compilando y ejecutando de verdad (no solo leyendo código): `colcon build` de los 6 paquetes, `colcon test` de los 4 `ament_python`, y ejecución real de `uned_crazyflie_gui`'s entry points.

Marca con `[x]` conforme se resuelva cada punto. Añade aquí cualquier hallazgo nuevo que surja al trabajar en el repo.

## Simples

- [ ] **Entry point roto**: `uned_crazyflie_driver/setup.py` registra `positioning_system = uned_crazyflie_driver.positioning_system:main`, pero `positioning_system.py` no existe en el repo. `ros2 run uned_crazyflie_driver positioning_system` falla con `ModuleNotFoundError`.
- [ ] **Fichero suelto `sys`**: `uned_crazyflie_gui/uned_crazyflie_gui/sys` es un blob de 0 bytes sin propósito aparente (no es un directorio). Parece basura.
- [ ] **IP hardcodeada**: `10.196.92.136` (probablemente el PC del Vicon) repetida tal cual en 4 ficheros (`uned_crazyflie_config/launch/swarm_formation.launch.py`, `demo_swarm_formation_distance_two_vicon.launch.py`, `demo_swarm_teleop_vicon.launch.py`, `swarm_teleop_vicon.launch.py`) en vez de un argumento de lanzamiento.
- [ ] **Metadatos de maintainer inconsistentes**: mezcla de `Francisco José Mañas Álvarez` / `Francisco-Jose Manas Alvarez` (con `fma527@ual.es`, distinto del resto) / `kiko` entre los 6 `package.xml`. Unificar a `fjmanas@dia.uned.es`.
- [ ] **Licencia inconsistente**: el `LICENSE` real del repo es BSD-3-Clause, pero `uned_crazyflie_config/package.xml` dice `GPLv3` y los otros 5 dicen `TODO: License declaration`.
- [ ] **Descripciones sin rellenar**: `uned_crazyflie_controllers`, `uned_crazyflie_gui`, `uned_crazyflie_task`, `uned_crazyflie_webots` siguen en `TODO: Package description` (plantilla de `ros2 pkg create` nunca completada).
- [ ] **Marker de paquete mal nombrado**: `uned_crazyflie_webots/setup.py` registra el índice de recursos en `resources/` (plural) en vez de `resource/` (singular, la convención `ament_python`). `colcon build` avisa en cada compilación: *"doesn't explicitly install a marker in the package index"*.

## Medias

- [ ] **Dependencias no declaradas**: los 4 `package.xml` de tipo `ament_python` (`driver`, `gui`, `task`, `webots`) no tienen ningún `<depend>` — solo `test_depend` de lint — pese a importar de verdad `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `visualization_msgs`, `tf2_ros`, `tf_transformations`, `qt_gui`, `rqt_gui_py`, `rqt_plot`, `uned_crazyflie_config` (mensajes propios), y dependencias cruzadas con otros repos del laboratorio: `vicon_receiver` (de `ros2-vicon-receiver`) y `multi_agent_pkg` (de `RoboticPark`, sin documentar en ningún README que este repo depende de él). `uned_crazyflie_controllers` tampoco declara `eigen3_cmake_module` pese a usarlo en `CMakeLists.txt`. `cflib` (librería del Crazyflie) no tiene clave rosdep — queda correctamente solo como dependencia documentada en el README, no en `package.xml`.
- [ ] **Nombre engañoso**: `uned_crazyflie_config/srv/AddTwoInts.srv` es literalmente el ejemplo del tutorial oficial de ROS 2 (`int64 a`/`int64 b`), reutilizado de verdad en `leader_follower.py` y `shape_based_formation_control.py` pero con el campo de respuesta cambiado a `uint32 delay` — el nombre no dice nada de lo que hace hoy. Renombrarlo implica actualizar los 2 nodos que lo usan y regenerar el paquete de mensajes.
- [ ] **Recurso Qt sobredimensionado**: `uned_crazyflie_gui/uned_crazyflie_gui/logo_rc.py` pesa **1.95 MB** (recurso Qt compilado desde `logo.qrc`). Revisar si la imagen fuente se puede comprimir/reducir antes de recompilar el recurso.
- [ ] **Tests de lint en rojo**: 10 de 12 tests (`ament_copyright`/`ament_flake8`/`ament_pep257`) fallan hoy en `driver`/`gui`/`task`/`webots`. La mayoría de los errores de estilo están en `uned_crazyflie_webots/controllers/*.py` (scripts de controlador de Webots, semi-generados). Decidir si se corrigen a mano o se excluyen del lint como código generado (mismo patrón ya usado en `uned_interface_ros` para los ficheros de pyuic5/pyrcc5).

## Complejas

- [ ] **Código muerto y roto en `uned_crazyflie_gui`**: dos implementaciones paralelas del nodo de interfaz. `interface_node.py` es el entry point real (`interface_node`) pero es un stub que solo abre una ventana vacía ("Development in progress..."). `interface_gui.py` es más completo (matplotlib, `rqt_plot`) pero no está conectado a ningún entry point y además está roto (`from main_ui import *`, `from shell_cmd import ShellCmd` — imports absolutos que fallan de verdad, confirmado ejecutándolo: `ModuleNotFoundError: No module named 'main_ui'`). Decidir: ¿se arregla y se conecta `interface_gui.py` como el entry point real, se descarta, o se fusionan?
- [ ] **Duplicación fuerte**: `leader_follower.py` (694 líneas) y `shape_based_formation_control.py` (701 líneas) comparten ~70% del contenido línea a línea (mismo bloque de imports, misma estructura de nodo). Candidatos a extraer una clase/base común.
- [ ] **Ficheros de un único módulo muy grandes**, difíciles de revisar/mantener: `crazyflie_agent.py` (1602 líneas), `crazyflie_driver.py` de `uned_crazyflie_webots` (1546), `swarm_driver.py` (1413). Sin bug conocido, pero candidatos a dividir en módulos más pequeños si se van a tocar de todos modos.
