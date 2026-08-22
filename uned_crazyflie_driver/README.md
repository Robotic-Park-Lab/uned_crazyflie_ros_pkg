# uned_crazyflie_driver

Paquete `ament_python` con el driver de comunicación con Crazyflies **físicos** vía [`cflib`](https://github.com/Robotic-Park-Lab/crazyflie-lib-python) (fork del laboratorio), y con la base de código común que también usa `uned_crazyflie_webots` para el driver **virtual** (Webots).

## Estructura

- **`swarm_driver.py`** (entry point `swarm_driver`, único nodo del paquete): `CFSwarmDriver`, gestiona un swarm de Crazyflies físicos de tamaño 1 o mayor. Instancia una `Crazyflie_ROS2` por dron.
- **`crazyflie_agent.py`**: clase `Crazyflie_ROS2`, el driver real de cada Crazyflie individual. Acepta hardware físico (`scf=`, vía `cflib`) **o** Webots (`webots_node=`) — es la misma clase la que hoy orquesta el modo físico desde `swarm_driver.py`; el modo Webots lo usa `uned_crazyflie_webots` a través de la base común de abajo. Contempla `config['type'] == 'digital_twin'` para el caso de gemelo digital.
- **`webots_bootstrap.py`**: inicialización de motores/sensores Webots y los 12 `PIDController` en cascada (posición/velocidad/actitud/velocidad angular) — extraído porque era **idéntico byte a byte** entre `Crazyflie_ROS2.virtualCrazyflie()` y el driver de `uned_crazyflie_webots`. Es la base común real de los dos drivers Webots del repo (`uned_crazyflie_webots/crazyflie_driver.py` y `crazyflie_driver_firmware.py`).
- **`agent.py`** / **`cmd_motion.py`**: clases `Agent` (seguimiento de vecinos en formación, con RViz y `high_level_commander`) y `CMD_Motion` (comando de movimiento hacia una pose objetivo), usadas por `crazyflie_agent.py`. **No** son las mismas clases que sus homónimas en `uned_crazyflie_task` — ahí los comandos de vuelo y el seguimiento son deliberadamente más simples, ver los comentarios en cada fichero.
- **`pid_controller.py`** / **`pid_params.py`**: `PIDController` y `apply_controller_params()` (aplica un mensaje `Pidcontroller` a un controlador según el eje). Compartidos por `crazyflie_agent.py`, `swarm_driver.py`, `uned_crazyflie_task` y `uned_crazyflie_webots`.
- **`crazyflie_ros2_test.py`**: `Crazyflie_ROS2_TEST`, código que parece no usarse en ningún sitio del repo (ver `AUDIT.md`) — movido aquí en vez de borrado, pendiente de que Francisco decida.

## Dependencias de otros repos del laboratorio

`multi_agent_pkg` (de `RoboticPark`) y `cflib` (sin clave rosdep, instalar aparte — ver el fork del laboratorio enlazado arriba) deben estar ya compilados/instalados en el mismo workspace.
