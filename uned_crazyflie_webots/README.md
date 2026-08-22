# uned_crazyflie_webots

Paquete `ament_python` con los drivers **virtuales** del Crazyflie 2.1 en [Webots](https://cyberbotics.com/), más los mundos/mallas de simulación.

## Los dos drivers

`webots_ros2_driver` **no** carga estos drivers con `ros2 run`: los instancia en tiempo de simulación por ruta de clase, desde la etiqueta `<plugin type="...">` del URDF del robot (`resources/crazyflie.urdf` / `crazyflie_firmware.urdf`). Por eso este paquete no tiene `console_scripts` en `setup.py`.

- **`crazyflie_driver.py`** (`CrazyflieWebotsDriver`, cargado desde `resources/crazyflie.urdf` y `crazyflie_IPC.urdf`): controlador geométrico propio en Python — formaciones por distancia/pose, con variantes ML1/ML2/ML3 y geometrías esfera/cono/elipsoide. Es el driver activo en las 6 demos de `launch/`.
- **`crazyflie_driver_firmware.py`** (`CrazyflieWebotsDriver`, cargado desde `resources/crazyflie_firmware.urdf`, nuevo): usa `cffirmware`, el firmware real de Bitcraze compilado para software-in-the-loop — la variante de **gemelo digital**: mismo código de control que vuela en el hardware real, en vez de una reimplementación en Python. Requiere `cffirmware` compilado aparte (`make bindings_python` en [`crazyflie-firmware`](https://github.com/Robotic-Park-Lab/crazyflie-firmware)) y la variable de entorno `CRAZYFLIE_FIRMWARE_PATH` — no está enlazado todavía desde ningún `.launch.py` de demo, solo desde su URDF.

Ambos comparten con `uned_crazyflie_driver` la inicialización de motores/sensores y los 12 `PIDController` en cascada (`uned_crazyflie_driver.webots_bootstrap`), el seguimiento de vecinos (`uned_crazyflie_driver.agent.Agent`) y `PIDController` (`uned_crazyflie_driver.pid_controller`). La lógica de control de formaciones en sí **no** está unificada con `Crazyflie_ROS2` (`uned_crazyflie_driver/crazyflie_agent.py`, driver del Crazyflie físico) — son algoritmos de gradiente reales y divergentes entre hardware y simulación; ver `AUDIT.md` (rama `doc`) para el detalle de por qué se dejaron separados.

## Estructura

- **`resources/`**: URDF/modelos del robot para Webots.
- **`worlds/`**: mundos `.wbt` de simulación (1 a 4 Crazyflies, formación esférica) y sus mallas.
- **`controllers/`**: controladores C/Python nativos de Webots (plantillas del propio simulador/Bitcraze), excluidos del lint por ser código generado.
- **`launch/`**: 6 `.launch.py` de demo, parcialmente duplicados con los de `uned_crazyflie_config` — se consolidarán con el launch único de la tarea 8 (ver `AUDIT.md`).

## Dependencias de otros repos del laboratorio

`multi_agent_pkg` (de `RoboticPark`) debe estar ya compilado en el mismo workspace.
