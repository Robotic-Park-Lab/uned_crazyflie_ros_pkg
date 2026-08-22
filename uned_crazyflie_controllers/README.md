# uned_crazyflie_controllers

Paquete `ament_cmake` (C++) con controladores de vuelo del Crazyflie 2.1, pensado como base docente: cada controlador es un nodo `rclcpp::Node` independiente, con la idea de que trabajos de alumnos añadan nuevas técnicas siguiendo el mismo patrón.

## Controladores incluidos

| Ejecutable | Clase / cabecera | Arquitectura |
|---|---|---|
| `periodic_pid_position_controller` | `PositionController` / `CrazyfliePositionController.hpp` | PID periódico de posición (X/Y/Z) |
| `eventbased_pid_position_controller` | `PositionController` / `CrazyfliePositionController.hpp` | PID de posición basado en eventos (umbral relativo, ver la publicación de Mañas-Álvarez et al. citada en el `README.md` de la raíz) |
| `periodic_pid_attituderate_controller` | `AttitudeRateController` / `CrazyflieAttitudeRateController.hpp` | PID periódico de actitud (roll/pitch/yaw) y velocidad angular |
| `generalized_predictive_controller` | (misma base que `PositionController`) / `CrazyfliePositionController.hpp` | Control predictivo generalizado (GPC) de posición |

Cada controlador lee sus ganancias por parámetro ROS (`ZKp`/`ZKi`/`ZKd`/..., `PitchKp`/..., según el eje/controlador) y publica/suscribe sobre los topics y mensajes propios de `uned_crazyflie_config` (`Pidcontroller`, `StateEstimate`).

## Añadir una nueva técnica de control

1. Nueva cabecera en `include/uned_crazyflie_controllers/Crazyflie<Nombre>Controller.hpp`, heredando de `rclcpp::Node` como las existentes.
2. Nueva fuente en `src/<nombre>_controller.cpp` implementando `initialize()`/`iterate()` (o el patrón equivalente que uses).
3. Registrar el ejecutable en `CMakeLists.txt` (`add_executable` + `ament_target_dependencies` + añadirlo a `install(TARGETS ...)`).

## Dependencias

`rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `Eigen3` (vía `eigen3_cmake_module`), y los mensajes propios de `uned_crazyflie_config`.
