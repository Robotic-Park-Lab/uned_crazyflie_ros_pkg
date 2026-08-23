# uned_crazyflie_controllers

Paquete `ament_cmake` (C++) con los controladores de vuelo del Crazyflie 2.1, diseñado como base docente: cada controlador es un `rclcpp::Node` independiente, pensado para que el trabajo de los estudiantes añada nuevas técnicas siguiendo el mismo patrón.

## Controladores incluidos

| Ejecutable | Clase / cabecera | Arquitectura |
|---|---|---|
| `periodic_pid_position_controller` | `PositionController` / `CrazyfliePositionController.hpp` | Control de posición PID periódico (X/Y/Z) |
| `eventbased_pid_position_controller` | `PositionController` / `CrazyfliePositionController.hpp` | Control de posición PID basado en eventos (umbral relativo, ver la publicación de Mañas-Álvarez et al. citada en el `README.md` raíz) |
| `periodic_pid_attituderate_controller` | `AttitudeRateController` / `CrazyflieAttitudeRateController.hpp` | Control PID periódico de actitud (roll/pitch/yaw) y velocidad angular |
| `generalized_predictive_controller` | (misma base que `PositionController`) / `CrazyfliePositionController.hpp` | Control Predictivo Generalizado (GPC) de posición |

Cada controlador lee sus ganancias de parámetros ROS (`ZKp`/`ZKi`/`ZKd`/..., `PitchKp`/..., según el eje/controlador) y publica/se suscribe en los topics y mensajes personalizados de `uned_crazyflie_config` (`Pidcontroller`, `StateEstimate`).

Nota: `periodic_pid_position_controller` y `eventbased_pid_position_controller` compilan ambos la *misma* clase `PositionController` declarada en `CrazyfliePositionController.hpp`, pero cada archivo `.cpp` aporta su propia definición de los métodos de esa clase (`pid_controller()`, `initialize()`, ...) y se compila en su propio ejecutable independiente — nunca se enlazan juntos. `generalized_predictive_controller` también incluye esa cabecera pero no define/usa `pid_controller()` en absoluto; resuelve el lazo de posición mediante sus propias matrices de espacio de estados del GPC.

## Tests

Hasta ahora este paquete solo tenía los tests de lint estándar de ROS 2 (`ament_lint_auto`/`ament_lint_common`) — ninguna cobertura funcional de las matemáticas del PID en sí. Se añadieron tres targets `ament_add_gtest` bajo `test/`, cada uno ejerciendo el `pid_controller()`/`init_controller()` *real, compilado* de un archivo `.cpp` concreto — nada se reimplementa ni se duplica en los tests:

- `test_periodic_pid_position_controller.cpp` — compila y enlaza directamente `src/periodic_pid_position_controller.cpp`. Verifica la copia de ganancias en `init_controller()`, que el término proporcional coincide con `Kp * error` en la primera llamada, que el término integral se acumula correctamente entre llamadas, que la salida satura a `[lowerlimit, upperlimit]`, y — una peculiaridad real y verificada de este archivo concreto — que la saturación se omite por completo cuando `upperlimit == 0.0` (`if (controller.upperlimit != 0.0)` en el código fuente).
- `test_eventbased_pid_position_controller.cpp` — misma clase `PositionController`, pero enlaza `src/eventbased_pid_position_controller.cpp`, cuyo `pid_controller()` es una definición compilada *distinta*: siempre pone a `true` el miembro `events` en cada llamada, verificado explícitamente.
- `test_attituderate_controller.cpp` — enlaza `src/periodic_pid_attituderate_controller.cpp` (clase `AttitudeRateController`). Verifica dos diferencias reales, ya verificadas, respecto a los controladores de posición: la saturación aquí es incondicional (satura incluso cuando `upperlimit == 0.0`, a diferencia de los dos archivos anteriores), y aplica una corrección anti-windup al término integral siempre que la salida satura (`integral -= (out - out_i) * sqrt(kp / ki)`) — el test calcula a mano la integral corregida esperada y comprueba que coincide exactamente.

### Cómo se hizo testeable sin cambiar el comportamiento

`pid_controller()`/`init_controller()` son métodos privados de `PositionController`/`AttitudeRateController`, y cada archivo `.cpp` también define su propio `int main()`, que colisionaría con el `main()` propio de gtest si se enlazara en un binario de test. Dos cambios pequeños y aditivos hicieron testeable de forma aislada la lógica PID *existente, sin modificar*:

1. El `main()` de cada archivo ahora está envuelto en `#ifndef UNED_CRAZYFLIE_CONTROLLERS_TEST_BUILD` / `#endif`. Las compilaciones normales no se ven afectadas (la macro no está definida); solo los targets de test la definen, vía `target_compile_definitions`, para excluir `main()` y enlazar el resto del archivo en el binario de gtest.
2. Cada cabecera añade una única declaración `friend class <Name>Test;` para el fixture de gtest correspondiente, de modo que los tests pueden llamar directamente a los métodos privados — no se ha cambiado la visibilidad de ningún método para los ejecutables reales de los nodos.

No se ha alterado ninguna matemática del PID, lógica de saturación ni comportamiento anti-windup en esta pasada; solo se ha añadido infraestructura de test.

### Lo que deliberadamente no está cubierto

`initialize()`/`iterate()` (lectura de parámetros, cableado de publicadores/suscriptores, el bucle de spin de ROS) y la lógica de espacio de estados del GPC de `generalized_predictive_controller` no se testean unitariamente aquí — necesitan un grafo de nodos en ejecución o fixtures de espacio de estados no triviales para ejercerse de forma significativa, y forzar un test superficial en torno a ellos no verificaría nada real. `eval_threshold()`/`init_triggering()` (la lógica de umbral/ruido basada en eventos en `CrazyfliePositionController.hpp`) tampoco están testeadas, por la misma razón: no son lo bastante puras como para testearlas de forma económica sin un esfuerzo mayor que el de esta pasada.

## Añadir una nueva técnica de control

1. Nueva cabecera en `include/uned_crazyflie_controllers/Crazyflie<Nombre>Controller.hpp`, heredando de `rclcpp::Node` como las existentes.
2. Nueva fuente en `src/<nombre>_controller.cpp` implementando `initialize()`/`iterate()` (o un patrón equivalente).
3. Registrar el ejecutable en `CMakeLists.txt` (`add_executable` + `ament_target_dependencies` + añadirlo a `install(TARGETS ...)`).
4. Si el nuevo controlador tiene lógica pura similar a un PID que merezca ser testeada, sigue el patrón anterior: protege `main()` con `#ifndef`, añade un fixture `friend class` de test y un nuevo target `ament_add_gtest`.

## Dependencias

`rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `Eigen3` (vía `eigen3_cmake_module`), y los mensajes personalizados de `uned_crazyflie_config`. Los tests requieren además `ament_cmake_gtest`.
