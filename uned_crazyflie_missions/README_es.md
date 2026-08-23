# uned_crazyflie_missions

Paquete `ament_python` con nodos de **misión de alto nivel** para uno o varios Crazyflies. Cada nodo aquí habla **solo por topics** — nunca directamente con `cflib` ni con Webots — así que el mismo nodo funciona sin cambios con un dron físico o virtual, y nunca necesita importar nada de `uned_crazyflie_driver`. Cada nodo se configura a partir de un `.yaml` (ver `uned_crazyflie_config/resources/`), no con valores fijos en el código ni un montón de parámetros ROS, así que una nueva experiencia solo necesita un nuevo archivo de configuración, no código nuevo.

Es un rediseño desde cero (2026-08-22): la versión anterior de este paquete (`leader_follower.py`, `shape_based_formation_control.py`, `formation_control_webots.py`, más sus clases de apoyo `agent.py`/`cmd_motion.py`) se conectaba **directamente** a `cflib.crazyflie.swarm.Swarm` — tres implementaciones de driver distintas, duplicadas y obsoletas, viviendo dentro de lo que se suponía que era un paquete independiente del driver, anteriores a la consolidación `swarm_driver`/`webots_driver` de `uned_crazyflie_driver`. Todo eso ha desaparecido; la ley de control de formación real de `shape_based_formation_control.py` se extrajo a `formation.py` (ver abajo), el resto era peso muerto.

## Nodos

- **`formation`** (`formation.py`): una instancia por robot. Se suscribe a su propia pose y a las de sus vecinos, publica su propia pose objetivo absoluta, usando una ley de consenso de distancia/offset (error medio de vecinos + término integral — ver el docstring del módulo para las matemáticas exactas y su procedencia). Configuración:
  ```yaml
  config:
    output: '/dron01/goal_pose'
    input: '/dron01/local_pose'
    period: 0.02
  neighbours:
    N0: {id: dron02, topic: '/dron02/local_pose', dx: 0.25, dy: 0.25, dz: 0.1}
  gains:
    integral_divisor_xy: 1.0
    integral_divisor_z: 5.0
  ```
  **Este esquema de configuración es una propuesta propia de este paquete** — Francisco describió el comportamiento del nodo ("suscribirse a las posiciones de los robots, publicar goal_pose") pero no proporcionó un `.yaml` de referencia para él como sí hizo con los dos nodos siguientes, así que todavía no hay un archivo de ejemplo al que remitirse. Se señala para revisión, no se ha asumido en silencio.
  ```
  ros2 run uned_crazyflie_missions formation --ros-args -p config:=/ruta/a/formation.yaml
  ```

- **`waypoints`** (`waypoints.py`): guía a un robot a través de una secuencia de puntos leída de un archivo de configuración — `take_off`, visitar todos los puntos (en el orden declarado, u optimizado por TSP, ver `shape`), `land`. Coincide exactamente con `uned_crazyflie_config/resources/demo_individual_waypoints.yaml` (ver ese archivo para el esquema completo: topics y tipos `output`/`input`, tolerancia `range`, `shape` (`polygon` o `tsp`), `repeat`, `period`, y un diccionario `points` con `x`/`y`/`z` por punto y un `t` de timeout opcional). El orden y la heurística TSP pura (`nearest_neighbor` + `two_opt`) siguen viviendo en `tsp.py`, sin cambios.
  ```
  ros2 run uned_crazyflie_missions waypoints --ros-args -p config:=/ruta/a/demo_individual_waypoints.yaml
  ```

- **`sequencer`** (`sequencer.py`): publica una secuencia guionizada de valores en topics arbitrarios, cada paso condicionado por un retardo fijo o esperando un valor concreto en un topic suscrito. Coincide exactamente con `uned_crazyflie_config/resources/demo_individual_waypoints_topics.yaml` (declara topics `publisher`/`subscription` + tipos, luego `cmd00`, `cmd01`, ... en orden, cada uno con `topic`/`type`/`value` y un `trigger`). Hoy solo está implementado `std_msgs/String`, ya que es el único tipo usado en la configuración de referencia — no es una limitación silenciosa, `SUPPORTED_TYPES` en el módulo lo deja explícito y lanza un error claro en caso contrario.
  ```
  ros2 run uned_crazyflie_missions sequencer --ros-args -p config:=/ruta/a/demo_individual_waypoints_topics.yaml
  ```

## Añadir una nueva misión

Mismo patrón que los tres nodos anteriores: un nodo que recibe un único parámetro `config` (ruta a un `.yaml`), habla solo por los topics nombrados en esa configuración, y nunca importa de `uned_crazyflie_driver` ni de `cflib`. Regístralo como `console_script` en `setup.py`.

## Tests

Además de los tests de lint estándar (`ament_copyright`, `ament_flake8`, `ament_pep257`), la lógica pura de cada nodo se testea unitariamente sin un grafo ROS en ejecución:

- **`test_tsp.py`** (4 tests): `nearest_neighbor` visita cada punto una sola vez, `two_opt` nunca empeora un recorrido, `solve_tsp` encuentra el óptimo real en un cuadrado de 4 puntos (el perímetro, no una diagonal cruzada), y el caso límite de 2 puntos.
- **`test_waypoints.py`** (4 tests): `load_points` conserva el orden de declaración del yaml y usa 0 por defecto para un `t` ausente; `order_for_shape('polygon', ...)` es el orden declarado; `order_for_shape('tsp', ...)` encuentra el óptimo real (mismo cuadrado que arriba, comprobado con una comparación real de longitud de recorrido, no solo "alguna permutación"); un caso límite de un solo punto.
- **`test_formation.py`** (4 tests): `compute_correction` es cero sin vecinos, cero cuando ya está en el offset deseado, tira hacia el offset deseado cuando está demasiado lejos, y es la *media* de los errores de varios vecinos (comprobado con valores elegidos para que la media tenga que cancelarse de verdad, no solo "cualquier número que no falle").
- **`test_sequencer.py`** (4 tests): `sorted_cmd_keys` ordena `cmd00`/`cmd01`/... correctamente (incluyendo números de dos dígitos) e ignora claves no relacionadas; `trigger_satisfied` para triggers de tipo `time` y `topic`, más un tipo de trigger desconocido.

**Deliberadamente sin test unitario**: las propias subclases de `Node` (`FormationNode`, `WaypointsNode`, `SequencerNode`) — cablear un test significativo para ellas necesita un grafo ROS en ejecución, fuera del alcance de un test unitario aquí. La máquina de estados de `waypoints.py` se comprobó con un test de integración real vía `ros2 run` contra un nodo harness durante el desarrollo de la versión original solo-TSP (ver `AUDIT.md` en la rama `doc`); ese test de harness necesita volver a ejecutarse contra la nueva interfaz basada en configuración, algo no hecho en esta pasada.
