# uned_crazyflie_missions

Paquete `ament_python` (antes `uned_crazyflie_task`) con nodos de **misión de alto nivel** para uno o varios Crazyflie: formaciones y recorrido de waypoints. Todos indiferentes a si el dron es físico o virtual (Webots) — hablan por topics con el driver, no directamente con `cflib` ni con Webots (excepto `leader_follower.py`/`shape_based_formation_control.py`, ver más abajo).

## Nodos

- **`tsp_waypoints`**: recorre un conjunto de waypoints resolviendo el orden de visita como un problema del viajero (TSP) con un heurístico vecino-más-cercano + 2-opt (`tsp.py`, testeado en `test/test_tsp.py` sin necesidad de ROS). Funciona igual con un Crazyflie físico o virtual: solo usa `<robot_id>/order` (`take_off`/`land`) y `<robot_id>/goal_pose` como comando, leyendo `<robot_id>/local_pose` como realimentación — el mismo contrato de topics que ya exponen `uned_crazyflie_driver`/`uned_crazyflie_webots`. Parámetros: `robot_id` (namespace del dron, por defecto `dron01`), `waypoints` (lista plana `[x1,y1,z1, x2,y2,z2, ...]`), `tolerance` (metros para considerar alcanzado un waypoint), `loop` (repetir el recorrido).
  ```
  ros2 run uned_crazyflie_missions tsp_waypoints --ros-args \
    -p robot_id:=dron01 \
    -p waypoints:="[0.0,0.0,1.0, 2.0,0.0,1.0, 2.0,2.0,1.0, 0.0,2.0,1.0]"
  ```
- **`formation_control_webots`**: control de formaciones puramente por topics (`<id>/goal_pose`, `<id>/pose`), sin dependencia de `cflib` — el patrón más "agnóstico al driver" de los tres nodos de formación.
- **`leader_follower`** / **`shape_based_formation_control`**: formación líder-seguidor y formación basada en forma. A diferencia de los dos anteriores, estos dos se conectan **directamente** a `cflib.crazyflie.swarm.Swarm` (solo hardware físico) en vez de hablar con el driver por topics — es una arquitectura distinta dentro del mismo paquete, heredada del código original; no se ha tocado en esta pasada.

## Añadir una nueva misión

Sigue el mismo patrón que `tsp_waypoints`: un nodo que publica `<robot_id>/order`/`<robot_id>/goal_pose` y se suscribe a `<robot_id>/local_pose`, sin hablar directamente con `cflib`/Webots, para que funcione igual con un dron físico o virtual. Regístralo como `console_script` en `setup.py`.

## Ideas para futuras misiones (no implementadas)

Navegación autónoma y exploración, mencionadas también como objetivo del paquete, no se han implementado en esta pasada — no hay una especificación concreta de qué algoritmo/sensor usar (a diferencia del TSP, que Francisco pidió explícitamente sobre "un conjunto de puntos"), y añadir un nodo vacío solo para completar la lista no aporta nada real.
