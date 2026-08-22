# Scripts

Ficheros auxiliares que no forman parte de ningún paquete de ROS 2: post-proceso de datos (conversión de `ros2 bag` a `.csv`, graficado) y modelos/identificación en Matlab/Simulink.

## Python (`Python/`)

- **`bag2csv.py`**: convierte todos los topics de un `ros2 bag` a un `.csv` por topic, según el tipo de mensaje (no requiere conocer los nombres de topic de antemano). Uso: `python3 bag2csv.py <carpeta_del_bag>`.
- **`bag2csv_crazyflie.py`**: conversor anterior, específico de los topics de `uned_crazyflie_driver` (`cf_data`, `cf_pose`, `cf_twist`, `pose`, `goal_pose`, `onboard_cmd`, `cf_order`) para un dron concreto. Uso: `python3 bag2csv_crazyflie.py <carpeta_del_bag> <id_dron>`.
- **`plot_csv.py`**: grafica con matplotlib los `.csv` generados por los dos scripts anteriores, una figura por fichero. Uso: `python3 plot_csv.py fichero1.csv [fichero2.csv ...]`.
- `basiclog.py`, `motion_cmd_test.py`, `motion_test.py`: **pendientes de revisión manual** (ver `AUDIT.md`, rama `doc`) — no se han tocado en esta pasada, decidir si siguen siendo útiles.

## Matlab (`Matlab/`)

- **`plot_csv.m`**: equivalente Matlab de `plot_csv.py`, lee directamente los `.csv` de `bag2csv.py` con `readtable`. Uso: `plot_csv('fichero1.csv', 'fichero2.csv')`. **No verificado en este entorno** (no hay Matlab disponible en el sandbox de esta sesión) — revisar antes de confiar en él.
- `Crazyflie_Graphs.m`: script de graficado existente, pero **no** lee los `.csv` de `bag2csv.py` — espera variables ya cargadas en el workspace (`references`, `dron_state`, `altitude_controller`, ...) que genera el pipeline `Crazyflie_Launch.m` + `bag2txt_sim.sh` + `Txt2Mat.m`. Se ha dejado intacto en vez de adaptarlo, para no romper ese pipeline existente.
- `Crazyflie_Launch.m`, `Crazyflie_Model.m`, `Crazyflie_Stats.m`, `Crazyflie_IAE.m`, `ROS_Controller_Example.m`, `ROS_test.m`, `ROS.slx`, y el contenido de `Controladores/`/`Modelos/`: **pendientes de revisión manual** (ver `AUDIT.md`, rama `doc`) — no se han tocado en esta pasada, decidir cuáles siguen siendo útiles y eliminar el resto.

## Shell (raíz de `scripts/`)

- `bag2txt_sim.sh`, `Txt2Mat.m`: pipeline anterior de conversión bag→txt→Matlab. **Pendientes de revisión manual** junto con el resto.
