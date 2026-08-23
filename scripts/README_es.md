# Scripts

Archivos auxiliares que no forman parte de ningún paquete ROS 2: post-procesado de datos (conversión de `ros2 bag` a `.csv`, gráficas) y modelos/identificación de Matlab/Simulink.

## Python (`Python/`)

- **`bag2csv.py`**: convierte cada topic de un `ros2 bag` en un archivo `.csv`, uno por topic, según el tipo de mensaje (no hace falta conocer los nombres de los topics de antemano). Uso: `python3 bag2csv.py <carpeta_bag>`.
- **`plot_csv.py`**: grafica con matplotlib los archivos `.csv` generados por los dos scripts anteriores, una figura por archivo. Uso: `python3 plot_csv.py archivo1.csv [archivo2.csv ...]`.
- `basiclog.py`, `motion_cmd_test.py`, `motion_test.py`: **pendientes de revisión manual** (ver `AUDIT.md` en la rama `doc`) — no se han tocado en esta pasada, hay que decidir si siguen siendo útiles.

## Matlab (`Matlab/`)

- **`plot_csv.m`**: equivalente en Matlab de `plot_csv.py`, lee directamente los `.csv` de `bag2csv.py` con `readtable`. Uso: `plot_csv('archivo1.csv', 'archivo2.csv')`. **No verificado en este entorno** (no hay Matlab disponible en el sandbox de esta sesión) — revisar antes de confiar en él.
- `Crazyflie_Graphs.m`: script de gráficas ya existente, pero **no** lee los `.csv` de `bag2csv.py` — espera variables ya cargadas en el workspace (`references`, `dron_state`, `altitude_controller`, ...) generadas por el pipeline `Crazyflie_Launch.m` + `bag2txt_sim.sh` + `Txt2Mat.m`. Se ha dejado sin tocar en lugar de adaptarlo, para no romper ese pipeline existente.
- `Crazyflie_Launch.m`, `Crazyflie_Model.m`, `Crazyflie_Stats.m`, `Crazyflie_IAE.m`, `ROS_Controller_Example.m`, `ROS_test.m`, `ROS.slx`, y el contenido de `Controladores/`/`Modelos/`: **pendientes de revisión manual** (ver `AUDIT.md` en la rama `doc`) — sin tocar en esta pasada, hay que decidir cuáles siguen siendo útiles y eliminar el resto.

## Shell (raíz de `scripts/`)

