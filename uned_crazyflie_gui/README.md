# uned_crazyflie_gui

Paquete `ament_python` con la interfaz gráfica PyQt5 para el manejo individual de un Crazyflie, más una perspectiva RQT y un fichero RViz genéricos (no ligados a ninguna demo concreta) para visualizar cualquier Crazyflie.

## Estructura

- `interface_gui.py` (entry point `interface_node`): ventana principal PyQt, carga `main.ui`. Versión básica a propósito: sin los paneles `rqt_plot`/`rqt_graph` embebidos que tenía una versión anterior del código, porque ese embebido dependía de `xdotool`/`QWindow.fromWinId()` (X11, frágil, sin declarar como dependencia) — ver `FUTURO_interface_gui.md` en la rama `doc` para la visión completa original.
- `main.ui` / `main_ui.py`: diseño de la ventana (Qt Designer) y su versión compilada a Python.
- `logo.qrc` / `logo_rc.py`: recursos gráficos (logos) empaquetados para Qt.
- `rqt/crazyflie.perspective`: perspectiva RQT genérica (`rqt_graph`+`rqt_plot`+`rqt_publisher`+`rqt_bag`+`rqt_console`+`rqt_service_caller`), con el namespace `dron01` como placeholder — ajusta los nombres de topic si tu dron usa otro id. Cárgala con `rqt --perspective-file $(ros2 pkg prefix uned_crazyflie_gui)/share/uned_crazyflie_gui/rqt/crazyflie.perspective`.
- `rviz/crazyflie.rviz`: `Grid` + `TF` (todos los frames habilitados), válido para cualquier Crazyflie sin editar nada. Cárgalo con `rviz2 -d $(ros2 pkg prefix uned_crazyflie_gui)/share/uned_crazyflie_gui/rviz/crazyflie.rviz`.

## Regenerar los ficheros compilados de Qt

Si editas `main.ui` o `logo.qrc`, hay que regenerar sus versiones Python:
```
pyuic5 -x main.ui -o main_ui.py
pyrcc5 -o logo_rc.py logo.qrc
```

## Uso

```
cd dev_ws
colcon build --symlink-install --packages-select uned_crazyflie_gui
ros2 run uned_crazyflie_gui interface_node
```
