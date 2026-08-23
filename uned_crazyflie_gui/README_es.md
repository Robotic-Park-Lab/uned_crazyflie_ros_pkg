# uned_crazyflie_gui

Paquete `ament_python` con la interfaz gráfica PyQt5 para manejar un único Crazyflie, más una perspectiva RQT genérica y un archivo RViz (no ligados a ninguna demo concreta) para visualizar cualquier Crazyflie.

## Estructura

- `interface_gui.py` (punto de entrada `interface_node`): ventana principal de PyQt, carga `main.ui`. Versión deliberadamente básica: sin paneles `rqt_plot`/`rqt_graph` embebidos como tenía una versión anterior del código, porque ese embebido dependía de `xdotool`/`QWindow.fromWinId()` (X11, frágil, nunca declarado como dependencia) — ver `FUTURO_interface_gui.md` en la rama `doc` para la visión completa original.
- `main.ui` / `main_ui.py`: diseño de la ventana (Qt Designer) y su versión Python compilada.
- `logo.qrc` / `logo_rc.py`: recursos gráficos (logotipos) empaquetados para Qt.
- `rqt/crazyflie.perspective`: perspectiva RQT genérica (`rqt_graph`+`rqt_plot`+`rqt_publisher`+`rqt_bag`+`rqt_console`+`rqt_service_caller`), usando el namespace `dron01` como marcador de posición — ajusta los nombres de topic si tu dron usa un id diferente. Cárgala con `rqt --perspective-file $(ros2 pkg prefix uned_crazyflie_gui)/share/uned_crazyflie_gui/rqt/crazyflie.perspective`.
- `rviz/crazyflie.rviz`: `Grid` + `TF` (todos los frames habilitados), válido para cualquier Crazyflie sin editar nada. Cárgalo con `rviz2 -d $(ros2 pkg prefix uned_crazyflie_gui)/share/uned_crazyflie_gui/rviz/crazyflie.rviz`.

## Regenerar los archivos Qt compilados

Si editas `main.ui` o `logo.qrc`, sus versiones Python necesitan regenerarse:
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

## Tests

`test/test_interface_gui.py`: un smoke test sin interfaz gráfica. Fuerza `QT_QPA_PLATFORM=offscreen` (no necesita display real), construye la `MainWindow` real a partir de `main.ui` con un nodo `rclpy` real, y comprueba que no falla al arrancar y se cierra limpiamente — no es un test de interacción/interfaz, solo que `interface_node` realmente arranca. Es exactamente el tipo de regresión que un simple flake8/pep257 nunca detectaría: el bug real que tenía esta clase antes del commit `a233491` de este refactor era que `main_ui.py` hacía un `import logo_rc` a secas, lo cual se rompió al mover el archivo dentro de un paquete Python adecuado. Más allá de eso, el árbol de widgets de la ventana y la carga de recursos Qt se ejercen de verdad pero no se comprueban campo a campo — un testeo de interfaz más profundo necesitaría un display real o un harness de test de Qt mucho más pesado, no intentado aquí.
