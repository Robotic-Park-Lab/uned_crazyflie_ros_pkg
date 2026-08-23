# uned_crazyflie_gui

`ament_python` package with the PyQt5 graphical interface for handling a single Crazyflie, plus a generic RQT perspective and RViz file (not tied to any specific demo) to visualize any Crazyflie.

## Structure

- `interface_gui.py` (entry point `interface_node`): main PyQt window, loads `main.ui`. Deliberately basic version: no embedded `rqt_plot`/`rqt_graph` panels like an earlier version of the code had, because that embedding depended on `xdotool`/`QWindow.fromWinId()` (X11, fragile, never declared as a dependency) — see `FUTURO_interface_gui.md` on the `doc` branch for the original full vision.
- `main.ui` / `main_ui.py`: window design (Qt Designer) and its compiled Python version.
- `logo.qrc` / `logo_rc.py`: graphical resources (logos) packaged for Qt.
- `rqt/crazyflie.perspective`: generic RQT perspective (`rqt_graph`+`rqt_plot`+`rqt_publisher`+`rqt_bag`+`rqt_console`+`rqt_service_caller`), using the `dron01` namespace as a placeholder — adjust the topic names if your drone uses a different id. Load it with `rqt --perspective-file $(ros2 pkg prefix uned_crazyflie_gui)/share/uned_crazyflie_gui/rqt/crazyflie.perspective`.
- `rviz/crazyflie.rviz`: `Grid` + `TF` (all frames enabled), valid for any Crazyflie without editing anything. Load it with `rviz2 -d $(ros2 pkg prefix uned_crazyflie_gui)/share/uned_crazyflie_gui/rviz/crazyflie.rviz`.

## Regenerating the compiled Qt files

If you edit `main.ui` or `logo.qrc`, their Python versions need regenerating:
```
pyuic5 -x main.ui -o main_ui.py
pyrcc5 -o logo_rc.py logo.qrc
```

## Usage

```
cd dev_ws
colcon build --symlink-install --packages-select uned_crazyflie_gui
ros2 run uned_crazyflie_gui interface_node
```

## Tests

`test/test_interface_gui.py`: a headless smoke test. Forces `QT_QPA_PLATFORM=offscreen` (no real display needed), builds the real `MainWindow` from `main.ui` with a real `rclpy` node, and checks it doesn't crash on startup and closes cleanly — not a UI/interaction test, just that `interface_node` actually starts. This is exactly the kind of regression a plain flake8/pep257 pass would never catch: the real bug this class had before this refactor's `a233491` commit was `main_ui.py` doing a bare `import logo_rc`, which broke once the file moved into a proper Python package. Beyond that, the window's widget tree and Qt resource loading are exercised for real but not asserted field-by-field — deeper UI testing would need a real display or a much heavier Qt test harness, not attempted here.
