# Trabajo futuro — `interface_gui.py` (gráficas y grafo embebidos)

La versión actual de `interface_gui.py` (commit `a233491`, rama `humble-dev`) es
deliberadamente básica: carga la ventana de `main.ui` tal cual, con una pestaña
placeholder en el hueco donde antes iban las gráficas. Este documento recoge la
visión original (más completa) que había en el código antes de arreglarlo, para
que quede como referencia si en el futuro se quiere retomar.

## Qué intentaba hacer la versión original

La `interface_gui.py` previa a la auditoría (ver `git show 0b82b34:uned_crazyflie_gui/uned_crazyflie_gui/interface_gui.py`
en `humble-dev`) tenía una clase `RQT_Panel` que:

1. Lanzaba `rqt_plot` / `rqt_graph` como subprocesos independientes
   (`subprocess.Popen([self._command])`).
2. Buscaba el `window_id` X11 de esa ventana externa vía `xdotool search --pid <pid>`,
   con reintentos durante 10s.
3. Envolvía ese `window_id` con `QWindow.fromWinId(...)` y lo incrustaba como un
   `QWidget` dentro de una pestaña de `GraphsWidget`, para que las gráficas de
   `rqt_plot` y el grafo de `rqt_graph` aparecieran "dentro" de la interfaz en vez
   de en ventanas sueltas.

La idea es razonable (evitar reimplementar `rqt_plot`/`rqt_graph` desde cero,
reutilizar las herramientas de ROS 2 tal cual), pero la implementación no era
viable tal y como estaba:

- **No estaba conectada a ningún entry point** — código muerto, nunca se ejecutaba.
- **Dependía de `xdotool`**, que no es una dependencia declarada en ningún
  `package.xml`/`setup.py` del repo, y no está garantizado que esté instalado en
  el entorno del laboratorio.
- **Embeber ventanas X11 ajenas es frágil por naturaleza**: depende de que el
  gestor de ventanas se comporte de una manera concreta, no funciona igual (o no
  funciona) bajo Wayland, y falla en cualquier entorno sin servidor X real (por
  ejemplo, para pruebas automatizadas con `QT_QPA_PLATFORM=offscreen`, como se ha
  usado para verificar el resto de este repo).
- El propio código de `cerrar()` reconoce la fragilidad: mata los procesos con
  `killall rqt` a la fuerza en vez de cerrarlos limpiamente.

## Qué se ha hecho en la versión básica actual

- Se ha quitado `RQT_Panel` y el lanzamiento de subprocesos/`xdotool`.
- El hueco donde iban las gráficas tiene un `QLabel` placeholder informativo.
- Se ha corregido el bug real que sí tenía la versión original: el icono de la
  ventana usaba la ruta de recurso Qt `:/figs/LogoRoboticPark.png`, que no existe
  (el prefijo declarado en `logo.qrc` es `logo`, no vacío) — la ruta correcta,
  verificada cargándola con `QPixmap` real, es `:/logo/figs/LogoRoboticPark.png`.
- Se ha dejado conectada como el entry point real `interface_node` (antes
  apuntaba a un stub vacío, `interface_node.py`, ahora eliminado).

## Alternativas a evaluar para una versión futura más completa

En vez de retomar el embebido X11 vía `xdotool`, valorar:

1. **Gráficas nativas con `matplotlib` + `FigureCanvasQTAgg`**, suscribiendo
   directamente a los topics de interés (posición, error de formación, batería,
   etc.) desde el propio nodo `rclpy` y actualizando el `canvas` con un
   `QTimer`. Es más trabajo que reusar `rqt_plot`, pero no depende de X11 ni de
   procesos externos, y funciona igual de bien headless (útil para tests).
2. **Reutilizar `rqt_gui_py`/`rqt_plot` como librería en vez de subproceso**:
   `rqt_plot` expone su lógica de plotting como plugin de `rqt_gui`; en vez de
   lanzar el binario y embeber su ventana, se podría instanciar el plugin
   directamente dentro del proceso Qt de `interface_gui.py`. Requiere estudiar
   la API de `rqt_gui_py.plugin.Plugin` — no explorado todavía.
3. **Para el grafo de nodos (`rqt_graph`)**: si no se necesita en tiempo real,
   una alternativa mucho más simple es un botón que lance
   `ros2 run rqt_graph rqt_graph` en una ventana normal (sin embeber nada) — se
   pierde la integración visual pero se gana toda la fiabilidad.
4. Sea cual sea la vía elegida, las dependencias nuevas (`matplotlib`,
   `xdotool`, etc.) deben declararse en `package.xml`/`setup.py`, cosa que la
   versión original no hacía.

Nada de esto está implementado — es una propuesta para cuando se quiera retomar
esta parte de la interfaz.
