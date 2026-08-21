# Documentación de ramas y guía de contribución — uned_crazyflie_ros_pkg

Esta rama (`doc`) no contiene código: solo explica para qué sirve cada rama del repositorio y cómo contribuir.

📋 **[AUDIT.md](AUDIT.md)** — checklist de la auditoría de `humble-dev` (2026-08-21), de lo más simple a lo más complejo. Se va tachando conforme se resuelve.

## Ramas de este repositorio

| Rama | Propósito |
|---|---|
| `humble-dev` | Desarrollo activo, sobre ROS 2 Humble. Rama por defecto del repo. Usa `cflib` (`crazyflie-lib-python`) directamente. |
| `benchmark` | **No se modifica ni se renombra.** Bloqueada en GitHub (`lock_branch`) incluso para administradores. Respalda un capítulo de libro sobre control publicado por Francisco Mañas. Cualquier actualización se reproduce reinstalando desde `RoboticPark/install.sh`, nunca con push directo. |
| `ros2-galactic-AGJ` | Controlador fuzzy de posición desarrollado en el Trabajo Fin de Máster de un alumno (iniciales AGJ). Se conserva tal cual: no se fusiona con `humble-dev` ni se elimina. |
| `ros-noetic` | Compatibilidad con ROS 1 Noetic. Incluye el submódulo `whoenig/crazyflie_ros`, que **no** se usa en `humble-dev` (ahí el driver ROS 2 habla directamente con `cflib`). |
| `doc` (esta) | Documentación de ramas y guía de contribución, común a todo el laboratorio. |

## Guía de contribución

Antes de abrir un Pull Request:
1. Comprueba que el paquete compila (`colcon build --packages-select <paquete>`) y, si aplica, que pasa `colcon test`.
2. Sigue la convención de nombres ya usada en el repo (`uned_<paquete>_<rol>`, p. ej. `_config`, `_driver`, `_task`, `_gui`, `_webots`).
3. Actualiza el README de la rama de desarrollo si cambias estructura, dependencias o forma de uso — y esta página si cambias el propósito de una rama o añades una nueva.
4. Si el cambio está ligado a un ejercicio, proyecto o publicación del laboratorio, indícalo en la descripción del PR.

### Estilo de código
- Python (`ament_python`): `ament_flake8` / `ament_pep257`.
- C++ (`ament_cmake`): sigue el estilo ya presente en el paquete (`-Wall -Wextra -Wpedantic`).

### Licencia
El código de este repositorio se publica bajo licencia BSD 3-Clause (ver `LICENSE` en `humble-dev`). Al contribuir, aceptas que tu aportación se publique bajo la misma licencia.
