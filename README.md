# F1TENTH – Controlador Reactivo

![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue) ![Python 3.10](https://img.shields.io/badge/Python-3.10-green) ![License: MIT](https://img.shields.io/badge/License-MIT-yellow)

> **DisparityExtenderNode**  |  Estable a **7 m/s** durante **10 vueltas**
> Basado en la estrategia *Follow‑the‑Gap* + extensión de disparidades

---

## 📑 Tabla de contenidos

|  Sección                                          |  Descripción                                |
| ------------------------------------------------- | ------------------------------------------- |
| [1. Enfoque](#1-enfoque)                          | Resumen de la lógica de control             |
| [2. Árbol de proyecto](#2-árbol-de-proyecto)      | Estructura de carpetas & ficheros           |
| [3. Instalación](#3-instalación)                  | Cómo clonar y compilar el paquete           |
| [4. Ejecución](#4-ejecución)                      | Comandos rápidos & lanzamiento con *launch* |
| [5. Parámetros](#5-parámetros)                    | Lista comentada de parámetros ROS 2         |
| [6. Vueltas & cronómetro](#6-vueltas--cronómetro) | Nodo auxiliar de métricas de carrera        |
| [7. Simulación Gazebo](#7-simulación-gazebo)      | Ejecutar en el mundo de competición         |
| [8. FAQ](#8-faq)                                  | Solución de problemas comunes               |
| [9. Licencia](#9-licencia)                        | Términos de uso                             |

---

## 1. Enfoque

El controlador amplía **Follow‑the‑Gap** mediante la **extensión de disparidades**:

1. **Filtrar & extender** las disparidades del LiDAR para simular un vehículo más ancho.
2. **Seleccionar el centro** del mayor hueco libre como dirección objetivo.
3. **Moduladores** para mantener estabilidad a alta velocidad:

   * *Limitador de aceleración* (`accel_step`)
   * *Limitador de timón* (`max_steer_rate_deg`)
   * *Look‑ahead* frontal y reducción proporcional de velocidad

<p align="center"><img src="https://raw.githubusercontent.com/widegonz/F1Tenth-Repository/main/docs/fg_scheme.png" width="520"></p>

---

## 2. Árbol de proyecto

```text
└── gap_follow_pkg/
    ├── gap_follower.py       # DisparityExtenderNode
    ├── lap_counter.py        # Vueltas + cronómetro
    ├── launch/
    │   └── race_launch.py    # Lanza ambos nodos
    ├── package.xml
    └── setup.py
```

---

## 3. Instalación

```bash
cd ~/ros2_ws/src
# ⬇️ Clona tu fork
git clone https://github.com/<TU‑USUARIO>/gap_follow_pkg.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

Requisitos: **ROS 2 Humble**, `numpy` (incluido) y Python ≥ 3.10.

---

## 4. Ejecución

### Solo controlador

```bash
ros2 run gap_follow_pkg gap_follower
```

### Controlador + métricas (recomendado)

```bash
ros2 launch gap_follow_pkg race_launch.py
```

Se inicia `rviz` con la línea de meta para el `LapCounterNode`.

---

## 5. Parámetros

| Parámetro             | Default    | Descripción                           |
| --------------------- | ---------- | ------------------------------------- |
| `speed_max`           | `9.0` m/s  | Velocidad máxima en recta             |
| `accel_step`          | `0.25` m/s | Aumento (↓) máximo por ciclo          |
| `max_steer_rate_deg`  | `20` °     | Cambio de timón permitido por *frame* |
| `front_sector_deg`    | `20` °     | Semicono frontal para anticipación    |
| `blocked_dist_thresh` | `1.8` m    | Se considera bloqueado si `<` valor   |
| `slowdown_dist`       | `2.5` m    | Escala velocidad según distancia      |

Modificar al vuelo:

```bash
ros2 param set /disparity_extender speed_max 10.0
```

---

## 6. Vueltas & cronómetro

`LapCounterNode` publica en `/lap_info`:

```yaml
total_laps:      10
last_lap_time:   11.97   # [s]
avg_lap_time:    12.11
best_lap_time:   11.80
```

Ver en directo:

```bash
ros2 topic echo /lap_info
```

*Implementación*: cruza un plano `map` definido en *x=0*, usa histéresis de 2 m para evitar doble conteo.

---

## 7. Simulación Gazebo

```bash
# Lanzar pista ejemplo
ros2 launch f1tenth_gazebo race_track.launch.py
# Ejecutar controlador
ros2 launch gap_follow_pkg race_launch.py
```

---

## 8. FAQ

<details>
<summary>Oscila en rectas</summary>
⬇️ Reduce `steer_smoothing_alpha` o incrementa `angle_speed_factor`.
</details>
<details>
<summary>Se queda girando en un punto</summary>
⬇️ Sube `blocked_min_scale` a ≥ 0.75 o `max_steer_rate_deg` a 25 °.
</details>
<details>
<summary>Choca en curvas cerradas</summary>
⬆️ Aumenta `front_sector_deg` a 25 ° y `blocked_dist_thresh` a 2 m.
</details>

---

## 9. Licencia

MIT © 2024 *Tu Nombre / Universidad XYZ*

---

¡A correr! 🏁🚀
