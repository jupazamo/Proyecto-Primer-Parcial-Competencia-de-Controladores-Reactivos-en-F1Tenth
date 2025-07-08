# F1TENTH – Controlador Reactivo

&#x20;&#x20;

> **DisparityExtenderNode**  |  Estable a **7 m/s** durante **10 vueltas**
> Basado en la estrategia *Follow‑the‑Gap* + extensión de disparidades

---

## 🎥 Evidencias en video

| Descripción                                                                      | Enlace                                       |
| -------------------------------------------------------------------------------- | -------------------------------------------- |
| Robot completando **10 vueltas** con contador de vueltas y cronómetro por vuelta | [🔗 Ver video](https://youtu.be/M5sZzdezW5M) |
| Robot **esquivando 5 obstáculos** durante 5 vueltas                              | [🔗 Ver video](https://youtu.be/yBp00ISa7pc) |

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
| [7. Mapas de simulación](#7-mapas-de-simulación)  | Pistas empleadas y cómo cargarlas           |
| [8. Simulación Gazebo](#8-simulación-gazebo)      | Ejecutar en el mundo de competición         |
| [9. FAQ](#9-faq)                                  | Solución de problemas comunes               |
| [10. Licencia](#10-licencia)                      | Términos de uso                             |

---

## 1. Enfoque

El controlador amplía **Follow‑the‑Gap** mediante la **extensión de disparidades**:

1. **Filtrar & extender** las disparidades del LiDAR para simular un vehículo más ancho.
2. **Seleccionar el centro** del mayor hueco libre como dirección objetivo.
3. **Moduladores** para mantener estabilidad a alta velocidad:

   * *Limitador de aceleración* (`accel_step`)
   * *Limitador de timón* (`max_steer_rate_deg`)
   * *Look‑ahead* frontal y reducción proporcional de velocidad

---

## 2. Árbol de proyecto

> **Nota:** el repositorio versionado incluye únicamente la carpeta `src/`; las rutas mostradas a continuación son relativas a ella.

```text
└── src/
    ├── gap_follow_pkg/
    │   ├── gap_follow_pkg/
    │   │   ├── gap_follower.py       # DisparityExtenderNode
    │   │   ├── lap_timer.py          # Vueltas + cronómetro
    │   ├── launch/
    │   │   └── dual_launch.launch.py    # Lanza ambos nodos
    │   └── setup.py
    └── f1tenth_gym_ros/
        └── maps/
            ├── Budapest_map.png          # pista sin obstáculos
            └── Budapest_map_modified.png # pista con 5 obstáculos
```

---

## 3. Instalación
Partimos desde tu HOME (~) — la ruta que usa sim.yaml busca exactamente ~/F1Tenth-Repository:
```bash
cd ~
# ⬇️ Clona el repositorio (o tu fork)
git clone https://github.com/jupazamo/Proyecto-Primer-Parcial-Competencia-de-Controladores-Reactivos-en-F1Tenth.git F1Tenth-Repository
cd F1Tenth-Repository
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
ros2 launch gap_follow_pkg dual_launch.launch
```

Se inicia `rviz` con la línea de meta para el `LapCounterNode`.

---

## 5. Parámetros

| Parámetro             | Default    | Descripción                           |
| --------------------- | ---------- | ------------------------------------- |
| `speed_max`           | `7.0` m/s  | Velocidad máxima en recta             |
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
lap_times_s:
  - 70.22  # Lap 1
  - 69.40  # Lap 2
  - 140.62 # Lap 3 (obstáculo + rebote)
  - 69.89  # Lap 4
  - 69.95  # Lap 5
  - 68.80  # Lap 6
  - 139.96 # Lap 7 (doble sobrepaso)
  - 70.11  # Lap 8
  - 68.06  # Lap 9
  - 69.04  # Lap 10
```

Ver en directo:

```bash
ros2 topic echo /lap_info
```

*Implementación*: cruza un plano `map` definido en *x = 0*, usa histéresis de 2 m para evitar doble conteo.

---

## 7. Mapas de simulación

> Los ficheros `.png` y `.yaml` se encuentran en `src/f1tenth_gym_ros/maps/` y se copian automáticamente al compilar el paquete.

### 7.1 Pista base

| Vista | Descripción                                                                                                                                             |
| ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
|       | Mapa **Budapest** (≈ 240 m) con 8 curvas (5 izquierda / 3 derecha). La línea de meta se sitúa en **x = 0 m** del *frame map* para el conteo de vueltas. |

### 7.2 Pista con obstáculos

| Vista | Descripción                                                                                                                                                                    |   |                                                                                                                                               |
| ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | - | --------------------------------------------------------------------------------------------------------------------------------------------- |
|       | Mismo trazado **Budapest** con **5 obstáculos** distribuidos en rectas y salidas de curva (cubos de \~0.25 m³). El desafío consiste en completar **5 vueltas** sin colisiones. |   | Mismo circuito con **5 cubos de 0.25 m³** colocados en rectas y salidas de curva. El reto consiste en completar **5 vueltas** sin colisiones. |

---

## 8. Simulación Gazebo

```bash
# Lanzar simulación (pista Budapest **o** variante con obstáculos)
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Ejecutar controlador + métricas
ros2 launch gap_follow_pkg dual_launch.launch
```

> 🔧 **Tip** Para alternar entre la pista base y la versión con **5 obstáculos**
> edita `src/f1tenth_gym_ros/config/sim.yaml` y ajusta la clave `map_path`
> apuntando a `Budapest_map.png` (base) o `Budapest_map_modified.png` (obstáculos).

---
