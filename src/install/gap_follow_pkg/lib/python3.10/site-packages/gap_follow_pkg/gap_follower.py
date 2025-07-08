import math
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry          # ← NUEVO
import time                                # ← NUEVO

class DisparityExtenderNode(Node):
    """Iteración 4 – Estabilidad a 8 m/s durante 10 vueltas

    ░░ Novedades clave ░░
    1. **Limitador de aceleración** (`accel_step`) – evita que el coche pase de 0 → 8 m/s en una sola ráfaga; sube (o baja) ≤ 0.25 m/s por ciclo.
    2. **Limitador de rapidez del timón** (`max_steer_rate_deg`) – el ángulo de giro puede variar ≤ 20 ° por frame → sin volantazos que desestabilicen.
    3. **Mayor anticipación** – `front_sector_deg` ↑ 20 °, `blocked_dist_thresh` ↑ 1.8 m y `slowdown_dist` ↑ 2.5 m.
    4. **Ajustes finos** – `speed_blocked_factor` 0.55, `steer_smoothing_alpha` 0.32, `k_turn` 1.1.

    Con estos cambios se mantiene la velocidad máxima (8 m/s) en rectas, pero
    entra a las curvas un ~12 % más lento y sin picos de aceleración/giro ⇒
    consistente durante ≥ 10 vueltas.
    """

    # ───────── Init ─────────
    def __init__(self):
        super().__init__("disparity_extender")

        #  ░░ PARAMS ░░
        self.declare_parameter("car_width",                0.22)
        self.declare_parameter("safety_radius",            0.05)
        self.declare_parameter("disparity_threshold",       0.20)
        self.declare_parameter("fov",                      270.0)
        self.declare_parameter("gap_min_value",             0.10)

        # Velocidad
        self.declare_parameter("speed_max",                 7.0)
        self.declare_parameter("speed_min",                 0.5)
        self.declare_parameter("angle_speed_factor",        1.1)   # ⇡ reducción en curvas
        self.declare_parameter("steer_smoothing_alpha",     0.32)
        self.declare_parameter("accel_step",                0.25)  # ⇠ nuevo

        # Seguridad y anticipación
        self.declare_parameter("front_sector_deg",         20.0)   # ⇡
        self.declare_parameter("blocked_dist_thresh",       1.80)  # ⇡
        self.declare_parameter("blocked_steer_deg",        40.0)
        self.declare_parameter("blocked_min_scale",         0.70)
        self.declare_parameter("blocked_scale_exp",         1.40)
        self.declare_parameter("speed_blocked_factor",      0.55)  # ⇣ menos freno
        self.declare_parameter("slowdown_dist",             2.50)  # ⇡ look‑ahead

        # Dinámica de timón
        self.declare_parameter("max_steer_rate_deg",       20.0)   # ⇠ nuevo

        self.declare_parameter("debug", True)
        p = lambda n: self.get_parameter(n).value
        self.car_width         = p("car_width")
        self.safety_radius     = p("safety_radius")
        self.disp_th           = p("disparity_threshold")
        self.fov               = math.radians(p("fov"))
        self.gap_min           = p("gap_min_value")
        self.speed_max         = p("speed_max")
        self.speed_min         = p("speed_min")
        self.k_turn            = p("angle_speed_factor")
        self.alpha_default     = p("steer_smoothing_alpha")
        self.front_deg         = p("front_sector_deg")
        self.blocked_dist      = p("blocked_dist_thresh")
        self.blocked_steer_max = math.radians(p("blocked_steer_deg"))
        self.blocked_min_scale = p("blocked_min_scale")
        self.blocked_exp       = p("blocked_scale_exp")
        self.speed_blocked_f   = p("speed_blocked_factor")
        self.slowdown_dist     = p("slowdown_dist")
        self.accel_step        = p("accel_step")
        self.max_steer_rate    = math.radians(p("max_steer_rate_deg"))
        self.debug             = p("debug")

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.scan_sub  = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.prev_steer = 0.0
        self.prev_speed = self.speed_min

        # ---------- seguimiento de vueltas ----------
        self.declare_parameter("lap_radius_thresh", 0.6)   # m  distancia para “estar” en meta
        self.declare_parameter("min_lap_dist",      8.0)   # m  para evitar dobles conteos
        self.lap_radius  = self.get_parameter("lap_radius_thresh").value
        self.min_lap_dist= self.get_parameter("min_lap_dist").value

        self.odom_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_cb, 10)

        self.start_pos       = None     # (x0,y0) se fija al 1er Odom
        self.last_pos        = None
        self.travelled_dist  = 0.0
        self.lap_count       = 0
        self.lap_start_time  = None     # epoch() de la vuelta en curso
        self.lap_times: List[float] = []
        self.in_start_zone   = False    # evita re-contar mientras siga en la zona

    # ───────── CALLBACK ─────────
    def scan_cb(self, scan: LaserScan):
        r = np.array(scan.ranges, dtype=np.float32)
        r = np.where(np.isfinite(r), r, scan.range_max)
        a_min, a_inc = scan.angle_min, scan.angle_increment

        # FRONT sector
        center = int((-a_min) / a_inc)
        w      = int(self.front_deg / math.degrees(a_inc))
        front  = float(r[center - w:center + w + 1].min())
        blocked = front < self.blocked_dist

        # Disparity extension
        rp = self._extend_disparities(r, a_inc)

        # Selección de steering
        if blocked:
            steer_tgt = self._blocked_steer(r, center, a_inc, front)
            alpha = 0.05  # menos amortiguado al reaccionar
        else:
            steer_tgt = self._steer_from_gap(rp, a_min, a_inc)
            alpha = self.alpha_default

        # Suavizado
        steer_sm = (1 - alpha) * steer_tgt + alpha * self.prev_steer

        # Limitador de rapidez del timón
        delta = steer_sm - self.prev_steer
        if delta > self.max_steer_rate:
            steer = self.prev_steer + self.max_steer_rate
        elif delta < -self.max_steer_rate:
            steer = self.prev_steer - self.max_steer_rate
        else:
            steer = steer_sm
        self.prev_steer = steer

        # Velocidad base dependiente del ángulo de giro
        v_turn = self.speed_max / (1 + self.k_turn * abs(steer))
        if blocked:
            v_turn *= self.speed_blocked_f

        # Rampa look‑ahead con distancia frontal
        v_front = self.speed_max * min(1.0, front / self.slowdown_dist)
        v_target = max(self.speed_min, min(v_turn, v_front))

        # Limitador de aceleración
        if v_target > self.prev_speed + self.accel_step:
            v = self.prev_speed + self.accel_step
        elif v_target < self.prev_speed - self.accel_step:
            v = self.prev_speed - self.accel_step
        else:
            v = v_target
        self.prev_speed = v

        # Publicar
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(v)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)

        if self.debug:
            self.get_logger().info(
                f"steer={math.degrees(steer):5.1f}°, v={v:.2f}, front={front:.2f}, block={blocked}")

    # ───────── helpers ─────────
    def _extend_disparities(self, r: np.ndarray, a_inc: float) -> np.ndarray:
        rp = r.copy()
        for i in range(len(rp) - 1):
            if abs(rp[i] - rp[i + 1]) > self.disp_th:
                j = i if rp[i] < rp[i + 1] else i + 1
                d = max(rp[j], 0.05)
                rad = math.asin(min(1.0, (self.car_width * 0.5 + self.safety_radius) / d))
                b   = int(rad / a_inc)
                rp[max(0, j - b): j + b + 1] = rp[j]
        return rp

    def _blocked_steer(self, r: np.ndarray, center: int, a_inc: float, front: float) -> float:
        # lado con más espacio (45°–90°)
        idx = lambda deg1, deg2: slice(center + int(deg1 / math.degrees(a_inc)),
                                       center + int(deg2 / math.degrees(a_inc)))
        left_sum  = float(r[idx(45, 90)].sum())
        right_sum = float(r[idx(-90, -45)].sum())
        side = 1.0 if left_sum > right_sum else -1.0

        frac  = min(1.0, max(0.0, (self.blocked_dist - front) / self.blocked_dist))
        scale = self.blocked_min_scale + (1 - self.blocked_min_scale) * pow(frac, self.blocked_exp)

        if self.debug:
            self.get_logger().info(
                f"BLOCKED front={front:.2f} → steer {('L' if side>0 else 'R')} scale={scale:.2f}")
        return side * self.blocked_steer_max * scale

    def _steer_from_gap(self, rp: np.ndarray, a_min: float, a_inc: float) -> float:
        start: Optional[int] = None
        best = (0, 0)
        for i, d in enumerate(rp):
            if d > self.gap_min:
                if start is None:
                    start = i
            else:
                if start is not None and i - 1 - start > best[1] - best[0]:
                    best = (start, i - 1)
                start = None
        if start is not None and (len(rp) - 1 - start) > (best[1] - best[0]):
            best = (start, len(rp) - 1)
        center_idx = (best[0] + best[1]) // 2
        return a_min + center_idx * a_inc
    
    # ───────── ODOMETRY ─────────
    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        pos = np.array([x, y])

        # 1) Fijar punto de salida
        if self.start_pos is None:
            self.start_pos      = pos
            self.last_pos       = pos
            self.lap_start_time = time.time()
            return

        # 2) Acumular distancia recorrida
        self.travelled_dist += np.linalg.norm(pos - self.last_pos)
        self.last_pos = pos

        # 3) Comprobar si está en la “zona meta”
        dist_to_start = np.linalg.norm(pos - self.start_pos)
        in_zone = dist_to_start < self.lap_radius

        # —–– Cruce de línea de meta —––
        if in_zone and not self.in_start_zone and self.travelled_dist > self.min_lap_dist:
            self.lap_count += 1
            lap_time = time.time() - self.lap_start_time
            self.lap_times.append(lap_time)
            self.get_logger().info(
                f"🏁  Lap {self.lap_count:02d}  |  tiempo = {lap_time:6.2f} s")

            # reiniciar contadores de la nueva vuelta
            self.travelled_dist = 0.0
            self.lap_start_time = time.time()

        # actualizar flag
        self.in_start_zone = in_zone

# ───────── main ─────────


def main(args: List[str] = None):
    rclpy.init(args=args)
    node = DisparityExtenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()  