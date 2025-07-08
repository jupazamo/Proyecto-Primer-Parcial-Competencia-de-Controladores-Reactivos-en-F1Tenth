#!/usr/bin/env python3
"""
LapCounterNode v3 – “gate method”

Detecta la entrada en un rectángulo pequeño (‘small gate’) y se rearma
cuando se abandona uno mayor (‘large gate’).  Sin dependencia de signo,
rumbo ni distancia recorrida.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class LapCounterNode(Node):
    def __init__(self):
        super().__init__('lap_counter')

        # ─── parámetros ───
        self.declare_parameter('finish_line_x',         0.0)   # centro de la meta
        self.declare_parameter('gate_tol_x',            0.15)  # ± en x  (pequeña)
        self.declare_parameter('gate_half_w',           0.40)  # ± en y  (pequeña)

        self.declare_parameter('rearm_tol_x',           1.00)  # ± en x  (grande)
        self.declare_parameter('rearm_half_w',          1.50)  # ± en y  (grande)

        self.declare_parameter('min_lap_time',          4.0)   # [s]  filtro

        p = lambda n: self.get_parameter(n).value
        self.x0          = float(p('finish_line_x'))
        self.gx          = float(p('gate_tol_x'))
        self.gy          = float(p('gate_half_w'))
        self.rx          = float(p('rearm_tol_x'))
        self.ry          = float(p('rearm_half_w'))
        self.min_dt      = float(p('min_lap_time'))

        # ─── estado ───
        self.armed               = True
        self.last_cross_stamp    = self.get_clock().now()
        self.lap_count           = 0
        self.last_lap_secs: Optional[float] = None
        self.best_lap_secs: Optional[float] = None
        self.total_secs          = 0.0

        # ─── ROS IO ───
        self.sub = self.create_subscription(Odometry, '/odom', self.cb_odom, 30)
        self.pub = self.create_publisher(String, '/lap_info', 10)
        self.get_logger().info('LapCounter v3 “gate method” inicializado.')

    # ──────────────────────────────────────────────────────────────

    def inside_rect(self, x, y, tol_x, half_w) -> bool:
        """¿Coord. dentro del rectángulo centrado en x0, ancho 2*tol_x × 2*half_w?"""
        return abs(x - self.x0) <= tol_x and abs(y) <= half_w

    # ──────────────────────────────────────────────────────────────

    def cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        now = self.get_clock().now()

        # ── cruce ──
        if self.armed and self.inside_rect(x, y, self.gx, self.gy):
            dt = (now - self.last_cross_stamp).nanoseconds * 1e-9
            if dt >= self.min_dt:                       # anti-rebote temporal
                self.lap_count += 1

                if self.lap_count > 1:                  # ≠ primera
                    self.last_lap_secs = dt
                    self.total_secs += dt
                    self.best_lap_secs = (dt if self.best_lap_secs is None
                                          else min(self.best_lap_secs, dt))

                self.last_cross_stamp = now
                self.armed = False                      # desarma hasta salir
                self.publish()
                lap_msg = (f"🏁  Vuelta {self.lap_count}  "
                           f"{'' if self.lap_count==1 else f't={dt:.2f}s'}")
                self.get_logger().info(lap_msg)

        # ── rearme ──
        if not self.armed and not self.inside_rect(x, y, self.rx, self.ry):
            self.armed = True

    # ──────────────────────────────────────────────────────────────

    def publish(self):
        out = String()
        avg = (self.total_secs / (self.lap_count - 1)) if self.lap_count > 1 else 0.0
        out.data = (f"total_laps: {self.lap_count}\n"
                    f"last_lap_time: {self.last_lap_secs or 0:.2f}\n"
                    f"avg_lap_time: {avg:.2f}\n"
                    f"best_lap_time: {self.best_lap_secs or 0:.2f}")
        self.pub.publish(out)


# ─── main ───
def main(args=None):
    rclpy.init(args=args)
    node = LapCounterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
