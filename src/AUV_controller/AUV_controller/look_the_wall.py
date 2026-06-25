"""
look_the_wall.py — Guidage et maintien de position face à un mur (Ping360).

Ce nœud implémente une machine d'états simple (10 Hz) pour :
  1. Attendre un scan complet du Ping360 (WAITING_FOR_SCAN).
  2. Orienter le robot face à la normale du filet (ORIENTATION_ALIGNMENT).
  3. Maintenir la distance et l'alignement face au mur (STATION_KEEPING).

Entrées :
  /perception/full_scan_ready  (std_msgs/msg/Bool)
  /perception/net_orientation  (geometry_msgs/msg/PoseStamped)
  /odometry/filtered           (nav_msgs/msg/Odometry)

Sorties :
  /cmd_vel_1 … /cmd_vel_8     (std_msgs/msg/Float64)

Paramètres ROS 2 :
  Kp_yaw   (float, défaut 5.0) — gain P pour l'asservissement en cap
  Kp_dist  (float, défaut 2.0) — gain P pour l'asservissement en distance
"""

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import numpy as np

# ── Thruster allocation (même configuration que station_keeping.py) ────────────
THRUST_COEFFS = [-0.002, 0.002, 0.002, -0.002, -0.002, 0.002, 0.002, -0.002]

SIN45 = 0.7071
LEVER = 0.1697

TAM = np.array([
    [ SIN45,  SIN45, -SIN45, -SIN45,  0.0,  0.0,  0.0,  0.0],  # Fx (surge)
    [ SIN45, -SIN45,  SIN45, -SIN45,  0.0,  0.0,  0.0,  0.0],  # Fy (sway)
    [ 0.0,    0.0,    0.0,    0.0,   -1.0,  1.0,  1.0, -1.0],  # Fz (heave)
    [ 0.0,    0.0,    0.0,    0.0,   0.218,0.218,0.218,0.218],  # Mx (roll)
    [ 0.0,    0.0,    0.0,    0.0,   0.12,-0.12, 0.12,-0.12],  # My (pitch)
    [LEVER,  -LEVER, -LEVER,  LEVER,  0.0,  0.0,  0.0,  0.0],  # Mz (yaw)
])
TAM_PINV = np.linalg.pinv(TAM)

MAX_FORCE   = 20.0  # N
MAX_TORQUE  = 8.0   # N·m
MAX_THRUST  = 5.0   # N par propulseur


def euler_from_quaternion(x, y, z, w) -> float:
    """Retourne uniquement le Yaw (rad) à partir d'un quaternion."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def normalize_angle(angle: float) -> float:
    """Normalise un angle dans [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class LookTheWallNode(Node):
    def __init__(self):
        super().__init__('look_the_wall')

        # ── Paramètres ─────────────────────────────────────────────────────────
        self.declare_parameter('Kp_yaw',  5.0)
        self.declare_parameter('Kp_dist', 2.0)

        # ── Variables d'état ───────────────────────────────────────────────────
        # 0 : WAITING_FOR_SCAN
        # 1 : ORIENTATION_ALIGNMENT
        # 2 : STATION_KEEPING
        self.state = 0

        self.full_scan_ready  = False
        self.latest_pose: PoseStamped | None = None
        self.current_yaw      = 0.0   # cap actuel du robot (rad), mis à jour par odométrie
        self.target_distance  = 0.0   # distance de consigne mémorisée à l'entrée de l'état 2

        # ── Subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(Bool,        '/perception/full_scan_ready', self._cb_ready, 10)
        self.create_subscription(PoseStamped, '/perception/net_orientation', self._cb_pose,  10)
        self.create_subscription(Odometry,    '/odometry/filtered',          self._cb_odom,  10)

        # ── Publishers (8 propulseurs) ─────────────────────────────────────────
        self.thrust_pubs = [
            self.create_publisher(Float64, f'/cmd_vel_{i}', 10)
            for i in range(1, 9)
        ]

        # ── Timer de contrôle @ 10 Hz ──────────────────────────────────────────
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            "Nœud look_the_wall démarré — état : WAITING_FOR_SCAN (0)"
        )

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _cb_ready(self, msg: Bool):
        self.full_scan_ready = msg.data

    def _cb_pose(self, msg: PoseStamped):
        self.latest_pose = msg

    def _cb_odom(self, msg: Odometry):
        o = msg.pose.pose.orientation
        self.current_yaw = euler_from_quaternion(o.x, o.y, o.z, o.w)

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _pose_distance(self, pose: PoseStamped) -> float:
        x = pose.pose.position.x
        y = pose.pose.position.y
        return math.sqrt(x * x + y * y)

    def _pose_yaw(self, pose: PoseStamped) -> float:
        o = pose.pose.orientation
        return euler_from_quaternion(o.x, o.y, o.z, o.w)

    def _publish_wrench(self, Fx: float, Fy: float, Fz: float, Mz: float):
        """Convertit un wrench [Fx, Fy, Fz, Mz] en commandes de propulseurs."""
        Fx = float(np.clip(Fx, -MAX_FORCE,  MAX_FORCE))
        Fy = float(np.clip(Fy, -MAX_FORCE,  MAX_FORCE))
        Fz = float(np.clip(Fz, -MAX_FORCE,  MAX_FORCE))
        Mz = float(np.clip(Mz, -MAX_TORQUE, MAX_TORQUE))

        tau    = np.array([Fx, Fy, Fz, 0.0, 0.0, Mz])
        thrusts = np.clip(TAM_PINV @ tau, -MAX_THRUST, MAX_THRUST)

        for i, (thrust, coeff) in enumerate(zip(thrusts, THRUST_COEFFS)):
            msg      = Float64()
            msg.data = float(thrust) * math.copysign(1.0, coeff)
            self.thrust_pubs[i].publish(msg)

    def _stop(self):
        """Arrête tous les propulseurs."""
        for pub in self.thrust_pubs:
            msg = Float64()
            msg.data = 0.0
            pub.publish(msg)

    # ── Boucle de contrôle ─────────────────────────────────────────────────────

    def _control_loop(self):
        Kp_yaw  = self.get_parameter('Kp_yaw').value
        Kp_dist = self.get_parameter('Kp_dist').value

        # ── ÉTAT 0 : WAITING_FOR_SCAN ──────────────────────────────────────────
        if self.state == 0:
            self._stop()
            if self.full_scan_ready and self.latest_pose is not None:
                self.target_distance = self._pose_distance(self.latest_pose)
                self.state = 1
                self.get_logger().info(
                    f"[→ ÉTAT 1] Scan valide reçu. "
                    f"Distance de consigne : {self.target_distance:.2f} m"
                )
            return

        # Sécurité : perte du filet → retour à 0
        if not self.full_scan_ready:
            self.get_logger().warn("[→ ÉTAT 0] Perte du signal full_scan_ready.")
            self.state = 0
            self._stop()
            return

        if self.latest_pose is None:
            self._stop()
            return

        # Cap cible de la normale au filet (dans le repère odom)
        target_yaw = self._pose_yaw(self.latest_pose)

        # Erreur de cap = différence entre cap cible et cap actuel du robot
        error_yaw  = normalize_angle(target_yaw - self.current_yaw)

        # Commande de couple en yaw (repère body)
        Mz = Kp_yaw * error_yaw

        # ── ÉTAT 1 : ORIENTATION_ALIGNMENT ────────────────────────────────────
        if self.state == 1:
            self._publish_wrench(0.0, 0.0, 0.0, Mz)
            self.get_logger().info(
                f"[ÉTAT 1] Erreur cap : {math.degrees(error_yaw):.2f}°  "
                f"robot_yaw={math.degrees(self.current_yaw):.1f}°  "
                f"cible={math.degrees(target_yaw):.1f}°"
            )
            if abs(error_yaw) < 0.05:   # ≈ 3°
                self.state = 2
                self.get_logger().info("[→ ÉTAT 2] Alignement terminé → STATION_KEEPING")
            return

        # ── ÉTAT 2 : STATION_KEEPING ───────────────────────────────────────────
        if self.state == 2:
            current_dist = self._pose_distance(self.latest_pose)
            error_dist   = current_dist - self.target_distance

            # Avance/recule dans le repère body (X = avant du robot)
            # Rotation de l'erreur monde → body
            dx_world = self.latest_pose.pose.position.x
            dy_world = self.latest_pose.pose.position.y
            cos_y    = math.cos(self.current_yaw)
            sin_y    = math.sin(self.current_yaw)
            # Composante frontale (X body) de la direction vers le filet
            fwd_body = cos_y * dx_world + sin_y * dy_world
            sign_fwd = math.copysign(1.0, fwd_body) if abs(fwd_body) > 0.01 else 1.0
            Fx = Kp_dist * error_dist * sign_fwd

            self._publish_wrench(Fx, 0.0, 0.0, Mz)
            self.get_logger().info(
                f"[ÉTAT 2] Dist: {current_dist:.2f}m (err={error_dist:+.2f}m)  "
                f"Yaw err: {math.degrees(error_yaw):.2f}°  "
                f"Fx={Fx:.2f} N  Mz={Mz:.2f} N·m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LookTheWallNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt du nœud.")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
