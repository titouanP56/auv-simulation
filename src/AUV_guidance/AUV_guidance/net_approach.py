#!/usr/bin/env python3
"""
net_approach.py
===============
Nœud ROS 2 de guidage pour la Phase 2 de la mission AUV : approche du filet
d'aquaculture.

Machine à états
---------------
  DESCENDING   → Descente à la profondeur cible et maintien.
  GLOBAL_SEARCH → Maintien en place pendant qu'un tour 360° complet du sonar
                  est effectué par ping360_nearest. L'AUV NE BOUGE PAS latérale-
                  ment : la perception accumule un tour complet et publie une
                  estimation robuste (signal /perception/full_scan_ready = True).
                  Dès que ce signal est reçu avec une orientation valide sur
                  /perception/net_orientation, on passe immédiatement en ALIGNING
                  sans attendre d'autres estimations ni calculer de médiane.
  ALIGNING     → PD sur le lacet jusqu'à l'alignement face au filet.
  APPROACHING  → Avance vers le filet jusqu'à la distance de standoff.
  STABILIZING  → Maintien de la position standoff pendant STABILIZE_TIME secondes.
  STANDOFF     → État final : publication continue de l'origine locale (Phase 3).

Modifications v2
----------------
- Suppression de l'état SCANNING et de son filtre médian (statistiques.median
  sur SCAN_MIN_ESTIMATES estimations).
- Nouvel état GLOBAL_SEARCH : transition immédiate vers ALIGNING dès la
  première estimation issue d'un tour complet (signalée par
  /perception/full_scan_ready).
- Souscription au topic /perception/full_scan_ready (std_msgs/Bool).

Topics ROS 2
------------
  Entrées :
    /odometry/filtered              → nav_msgs/Odometry
    /perception/net_orientation     → geometry_msgs/PoseStamped
    /perception/full_scan_ready     → std_msgs/Bool
    /sonoptix/points                → sensor_msgs/PointCloud2

  Sorties :
    /auv/command_wrench             → geometry_msgs/Wrench
    /mission/phase                  → std_msgs/String
    /mission/phase2_done            → std_msgs/Bool
    /mission/local_origin           → geometry_msgs/PoseStamped

Auteur  : titou
Package : AUV_guidance
"""

import math
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64, Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, PoseStamped, Wrench
from tf2_ros import TransformBroadcaster


# ── Constantes ─────────────────────────────────────────────────────────────────

TARGET_DEPTH      = -2.0    # [m] profondeur cible
DEPTH_TOLERANCE   = 0.2     # [m] tolérance d'erreur de profondeur
DEPTH_HOLD_TIME   = 2.0     # [s] durée de stabilisation de profondeur requise

YAW_TOLERANCE     = math.radians(10.0)   # [rad] tolérance d'erreur de lacet
YAW_HOLD_TIME     = 1.0                  # [s] durée de maintien d'alignement

STANDOFF_DIST     = 1.5     # [m] distance de standoff face au filet
APPROACH_TOL      = 0.10    # [m] tolérance d'atteinte du standoff
STABILIZE_TIME    = 3.0     # [s] temps de stabilisation avant fin de phase

# ── Gains P/PD ─────────────────────────────────────────────────────────────────
KP_DEPTH              = 15.0
BUOYANCY_COMPENSATION = 3.0    # [N] compensation statique de la flottabilité
KP_YAW                = 5.0
KD_YAW                = 2.0
KP_SURGE              = 6.0

# ── Limites de commande ────────────────────────────────────────────────────────
MAX_DEPTH_CMD  = 20.0   # [N]
MAX_YAW_CMD    = 40.0   # [N·m]
MAX_SURGE_CMD  = 25.0   # [N]

# ── Sonoptix (sonar avant pour la distance au filet) ──────────────────────────
SONOPTIX_BORESIGHT_HALF_ANGLE = math.radians(20.0)

# ── Contrôle ──────────────────────────────────────────────────────────────────
CONTROL_RATE_HZ = 10.0

# ── Timeout GLOBAL_SEARCH ─────────────────────────────────────────────────────
# Si aucune estimation n'est reçue après ce délai, on affiche une alerte et
# on réarme le timer (sans blocage définitif).
GLOBAL_SEARCH_TIMEOUT_SEC = 60.0   # [s]


# ── États de la machine à états ───────────────────────────────────────────────

class State:
    DESCENDING    = "DESCENDING"
    GLOBAL_SEARCH = "GLOBAL_SEARCH"   # Remplace SCANNING : attend 1 tour complet
    ALIGNING      = "ALIGNING"
    APPROACHING   = "APPROACHING"
    STABILIZING   = "STABILIZING"
    STANDOFF      = "STANDOFF"


# ── Fonctions utilitaires ─────────────────────────────────────────────────────

def _yaw_from_odom(odom: Odometry) -> float:
    """Extrait le yaw courant depuis la quaternion de l'odométrie."""
    q = odom.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a: float, b: float) -> float:
    """Différence d'angles normalisée dans [-π, π]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _min_sonoptix_range(msg: PointCloud2) -> float | None:
    """
    Extrait la distance minimale du nuage de points Sonoptix dans le cône
    frontal du robot (±SONOPTIX_BORESIGHT_HALF_ANGLE).

    Retourne None si aucun point valide n'est trouvé dans le cône.
    """
    field_map = {f.name: f for f in msg.fields}
    if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
        return None

    x_off      = field_map['x'].offset
    y_off      = field_map['y'].offset
    z_off      = field_map['z'].offset
    point_step = msg.point_step
    data       = msg.data

    min_range = float('inf')
    for i in range(msg.width * msg.height):
        base = i * point_step
        try:
            px = struct.unpack_from('f', data, base + x_off)[0]
            py = struct.unpack_from('f', data, base + y_off)[0]
            pz = struct.unpack_from('f', data, base + z_off)[0]
        except struct.error:
            continue

        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
            continue

        horiz_angle = abs(math.atan2(py, px))
        if horiz_angle > SONOPTIX_BORESIGHT_HALF_ANGLE:
            continue

        r = math.sqrt(px * px + py * py + pz * pz)
        if r > 0.01:
            min_range = min(min_range, r)

    return min_range if math.isfinite(min_range) else None


# ── Nœud principal ─────────────────────────────────────────────────────────────

class Phase2MissionNode(Node):
    """
    Nœud ROS 2 de guidage pour la Phase 2 de la mission AUV (approche filet).

    Machine à états :
      DESCENDING → GLOBAL_SEARCH → ALIGNING → APPROACHING → STABILIZING → STANDOFF

    Différences par rapport à la version précédente :
    - L'état SCANNING est remplacé par GLOBAL_SEARCH.
    - Pas de filtre médian : la transition vers ALIGNING se fait dès la
      première estimation robuste (tour complet, signalée par le topic Bool
      /perception/full_scan_ready).
    - Le yaw cible est extrait directement du PoseStamped de la perception.
    """

    def __init__(self) -> None:
        super().__init__('phase2_mission')

        # ── QoS ─────────────────────────────────────────────────────────────
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Souscriptions ───────────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self._odom_cb,
            10,
        )

        # Orientation du filet (PoseStamped) depuis ping360_nearest
        self.ping360_sub = self.create_subscription(
            PoseStamped,
            '/perception/net_orientation',
            self._net_orientation_cb,
            best_effort_qos,
        )

        # Signal "tour complet prêt" depuis ping360_nearest
        # Reçoit True quand une estimation fraîche 360° vient d'être publiée.
        self.full_scan_sub = self.create_subscription(
            Bool,
            '/perception/full_scan_ready',
            self._full_scan_ready_cb,
            best_effort_qos,
        )

        self.sonoptix_sub = self.create_subscription(
            PointCloud2,
            '/sonoptix/points',
            self._sonoptix_cb,
            best_effort_qos,
        )

        # ── Publications ─────────────────────────────────────────────────────
        self.wrench_pub = self.create_publisher(Wrench, '/auv/command_wrench', 10)

        latching_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.phase_pub  = self.create_publisher(String,      '/mission/phase',        10)
        self.done_pub   = self.create_publisher(Bool,        '/mission/phase2_done',  latching_qos)
        self.origin_pub = self.create_publisher(PoseStamped, '/mission/local_origin', latching_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── État interne ─────────────────────────────────────────────────────
        self.state: str = State.DESCENDING

        # Odométrie
        self.current_x    = 0.0
        self.current_y    = 0.0
        self.current_z    = 0.0
        self.current_yaw  = 0.0
        self.current_vyaw = 0.0

        # Cible de lacet (déterminée en GLOBAL_SEARCH, utilisée en ALIGNING+)
        self.target_yaw: float = 0.0

        # Distance Sonoptix (mise à jour en APPROACHING et STABILIZING)
        self.sonoptix_range: float | None = None

        # Drapeaux de disponibilité des données
        self._have_odom:   bool = False
        self._have_orient: bool = False   # True dès réception d'une orientation valide

        # ── État GLOBAL_SEARCH ───────────────────────────────────────────────
        # _pending_yaw      : dernière orientation reçue de ping360_nearest,
        #                     en attente de la confirmation "full_scan_ready".
        # _pending_yaw_valid: True si _pending_yaw a été rempli.
        # _full_scan_flag   : True si le topic /perception/full_scan_ready
        #                     vient de signaler un tour complet.
        # _search_start_time: timestamp de début de GLOBAL_SEARCH (pour timeout).
        self._pending_yaw:        float       = 0.0
        self._pending_yaw_valid:  bool        = False
        self._full_scan_flag:     bool        = False
        self._search_start_time:  float | None = None
        # Timestamp de la première orientation reçue (pour le fallback)
        self._orient_received_at: float | None = None

        # Timers de stabilisation
        self._depth_ok_since:    float | None = None
        self._yaw_ok_since:      float | None = None
        self._stabilize_start:   float | None = None

        # ── Timer de contrôle ─────────────────────────────────────────────────
        self.declare_parameter('control_rate_hz', CONTROL_RATE_HZ)
        _rate = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / _rate, self._control_loop)

        self._publish_done_status()
        self.get_logger().info(
            f"[Phase2MissionNode] Démarré → état initial : DESCENDING "
            f"(taux de contrôle = {_rate:.0f} Hz)\n"
            f"  Profondeur cible : {TARGET_DEPTH} m\n"
            f"  Standoff         : {STANDOFF_DIST} m\n"
            f"  Timeout recherche: {GLOBAL_SEARCH_TIMEOUT_SEC} s"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        """Met à jour la pose et la vitesse de lacet courantes."""
        self.current_x    = msg.pose.pose.position.x
        self.current_y    = msg.pose.pose.position.y
        self.current_z    = msg.pose.pose.position.z
        self.current_yaw  = _yaw_from_odom(msg)
        self.current_vyaw = msg.twist.twist.angular.z
        self._have_odom   = True

    def _net_orientation_cb(self, msg: PoseStamped) -> None:
        """
        Callback pour /perception/net_orientation (depuis ping360_nearest).

        On stocke le yaw en tant qu'estimation "en attente". La transition
        vers ALIGNING ne sera déclenchée que si _full_scan_flag est aussi
        True (i.e., le signal /perception/full_scan_ready a été reçu).

        On ignore les messages reçus dans des états autres que GLOBAL_SEARCH
        afin d'éviter de modifier le yaw cible une fois l'alignement commencé.
        """
        if self.state != State.GLOBAL_SEARCH:
            return

        # Extraction du yaw depuis le quaternion (roll = pitch = 0)
        q = msg.pose.orientation
        world_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self._pending_yaw       = world_yaw
        self._pending_yaw_valid = True

        self.get_logger().debug(
            f"[GLOBAL_SEARCH] Orientation reçue : "
            f"yaw_filet={math.degrees(world_yaw):.1f}°  "
            f"pt_filet=({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})  "
            f"full_scan_ready={self._full_scan_flag}"
        )

        # Tentative de transition immédiate si le signal "tour complet" est
        # déjà arrivé avant ce message (cas possible selon l'ordre de livraison)
        self._try_transition_to_aligning()

    def _full_scan_ready_cb(self, msg: Bool) -> None:
        """
        Callback pour /perception/full_scan_ready.

        Reçoit True quand ping360_nearest a terminé un tour complet et a
        publié une estimation fraîche. On lève le drapeau _full_scan_flag
        puis on tente la transition.
        """
        if not msg.data:
            return   # on ignore les False

        if self.state != State.GLOBAL_SEARCH:
            return

        self._full_scan_flag = True
        self.get_logger().info(
            "[GLOBAL_SEARCH] Signal 'full_scan_ready' reçu → "
            f"estimation valide = {self._pending_yaw_valid}"
        )

        self._try_transition_to_aligning()

    def _try_transition_to_aligning(self) -> None:
        """
        Déclenche la transition GLOBAL_SEARCH → ALIGNING dès qu'une orientation
        valide est disponible.

        On ne bloque plus sur _full_scan_flag : ping360_nearest publie les deux
        topics (/perception/net_orientation et /perception/full_scan_ready) de
        manière quasi-simultanée. Attendre les deux créait un possible deadlock
        si l'un des deux messages était perdu (QoS Best-Effort).

        Le fait que ping360_nearest exécute son pipeline uniquement à la fin
        d'un tour (ou toutes les _min_period_sec secondes avec le fallback)
        garantit déjà la qualité de l'estimation.
        """
        if self._pending_yaw_valid:
            self.target_yaw = self._pending_yaw
            self.get_logger().info(
                f"[GLOBAL_SEARCH → ALIGNING] Estimation reçue. "
                f"Yaw cible : {math.degrees(self.target_yaw):.1f}°  "
                f"(full_scan_flag={self._full_scan_flag})"
            )
            self.state = State.ALIGNING

            # Réinitialisation des flags
            self._pending_yaw_valid = False
            self._full_scan_flag    = False
            self._search_start_time = None

    def _sonoptix_cb(self, msg: PointCloud2) -> None:
        """
        Met à jour la distance au filet via le sonar Sonoptix frontal.
        N'est actif qu'en phase d'approche et de stabilisation.
        """
        if self.state in (State.APPROACHING, State.STABILIZING, State.STANDOFF):
            self.sonoptix_range = _min_sonoptix_range(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Boucle de contrôle principale
    # ─────────────────────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        """Boucle de contrôle cadencée à CONTROL_RATE_HZ Hz."""
        if not self._have_odom:
            return

        self._publish_state()
        self._publish_done_status()

        if self.state == State.DESCENDING:
            self._do_descending()
        elif self.state == State.GLOBAL_SEARCH:
            self._do_global_search()
        elif self.state == State.ALIGNING:
            self._do_aligning()
        elif self.state == State.APPROACHING:
            self._do_approaching()
        elif self.state == State.STABILIZING:
            self._do_stabilizing()
        elif self.state == State.STANDOFF:
            self._do_standoff()

    # ─────────────────────────────────────────────────────────────────────────
    # Handlers des états
    # ─────────────────────────────────────────────────────────────────────────

    def _do_descending(self) -> None:
        """
        Descend à TARGET_DEPTH et passe en GLOBAL_SEARCH une fois la profondeur
        stabilisée pendant DEPTH_HOLD_TIME secondes.
        """
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = float(np.clip(
            KP_DEPTH * depth_error - BUOYANCY_COMPENSATION,
            -MAX_DEPTH_CMD, MAX_DEPTH_CMD,
        ))
        self._set_Fz(depth_cmd)
        self._set_Mz(0.0)
        self._set_Fx(0.0)

        now = self.get_clock().now().nanoseconds * 1e-9
        if abs(depth_error) < DEPTH_TOLERANCE:
            if self._depth_ok_since is None:
                self._depth_ok_since = now
            elif (now - self._depth_ok_since) >= DEPTH_HOLD_TIME:
                self.get_logger().info(
                    f"[DESCENDING → GLOBAL_SEARCH] Profondeur stable : "
                    f"{self.current_z:.2f} m"
                )
                self.state = State.GLOBAL_SEARCH
                self._depth_ok_since = None
        else:
            self._depth_ok_since = None

    def _do_global_search(self) -> None:
        """
        Maintient l'AUV en place (profondeur + cap nul) pendant que le sonar
        Ping360 effectue un tour complet.

        La transition vers ALIGNING est déclenchée par les callbacks
        (_net_orientation_cb et _full_scan_ready_cb) dès que les deux
        conditions sont réunies.

        Un timeout GLOBAL_SEARCH_TIMEOUT_SEC est utilisé pour afficher
        une alerte si aucune estimation n'est reçue (nœud perception
        non démarré, TF manquant…).
        """
        # Maintien de profondeur strict (Fx = 0 : pas de translation avant)
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = float(np.clip(
            KP_DEPTH * depth_error - BUOYANCY_COMPENSATION,
            -MAX_DEPTH_CMD, MAX_DEPTH_CMD,
        ))
        self._set_Fz(depth_cmd)
        self._set_Mz(0.0)   # pas de rotation non plus : le sonar balaie seul
        self._set_Fx(0.0)

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._search_start_time is None:
            self._search_start_time = now
            self.get_logger().info(
                "[GLOBAL_SEARCH] En attente d'une estimation robuste "
                "depuis ping360_nearest (tour complet 360°)…\n"
                "  L'AUV maintient sa position. Ne pas commander de mouvement."
            )

        elapsed = now - self._search_start_time

        # ── Timeout guard ─────────────────────────────────────────────────────
        if elapsed >= GLOBAL_SEARCH_TIMEOUT_SEC and not self._pending_yaw_valid:
            self.get_logger().error(
                f"[GLOBAL_SEARCH] Timeout ({GLOBAL_SEARCH_TIMEOUT_SEC:.0f} s) : "
                "aucune estimation reçue depuis /perception/net_orientation. "
                "Vérifiez que ping360_nearest est actif et que TF2 est disponible."
            )
            # Réarme le timer sans bloquer définitivement
            self._search_start_time = now

    def _do_aligning(self) -> None:
        """
        Contrôle PD du lacet jusqu'à l'alignement face au filet.

        La profondeur est maintenue ; aucun mouvement d'avance.
        Passe en APPROACHING une fois le lacet stable pendant YAW_HOLD_TIME s.
        """
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = float(np.clip(
            KP_DEPTH * depth_error - BUOYANCY_COMPENSATION,
            -MAX_DEPTH_CMD, MAX_DEPTH_CMD,
        ))
        self._set_Fz(depth_cmd)
        self._set_Fx(0.0)

        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        mz_cmd    = float(np.clip(
            KP_YAW * yaw_error - KD_YAW * self.current_vyaw,
            -MAX_YAW_CMD, MAX_YAW_CMD,
        ))
        self._set_Mz(mz_cmd)

        now = self.get_clock().now().nanoseconds * 1e-9
        if abs(yaw_error) < YAW_TOLERANCE:
            if self._yaw_ok_since is None:
                self._yaw_ok_since = now
            elif (now - self._yaw_ok_since) >= YAW_HOLD_TIME:
                self.get_logger().info(
                    f"[ALIGNING → APPROACHING] Aligné : "
                    f"erreur lacet = {math.degrees(yaw_error):.2f}°"
                )
                self.state = State.APPROACHING
                self._yaw_ok_since = None
        else:
            self._yaw_ok_since = None

    def _do_approaching(self) -> None:
        """
        Avance vers le filet en maintenant le cap et la profondeur.

        Si le Sonoptix n'est pas encore disponible, avance à 30 % de la
        poussée maximale (mode aveugle temporaire).
        Passe en STABILIZING lorsque la distance de standoff est atteinte.
        """
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = float(np.clip(
            KP_DEPTH * depth_error - BUOYANCY_COMPENSATION,
            -MAX_DEPTH_CMD, MAX_DEPTH_CMD,
        ))
        self._set_Fz(depth_cmd)

        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        mz_cmd    = float(np.clip(
            KP_YAW * yaw_error - KD_YAW * self.current_vyaw,
            -MAX_YAW_CMD, MAX_YAW_CMD,
        ))
        self._set_Mz(mz_cmd)

        if self.sonoptix_range is None:
            # Pas encore de mesure de distance → avance lente
            self._set_Fx(MAX_SURGE_CMD * 0.3)
            return

        surge_cmd = float(np.clip(
            KP_SURGE * (self.sonoptix_range - STANDOFF_DIST),
            0.0, MAX_SURGE_CMD,
        ))
        self._set_Fx(surge_cmd)

        if (self.sonoptix_range - STANDOFF_DIST) <= APPROACH_TOL:
            self.get_logger().info(
                f"[APPROACHING → STABILIZING] Distance standoff atteinte : "
                f"{self.sonoptix_range:.2f} m. Stabilisation…"
            )
            self.state = State.STABILIZING
            self._stabilize_start = self.get_clock().now().nanoseconds * 1e-9

    def _do_stabilizing(self) -> None:
        """
        Maintient la position standoff pendant STABILIZE_TIME secondes.
        Passe ensuite en STANDOFF et publie l'origine locale pour la Phase 3.
        """
        depth_error = TARGET_DEPTH - self.current_z
        depth_cmd   = float(np.clip(
            KP_DEPTH * depth_error - BUOYANCY_COMPENSATION,
            -MAX_DEPTH_CMD, MAX_DEPTH_CMD,
        ))
        self._set_Fz(depth_cmd)

        yaw_error = _angle_diff(self.target_yaw, self.current_yaw)
        mz_cmd    = float(np.clip(
            KP_YAW * yaw_error - KD_YAW * self.current_vyaw,
            -MAX_YAW_CMD, MAX_YAW_CMD,
        ))
        self._set_Mz(mz_cmd)

        if self.sonoptix_range is not None:
            surge_cmd = float(np.clip(
                KP_SURGE * (self.sonoptix_range - STANDOFF_DIST),
                -MAX_SURGE_CMD, MAX_SURGE_CMD,
            ))
            self._set_Fx(surge_cmd)
        else:
            self._set_Fx(0.0)

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._stabilize_start and (now - self._stabilize_start) >= STABILIZE_TIME:
            self.get_logger().info(
                "[STABILIZING → STANDOFF] Robot stabilisé. Définition de l'origine locale."
            )
            self._define_local_origin()
            self.state = State.STANDOFF

    def _do_standoff(self) -> None:
        """
        État final : publication continue de l'origine locale et du TF
        pour la Phase 3 d'inspection.
        """
        if hasattr(self, '_local_origin_pose'):
            self._local_origin_pose.header.stamp = self.get_clock().now().to_msg()
            self.origin_pub.publish(self._local_origin_pose)

        if hasattr(self, '_local_origin_transform'):
            self._local_origin_transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self._local_origin_transform)

    # ─────────────────────────────────────────────────────────────────────────
    # Définition de l'origine locale (transition vers Phase 3)
    # ─────────────────────────────────────────────────────────────────────────

    def _define_local_origin(self) -> None:
        """
        Calcule et stocke l'origine du repère local (local_origin) utilisé
        par la Phase 3. L'origine est placée au point standoff projeté sur
        le filet, face à l'AUV.
        """
        origin_x   = self.current_x + STANDOFF_DIST * math.cos(self.target_yaw)
        origin_y   = self.current_y + STANDOFF_DIST * math.sin(self.target_yaw)
        origin_z   = self.current_z
        origin_yaw = self.target_yaw + math.pi   # face à l'AUV

        self.get_logger().info(
            f"[PHASE 3] Origine locale : "
            f"x={origin_x:.2f}  y={origin_y:.2f}  z={origin_z:.2f}  "
            f"yaw={math.degrees(origin_yaw):.1f}°"
        )

        # ── TF ────────────────────────────────────────────────────────────────
        tf_msg = TransformStamped()
        tf_msg.header.frame_id    = 'odom'
        tf_msg.child_frame_id     = 'local_origin'
        tf_msg.transform.translation.x = origin_x
        tf_msg.transform.translation.y = origin_y
        tf_msg.transform.translation.z = origin_z
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(origin_yaw / 2.0)
        tf_msg.transform.rotation.w = math.cos(origin_yaw / 2.0)
        self._local_origin_transform = tf_msg

        # ── PoseStamped ───────────────────────────────────────────────────────
        pose_msg = PoseStamped()
        pose_msg.header.frame_id         = 'odom'
        pose_msg.pose.position.x         = origin_x
        pose_msg.pose.position.y         = origin_y
        pose_msg.pose.position.z         = origin_z
        pose_msg.pose.orientation.x      = 0.0
        pose_msg.pose.orientation.y      = 0.0
        pose_msg.pose.orientation.z      = math.sin(origin_yaw / 2.0)
        pose_msg.pose.orientation.w      = math.cos(origin_yaw / 2.0)
        self._local_origin_pose = pose_msg

    # ─────────────────────────────────────────────────────────────────────────
    # Helpers de commande
    # ─────────────────────────────────────────────────────────────────────────

    def _set_Fz(self, fz: float) -> None:
        self._cmd_Fz = float(fz)

    def _set_Mz(self, mz: float) -> None:
        self._cmd_Mz = float(mz)

    def _set_Fx(self, fx: float) -> None:
        self._cmd_Fx = float(fx)

    # ─────────────────────────────────────────────────────────────────────────
    # Publications d'état
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_done_status(self) -> None:
        """Publie True sur /mission/phase2_done lorsque l'état STANDOFF est atteint."""
        done_msg = Bool()
        done_msg.data = (self.state == State.STANDOFF)
        self.done_pub.publish(done_msg)

    def _publish_state(self) -> None:
        """Publie le nom de l'état courant et les commandes Wrench calculées."""
        # En état STANDOFF, on ne publie plus de Wrench (contrôle MPC actif)
        if self.state == State.STANDOFF:
            phase_msg = String()
            phase_msg.data = self.state
            self.phase_pub.publish(phase_msg)
            return

        Fx = getattr(self, '_cmd_Fx', 0.0)
        Fz = getattr(self, '_cmd_Fz', 0.0)
        Mz = getattr(self, '_cmd_Mz', 0.0)

        wrench_msg = Wrench()
        wrench_msg.force.x  = Fx
        wrench_msg.force.y  = 0.0
        wrench_msg.force.z  = Fz
        wrench_msg.torque.z = Mz
        self.wrench_pub.publish(wrench_msg)

        # Remise à zéro des commandes pour éviter une répétition accidentelle
        self._cmd_Fx = 0.0
        self._cmd_Fz = 0.0
        self._cmd_Mz = 0.0

        phase_msg = String()
        phase_msg.data = self.state
        self.phase_pub.publish(phase_msg)


# ── Point d'entrée ─────────────────────────────────────────────────────────────

def main(args=None) -> None:
    """Point d'entrée standard ROS 2."""
    rclpy.init(args=args)
    node = Phase2MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[Phase2MissionNode] Interruption clavier — arrêt propre.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()