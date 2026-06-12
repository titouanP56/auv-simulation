#!/usr/bin/env python3
"""
ping360_nearest.py
==================
Nœud ROS 2 de perception avancée pour le sonar mécanique Ping360.

Pipeline de traitement (version 3 – sélection par ratio d'inliers RANSAC)
---------------------------------------------------------------------------
  1. Réception des balayages (LaserScan) depuis /ping360/scan.
  2. Pour chaque point valide du scan, transformation TF2 du repère
     local du capteur (ping360_link) vers le repère fixe (odom) afin
     de compenser les mouvements du robot pendant l'accumulation.
  3. Accumulation dans un buffer à fenêtre temporelle correspondant à
     UN tour complet du sonar (durée = angle_range / vitesse_angulaire).
     L'AUV se maintient en place pendant ce tour (pas de mouvement avance).
  4. À la fin d'un tour complet (nouvelle estimation "fraîche") :
       a. DBSCAN  : clustering des points pour isoler les entités.
       b. Sélection par ratio d'inliers RANSAC : pour chaque cluster,
          on ajuste un polynôme deg-2 et on mesure le ratio d'inliers.
          Un filet (même très courbé) colle à la courbe → ratio élevé.
          Un banc de poissons volumétrique → ratio très faible.
          On sélectionne le cluster au meilleur ratio ≥ ransac_min_inlier_ratio.
       c. Point le plus proche : on cherche le point de la courbe RANSAC
          le plus proche de l'AUV, on calcule la normale à la tangente.
       d. Publication du résultat : yaw cible sous forme de Quaternion
          dans un PoseStamped sur /perception/net_orientation.
          → Flag "full_rotation_complete" dans un champ Bool séparé
            pour signaler à net_approach.py qu'une estimation robuste
            (tour complet) vient d'être produite.

Paramètres ROS 2 déclarés (tous configurables via CLI / YAML)
--------------------------------------------------------------
  sonar_topic             : topic LaserScan d'entrée   (défaut: /ping360/scan)
  output_topic            : topic PoseStamped de sortie (défaut: /perception/net_orientation)
  ready_topic             : topic Bool "tour complet"   (défaut: /perception/full_scan_ready)
  source_frame            : repère du capteur           (défaut: ping360_link)
  target_frame            : repère fixe de référence    (défaut: odom)
  window_sec              : durée de la fenêtre (0 = auto depuis scan) (défaut: 0.0)
  ignore_fraction         : fraction de range_max pour rejeter les échos saturés (défaut: 0.95)
  min_range_m             : distance minimale valide en mètres (défaut: 0.3)
  dbscan_eps              : rayon de voisinage DBSCAN en mètres  (défaut: 0.25)
  dbscan_min_samples      : échantillons minimum pour un cluster  (défaut: 5)
  ransac_min_inlier_ratio : ratio inliers/cluster minimum pour valider "filet" (défaut: 0.30)
  ransac_residual         : seuil de résidu RANSAC en mètres      (défaut: 0.1)
  ransac_min_samples      : nombre de points minimum pour RANSAC  (défaut: 10)
  min_cluster_pts         : taille minimale du cluster candidat   (défaut: 15)
  max_range_m             : distance maximale absolue acceptée [m] (défaut: 5.0)
                            Permet d'isoler le filet des parois du bassin.
                            Mettre à 0.0 pour désactiver (utilise ignore_fraction).

Auteur  : titou
Package : auv_perception
Topics  : entrée → /ping360/scan  |  sortie → /perception/net_orientation
                                             → /perception/full_scan_ready
"""

# ── Bibliothèques standard ────────────────────────────────────────────────────
import math
from collections import deque

# ── Calcul scientifique ───────────────────────────────────────────────────────
import numpy as np
from sklearn.cluster import DBSCAN

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ── Messages & TF2 ────────────────────────────────────────────────────────────
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — enregistre PointStamped dans tf2


# ── Constantes par défaut (surchargées par les paramètres ROS 2) ──────────────

_DEFAULT_SONAR_TOPIC       = "/ping360/scan"
_DEFAULT_OUTPUT_TOPIC      = "/perception/net_orientation"
_DEFAULT_READY_TOPIC       = "/perception/full_scan_ready"
_DEFAULT_SOURCE_FRAME      = "ping360_link"
_DEFAULT_TARGET_FRAME      = "odom"
_DEFAULT_WINDOW_SEC        = 0.0    # 0 = durée automatique d'un tour complet
_DEFAULT_IGNORE_FRAC       = 0.95   # ignore les échos à > 95 % de range_max
_DEFAULT_MIN_RANGE_M       = 0.3    # zone morte proche du robot
_DEFAULT_DBSCAN_EPS        = 0.25   # [m] rayon de voisinage DBSCAN
_DEFAULT_DBSCAN_MIN_PTS    = 5      # points minimum pour constituer un cluster
_DEFAULT_RANSAC_MIN_INLIER = 0.30   # ratio inliers/cluster minimum pour valider "filet"
_DEFAULT_RANSAC_RESID      = 0.10   # [m] seuil de résidu RANSAC
_DEFAULT_RANSAC_MIN_PTS    = 10     # points minimum pour l'ajustement polynomial
_DEFAULT_MIN_CLUSTER       = 15     # taille minimale d'un cluster candidat
_DEFAULT_MAX_RANGE_M       = 5.0    # [m] coupure absolue — isole filet (3-5m) des parois (>5m)
                                    # mettre à 0.0 pour désactiver


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """
    Convertit un angle yaw (rad) en quaternion (x, y, z, w).
    Roll et Pitch sont supposés nuls (plan horizontal).
    """
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)



def _poly2_closest_point(coeffs: np.ndarray, pts: np.ndarray) -> tuple[float, float]:
    """
    Trouve le point de la parabole y = a*x^2 + b*x + c le plus proche
    du centroïde des points du cluster.

    La recherche est effectuée par échantillonnage dense sur la plage X du
    cluster, ce qui est suffisamment précis pour des données sonar (~mm).

    Args:
        coeffs : [a, b, c] du polynôme (degré 2, axe X principal).
        pts    : nuage de points du cluster (N, 2).

    Returns:
        (x_closest, y_closest) sur la courbe.
    """
    x_min, x_max = float(pts[:, 0].min()), float(pts[:, 0].max())
    # 500 échantillons sur la plage couverte par le cluster
    x_samples = np.linspace(x_min, x_max, 500)
    y_samples = np.polyval(coeffs, x_samples)

    centroid = pts.mean(axis=0)   # position de référence = centroïde du cluster

    dist2 = (x_samples - centroid[0]) ** 2 + (y_samples - centroid[1]) ** 2
    idx = int(np.argmin(dist2))
    return float(x_samples[idx]), float(y_samples[idx])


def _poly2_normal_yaw(coeffs: np.ndarray, x0: float, toward_origin: np.ndarray) -> float:
    """
    Calcule l'angle yaw du vecteur normal à la tangente de la parabole
    au point (x0, y(x0)), orienté vers l'origine (= vers l'AUV).

    La tangente en x0 a pour pente dy/dx = 2*a*x0 + b.
    La normale perpendiculaire est dans la direction (-dy/dx, 1) normalisée.

    Args:
        coeffs        : [a, b, c] du polynôme.
        x0            : abscisse du point d'intérêt sur la courbe.
        toward_origin : vecteur de référence indiquant la direction de l'AUV
                        (typiquement [0,0] - centroïde, non normalisé).

    Returns:
        yaw_rad : angle yaw (rad) du vecteur normal, orienté vers l'AUV.
    """
    a, b = float(coeffs[0]), float(coeffs[1])
    slope_tangent = 2.0 * a * x0 + b          # dy/dx en x0

    # Normal perpendiculaire à la tangente (rotation 90°)
    normal = np.array([-slope_tangent, 1.0])
    norm_mag = np.linalg.norm(normal)
    if norm_mag < 1e-9:
        normal = np.array([0.0, 1.0])
    else:
        normal /= norm_mag

    # Orienter la normale vers l'AUV (en direction de toward_origin)
    if np.dot(normal, toward_origin) < 0:
        normal = -normal

    return math.atan2(normal[1], normal[0])


# ─────────────────────────────────────────────────────────────────────────────
# Nœud principal
# ─────────────────────────────────────────────────────────────────────────────

class Ping360NearestNode(Node):
    """
    Nœud ROS 2 de perception avancée du filet de pêche par sonar Ping360.

    Nouveautés v3 :
    - Fenêtre temporelle = durée d'UN tour complet du sonar (auto-calculée).
    - Sélection du cluster filet par ratio d'inliers RANSAC (robuste à la
      courbure forte du filet due aux courants — la PCA globale échouait).
    - RANSAC polynôme deg 2 pour modéliser la courbure du filet.
    - Point le plus proche sur la courbe + normale → yaw cible.
    - Publication d'un Bool sur /perception/full_scan_ready pour signaler
      au nœud de guidage qu'une estimation "tour complet" est disponible.
    """

    def __init__(self) -> None:
        super().__init__("ping360_nearest")

        # ── Déclaration des paramètres ────────────────────────────────────────
        self.declare_parameter("sonar_topic",          _DEFAULT_SONAR_TOPIC)
        self.declare_parameter("output_topic",         _DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter("ready_topic",          _DEFAULT_READY_TOPIC)
        self.declare_parameter("source_frame",         _DEFAULT_SOURCE_FRAME)
        self.declare_parameter("target_frame",         _DEFAULT_TARGET_FRAME)
        self.declare_parameter("window_sec",           _DEFAULT_WINDOW_SEC)
        self.declare_parameter("ignore_fraction",      _DEFAULT_IGNORE_FRAC)
        self.declare_parameter("min_range_m",          _DEFAULT_MIN_RANGE_M)
        self.declare_parameter("dbscan_eps",              _DEFAULT_DBSCAN_EPS)
        self.declare_parameter("dbscan_min_samples",      _DEFAULT_DBSCAN_MIN_PTS)
        self.declare_parameter("ransac_min_inlier_ratio", _DEFAULT_RANSAC_MIN_INLIER)
        self.declare_parameter("ransac_residual",         _DEFAULT_RANSAC_RESID)
        self.declare_parameter("ransac_min_samples",      _DEFAULT_RANSAC_MIN_PTS)
        self.declare_parameter("min_cluster_pts",         _DEFAULT_MIN_CLUSTER)
        self.declare_parameter("max_range_m",             _DEFAULT_MAX_RANGE_M)

        # ── Lecture des paramètres ────────────────────────────────────────────
        self._sonar_topic         = self.get_parameter("sonar_topic").value
        self._output_topic        = self.get_parameter("output_topic").value
        self._ready_topic         = self.get_parameter("ready_topic").value
        self._source_frame        = self.get_parameter("source_frame").value
        self._target_frame        = self.get_parameter("target_frame").value
        self._window_sec          = float(self.get_parameter("window_sec").value)
        self._ignore_frac         = float(self.get_parameter("ignore_fraction").value)
        self._min_range_m         = float(self.get_parameter("min_range_m").value)
        self._dbscan_eps          = float(self.get_parameter("dbscan_eps").value)
        self._dbscan_min_pts      = int(self.get_parameter("dbscan_min_samples").value)
        self._ransac_min_inlier   = float(self.get_parameter("ransac_min_inlier_ratio").value)
        self._ransac_resid        = float(self.get_parameter("ransac_residual").value)
        self._ransac_min_pts      = int(self.get_parameter("ransac_min_samples").value)
        self._min_cluster         = int(self.get_parameter("min_cluster_pts").value)
        self._max_range_m         = float(self.get_parameter("max_range_m").value)

        # ── Buffer d'accumulation ─────────────────────────────────────────────
        # Chaque entrée : (timestamp_sec: float, x: float, y: float)
        self._point_buffer: deque[tuple[float, float, float]] = deque()

        # ── Suivi du tour complet ─────────────────────────────────────────────
        # Trois mécanismes de détection de fin de tour (du plus prioritaire au
        # moins prioritaire) :
        #
        #  A) Scan 360° en un seul message (Gazebo) :
        #     Si angle_max - angle_min >= 300° le scan est déjà complet.
        #     On publie le pipeline toutes les _min_period_sec secondes.
        #
        #  B) Wrap-around angulaire (vrai Ping360 mécanique) :
        #     Quand angle_min redescend de ~+π vers ~-π entre deux messages.
        #
        #  C) Fallback temporel :
        #     Si ni A ni B ne se déclenche et que le buffer est suffisant,
        #     on publie quand même toutes les _min_period_sec secondes.
        #
        # _min_period_sec : intervalle minimal entre deux publications
        #                   (évite de saturer le bus avec des estimations
        #                   identiques si le sonar publie très vite).
        self._last_angle_rad: float | None    = None
        self._tour_start_sec: float | None    = None
        self._tour_duration_sec: float | None = None
        self._full_tour_ready: bool           = False
        self._last_pipeline_sec: float        = 0.0   # timestamp dernière publication
        self._min_period_sec: float           = 2.0   # [s] anti-spam inter-estimations

        # ── TF2 ───────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS : Best Effort pour correspondre au bridge Gazebo ───────────────
        sonar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Abonnement sonar ───────────────────────────────────────────────────
        self._sonar_sub = self.create_subscription(
            LaserScan,
            self._sonar_topic,
            self._scan_callback,
            sonar_qos,
        )

        # ── Publications ───────────────────────────────────────────────────────
        self._orient_pub = self.create_publisher(
            PoseStamped,
            self._output_topic,
            10,
        )
        # Signal booléen : True = estimation fraîche d'un tour complet disponible
        self._ready_pub = self.create_publisher(
            Bool,
            self._ready_topic,
            10,
        )

        # ── Compteurs de diagnostic ────────────────────────────────────────────
        self._n_scans_received = 0
        self._n_points_added   = 0
        self._n_tf_failures    = 0
        self._n_estimates_pub  = 0

        self.get_logger().info(
            f"\n[ping360_nearest] Nœud démarré (v3 – Sélection RANSAC inlier-ratio)\n"
            f"  Sonar topic       : {self._sonar_topic}\n"
            f"  Output topic      : {self._output_topic}\n"
            f"  Ready topic       : {self._ready_topic}\n"
            f"  Frames            : {self._source_frame} → {self._target_frame}\n"
            f"  Fenêtre           : {'auto (1 tour)' if self._window_sec == 0.0 else f'{self._window_sec} s'}\n"
            f"  Filtre distance   : [{self._min_range_m:.2f} m — "
            f"{'désactivé (ignore_fraction)' if self._max_range_m == 0.0 else f'{self._max_range_m:.2f} m'}]\n"
            f"  DBSCAN            : eps={self._dbscan_eps} m, min_pts={self._dbscan_min_pts}\n"
            f"  RANSAC inlier ratio ≥ {self._ransac_min_inlier:.0%} (sélection filet)\n"
            f"  RANSAC poly2      : résidu={self._ransac_resid} m, min_pts={self._ransac_min_pts}\n"
            f"  Min cluster       : {self._min_cluster} points"
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Callback principal
    # ──────────────────────────────────────────────────────────────────────────

    def _scan_callback(self, msg: LaserScan) -> None:
        """
        Traite un scan LaserScan du Ping360.

        Détection de fin de tour (par ordre de priorité) :
          A) Le message couvre déjà ≥ 300° → scan complet en un seul message
             (typique de Gazebo). On publie si _min_period_sec est écoulé.
          B) Wrap-around angulaire entre deux messages consécutifs → vrai
             Ping360 mécanique tournant par incréments.
          C) Fallback temporel : on publie si _min_period_sec est écoulé et
             le buffer est suffisant (garde-fou contre A et B non déclenchés).
        """
        self._n_scans_received += 1

        stamp_sec       = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        max_valid_range = msg.range_max * self._ignore_frac

        # ── Debug : afficher les ranges reçues (toutes les 5 réceptions) ──────
        if self._n_scans_received % 5 == 1:
            finite_ranges = [r for r in msg.ranges if math.isfinite(r)]
            if finite_ranges:
                self.get_logger().info(
                    f"[ping360_nearest] [DEBUG RANGES] scan #{self._n_scans_received}: "
                    f"n_rays={len(msg.ranges)}, finite={len(finite_ranges)}, "
                    f"min={min(finite_ranges):.3f} m, max={max(finite_ranges):.3f} m, "
                    f"range_max={msg.range_max:.1f} m, seuil_valid={max_valid_range:.3f} m"
                )
            else:
                self.get_logger().warn(
                    f"[ping360_nearest] [DEBUG RANGES] scan #{self._n_scans_received}: "
                    f"AUCUNE range finie parmi {len(msg.ranges)} rayons ! "
                    f"(tout est inf/NaN)"
                )

        # ── A) Scan déjà 360° en un seul message (Gazebo / simulation) ────────
        scan_angular_range = abs(msg.angle_max - msg.angle_min)
        is_full_scan_msg   = scan_angular_range >= math.radians(300.0)

        # ── B) Wrap-around angulaire (vrai Ping360 mécanique) ─────────────────
        wrap_detected = False
        current_angle = msg.angle_min
        if self._last_angle_rad is not None and not is_full_scan_msg:
            angle_delta = current_angle - self._last_angle_rad
            if angle_delta < -math.pi:      # saut de ~+π → ~-π = nouveau tour
                wrap_detected = True
                if self._tour_start_sec is not None:
                    self._tour_duration_sec = stamp_sec - self._tour_start_sec
                    self.get_logger().info(
                        f"[ping360_nearest] Tour complet (wrap-around) — "
                        f"durée: {self._tour_duration_sec:.2f} s"
                    )
                self._tour_start_sec = stamp_sec
                self._full_tour_ready = True
        else:
            if self._tour_start_sec is None:
                self._tour_start_sec = stamp_sec
        self._last_angle_rad = current_angle

        # ── Transformation TF2 globale (une seule fois par message) ───────────
        frame_id = msg.header.frame_id if msg.header.frame_id else self._source_frame
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ) as exc:
            self._n_tf_failures += 1
            if self._n_tf_failures % 5 == 1:
                self.get_logger().warn(
                    f"[ping360_nearest] TF2 Exception #{self._n_tf_failures} "
                    f"({type(exc).__name__}) : {exc}  "
                    f"[frames: {frame_id} → {self._target_frame}]"
                )
            return

        # ── Traitement de chaque rayon ─────────────────────────────────────────
        # Seuil de distance supérieure : max_range_m (absolu) si actif, sinon
        # ignore_fraction * range_max comme avant.
        if self._max_range_m > 0.0:
            effective_max = self._max_range_m
        else:
            effective_max = max_valid_range

        n_pts_added_this_scan = 0
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < self._min_range_m or r >= effective_max:
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            x_local   = r * math.cos(angle_rad)
            y_local   = r * math.sin(angle_rad)

            pt_in = PointStamped()
            pt_in.point.x = x_local
            pt_in.point.y = y_local
            pt_in.point.z = 0.0
            pt_out = tf2_geometry_msgs.do_transform_point(pt_in, transform)

            self._point_buffer.append((stamp_sec, pt_out.point.x, pt_out.point.y))
            self._n_points_added   += 1
            n_pts_added_this_scan  += 1

        # Log pour calibrer max_range_m : combien de points ont passé le filtre ?
        if self._n_scans_received % 5 == 1:
            self.get_logger().info(
                f"[ping360_nearest] [FILTRE] scan #{self._n_scans_received}: "
                f"{n_pts_added_this_scan}/{len(msg.ranges)} rayons acceptés "
                f"(seuil: {self._min_range_m:.2f}—{effective_max:.2f} m), "
                f"buffer={len(self._point_buffer)} pts"
            )

        # ── Purge de la fenêtre glissante ─────────────────────────────────────
        if self._window_sec > 0.0:
            window = self._window_sec
        elif self._tour_duration_sec is not None:
            window = self._tour_duration_sec
        else:
            window = 10.0   # valeur conservatrice en attendant la calibration

        cutoff = stamp_sec - window
        while self._point_buffer and self._point_buffer[0][0] < cutoff:
            self._point_buffer.popleft()

        # ── Décision de déclenchement du pipeline ─────────────────────────────
        time_since_last = stamp_sec - self._last_pipeline_sec
        throttled_ok    = time_since_last >= self._min_period_sec
        buffer_ok       = len(self._point_buffer) >= self._min_cluster

        trigger = False
        trigger_reason = ""

        if is_full_scan_msg and throttled_ok and buffer_ok:
            # Cas A : scan Gazebo déjà complet
            trigger = True
            trigger_reason = f"scan_360° ({math.degrees(scan_angular_range):.0f}°)"
        elif wrap_detected and buffer_ok:
            # Cas B : vrai Ping360 mécanique
            trigger = True
            trigger_reason = "wrap-around détecté"
        elif throttled_ok and buffer_ok and self._n_scans_received > 5:
            # Cas C : fallback temporel (ni A ni B déclenchés)
            trigger = True
            trigger_reason = f"fallback temporel ({time_since_last:.1f} s écoulés)"

        if trigger:
            self.get_logger().info(
                f"[ping360_nearest] ▶ Pipeline déclenché — raison: {trigger_reason}  "
                f"buffer={len(self._point_buffer)} pts"
            )
            self._last_pipeline_sec = stamp_sec
            self._detect_and_publish(stamp_sec, msg.header.stamp)
        elif is_full_scan_msg and buffer_ok and not throttled_ok:
            pass   # scan valide mais trop fréquent → silence (pas de log inutile)
        elif not buffer_ok and time_since_last >= self._min_period_sec:
            self.get_logger().warn(
                f"[ping360_nearest] Buffer insuffisant depuis {time_since_last:.1f} s — "
                f"{len(self._point_buffer)}/{self._min_cluster} pts requis. "
                f"scans_reçus={self._n_scans_received}, pts_ajoutés_total={self._n_points_added}, "
                f"tf_failures={self._n_tf_failures}. "
                "Vérifiez la portée du sonar, l'arbre TF et les paramètres de filtrage."
            )

    # ──────────────────────────────────────────────────────────────────────────
    # Pipeline de détection : DBSCAN → sélection RANSAC → publication
    # ──────────────────────────────────────────────────────────────────────────

    def _detect_and_publish(self, stamp_sec: float, ros_stamp) -> None:
        """
        Exécute le pipeline de détection sur le buffer courant et publie
        le résultat.

        Étapes internes :
          1. Conversion du buffer en matrice NumPy (N, 2).
          2. DBSCAN → clusters candidats.
          3. Sélection par ratio d'inliers RANSAC : le cluster dont le plus
             grand ratio de points colle à une parabole deg-2 est identifié
             comme le filet. Retourne directement les coeffs + inliers.
          4. Point de la courbe le plus proche de l'AUV → normale → yaw cible.
          5. Construction et publication du PoseStamped + Bool "ready".
        """
        # ── 1. Extraction de la matrice XY ────────────────────────────────────
        pts = np.array([(x, y) for _, x, y in self._point_buffer], dtype=np.float64)

        # ── 2. Clustering DBSCAN ──────────────────────────────────────────────
        labels = self._run_dbscan(pts)
        if labels is None:
            return

        # ── 3. Sélection du cluster filet + RANSAC poly2 ──────────────────────
        # _select_net_cluster_ransac itère sur les clusters, lance RANSAC sur
        # chacun et retourne le meilleur (ratio inliers le plus élevé).
        selection = self._select_net_cluster_ransac(pts, labels)
        if selection is None:
            return   # logs d'erreur déjà émis dans la méthode

        net_pts, coeffs, inlier_pts = selection
        # coeffs = [a, b, c] dans l'espace de régression (éventuellement permuté)
        # inlier_pts est dans le même espace (pour _poly2_closest_point)

        # ── 4. Point le plus proche + normale ─────────────────────────────────
        # En pratique, on cherche le point de la courbe le plus proche du
        # centroïde du cluster (meilleure approximation de "proche de l'AUV"
        # dans le repère odom courant).
        x_close, y_close = _poly2_closest_point(coeffs, inlier_pts)

        # Vecteur de l'AUV (approximation : origine du repère odom = [0,0])
        # vers le cluster, pour orienter la normale vers l'AUV.
        net_centroid = inlier_pts.mean(axis=0)
        toward_auv   = np.array([0.0, 0.0]) - net_centroid   # AUV ≈ at odom origin

        yaw_target = _poly2_normal_yaw(coeffs, x_close, toward_auv)

        # ── 5. Construction et publication du PoseStamped ─────────────────────
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = ros_stamp
        pose_msg.header.frame_id = self._target_frame

        # Position = point le plus proche de l'AUV sur la courbe fittée
        pose_msg.pose.position.x = x_close
        pose_msg.pose.position.y = y_close
        pose_msg.pose.position.z = 0.0

        qx, qy, qz, qw = _quaternion_from_yaw(yaw_target)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self._orient_pub.publish(pose_msg)
        self._n_estimates_pub += 1

        # Signale qu'une estimation fraîche (tour complet) est disponible
        ready_msg = Bool()
        ready_msg.data = True
        self._ready_pub.publish(ready_msg)

        self.get_logger().info(
            f"[ping360_nearest] ✔ Estimation #{self._n_estimates_pub} (tour complet) : "
            f"yaw={math.degrees(yaw_target):.1f}°  "
            f"pt_proche=({x_close:.2f}, {y_close:.2f})  "
            f"cluster={len(net_pts)} pts → inliers={len(inlier_pts)} "
            f"({len(inlier_pts)/len(net_pts):.0%})  "
            f"buffer={len(pts)} pts"
        )

    # ──────────────────────────────────────────────────────────────────────────
    # DBSCAN
    # ──────────────────────────────────────────────────────────────────────────

    def _run_dbscan(self, pts: np.ndarray) -> np.ndarray | None:
        """
        Lance DBSCAN sur la matrice de points (N, 2).

        Retourne le tableau de labels (−1 = bruit/poisson) ou None en cas d'erreur.
        """
        try:
            db = DBSCAN(
                eps=self._dbscan_eps,
                min_samples=self._dbscan_min_pts,
                metric="euclidean",
                n_jobs=1,  # déterministe, pas de parallélisme
            )
            labels = db.fit_predict(pts)
            return labels
        except Exception as exc:
            self.get_logger().warn(f"[ping360_nearest] DBSCAN échoué : {exc}")
            return None

    # ──────────────────────────────────────────────────────────────────────────
    # Sélection du cluster filet par ratio d'inliers RANSAC
    # ──────────────────────────────────────────────────────────────────────────

    def _select_net_cluster_ransac(
        self,
        pts: np.ndarray,
        labels: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """
        Sélectionne le cluster le plus susceptible d'être le filet en testant
        le ratio d'inliers d'un ajustement RANSAC polynôme deg-2 sur chaque
        cluster candidat.

        Raisonnement :
        - Un filet de pêche, même très courbé par les courants, est une
          structure surfacique quasi-2D → les points collent bien à une
          parabole → ratio d'inliers élevé (≥ ransac_min_inlier_ratio).
        - Un banc de poissons est volumétrique → les points sont épars dans
          le repère 2D du sonar → ratio d'inliers très faible.

        Cette heuristique est robuste là où la PCA échoue : un filet en
        « ventre » profond a une variance isotrope (ratio PCA ≈ 1) mais
        toujours un fort ratio d'inliers RANSAC.

        Args:
            pts    : matrice (N, 2) de tous les points du buffer (repère odom).
            labels : tableau de labels DBSCAN (−1 = bruit).

        Returns:
            (cluster_pts, coeffs, inlier_pts_for_helpers) ou None.
            - cluster_pts            : points bruts du cluster sélectionné (M, 2)
            - coeffs                 : [a, b, c] du polynôme RANSAC retenu
            - inlier_pts_for_helpers : points inliers dans l'espace de régression
                                       (pour _poly2_closest_point et _poly2_normal_yaw)
        """
        unique_labels = set(labels)
        unique_labels.discard(-1)   # ignorer le bruit DBSCAN

        if not unique_labels:
            self.get_logger().warn(
                f"[ping360_nearest] ✗ DBSCAN : 0 cluster trouvé (tout = bruit). "
                f"n_points={len(pts)}, eps={self._dbscan_eps}, "
                f"min_samples={self._dbscan_min_pts}"
            )
            return None

        best_label        = None
        best_inlier_ratio = -1.0
        best_result       = None   # (coeffs, inlier_pts_for_helpers)
        best_cluster_pts  = None
        cluster_info      = []     # pour le log de diagnostic
        too_small_count   = 0

        for lbl in unique_labels:
            cluster_pts = pts[labels == lbl]
            n = len(cluster_pts)

            if n < self._min_cluster:
                too_small_count += 1
                continue

            # Lance RANSAC poly-2 sur ce cluster
            result = self._run_ransac_poly2(cluster_pts)
            if result is None:
                # RANSAC n'a pas convergé → cluster rejeté (log déjà émis)
                cluster_info.append((lbl, n, 0.0))
                continue

            coeffs, inlier_pts_helpers = result
            inlier_ratio = len(inlier_pts_helpers) / n
            cluster_info.append((lbl, n, inlier_ratio))

            if inlier_ratio > best_inlier_ratio:
                best_inlier_ratio = inlier_ratio
                best_label        = lbl
                best_result       = (coeffs, inlier_pts_helpers)
                best_cluster_pts  = cluster_pts

        # ── Log de diagnostic (toujours visible au level par défaut) ───────────
        if cluster_info:
            info_str = "  ".join(
                f"[lbl={l}, n={n}, inliers={r:.0%}]" for l, n, r in cluster_info
            )
            self.get_logger().warn(
                f"[ping360_nearest] Clusters RANSAC : {info_str}  "
                f"→ Meilleur label={best_label} "
                f"(ratio={best_inlier_ratio:.0%}, seuil={self._ransac_min_inlier:.0%})"
            )
        else:
            # Tous les clusters sont sous min_cluster_pts
            n_labels  = len(unique_labels)
            noise_pts = int(np.sum(labels == -1))
            self.get_logger().warn(
                f"[ping360_nearest] ✗ DBSCAN a trouvé {n_labels} cluster(s) mais TOUS "
                f"ont < {self._min_cluster} pts (min_cluster_pts). "
                f"Bruit DBSCAN: {noise_pts}/{len(pts)} pts."
            )
            return None

        # ── Validation du meilleur candidat ───────────────────────────────────
        if best_result is None or best_inlier_ratio < self._ransac_min_inlier:
            self.get_logger().warn(
                f"[ping360_nearest] ✗ RANSAC : aucun cluster ne dépasse le seuil "
                f"d'inliers ({self._ransac_min_inlier:.0%}). "
                f"Meilleur ratio obtenu : {best_inlier_ratio:.0%}. "
                "Peut-être uniquement des bancs de poissons volumétriques ?"
            )
            return None

        coeffs, inlier_pts_helpers = best_result
        return best_cluster_pts, coeffs, inlier_pts_helpers

    # ──────────────────────────────────────────────────────────────────────────
    # RANSAC Polynôme de degré 2 (courbe du filet)
    # ──────────────────────────────────────────────────────────────────────────

    def _run_ransac_poly2(
        self,
        pts: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """
        Ajuste un polynôme de degré 2 (parabole) sur les points du cluster via
        RANSAC manuel pour être robuste aux outliers résiduels après DBSCAN.

        Stratégie :
        - On détermine l'axe principal du cluster via PCA pour choisir si on
          fait la régression y=f(x) ou x=f(y), évitant ainsi la dégénérescence
          sur les filets quasi-verticaux.
        - RANSAC : tirage aléatoire de 3 points, ajustement d'un poly deg-2,
          comptage des inliers (résidu < _ransac_resid).
        - On retourne les coefficients du meilleur modèle et les points inliers.

        Returns:
            (coeffs, inlier_pts) où coeffs = [a, b, c] (poly deg-2 dans
            l'espace éventuellement permuté), ou None en cas d'échec.
        """
        if len(pts) < self._ransac_min_pts:
            self.get_logger().debug(
                f"[ping360_nearest] RANSAC poly2 ignoré : {len(pts)} pts "
                f"< min={self._ransac_min_pts}."
            )
            return None

        # ── Choix de l'axe de régression via PCA ─────────────────────────────
        # On calcule la variance de chaque axe. L'axe avec le plus de variance
        # devient l'axe indépendant X pour la régression.
        var_x = float(np.var(pts[:, 0]))
        var_y = float(np.var(pts[:, 1]))
        swap_axes = var_y > var_x   # si le filet est quasi-vertical, on permute

        if swap_axes:
            X_fit = pts[:, 1].copy()  # y devient l'axe indépendant
            Y_fit = pts[:, 0].copy()  # x devient la variable dépendante
        else:
            X_fit = pts[:, 0].copy()
            Y_fit = pts[:, 1].copy()

        # ── RANSAC manuel sur polynôme deg 2 ──────────────────────────────────
        n_pts = len(pts)
        n_trials = 300
        best_inlier_mask = None
        best_n_inliers   = 0
        best_coeffs      = None

        rng = np.random.default_rng(seed=42)

        for _ in range(n_trials):
            # Tirage de 3 points (minimum pour un poly deg-2)
            idx = rng.choice(n_pts, size=3, replace=False)
            X_s, Y_s = X_fit[idx], Y_fit[idx]

            try:
                coeffs = np.polyfit(X_s, Y_s, deg=2)
            except (np.linalg.LinAlgError, ValueError):
                continue

            # Calcul des résidus sur tous les points
            Y_pred   = np.polyval(coeffs, X_fit)
            residuals = np.abs(Y_fit - Y_pred)
            inlier_mask = residuals < self._ransac_resid
            n_inliers   = int(inlier_mask.sum())

            if n_inliers > best_n_inliers:
                best_n_inliers   = n_inliers
                best_inlier_mask = inlier_mask
                best_coeffs      = coeffs

        if best_coeffs is None or best_n_inliers < self._ransac_min_pts:
            self.get_logger().warn(
                f"[ping360_nearest] RANSAC poly2 n'a pas convergé "
                f"(meilleur inliers={best_n_inliers}, requis={self._ransac_min_pts})."
            )
            return None

        # ── Ré-ajustement sur tous les inliers (solution finale plus stable) ──
        X_in = X_fit[best_inlier_mask]
        Y_in = Y_fit[best_inlier_mask]
        try:
            final_coeffs = np.polyfit(X_in, Y_in, deg=2)
        except (np.linalg.LinAlgError, ValueError) as exc:
            self.get_logger().warn(
                f"[ping360_nearest] RANSAC poly2 re-fit échoué : {exc}"
            )
            return None

        # Reconstruire les points inliers dans le repère ORIGINAL (non-permuté)
        if swap_axes:
            # X_fit = pts[:,1], Y_fit = pts[:,0]
            inlier_pts = pts[best_inlier_mask]   # points originaux filtrés
        else:
            inlier_pts = pts[best_inlier_mask]

        self.get_logger().debug(
            f"[ping360_nearest] RANSAC poly2 : "
            f"inliers={best_n_inliers}/{n_pts}  "
            f"a={final_coeffs[0]:.4f}  b={final_coeffs[1]:.4f}  "
            f"swap_axes={swap_axes}"
        )

        # On stocke l'info de permutation dans les coefficients en retournant
        # un tuple avec les coefficients dans l'espace permuté + le flag.
        # La fonction appelante (_detect_and_publish) utilise les helpers
        # _poly2_closest_point / _poly2_normal_yaw qui opèrent dans cet espace.
        # Pour rester cohérent : si swap_axes, on retourne les coefficients tels
        # quels mais les points inliers sont dans l'espace permuté.
        if swap_axes:
            # Retourner les points inliers dans l'espace permuté pour les
            # helpers (ils s'attendent à [X_indep, Y_dep])
            inlier_pts_for_helpers = np.column_stack([X_in, Y_in])
        else:
            inlier_pts_for_helpers = inlier_pts

        return final_coeffs, inlier_pts_for_helpers


# ─────────────────────────────────────────────────────────────────────────────
# Point d'entrée
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    """Point d'entrée standard ROS 2."""
    rclpy.init(args=args)
    node = Ping360NearestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[ping360_nearest] Interruption clavier — arrêt propre.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
