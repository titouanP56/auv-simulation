#!/usr/bin/env python3
"""
ping360_mapper_node.py  —  v2 (Enhanced)
=========================================
Nœud ROS 2 de cartographie sonar 2D en temps réel pour le Ping360
(Blue Robotics).  Version améliorée avec :

  • Fusion attitude : yaw du compas (/sensor/attitude) intégré dans la
    projection cartésienne pour un repère global cohérent.
  • Filtrage 1D médian (kernel 5) sur le tableau d'intensités brutes
    pour éliminer le speckle acoustique avant l'analyse.
  • Extraction du pic d'intensité maximale (et non du premier point
    dépassant le seuil), qui correspond mieux au mur physique réel.
  • Post-traitement image : medianBlur spatial (7×7) + fermeture
    morphologique (5×5) pour relier les points fragmentés en murs nets.
  • Mode hors-ligne : lecture directe d'un fichier .bag (ROS 1 ou ROS 2)
    via rosbags, sans dépendance à un broker ROS 2 actif.
  • Touches interactives : [c] effacer, [s] sauvegarder PNG, [q] quitter.

Auteur  : titou
Package : auv_perception
Topics  : /sensor/ping360 · /sensor/attitude
"""

# ── Bibliothèques standard ────────────────────────────────────────────────────
import argparse
import bisect
import math
import sys
import time
from pathlib import Path

# ── Calcul scientifique ───────────────────────────────────────────────────────
import numpy as np
import cv2

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node

# ── Import optionnel du message ROS 2 Ping360 ─────────────────────────────────
# (non disponible si le package 'sensors' n'est pas buildé dans le workspace)
try:
    from sensors.msg import Ping360   # type: ignore
except ImportError:
    Ping360 = None

try:
    from sensors.msg import Attitude  # type: ignore
except ImportError:
    Attitude = None


# ═════════════════════════════════════════════════════════════════════════════
# Paramètres de la carte (valeurs par défaut)
# ═════════════════════════════════════════════════════════════════════════════
MAP_SIZE_PX      = 1000     # Côté de l'image carrée en pixels
SCALE_PX_PER_M   = 50.0    # 50 px = 1 m  →  image = 20 m × 20 m
MAIN_BANG_MIN_M  = 0.5     # Début de la zone morte (m)
MAIN_BANG_MAX_M  = 1.0     # Fin de la zone morte  (m)
DEFAULT_THRESHOLD = 50     # Seuil d'intensité minimal pour valider un écho
SIGNAL_KERNEL    = 5       # Taille du noyau médian 1D sur le signal brut
DISPLAY_FPS      = 20.0    # Fréquence de rafraîchissement de la fenêtre


# ═════════════════════════════════════════════════════════════════════════════
# Noyau morphologique pour la fermeture (closing) — crée des murs continus
# ═════════════════════════════════════════════════════════════════════════════
MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))


# ─────────────────────────────────────────────────────────────────────────────
# Helpers de signal
# ─────────────────────────────────────────────────────────────────────────────

def median_filter_1d(data: np.ndarray, kernel: int) -> np.ndarray:
    """
    Filtrage médian 1D sur un signal 1-D (numpy uint8 ou float).
    Utilise np.pad pour gérer les bords (mode 'edge').
    """
    half = kernel // 2
    padded = np.pad(data.astype(np.float32), half, mode='edge')
    out = np.empty_like(data, dtype=np.float32)
    for i in range(len(data)):
        out[i] = np.median(padded[i: i + kernel])
    return out.astype(data.dtype)


def find_peak_after_deadzone(
    data: np.ndarray,
    main_bang_idx: int,
    threshold: int,
) -> int | None:
    """
    Retourne l'index du pic d'intensité maximale *après* la zone morte.
    Retourne None si aucune valeur ne dépasse le seuil.

    Utilise np.argmax pour la performance.
    """
    segment = data[main_bang_idx:]
    if len(segment) == 0:
        return None
    peak_val = int(np.max(segment))
    if peak_val <= threshold:
        return None
    return main_bang_idx + int(np.argmax(segment))


# ═════════════════════════════════════════════════════════════════════════════
# Nœud ROS 2
# ═════════════════════════════════════════════════════════════════════════════

class Ping360MapperNode(Node):
    """
    Nœud ROS 2 de cartographie sonar 2D en temps réel.

    Pipeline de traitement pour chaque rayon sonar
    -----------------------------------------------
    1. Filtrage médian 1D du signal brut (suppression du speckle).
    2. Calcul de la zone morte (Main Bang) en index.
    3. Détection du pic d'intensité maximale.
    4. Projection polaire → cartésien local (repère sonar).
    5. Rotation par le yaw du compas → repère global (map).
    6. Conversion en coordonnées pixel et écriture dans map_image.

    Post-traitement (affichage seulement, ne modifie pas map_image)
    ----------------------------------------------------------------
    - medianBlur spatial 7×7.
    - Fermeture morphologique 5×5 pour des murs continus.
    """

    # ── Initialisation ────────────────────────────────────────────────────────
    def __init__(self):
        super().__init__('ping360_mapper_node')

        # ── Paramètres ROS 2 (configurables via CLI ou YAML) ─────────────────
        self.declare_parameter('topic_sonar',       '/sensor/ping360')
        self.declare_parameter('topic_attitude',    '/sensor/attitude')
        self.declare_parameter('map_size_px',       MAP_SIZE_PX)
        self.declare_parameter('scale_px_per_m',    SCALE_PX_PER_M)
        self.declare_parameter('main_bang_min_m',   MAIN_BANG_MIN_M)
        self.declare_parameter('main_bang_max_m',   MAIN_BANG_MAX_M)
        self.declare_parameter('default_threshold', DEFAULT_THRESHOLD)
        self.declare_parameter('signal_kernel',     SIGNAL_KERNEL)
        self.declare_parameter('display_fps',       DISPLAY_FPS)

        self.topic_sonar    = self.get_parameter('topic_sonar').value
        self.topic_attitude = self.get_parameter('topic_attitude').value
        self.map_size       = self.get_parameter('map_size_px').value
        self.scale          = self.get_parameter('scale_px_per_m').value
        self.mb_min_m       = self.get_parameter('main_bang_min_m').value
        self.mb_max_m       = self.get_parameter('main_bang_max_m').value
        self.def_thresh     = self.get_parameter('default_threshold').value
        self.sig_kernel     = self.get_parameter('signal_kernel').value
        display_fps         = self.get_parameter('display_fps').value

        self.center = self.map_size // 2

        # ── Dernière attitude connue (yaw en radians) ─────────────────────────
        # Initialisée à 0 (cap Nord) jusqu'à la réception du premier message.
        self.current_yaw: float = 0.0

        # ── Image de la carte : fond noir, BGR, persistante ───────────────────
        self.map_image = np.zeros((self.map_size, self.map_size, 3), dtype=np.uint8)
        self._draw_overlay()

        # ── Compteurs de débogage ─────────────────────────────────────────────
        self._count_processed  = 0
        self._count_ignored    = 0

        # ── Abonnements ROS 2 (uniquement si les types de messages sont dispo) ─
        if Ping360 is not None:
            self.create_subscription(
                Ping360,
                self.topic_sonar,
                self._sonar_callback,
                10,
            )
        else:
            self.get_logger().warn(
                "sensors/msg/Ping360 introuvable — mode ROS 2 live désactivé "
                "(utiliser --bag pour le mode hors-ligne)."
            )

        if Attitude is not None:
            self.create_subscription(
                Attitude,
                self.topic_attitude,
                self._attitude_callback,
                10,
            )
        else:
            self.get_logger().warn(
                "sensors/msg/Attitude introuvable — yaw figé à 0.0 rad."
            )

        # ── Timer d'affichage OpenCV ──────────────────────────────────────────
        self.create_timer(1.0 / display_fps, self._display_callback)

        self.get_logger().info(
            f"\nPing360MapperNode v2 démarré\n"
            f"  Sonar topic   : {self.topic_sonar}\n"
            f"  Attitude topic: {self.topic_attitude}\n"
            f"  Carte         : {self.map_size}×{self.map_size} px\n"
            f"  Échelle       : {self.scale:.0f} px/m "
            f"({self.map_size / self.scale:.1f} m × {self.map_size / self.scale:.1f} m)\n"
            f"  Zone morte    : [{self.mb_min_m}, {self.mb_max_m}] m\n"
            f"  Seuil défaut  : {self.def_thresh}\n"
            f"  Noyau signal  : {self.sig_kernel}\n"
            f"  Affichage     : {display_fps:.0f} Hz"
        )

    # ── Dessin des repères permanents de la carte ─────────────────────────────
    def _draw_overlay(self) -> None:
        """Dessine la croix centrale et le quadrillage de référence."""
        cx = self.center

        # Quadrillage kilométrique discret (une ligne tous les 5 m)
        step_px = int(5 * self.scale)
        grid_color = (20, 20, 20)
        if step_px > 0:
            for offset in range(step_px, cx, step_px):
                cv2.line(self.map_image, (cx - offset, 0), (cx - offset, self.map_size), grid_color, 1)
                cv2.line(self.map_image, (cx + offset, 0), (cx + offset, self.map_size), grid_color, 1)
                cv2.line(self.map_image, (0, cx - offset), (self.map_size, cx - offset), grid_color, 1)
                cv2.line(self.map_image, (0, cx + offset), (self.map_size, cx + offset), grid_color, 1)

        # Axes principaux (plus lumineux)
        axis_color = (35, 35, 35)
        cv2.line(self.map_image, (cx, 0), (cx, self.map_size), axis_color, 1)
        cv2.line(self.map_image, (0, cx), (self.map_size, cx), axis_color, 1)

        # Marqueur robot (centre)
        cv2.drawMarker(
            self.map_image, (cx, cx),
            color=(0, 180, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=16, thickness=2,
        )

    # ── Callback attitude ─────────────────────────────────────────────────────
    def _attitude_callback(self, msg) -> None:
        """Met à jour le yaw courant du robot depuis /sensor/attitude."""
        self.current_yaw = float(msg.yaw)

    # ── Callback sonar (cœur du traitement) ──────────────────────────────────
    def _sonar_callback(self, msg) -> None:
        """
        Traite un message Ping360 et projette le point détecté sur la carte.

        Paramètres
        ----------
        msg : objet Ping360 (ROS 2) ou objet rosbags désérialisé
        """
        # ── 0. Extraction et validation des champs ────────────────────────────
        sonar_range  = float(msg.sonar_range)
        num_samples  = int(msg.number_of_samples)
        angle_deg    = float(msg.angle_deg)
        threshold    = int(msg.threshold) if int(msg.threshold) > 0 else self.def_thresh
        raw_data     = msg.data

        if num_samples == 0 or sonar_range <= 0.0:
            return

        # Convertir en tableau numpy (robuste aux bytes, list, array)
        data = np.frombuffer(bytes(raw_data), dtype=np.uint8) if isinstance(raw_data, (bytes, bytearray)) \
               else np.asarray(raw_data, dtype=np.uint8)

        # Garde-fou : tronquer si le tableau est plus court qu'annoncé
        if len(data) < num_samples:
            num_samples = len(data)
        if num_samples == 0:
            return

        data = data[:num_samples]

        # ── 1. Filtrage médian 1D du signal brut (déspeckle) ─────────────────
        data = median_filter_1d(data, self.sig_kernel)

        # ── 2. Calcul des bornes de la zone morte (Main Bang) ─────────────────
        spm = num_samples / sonar_range          # samples par mètre
        idx_min = int(math.ceil(self.mb_min_m * spm))
        idx_max = int(math.ceil(self.mb_max_m * spm))
        idx_max = min(idx_max, num_samples)

        # ── 3. Extraction du pic d'intensité maximale ─────────────────────────
        peak_idx = find_peak_after_deadzone(data, idx_max, threshold)
        if peak_idx is None:
            self._count_ignored += 1
            return

        # ── 4. Distance en mètres ─────────────────────────────────────────────
        distance_m = peak_idx * (sonar_range / num_samples)

        # ── 5. Projection polaire → cartésien local (repère sonar, Z=0) ───────
        #   Convention : X avant du robot, Y gauche
        angle_rad = math.radians(angle_deg)
        x_local   =  distance_m * math.cos(angle_rad)
        y_local   =  distance_m * math.sin(angle_rad)

        # ── 6. Rotation par le yaw du compas → repère global (map) ───────────
        #   [x_global]   [cos(yaw)  -sin(yaw)] [x_local]
        #   [y_global] = [sin(yaw)   cos(yaw)] [y_local]
        yaw = self.current_yaw
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        x_global = cos_y * x_local - sin_y * y_local
        y_global = sin_y * x_local + cos_y * y_local

        # ── 7. Conversion en pixels ───────────────────────────────────────────
        #   Axe image : col (u) = center + y_global * scale  (Y → droite)
        #               row (v) = center - x_global * scale  (X → haut)
        px_u = int(round(self.center + y_global * self.scale))
        px_v = int(round(self.center - x_global * self.scale))

        # ── 8. Écriture sur la carte (zone 2×2 pixels) ────────────────────────
        if 0 <= px_u < self.map_size and 0 <= px_v < self.map_size:
            # Intensité proportionnelle à la valeur du pic (nuance de gris-bleu)
            intensity = int(data[peak_idx])
            color = (intensity, intensity, 255)   # teinte bleuâtre
            cv2.circle(self.map_image, (px_u, px_v), radius=2,
                       color=color, thickness=-1)

        self._count_processed += 1

        self.get_logger().debug(
            f"[{self._count_processed}] "
            f"θ={angle_deg:.1f}° | d={distance_m:.2f} m | "
            f"yaw={math.degrees(yaw):.1f}° | "
            f"px=({px_u},{px_v})"
        )

    # ── Callback timer : post-traitement + affichage ──────────────────────────
    def _display_callback(self) -> None:
        """
        Génère et affiche l'image de carte post-traitée.
        Appelé périodiquement par le timer ROS 2 (même thread que spin).

        Le post-traitement est appliqué sur une COPIE de map_image
        pour ne pas corrompre les données brutes accumulées.
        """
        # Copie de travail (ne jamais modifier self.map_image ici)
        display = self.map_image.copy()

        # ── Post-traitement 1 : flou médian spatial 7×7 ───────────────────────
        #    Lisse le bruit résiduel sans créer d'artefacts de bord.
        display = cv2.medianBlur(display, 7)

        # ── Post-traitement 2 : fermeture morphologique 5×5 ──────────────────
        #    Dilatation suivie d'érosion → comble les trous entre points
        #    voisins et produit des contours de murs continus.
        display = cv2.morphologyEx(display, cv2.MORPH_CLOSE, MORPH_KERNEL)

        # ── HUD textuel ───────────────────────────────────────────────────────
        hud_lines = [
            ("Ping360 Sonar Map  v2", (10, 28), 0.75, (220, 220, 220), 2),
            (f"Scale: {self.scale:.0f} px/m  |  FOV: {self.map_size / self.scale:.0f} m",
             (10, 52), 0.45, (140, 140, 140), 1),
            (f"Yaw: {math.degrees(self.current_yaw):+.1f} deg",
             (10, 70), 0.45, (80, 200, 120), 1),
            (f"Points: {self._count_processed}  ignored: {self._count_ignored}",
             (10, 88), 0.42, (100, 100, 100), 1),
            ("[c] Clear  [s] Save  [q] Quit",
             (10, self.map_size - 12), 0.40, (80, 80, 80), 1),
        ]
        for text, org, scale, color, thick in hud_lines:
            cv2.putText(display, text, org,
                        cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick,
                        cv2.LINE_AA)

        cv2.imshow("Ping360 Sonar Map", display)

        # ── Gestion des touches ───────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            # Effacer la carte et redessiner les repères
            self.map_image[:] = 0
            self._draw_overlay()
            self._count_processed = 0
            self._count_ignored   = 0
            self.get_logger().info("Carte effacée (touche 'c').")

        elif key == ord('s'):
            filename = f"ping360_map_{int(time.time())}.png"
            cv2.imwrite(filename, display)
            self.get_logger().info(f"Carte sauvegardée → {filename}")

        elif key == ord('q'):
            self.get_logger().info("Arrêt demandé (touche 'q').")
            rclpy.shutdown()


# ═════════════════════════════════════════════════════════════════════════════
# Point d'entrée
# ═════════════════════════════════════════════════════════════════════════════

def _run_offline(node: Ping360MapperNode, bag_path: Path) -> None:
    """
    Lecture hors-ligne d'un fichier bag (ROS 1 ou ROS 2) avec rosbags.
    Fusionne les topics sonar ET attitude avec synchronisation temporelle
    par plus proche voisin (bisect, O(log n)).

    Paramètres
    ----------
    node     : nœud Ping360MapperNode déjà initialisé
    bag_path : chemin vers le fichier .bag
    """
    from rosbags.highlevel import AnyReader

    TOPIC_SONAR = node.topic_sonar
    TOPIC_ATT   = node.topic_attitude

    node.get_logger().info(f"Mode hors-ligne  →  bag : {bag_path}")

    # ── Passe 1 : cache des attitudes ─────────────────────────────────────────
    att_cache: list[tuple[int, float]] = []   # (timestamp_ns, yaw)

    with AnyReader([bag_path]) as reader:
        att_conns = [c for c in reader.connections if c.topic == TOPIC_ATT]
        if not att_conns:
            node.get_logger().warn(
                f"Topic attitude '{TOPIC_ATT}' absent du bag — yaw = 0.0."
            )
        else:
            node.get_logger().info(f"Passe 1 : mise en cache du topic '{TOPIC_ATT}'…")
            for conn, ts_ns, rawdata in reader.messages(connections=att_conns):
                msg = reader.deserialize(rawdata, conn.msgtype)
                att_cache.append((int(ts_ns), float(msg.yaw)))
            node.get_logger().info(f"  {len(att_cache)} échantillons d'attitude mis en cache.")

    att_ts = [e[0] for e in att_cache]   # timestamps triés (extraction une fois)

    def lookup_yaw(query_ts: int) -> float:
        """Retourne le yaw le plus proche dans le cache."""
        if not att_cache:
            return 0.0
        idx = bisect.bisect_left(att_ts, query_ts)
        if idx == 0:
            return att_cache[0][1]
        if idx >= len(att_cache):
            return att_cache[-1][1]
        before = att_cache[idx - 1]
        after  = att_cache[idx]
        return (before if (query_ts - before[0]) <= (after[0] - query_ts) else after)[1]

    # ── Passe 2 : lecture et projection des données sonar ─────────────────────
    with AnyReader([bag_path]) as reader:
        sonar_conns = [c for c in reader.connections if c.topic == TOPIC_SONAR]
        if not sonar_conns:
            available = [c.topic for c in reader.connections]
            node.get_logger().error(
                f"Topic sonar '{TOPIC_SONAR}' absent du bag.\n"
                f"Topics disponibles : {available}"
            )
            return

        node.get_logger().info(f"Passe 2 : lecture des données sonar '{TOPIC_SONAR}'…")

        for conn, ts_ns, rawdata in reader.messages(connections=sonar_conns):
            # Injecter le yaw synchronisé avant chaque callback sonar
            node.current_yaw = lookup_yaw(int(ts_ns))

            msg = reader.deserialize(rawdata, conn.msgtype)
            node._sonar_callback(msg)
            node._display_callback()

    # ── Fin de lecture : affichage persistant ─────────────────────────────────
    node.get_logger().info(
        f"\nLecture terminée — {node._count_processed} points tracés, "
        f"{node._count_ignored} rayons ignorés.\n"
        "La carte reste affichée. [s] sauvegarder  [q] quitter."
    )
    while rclpy.ok():
        node._display_callback()
        time.sleep(0.05)


def main(args=None) -> None:
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        prog='ping360_mapper_node',
        description='Ping360 2D Sonar Mapper (v2 — avec fusion attitude)',
    )
    parser.add_argument(
        '--bag', type=str, default=None,
        help="Chemin vers un fichier .bag (ROS 1 ou ROS 2) pour lecture hors-ligne.",
    )
    parsed, ros_args = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=ros_args)
    node = Ping360MapperNode()

    try:
        if parsed.bag:
            _run_offline(node, Path(parsed.bag))
        else:
            node.get_logger().info("Mode ROS 2 live activé.")
            rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Interruption clavier — arrêt propre.")
    finally:
        cv2.destroyAllWindows()
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
