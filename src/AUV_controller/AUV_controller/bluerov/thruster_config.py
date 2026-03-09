"""
thruster_config.py — Source de vérité unique pour la géométrie des propulseurs.

Si tu modifies le robot (URDF), c'est ICI que tu mets à jour les valeurs.
Tous les scripts (move_forward, thruster_visualizer, mpc, etc.) doivent importer depuis ici.

Comment mapper l'URDF → ce fichier :
  - THRUST_COEFFS[i] = thrust_coeff de <xacro:add_thruster prefix="thruster_{i+1}" ...>
  - POSITIONS[i]     = (x, y, z) de <xacro:add_thruster prefix="thruster_{i+1}" ...>
  - DIRECTIONS[i]    = vecteur de poussée résultant dans le repère base_link.
                       Pour les propulseurs horizontaux à 45° (config X):
                         - Front-right (T1): (-sin45, -sin45, 0)
                         - Front-left  (T2): (+sin45, -sin45, 0)
                         - Rear-right  (T3): (-sin45, +sin45, 0)
                         - Rear-left   (T4): (+sin45, +sin45, 0)
                       Pour les propulseurs verticaux: (0, 0, ±1)
"""

import math

_SIN45 = math.sqrt(2) / 2  # ≈ 0.7071

# ── THRUST_COEFFS ────────────────────────────────────────────────────────────
# Coefficient de thrust de chaque propulseur (issu du champ thrust_coeff de l'URDF).
# f = coeff * w * |w| où w est la vitesse de rotation envoyée.
# Unité : N·s²/rad²  (ou m-equivalent selon Gazebo)
THRUST_COEFFS = [-0.02, 0.02, -0.02, 0.02, -0.02, 0.02, 0.02, -0.02]

# ── POSITIONS ────────────────────────────────────────────────────────────────
# Position (x, y, z) de chaque propulseur dans le repère base_link (en mètres).
# Copie directe des attributs x, y, z de <xacro:add_thruster> dans l'URDF.
POSITIONS = [
    [ 0.135, -0.11,  0.0],  # T1 — avant-droit  (horizontal)
    [ 0.135,  0.11,  0.0],  # T2 — avant-gauche (horizontal)
    [-0.135, -0.11,  0.0],  # T3 — arrière-droit  (horizontal)
    [-0.135,  0.11,  0.0],  # T4 — arrière-gauche (horizontal)
    [ 0.12, -0.218,  0.0],  # T5 — avant-droit  (vertical)
    [ 0.12,  0.218,  0.0],  # T6 — avant-gauche (vertical)
    [-0.12, -0.218,  0.0],  # T7 — arrière-droit  (vertical)
    [-0.12,  0.218,  0.0],  # T8 — arrière-gauche (vertical)
]

# ── DIRECTIONS ───────────────────────────────────────────────────────────────
# Direction de poussée de chaque propulseur dans le repère base_link.
# Vecteur unitaire. Pour une force positive (coeff*w*|w| > 0), le vecteur indique
# le sens de la poussée. Pour une force négative, la poussée est inversée.
DIRECTIONS = [
    [ _SIN45,  _SIN45, 0.0],  # T1 [COEFF=-0.02] empiriquement calibré
    [ _SIN45, -_SIN45, 0.0],  # T2 [COEFF=+0.02] inversé (coeff positif)
    [ _SIN45, -_SIN45, 0.0],  # T3 [COEFF=-0.02]
    [ _SIN45,  _SIN45, 0.0],  # T4 [COEFF=+0.02] inversé
    [0.0, 0.0, -1.0],          # T5 [COEFF=-0.02]
    [0.0, 0.0,  1.0],          # T6 [COEFF=+0.02] inversé ← sens réel +Z
    [0.0, 0.0,  1.0],          # T7 [COEFF=+0.02] inversé ← sens réel +Z
    [0.0, 0.0, -1.0],          # T8 [COEFF=-0.02]
]

NUM_THRUSTERS = 8
