# Concepts Informatiques et Algorithmiques — Projet AUV BlueROV2

> Ce document recense l'ensemble des notions informatiques, algorithmiques et mathématiques
> mises en œuvre dans le projet de simulation et de contrôle autonome d'un véhicule sous-marin
> (AUV) BlueROV2 pour l'inspection de filets d'aquaculture.

---

## Table des matières

1. [Architecture logicielle — ROS 2](#1-architecture-logicielle--ros-2)
2. [Modélisation et simulation robotique](#2-modélisation-et-simulation-robotique)
3. [Contrôle automatique](#3-contrôle-automatique)
4. [Localisation et fusion de capteurs](#4-localisation-et-fusion-de-capteurs)
5. [Traitement du signal et des données capteurs](#5-traitement-du-signal-et-des-données-capteurs)
6. [Perception et analyse de nuages de points](#6-perception-et-analyse-de-nuages-de-points)
7. [Gestion des missions — Machines à états finis](#7-gestion-des-missions--machines-à-états-finis)
8. [Optimisation numérique et résolution sous contraintes](#8-optimisation-numérique-et-résolution-sous-contraintes)
9. [Algèbre linéaire appliquée](#9-algèbre-linéaire-appliquée)
10. [Géométrie et transformations spatiales](#10-géométrie-et-transformations-spatiales)
11. [Programmation système et interopérabilité](#11-programmation-système-et-interopérabilité)
12. [Informatique embarquée et temps réel](#12-informatique-embarquée-et-temps-réel)
13. [Infrastructure DevOps](#13-infrastructure-devops)
14. [Récapitulatif par fichier source](#14-récapitulatif-par-fichier-source)

---

## 1. Architecture logicielle — ROS 2

### 1.1 Paradigme publish/subscribe (messagerie)

Tous les nœuds du projet communiquent exclusivement via des **topics ROS 2** sans appel direct de fonction entre elles. Chaque nœud publie ses sorties et souscrit aux entrées dont il a besoin, ce qui garantit un **couplage faible** entre les composants.

- **Publishers / Subscribers** : pattern observateur implémenté nativement par `rclpy` (Python) et `rclcpp` (C++).
- **Types de messages standards** : `std_msgs/Float64`, `nav_msgs/Odometry`, `sensor_msgs/PointCloud2`, `sensor_msgs/LaserScan`, `geometry_msgs/Wrench`, `geometry_msgs/TwistWithCovarianceStamped`.
- **Topics principaux** :
  - `/odometry/filtered` — sortie EKF (localisation fusionnée)
  - `/auv/command_wrench` — commande de force/couple 6-DOF centralisée
  - `/cmd_vel_1` à `/cmd_vel_8` — commandes individuelles par propulseur
  - `/mission/phase` — état courant de la mission

### 1.2 Qualité de service (QoS)

Deux profils QoS distincts sont utilisés selon la criticité des données :

| Profil | Fiabilité | Usage |
|---|---|---|
| **Best-Effort** | Peut perdre des messages | Données sonar temps réel (Ping360, Sonoptix) |
| **Reliable** | Livraison garantie | Signaux de changement d'état (`/mission/phase2_done`) |

La politique `DurabilityPolicy.VOLATILE` vs la conservation des derniers messages (`KEEP_LAST`) est choisie selon que l'abonné peut rejoindre en cours de route.

### 1.3 Timers et boucles de contrôle

Chaque nœud exécute sa boucle principale via un **timer ROS 2** (`create_timer`) à une fréquence fixe :

| Nœud | Fréquence |
|---|---|
| MPC (contrôleur) | 4 Hz |
| Station-keeping (PID) | 20 Hz |
| Phase 2 & 3 (guidage) | 10 Hz |

Ce modèle s'apparente au **pattern réacteur** : la boucle d'événements de `rclpy.spin()` dispatche callbacks et timers de manière coopérative.

### 1.4 Architecture en packages modulaires

Le workspace est découpé en 6 packages `colcon` indépendants, suivant le principe de **séparation des responsabilités** :

```
AUV_description   → description physique, monde de simulation
AUV_guidance      → cerveau de mission (guidage, bridges)
AUV_controller    → contrôleurs avancés (MPC, station-keeping)
auv_perception    → traitement sonar, estimation du filet
my_auv_localization → configuration EKF
auv_dvl_bridge    → pont Gazebo → ROS 2 (C++)
```

### 1.5 TF2 — Arbre de transformations

Le système de transformation de repères `tf2_ros` maintient en temps réel l'**arbre de repères** du robot :

- `odom` → `base_link` (odométrie)
- `odom` → `local_origin` (repère local défini lors de l'approche du filet)

Les nœuds utilisent `TransformBroadcaster` pour publier et `tf2_ros.Buffer` + `TransformListener` pour consulter n'importe quelle transformation à n'importe quel instant passé.

---

## 2. Modélisation et simulation robotique

### 2.1 Description URDF/XML du robot

Le robot BlueROV2 est décrit en **URDF** (Unified Robot Description Format), un format XML déclaratif qui spécifie :

- La **cinématique** : liens (`<link>`) et articulations (`<joint>`) formant un arbre rigide.
- L'**inertie** : masse, centre de masse, tenseur d'inertie de chaque corps rigide.
- La **géométrie de collision** et la géométrie visuelle (maillages 3D).
- Les **plugins Gazebo** : propulseurs hydrodynamiques, capteurs IMU, DVL, sonar.

Trois variantes URDF existent selon le niveau de réalisme :
- `BlueROV2.urdf.xml` — modèle minimal
- `BlueROV2captors.urdf.xml` — avec capteurs
- `Bluerov2_realistic.urdf.xml` — physique hydrodynamique complète

### 2.2 Simulation physique (Gazebo Harmonic)

**Gazebo Harmonic** (gz-sim) simule la physique du robot dans des mondes SDF (Simulation Description Format). Les mondes disponibles permettent de tester différentes conditions :

- `small_net.xml` — filet aquacole (rayon ≈ 3,4 m)
- `ocean_40m.xml` — grand filet (rayon ≈ 20 m)
- `Bassin_ntnu_waves.xml` — piscine avec vagues (plugin `gz-waves`)

Le moteur physique gère : flottabilité, interactions rigides, dynamique des fluides (via plugins hydrodynamiques).

### 2.3 Modèle dynamique du robot sous-marin (6-DOF)

Le modèle physique implémenté dans les contrôleurs MPC est un modèle **6 degrés de liberté** standard pour AUV :

#### Cinématique (repère Terre → repère robot)
```
ẋ = u·cos(ψ) - v·sin(ψ)
ẏ = u·sin(ψ) + v·cos(ψ)
ż = w
ψ̇ = r
```

#### Dynamique (2e loi de Newton avec masse ajoutée et trainée)
```
(m + m_added_u)·u̇ = F_surge - Xu_lin·u - Xu_quad·u·|u|
(m + m_added_v)·v̇ = F_sway  - Yv_lin·v - Yv_quad·v·|v|
(m + m_added_w)·ẇ = F_heave - Zw_lin·w - Zw_quad·w·|w| + F_buoyancy
(Izz + Izz_added)·ṙ = M_yaw  - Nr_lin·r - Nr_quad·r·|r|
```

Les paramètres clés :
- **Masse ajoutée** (`added mass`) : effet de l'eau entraînée par le corps.
- **Trainée linéaire et quadratique** : résistance hydrodynamique proportionnelle à `v` et `v²`.
- **Flottabilité nette** (`BUOYANCY_NET = 2.0 N`) : différence entre la poussée d'Archimède et le poids.

---

## 3. Contrôle automatique

### 3.1 Régulateur PID (Proportionnel-Intégral-Dérivé)

**Fichiers** : `station_keeping.py`, `net_approach.py`, `phase3_inspection.py`

Le PID est le régulateur fondamental du projet, utilisé dans toutes les phases de guidage.

#### Loi de commande
```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·ė(t)
```

- **Proportionnel (Kp)** : réaction immédiate à l'erreur courante.
- **Intégral (Ki)** : élimination de l'erreur statique (ex : compensation de la flottabilité).
- **Dérivé (Kd)** : amortissement, prévention du dépassement.

#### Anti-windup de l'intégrateur
L'intégrale est bornée (`np.clip`) pour éviter la saturation de l'actionneur :
```python
self._integral = np.clip(self._integral, -self._integral_limit, self._integral_limit)
```

#### Limiteur de taux (rate limiter)
Sur le signal de couple de lacet `Mz`, un limiteur de variation maximale par pas de temps est appliqué pour éviter les à-coups brutaux :
```python
Mz_delta = np.clip(Mz_raw - self._last_Mz, -MZ_RATE_LIMIT, MZ_RATE_LIMIT)
```

#### Axe de contrôle par PID (Phase 3)
| PID | Erreur contrôlée | Sortie |
|---|---|---|
| `_pid_depth` | Erreur de profondeur (Z) | Force verticale `Fz` |
| `_pid_dist` | Distance au filet - consigne | Force d'avance `Fx` |
| `_pid_yaw` | Erreur d'orientation (lacet) | Couple `Mz` |
| `_pid_pitch` | Asymétrie top/bottom sonar | Couple `My` (mode cône) |
| `_pid_velocity_sway` | Erreur de vitesse latérale | Force latérale `Fy` |

### 3.2 Régulateur MPC (Model Predictive Control)

**Fichiers** : `mpc_controller_blueROV.py`, `mpc_controller_sensors.py`

Le MPC est un contrôleur avancé qui, à chaque pas de temps, résout un **problème d'optimisation en ligne** pour calculer la meilleure séquence de commandes sur un horizon temporel futur.

#### Principe
1. **Modèle interne** : le contrôleur dispose du modèle dynamique 8-états du robot (décrit ci-dessus).
2. **Horizon de prédiction** : N=10 à 12 pas de temps dans le futur (1 à 1,8 s).
3. **Fonction de coût** à minimiser :
   - **Coût terminal `mterm`** : erreur de position finale × pondération forte.
   - **Coût de stage `lterm`** : erreur de position à chaque étape + pénalité sur la vitesse de lacet + pénalité énergétique sur les commandes.
   - **Rterm** : pénalité sur la variation de commande entre deux pas (lissage).
4. **Contraintes** :
   - Bornes sur les forces de chaque propulseur `[-5 N, +5 N]`.
   - Contraintes non-linéaires sur les couples de roulis et de tangage (équilibre des propulseurs verticaux).
   - Contraintes souples (soft constraints) sur les vitesses maximales.

#### Bibliothèques utilisées
- **`do_mpc`** : framework Python pour la définition de modèle, le contrôleur MPC et l'estimateur.
- **`CasADi`** : calcul symbolique et différentiation automatique pour la génération du problème NLP.
- **IPOPT** (Interior Point OPTimizer) : solveur NLP sous-jacent pour résoudre le problème d'optimisation.

#### Warm-start
L'option `ipopt.warm_start_init_point: yes` initialise chaque résolution à partir de la solution précédente, réduisant le nombre d'itérations nécessaires.

---

## 4. Localisation et fusion de capteurs

### 4.1 Filtre de Kalman Étendu (EKF)

**Fichier de configuration** : `my_auv_localization/config/ekf.yaml`  
**Package** : `robot_localization` (package ROS 2 standard)

L'EKF estime de façon optimale l'état complet du robot (position + vitesse) à partir de plusieurs capteurs bruités.

#### Principe de l'EKF
1. **Prédiction** : propagation de l'état via le modèle du système.
2. **Mise à jour** : correction de l'état en intégrant la mesure du capteur, pondérée par sa confiance relative.

La matrice de covariance `P` résume l'incertitude courante sur l'état estimé.

#### Capteurs fusionnés

| Capteur | Grandeur mesurée | Confiance |
|---|---|---|
| **IMU** (`/imu/fixed`) | Vitesses angulaires (ωx, ωy, ωz) | Haute (gyroscope) |
| **DVL** (`/dvl/velocity_ros`) | Vitesses linéaires (vx, vy, vz) dans `base_link` | Haute |
| **Capteur de profondeur** (`/depth/pose`) | Position absolue en Z | Moyenne (bruit gaussien 0,02 m) |

#### Matrice de covariance du bruit de processus
La matrice 15×15 diagonale `process_noise_covariance` quantifie l'incertitude sur le modèle de propagation d'état. Des valeurs faibles sur les diagonales indiquent que le filtre "fait confiance" au modèle et est peu sensible aux imperfections des capteurs.

### 4.2 DVL — Doppler Velocity Log

**Fichier** : `auv_dvl_bridge/src/dvl_bridge_node.cpp`

Le DVL mesure les vitesses 3D du robot par effet Doppler (écho acoustique sur le fond). Le nœud bridge :
- Souscrit au topic Gazebo (format **protobuf** `gz::msgs::DVLVelocityTracking`)
- Convertit vers un message ROS 2 `TwistWithCovarianceStamped`
- Remplit la matrice de covariance 6×6 (seule la sous-matrice 3×3 des vitesses linéaires est pertinente)

### 4.3 Simulation du capteur de profondeur

**Fichier** : `simulated_depth_sensor.py`

Pour modéliser la réalité d'un capteur de pression :
- Extraction de la position Z exacte depuis Gazebo (`/odom`)
- Ajout d'un **bruit gaussien** `N(0, 0.02)` (écart-type 2 cm)
- Publication avec une covariance appropriée (`covariance[14] = 0.0004`)

La covariance 36-éléments (matrice 6×6 pour pose complète) permet à l'EKF de n'utiliser que l'axe Z (`[false, false, true, ...]`).

---

## 5. Traitement du signal et des données capteurs

### 5.1 Filtre médian glissant

**Fichier** : `phase3_inspection.py` → classe `MovingMedian`

Un **buffer circulaire** (`collections.deque(maxlen=window)`) stocke les N dernières mesures du sonar Sonoptix. À chaque nouvelle mesure, la médiane est recalculée sur la fenêtre.

```python
class MovingMedian:
    def __init__(self, window: int = 7):
        self._buf = collections.deque(maxlen=window)

    def update(self, value: float) -> float:
        self._buf.append(value)
        return float(np.median(list(self._buf)))
```

La médiane est préférée à la moyenne car elle est **robuste aux valeurs aberrantes** (outliers sonar).

### 5.2 Filtre de moyenne glissante

**Fichier** : `net_local_estimator.py`

Un `deque(maxlen=5)` stocke les 5 dernières estimations de distance et d'angle. La moyenne simple est calculée pour lisser les mesures PCA.

### 5.3 Filtre à moyenne exponentielle (EMA — Exponential Moving Average)

**Fichier** : `phase3_inspection.py`

```python
self._smoothed_yaw_error += alpha * (raw_error - self._smoothed_yaw_error)
```

L'EMA atténue les variations brusques de l'erreur d'angle (le sonar Sonoptix publie à ~2 Hz, créant des sauts). Le paramètre `alpha` est réglable via les paramètres ROS 2 (`yaw_ema_alpha`).

### 5.4 Détection et rejet des pics (spike rejection)

**Fichier** : `phase3_inspection.py`

Un saut de mesure sonar supérieur à `SPIKE_THRESHOLD = 0.5 m` entre deux acquisitions consécutives est considéré comme un artefact et la mesure précédente est conservée :

```python
if abs(raw - prev_range) > SPIKE_THRESHOLD:
    raw = prev_range  # Rejet du pic
```

### 5.5 Bruit gaussien (simulation réaliste)

**Fichier** : `simulated_depth_sensor.py`

Le bruit du capteur de profondeur est modélisé par une distribution normale `N(μ=0, σ=0.02)` via `random.gauss()`, conformément au modèle de bruit des capteurs de pression réels.

### 5.6 Accumulation et médiane de mesures angulaires (robustesse de cap)

**Fichier** : `net_approach.py`

Pendant la phase de scan (Ping360), plusieurs lectures du cap vers le filet sont accumulées sur 4 secondes, puis leur **médiane** est utilisée comme cap cible. Cela rend la décision robuste aux retours parasites du sonar :

```python
import statistics
self.target_yaw = statistics.median(self._scan_angles)
```

---

## 6. Perception et analyse de nuages de points

### 6.1 Traitement de PointCloud2

**Fichiers** : `sonar_filter_node.py`, `phase3_inspection.py`, `net_local_estimator.py`

Le format `sensor_msgs/PointCloud2` encode un nuage de points 3D en mémoire binaire contiguë (format optimisé). Le projet utilise deux approches de décodage :

- **`sensor_msgs_py.point_cloud2.read_points()`** : itérateur Python générique.
- **`np.frombuffer(msg.data, dtype=np.float32)`** suivi d'un `reshape` : décodage vectoriel rapide avec NumPy, évitant la boucle Python.

#### Filtrage par distance (SonarFilterNode)
Seuls les points à distance ≤ 4 m du capteur sont conservés :
```python
distances = np.sqrt(x**2 + y**2 + z**2)
filtered_points = points_array[distances <= 4.0]
```

#### Extraction du point le plus proche (percentile)
**Fichier** : `phase3_inspection.py`

Plutôt que de prendre le minimum absolu (sensible au bruit), les **10% des points les plus proches** sont sélectionnés, puis leur distance et angle moyens sont calculés. Cela constitue un estimateur robuste de la surface du filet.

### 6.2 Analyse en Composantes Principales (PCA) pour l'estimation du filet

**Fichier** : `net_local_estimator.py`

La PCA est utilisée ici comme **ajustement de droite robuste** (Total Least Squares) sur les points détectés du filet :

1. Calcul du centroïde (moyenne) des points projetés en 2D.
2. Centrage des données.
3. Calcul de la matrice de covariance 2×2.
4. Décomposition en valeurs propres (`np.linalg.eigh`) : le vecteur propre associé à la **plus petite valeur propre** est la **normale à la droite** (direction la moins variable = direction perpendiculaire au filet).
5. La distance du capteur au filet est le produit scalaire `normal · centroid`.

```python
cov_matrix = np.cov(centered_x, centered_y)
eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
normal = eigenvectors[:, 0]  # vecteur propre minimal
distance = np.dot(normal, mean_vector)
```

### 6.3 Parsing binaire de PointCloud2 (struct)

**Fichier** : `net_approach.py`

Pour extraire les champs x, y, z d'un `PointCloud2` manuellement (sans librairie helper), le module `struct` Python est utilisé :

```python
px = struct.unpack_from('f', data, base + x_offset)[0]
```

Cela démontre la compréhension de la disposition mémoire (`point_step`, `offset` par champ).

---

## 7. Gestion des missions — Machines à états finis

### 7.1 Phase 2 : Approche du filet

**Fichier** : `net_approach.py`

La mission d'approche est modélisée comme un **automate fini déterministe (DFA)** à 6 états :

```
DESCENDING → SCANNING → ALIGNING → APPROACHING → STABILIZING → STANDOFF
```

| État | Action | Condition de transition |
|---|---|---|
| `DESCENDING` | Plongée à la profondeur cible | Profondeur stable depuis 2 s |
| `SCANNING` | Accumulation de lectures Ping360 | ≥2 lectures sur 4 s |
| `ALIGNING` | Rotation vers le cap du filet | Erreur yaw < 10° stable 1 s |
| `APPROACHING` | Avance vers le filet | Distance Sonoptix ≤ 1.5 m + tolérance |
| `STABILIZING` | Stabilisation à la distance de consigne | 3 secondes écoulées |
| `STANDOFF` | Publication du repère local, attente phase 3 | — |

Les **conditions de tenue** (hysteresis temporelle) évitent les transitions prématurées :
```python
if abs(error) < TOLERANCE:
    if self._ok_since is None:
        self._ok_since = now
    elif (now - self._ok_since) >= HOLD_TIME:
        self.state = NEXT_STATE
else:
    self._ok_since = None
```

### 7.2 Phase 3 : Inspection par orbite

**Fichier** : `phase3_inspection.py`

La phase d'inspection est aussi une machine à états :

```
WAITING → WALKING_THE_NET ⇌ LOST_WALL → LAP_COMPLETED
```

Le nœud "marche le long du filet" en maintenant une distance constante, en contrôlant sa vitesse latérale. La **détection de perte du sonar** déclenche un état de récupération `LOST_WALL`.

#### Suivi angulaire et comptage de tours
L'angle parcouru est intégré numériquement à chaque pas de temps :
```python
delta = angle_diff(current_yaw, prev_yaw)
accumulated_yaw += delta
```
Un tour complet correspond à `|accumulated_yaw| ≥ 2π`.

#### Détection du mode cône
En comparant le rayon d'orbite estimé (`arc_length / angle`) à la référence initiale, le système détecte que le robot approche du bas du filet (forme conique) et active un PID de contrôle du tangage basé sur l'asymétrie top/bottom des retours sonar.

---

## 8. Optimisation numérique et résolution sous contraintes

### 8.1 Problème d'optimisation non-linéaire (NLP)

Le MPC formule, à chaque pas de contrôle, un **problème NLP** de la forme :

```
minimiser   J = Σ lterm(x_k, u_k) + mterm(x_N)
sous         x_{k+1} = f(x_k, u_k)   [modèle dynamique]
             u_min ≤ u_k ≤ u_max      [contraintes d'entrée]
             g(x_k, u_k) ≤ 0          [contraintes non-linéaires]
```

- **Variables de décision** : séquence de commandes `[u_0, u_1, ..., u_{N-1}]` (8 propulseurs × N pas).
- **Contraintes d'égalité** : le modèle dynamique est respecté à chaque pas.
- **Contraintes non-linéaires souples** : via pénalisation dans la fonction de coût (slack variables).

### 8.2 Différentiation symbolique automatique (CasADi)

**Bibliothèque** : `casadi`

CasADi représente les équations du modèle comme des **graphes de calcul symboliques**. Cela permet de calculer analytiquement le gradient et le hessien de la fonction de coût, nécessaires pour IPOPT. La formule `ca.sqrt(u**2 + eps)` évite la non-différentiabilité en `u=0`.

### 8.3 IPOPT — Interior Point OPTimizer

IPOPT résout le NLP par méthode de **points intérieurs** (méthode barrière). Les options configurées :

| Option | Valeur | Signification |
|---|---|---|
| `max_iter` | 25–40 | Nombre maximal d'itérations |
| `tol` | 1e-2 à 1e-3 | Tolérance de convergence |
| `warm_start_init_point` | yes | Réutilisation de la solution précédente |
| `print_level` | 0 | Pas de sortie verboseuse |

### 8.4 Contraintes d'équilibre de couple (stabilité roulis/tangage)

Pour éviter que le MPC commande des combinaisons de propulseurs qui généreraient un roulis ou un tangage excessif, des contraintes non-linéaires explicites sont imposées sur les couples de stabilisation :

```python
pitch_torque_balance = (t5 - t6 + t7 - t8) + (z_arm / pitch_arm) * F_surge
self.mpc.set_nl_cons('eq_pitch_max', pitch_torque_balance, ub=0.5)
```

---

## 9. Algèbre linéaire appliquée

### 9.1 Matrice d'allocation des propulseurs (TAM)

**Fichiers** : `station_keeping.py`, `sim_thruster_bridge.py`, `mpc_controller_sensors.py`

La **Thruster Allocation Matrix** (TAM) est une matrice 6×8 qui décrit la contribution géométrique de chaque propulseur aux 6 degrés de liberté du corps rigide :

```
τ = TAM · [t1, t2, ..., t8]ᵀ
```

avec `τ = [Fx, Fy, Fz, Mx, My, Mz]ᵀ` le vecteur de forces et couples appliqués.

La configuration angulaire à 45° des propulseurs horizontaux se traduit par :
```
Fx = sin(45°) · (t1 + t2 - t3 - t4) = 0.7071 · (t1 + t2 - t3 - t4)
Fy = sin(45°) · (t1 - t2 + t3 - t4)
```

### 9.2 Pseudo-inverse de Moore-Penrose

**Fichier** : `station_keeping.py`, `sim_thruster_bridge.py`

Le problème inverse (trouver les forces individuelles `t_i` à partir d'un effort désiré `τ`) est un **système sous-déterminé** (6 équations, 8 inconnues). La solution de **norme minimale** est donnée par la pseudo-inverse :

```python
TAM_PINV = np.linalg.pinv(TAM)
thrusts = TAM_PINV @ tau
```

La pseudo-inverse minimise `Σ ti²` (minimisation de la consommation d'énergie) parmi toutes les solutions valides.

### 9.3 Produit matriciel-vecteur (CasADi)

**Fichier** : `mpc_controller_sensors.py`

Dans le cadre symbolique CasADi, le produit `TAM · u_vec` est calculé via `ca.mtimes()`, qui génère le code NLP correspondant avec ses dérivées analytiques.

### 9.4 Décomposition spectrale (valeurs/vecteurs propres)

**Fichier** : `net_local_estimator.py`

`np.linalg.eigh()` calcule les valeurs et vecteurs propres d'une matrice de covariance symétrique réelle. La propriété fondamentale utilisée : les vecteurs propres d'une matrice de covariance sont alignés avec les **axes de variance principale** des données.

---

## 10. Géométrie et transformations spatiales

### 10.1 Quaternions et angles d'Euler

**Présent dans** : `mpc_controller_blueROV.py`, `station_keeping.py`, `net_approach.py`, `net_local_estimator.py`

La représentation d'orientation standard en robotique est le **quaternion** `(x, y, z, w)`, qui évite les singularités des angles d'Euler (blocage de cardan).

Conversion quaternion → angle de lacet (yaw) utilisée partout :
```python
siny_cosp = 2 * (o.w * o.z + o.x * o.y)
cosy_cosp = 1 - 2 * (o.y**2 + o.z**2)
psi = np.arctan2(siny_cosp, cosy_cosp)
```

Conversion angles d'Euler → quaternion :
```python
qz = math.sin(yaw / 2.0)
qw = math.cos(yaw / 2.0)  # (pour une rotation pure autour de Z)
```

### 10.2 Différence angulaire normalisée

Pour calculer la plus courte rotation entre deux angles :
```python
def _angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))
```

Cette formule projette la différence dans `[-π, π]` en exploitant la périodicité des fonctions trigonométriques, évitant les discontinuités à `±π`.

### 10.3 Changement de repère (Corps → Monde)

**Fichier** : `station_keeping.py`

L'erreur de position calculée dans le repère Monde doit être exprimée dans le repère du robot (Body frame) pour commander les propulseurs :

```python
ex_body =  cos(yaw) * ex_world + sin(yaw) * ey_world
ey_body = -sin(yaw) * ex_world + cos(yaw) * ey_world
```

C'est une rotation 2D d'angle `-yaw` (rotation inverse pour passer du repère Monde au repère Corps).

### 10.4 Modèle de force des propulseurs

Conversion entre la vitesse de rotation des hélices et la force générée via la formule hydrodynamique :
```
F = ρ · c · ω²
```
soit :
```python
omega = copysign(sqrt(abs(F / (rho * c))), F / (rho * c))
```

### 10.5 Transformation TF2

**Fichier** : `net_local_estimator.py`

La pose du filet estimée dans le repère capteur est convertie dans le repère `odom` via une transformation TF2 :
```python
transform = tf_buffer.lookup_transform('odom', sensor_frame, Time())
pose_odom = tf2_geometry_msgs.do_transform_pose(pose_sensor.pose, transform)
```

### 10.6 Estimation de rayon par intégration de trajectoire

**Fichier** : `phase3_inspection.py`

Le rayon de l'orbite autour du filet est estimé géométriquement :
```
R = distance_parcourue / angle_accumulé = ∫|vy|dt / ∫|dψ|
```
Cette estimation permet de détecter le changement de géométrie (passage d'un cylindre à un cône).

---

## 11. Programmation système et interopérabilité

### 11.1 Bridge Gazebo ↔ ROS 2 (C++)

**Fichier** : `dvl_bridge_node.cpp`

Le nœud DVL bridge est écrit en **C++** pour des raisons de performance. Il utilise :
- **`gz::transport::Node`** : API de transport Gazebo (publication/souscription native Gazebo)
- **Protobuf** (`gz::msgs::DVLVelocityTracking`) : format de sérialisation binaire efficace utilisé par Gazebo
- **`rclcpp`** : API ROS 2 C++

La conversion Protobuf → ROS 2 est faite manuellement champ par champ.

### 11.2 Bridge matériel réel (MAVROS / ArduSub)

**Fichier** : `bluerov2_bridge.py`

Pour le déploiement sur le vrai BlueROV2, le bridge convertit les forces/couples abstraits en signaux **PWM** pour le pilote de vol **ArduSub** via **MAVROS** :

```python
# Normalisation → PWM [1100, 1900 µs]
norm = np.clip(value / max_val, -1.0, 1.0)
pwm = int(1500 + norm * 400)  # 1500 = neutre, ±400 = pleine puissance
```

Le protocole **MAVLink** (via MAVROS) est la couche de communication standard pour les autopilotes de drones/ROV.

### 11.3 Nœud bridge simulation → propulseurs

**Fichier** : `sim_thruster_bridge.py`

Dans la simulation, le Wrench (force/couple) est converti en commandes individuelles de propulseurs via la pseudo-inverse de la TAM. Il s'agit d'une **couche d'abstraction** qui découple le guidage de l'actionneur physique.

### 11.4 Parsing binaire structuré

**Fichier** : `net_approach.py`

Le module `struct` Python permet d'interpréter des buffers binaires bruts :
```python
px = struct.unpack_from('f', data, base + x_offset)[0]
```
`'f'` signifie `float32` (4 octets), `base` est l'offset du point dans le buffer.

---

## 12. Informatique embarquée et temps réel

### 12.1 Budgets de temps de calcul

Le MPC surveille son temps de résolution et émet un avertissement si le solveur dépasse 250 ms (budget alloué pour un cycle de contrôle à 4 Hz) :
```python
if solve_ms > 250.0:
    self.get_logger().warn(f"MPC solve too slow: {solve_ms:.0f}ms")
```

### 12.2 Mesure du Real-Time Factor (RTF)

**Fichier** : `phase3_inspection.py`

Le RTF mesure la vitesse d'exécution de la simulation par rapport au temps réel :
```python
rtf = sim_elapsed / real_elapsed
```
- `RTF > 1` : la simulation va plus vite que le temps réel.
- `RTF < 1` : la simulation est plus lente (machine trop lente ou physique trop complexe).

### 12.3 Gestion des timeouts capteurs

Si le sonar Sonoptix ne publie pas de données depuis plus de 2 secondes (`LOST_WALL_TIMEOUT`), le nœud bascule dans l'état `LOST_WALL` et exécute un comportement de récupération, illustrant la **robustesse aux pannes capteurs**.

### 12.4 Paramètres dynamiques ROS 2

Les paramètres configurables (fréquence de contrôle, coefficient EMA, etc.) sont déclarés via `declare_parameter()` et lisibles en ligne de commande ou depuis un fichier YAML, permettant une adaptation sans recompilation :

```python
self.declare_parameter('yaw_ema_alpha', 1.0)
alpha = self.get_parameter('yaw_ema_alpha').value
```

### 12.5 Vectorisation NumPy pour la performance

**Fichier** : `phase3_inspection.py`

Le traitement de milliers de points sonar par itération utilise des opérations vectorielles NumPy plutôt que des boucles Python, réduisant drastiquement le temps de traitement :
```python
# Vectorisé (rapide) vs boucle Python (lent)
payload = np.frombuffer(msg.data, dtype=np.float32)
points = payload.reshape(-1, floats_per_point)
dists = np.sqrt(px**2 + py**2 + pz**2)
valid_mask = (dists >= 0.3) & (dists <= 7.0) & np.isfinite(dists)
```

---

## 13. Infrastructure DevOps

### 13.1 Conteneurisation Docker

Un `Dockerfile` produit une image autonome contenant l'intégralité de l'environnement (ROS 2 Jazzy, Gazebo Harmonic, dépendances Python, workspace compilé). Cela garantit la **reproductibilité** de l'expérience sur n'importe quelle machine Linux.

Techniques employées :
- **Multi-stage build** implicite (base image `osrf/ros:jazzy-desktop`)
- **Partage d'affichage X11** (`-v /tmp/.X11-unix:/tmp/.X11-unix`) pour la GUI Gazebo
- **Réseau hôte** (`--net=host`) pour la communication ROS 2 inter-conteneurs
- **`.dockerignore`** pour exclure les artefacts de build locaux

### 13.2 Build système (colcon)

`colcon` est le système de build CMake/Python unifié pour ROS 2 :
- Gestion automatique de l'**ordre de compilation** selon les dépendances (`package.xml`)
- **Build incrémentiel** par package (`--packages-select`)
- **Installation par liens symboliques** (`--symlink-install`) pour les packages Python

### 13.3 Gestion des dépendances (rosdep)

`rosdep` résout et installe automatiquement toutes les dépendances déclarées dans les `package.xml`, en distinguant les packages ROS et les paquets système.

### 13.4 Optimisation de la simulation (mode performance)

Un mode `optimize:=True` applique à la volée (en mémoire, sans modifier les fichiers sources) des patchs XML qui :
- Réduisent le pas de temps physique (1 ms → 6 ms, accélération ×6)
- Diminuent les fréquences de publication des capteurs
- Ajustent les paramètres de filtre (EMA plus agressif)

### 13.5 Visualisation Foxglove Studio

Le bridge `foxglove_bridge` expose les topics ROS 2 via **WebSocket** (port 8765), permettant la visualisation temps réel depuis n'importe quel navigateur web, indépendamment de l'OS et sans installer ROS.

---

## 14. Récapitulatif par fichier source

| Fichier | Package | Principaux concepts |
|---|---|---|
| `mpc_controller_blueROV.py` | `AUV_controller` | MPC, CasADi, IPOPT, modèle 6-DOF, quaternions |
| `mpc_controller_sensors.py` | `AUV_controller` | MPC + EKF, TAM matricielle, NLP, contraintes NL |
| `station_keeping.py` | `AUV_controller` | PID (PID), pseudo-inverse TAM, changement de repère, anti-windup |
| `net_approach.py` | `AUV_guidance` | Machine à états (FSM), Ping360, Sonoptix, parsing binaire, PD |
| `phase3_inspection.py` | `AUV_guidance` | PID multi-axes, filtre médian, EMA, RTF, détection cône, vectorisation NumPy |
| `phase3_inspection_big_net.py` | `AUV_guidance` | Variante grande échelle (même concepts) |
| `bluerov2_bridge.py` | `AUV_guidance` | MAVROS, PWM, ArduSub, normalisation |
| `sim_thruster_bridge.py` | `AUV_guidance` | Pseudo-inverse, TAM, abstraction couche actionneur |
| `dvl_bridge_node.cpp` | `auv_dvl_bridge` | C++, gz-transport, Protobuf, covariance, ROS/Gazebo bridge |
| `net_local_estimator.py` | `auv_perception` | PCA, valeurs propres, PointCloud2, TF2, quaternions |
| `sonar_filter_node.py` | `auv_perception` | Filtrage PointCloud2, NumPy vectorisé |
| `auto_saver_node.py` | `auv_perception` | OctoMap, persistence |
| `simulated_depth_sensor.py` | `AUV_description` | Bruit gaussien, covariance 6×6, simulation capteur |
| `imu_republisher.py` | `AUV_description` | Re-publication IMU, topics ROS 2 |
| `ekf.yaml` | `my_auv_localization` | EKF, fusion de capteurs, matrices de covariance |
| `BlueROV2*.urdf.xml` | `AUV_description` | URDF, cinématique, inertie, plugins Gazebo |
| `*.xml` (worlds) | `AUV_description` | SDF, simulation physique, environnements sous-marins |

---

*Document généré automatiquement à partir de l'analyse du code source du projet.*  
*Projet : ROS 2 AUV BlueROV2 — Inspection autonome de filets d'aquaculture.*
