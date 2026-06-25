# 🤿 Workspace Overview — `ros2_AUV`

> **Auteur** : titou  
> **Plateforme** : BlueROV2 (8 propulseurs)  
> **Framework** : ROS 2 (Humble/Jazzy) + Gazebo (gz-sim)  
> **Objectif** : Inspection autonome d'un filet d'aquaculture sous-marin

---

## 📁 Structure globale

```
ros2_AUV/
├── src/                          ← Tous les packages ROS 2
│   ├── AUV_guidance/             ← Guidage & missions (Python)
│   ├── AUV_controller/           ← Contrôleurs bas niveau (Python)
│   ├── AUV_description/          ← Description robot, URDF, mondes Gazebo
│   ├── auv_perception/           ← Traitement sonar & perception (Python)
│   ├── my_auv_localization/      ← Localisation EKF (config ROS 2)
│   ├── auv_dvl_bridge/           ← Pont Gazebo DVL → ROS 2 (C++)
│   └── asv_wave_sim/             ← Simulation de vagues (gz-waves, dépendance externe)
├── build/                        ← Artefacts de compilation (colcon)
├── install/                      ← Installation colcon
├── log/                          ← Logs colcon
├── scratch/                      ← Fichiers temporaires / essais
├── 2026-02-05_*.bag              ← Bags d'enregistrement réels (~1.2–1.4 Go chacun)
├── Dockerfile                    ← Image Docker du projet
├── bag_info.py                   ← Script d'inspection des bags
├── test.urdf                     ← URDF de test
└── README.md                     ← README principal du projet
```

---

## 🧭 Architecture générale de la mission

Le projet implémente une **mission en 3 phases** pour l'inspection autonome d'un filet de cage aquacole :

```
Phase 1 — Descente initiale
      │
      ▼
Phase 2 — Approche du filet  [AUV_guidance / net_approach]
      │  DESCENDING → GLOBAL_SEARCH → ALIGNING → APPROACHING → STABILIZING → STANDOFF
      ▼
Phase 3 — Inspection orbitale [AUV_guidance / phase3_inspection]
           WAITING → WALKING_THE_NET → (LOST_WALL) → LAP_COMPLETED
```

### Flux de données principal

```
Gazebo Simulation
  ├── /ping360/scan        (LaserScan)     →  auv_perception/ping360_nearest
  │                                               → /perception/net_orientation (PoseStamped)
  │                                               → /perception/full_scan_ready (Bool)
  │
  ├── /sonoptix/points     (LaserScan)     →  auv_perception/sonoptix_2D_perception
  │                                               → /perception/net_distance (Float32)
  │                                               → /perception/net_yaw_target (Float32)
  │                                               → /perception/perception_valid (Bool)
  │
  ├── /imu                 (IMU)           →  imu_republisher → /imu/fixed
  ├── /dvl/velocity        (Gz protobuf)   →  auv_dvl_bridge  → /dvl/velocity_ros
  └── /odom                (Odometry)      → [brut Gazebo]
                                                    ↓
                                        my_auv_localization (EKF)
                                                    ↓
                                        /odometry/filtered
                                                    ↓
                              AUV_guidance (Phase 2 & 3)
                                                    ↓
                                        /auv/command_wrench (Wrench)
                                                    ↓
                              AUV_guidance/sim_thruster_bridge
                                                    ↓
                                        /cmd_vel_1..8 (Float64)
                                                    ↓
                                     8 propulseurs Gazebo
```

---

## 📦 Package `AUV_guidance`

**Rôle** : Guidage de haut niveau — machines à états de mission.

### Nœuds

| Fichier | Nœud ROS 2 | Phase | Description |
|---|---|---|---|
| `net_approach.py` | `phase2_mission` | Phase 2 | Approche du filet via Ping360 (3D Sonoptix) |
| `net_approach_2D_sono.py` | `phase2_mission` | Phase 2 | **Variante 2D** : Sonoptix LaserScan + Float32 |
| `phase3_inspection.py` | `phase3_inspection` | Phase 3 | Orbite autour du filet (Sonoptix 3D) |
| `phase3_inspection_2D_sono.py` | `phase3_inspection` | Phase 3 | **Variante 2D** (Sonoptix LaserScan) |
| `phase3_inspection_big_net.py` | `phase3_inspection` | Phase 3 | Variante grands filets |
| `sim_thruster_bridge.py` | `sim_thruster_bridge` | Infra | Convertit `/auv/command_wrench` → `/cmd_vel_1..8` |
| `bluerov2_bridge.py` | `bluerov2_bridge` | Infra | Pont MAVROS pour le vrai BlueROV2 |
| `real_test/inspection_real_test.py` | — | Tests réels | Version adaptée pour tests en bassin réel |
| `real_test/net_approach_real_test.py` | — | Tests réels | Approche adaptée pour tests réels |

### Machine à états — Phase 2 (`net_approach_2D_sono.py`)

```
DESCENDING
    │  (profondeur cible atteinte pendant 2s)
    ▼
GLOBAL_SEARCH
    │  (rotation 360° du Ping360, détection orientation du filet)
    ▼
ALIGNING
    │  (PD yaw jusqu'à < 10° d'erreur pendant 1s)
    ▼
APPROACHING
    │  (avance vers le filet, contrôle P sur distance Sonoptix)
    ▼
STABILIZING
    │  (maintien position pendant 3s)
    ▼
STANDOFF
    │  (publie TF local_origin pour Phase 3)
    ▼  → /mission/phase2_done = True
```

**Topics clés (Phase 2)** :
- Entrées : `/odometry/filtered`, `/perception/net_orientation`, `/perception/full_scan_ready`, `/perception/net_distance`
- Sorties : `/auv/command_wrench`, `/mission/phase`, `/mission/phase2_done`, `/mission/local_origin`

### Machine à états — Phase 3 (`phase3_inspection.py`)

```
WAITING
    │  (/mission/phase2_done = True)
    ▼
WALKING_THE_NET
    │  (orbite le long du filet, contrôle PID Fz/Fx/Fy/Mz)
    │  (détecte fin de tour via yaw accumulé ≥ 2π)
    │  ← → LOST_WALL (si sonar perdu > 2s)
    ▼
LAP_COMPLETED
    │  (descend d'un palier de profondeur ou fin de mission)
```

**Contrôle Phase 3** :
- PID profondeur (Fz)
- PID distance filet (Fx)
- PID vitesse sway (Fy — vitesse orbitale)
- PID yaw (Mz — alignement perpendiculaire au filet)
- PID pitch (My — mode cône, uniquement si `in_cone_mode`)

### Fichiers launch

| Fichier | Description |
|---|---|
| `net_full_inspection.launch.py` | Mission complète (Sonoptix 3D) |
| `net_full_inspection_true_auv.launch.py` | **Mission principale** : Sonoptix 2D + Ping360, supporte `use_hardware:=True` |
| `net_inspection_big_net.launch.py` | Variante grands filets |
| `real_test.launch.py` | Tests en bassin réel |

**Paramètres du launch principal** (`net_full_inspection_true_auv`) :
```bash
headless:=False        # Mode sans interface graphique Gazebo
rviz:=False            # Lancer RViz2
world_file:=small_net.xml
gz_delay:=8.0          # Délai avant démarrage des nœuds mission
use_hardware:=False    # True = utilise MAVROS + BlueROV2 réel
optimize:=False        # True = physique allégée, contrôle à 5 Hz
```

---

## 📦 Package `AUV_controller`

**Rôle** : Contrôleurs de bas niveau.

### Nœuds

| Fichier | Nœud | Description |
|---|---|---|
| `mpc_controller_blueROV.py` | `mpc_controller_bluerov` | MPC non-linéaire (do_mpc + CasADi) — simulation parfaite |
| `mpc_controller_sensors.py` | — | MPC avec fusion de capteurs (EKF) |
| `station_keeping.py` | `station_keeping` | Maintien de position PD (baseline) |
| `look_the_wall.py` | — | Contrôleur basique face au mur |

### `mpc_controller_blueROV.py` — Architecture

- **Modèle** : 8 états `[x, y, z, ψ, u, v, w, r]`, 8 entrées (poussées T1..T8)
- **Dynamique** : cinématique + hydrodynamique (masse ajoutée, traînée linéaire + quadratique)
- **Horizon** : N=12 pas, Δt=0.1s → prévision sur 1.2s
- **Solveur** : IPOPT (max 40 itérations, warm-start)
- **Coût** : erreur position (×50), erreur profondeur (×100), régularisation vitesses angulaires et poussées

### `station_keeping.py` — Architecture

- **Contrôle** : PD (position XY + yaw) + PID (profondeur Z avec intégrateur anti-windup)
- **Allocation** : Matrice TAM 6×8 + pseudo-inverse Moore-Penrose
- **Cadence** : 20 Hz

### `tools/`

| Fichier | Description |
|---|---|
| `move_down.py` | Descente simple |
| `move_forward.py` | Avance simple |

---

## 📦 Package `auv_perception`

**Rôle** : Traitement des données sonar pour détecter et localiser le filet.

### Nœuds

| Fichier | Nœud | Sonar | Méthode |
|---|---|---|---|
| `ping360_nearest.py` | `ping360_nearest` | Ping360 (LaserScan) | DBSCAN + RANSAC polynôme deg-2 + ratio inliers |
| `sonoptix_perception.py` | `sonoptix_perception` | Sonoptix ECHO (PointCloud2) | RANSAC plan 3D (Open3D ou sklearn) |
| `sonoptix_2D_perception.py` | `sonoptix_2D_perception` | Sonoptix ECHO (LaserScan) | RANSAC polynôme deg-2 2D |
| `ping360_circle_fitting.py` | `ping360_circle_fitting` | Ping360 (PointCloud2) | RANSAC cercle → rayon/centre cage |
| `ping360_bridge_player.py` | — | — | Rejoue des bags Ping360 |
| `auto_saver_node.py` | — | — | Sauvegarde automatique OctoMap |
| `bag_to_ply.py` | — | — | Convertit bags → fichiers PLY |
| `bt_to_ply.py` | — | — | Convertit OctoMap .bt → PLY |

### `ping360_nearest.py` — Pipeline (v3)

```
LaserScan → TF2 transform (sensor → odom) → buffer circulaire (1 rotation)
    ↓
Déclenchement : fin de rotation (360° ou angular wrap-around ou fallback temporel)
    ↓
DBSCAN (eps=0.25m, min_pts=5) → clusters
    ↓
Pour chaque cluster : RANSAC polynôme deg-2 → ratio inliers
    ↓
Sélection : cluster avec ratio ≥ 30% (= filet, pas banc de poissons)
    ↓
Point le plus proche sur la courbe → tangente → normale → yaw cible
    ↓
/perception/net_orientation (PoseStamped) + /perception/full_scan_ready (Bool)
```

### `sonoptix_2D_perception.py` — Pipeline

```
LaserScan @ 25 Hz → filtre range [0.3m, 7.0m] → tableau (N, 2) Cartésien
    ↓
RANSAC polynôme deg-2 (heuristique swap-axes pour singularité verticale)
    ↓
Point le plus proche de l'origine → distance + normale → yaw (filtre EMA α=0.25)
    ↓
/perception/net_distance (Float32) + /perception/net_yaw_target (Float32) + valid
```

### `sonoptix_perception.py` — Pipeline (3D)

```
PointCloud2 → décodage NumPy vectorisé → filtre range 3D
    ↓
RANSAC plan 3D (Open3D segment_plane ou sklearn RANSACRegressor en fallback)
    ↓
normale [nx, ny, nz] → distance = |d| → yaw = atan2(ny,nx) → pitch = arcsin(nz)
    ↓
Encodage quaternion ZYX → /sonoptix/perception (PoseStamped) + /sonoptix/perception_valid
```

---

## 📦 Package `AUV_description`

**Rôle** : Description physique du robot et environnements de simulation.

### URDF

| Fichier | Description |
|---|---|
| `BlueROV2.urdf.xml` | URDF minimal |
| `BlueROV2captors.urdf.xml` | Avec tous les capteurs |
| `Bluerov2_realistic.urdf.xml` | Version réaliste (Sonoptix 3D) |
| `Bluerov2_realistic_2D.urdf.xml` | **Version utilisée en mission** (Sonoptix 2D LaserScan) |

**Capteurs simulés** :
- Ping360 (`LibRayPlugin` → LaserScan 360°)
- Sonoptix ECHO (`LibRayPlugin` → LaserScan ou PointCloud2)
- IMU
- DVL (Doppler Velocity Log → Gazebo protobuf)
- Caméra (optionnelle)

### Mondes Gazebo (`world/`)

| Fichier | Description |
|---|---|
| `small_net.xml` | **Monde principal** — petit filet carré dans un bassin |
| `small_net_current.xml` | Avec courant sous-marin |
| `small_net_deforme.xml` | Filet déformé |
| `Bassin_ntnu.xml` | Bassin NTNU (environnement réaliste) |
| `Bassin_ntnu_waves.xml` | Bassin NTNU avec vagues (asv_wave_sim) |
| `ocean_40m.xml` | Océan ouvert 40m de profondeur |
| `cube_obstacle.xml` | Obstacle cube (tests d'évitement) |

### Modèles 3D (`models/`)

- `fish_net/` — Filet aquacole
- `flexible_net/` — Filet déformable

### Scripts utilitaires (`scripts/`)

- `simulated_depth_sensor` — Publie la profondeur depuis `/odom` sur `/depth/pose`
- `imu_republisher` — Reformate le topic IMU

---

## 📦 Package `my_auv_localization`

**Rôle** : Fusion de capteurs via filtre de Kalman étendu (EKF).

### Configuration EKF (`config/ekf.yaml`)

| Capteur | Topic | Variables fusionnées |
|---|---|---|
| IMU | `/imu/fixed` | Vitesses angulaires (ωx, ωy, ωz) |
| DVL | `/dvl/velocity_ros` | Vitesses linéaires corps (Vx, Vy, Vz) |
| Profondeur | `/depth/pose` | Position Z absolue |

**Sortie** : `/odometry/filtered` à **30 Hz**

**Cadence** : 30 Hz, mode 3D complet (pas de `two_d_mode`), TF `odom → base_link` publié.

---

## 📦 Package `auv_dvl_bridge`

**Rôle** : Pont C++ entre Gazebo DVL et ROS 2.

### `dvl_bridge_node.cpp`

- **Entrée** : `/dvl/velocity` (Gazebo Transport, protobuf `DVLVelocityTracking`)
- **Sortie** : `/dvl/velocity_ros` (`geometry_msgs/TwistWithCovarianceStamped`)
- **Correction** : Frame forcée à `base_link` pour compatibilité EKF
- **Covariance** : Propagée depuis Gazebo ou fallback diagonal `0.01`

---

## 📦 Package `asv_wave_sim`

**Rôle** : Simulation de vagues Gazebo (dépendance externe, gz-waves).

Composé de :
- `gz-waves/` — Plugin Gazebo de génération de vagues
- `gz-waves-models/` — Modèles Gazebo pour les vagues

Utilisé dans le monde `Bassin_ntnu_waves.xml`.

---

## 🔗 Graphe des topics principaux

```
Ping360                                         Sonoptix ECHO
/ping360/scan (LaserScan)                      /sonoptix/points (LaserScan)
       │                                                │
       ▼                                                ▼
ping360_nearest                               sonoptix_2D_perception
       │                                                │
       ├─ /perception/net_orientation                   ├─ /perception/net_distance
       └─ /perception/full_scan_ready                   ├─ /perception/net_yaw_target
                          │                             └─ /perception/perception_valid
                          │                                          │
                          └──────────────┬──────────────────────────┘
                                         ▼
                               net_approach_2D_sono  (Phase 2)
                                         │
                               /mission/phase2_done = True
                                         │
                                         ▼
                               phase3_inspection_2D_sono  (Phase 3)
                                         │
                               /auv/command_wrench (Wrench)
                                         │
                               sim_thruster_bridge
                                         │
                               /cmd_vel_1..8 (Float64)
                                         │
                               8 propulseurs BlueROV2
```

---

## 🎯 Lancement typique

### Simulation complète (mission filet)

```bash
ros2 launch AUV_guidance net_full_inspection_true_auv.launch.py
```

### Options avancées

```bash
# Sans interface graphique (headless)
ros2 launch AUV_guidance net_full_inspection_true_auv.launch.py headless:=True

# Avec RViz2
ros2 launch AUV_guidance net_full_inspection_true_auv.launch.py rviz:=True

# Mode performance (physique allégée, 5 Hz)
ros2 launch AUV_guidance net_full_inspection_true_auv.launch.py optimize:=True

# Sur vrai BlueROV2 (MAVROS)
ros2 launch AUV_guidance net_full_inspection_true_auv.launch.py use_hardware:=True

# Filet déformé avec courant
ros2 launch AUV_guidance net_full_inspection_true_auv.launch.py world_file:=small_net_current.xml
```

### Visualisation Foxglove

```bash
ros2 run foxglove_bridge foxglove_bridge
```

---

## 📊 Fichiers de données

| Fichier | Taille | Date |
|---|---|---|
| `2026-02-05_11-03-27_data.bag` | ~1.11 Go | 5 fév. 2026, 11h03 |
| `2026-02-05_11-08-20_data.bag` | ~1.19 Go | 5 fév. 2026, 11h08 |
| `2026-02-05_11-16-19_data.bag` | ~1.35 Go | 5 fév. 2026, 11h16 |

Ces bags contiennent des enregistrements de tests réels (vraisemblablement en bassin avec le vrai BlueROV2).

---

## 🛠️ Dépendances principales

| Bibliothèque | Usage |
|---|---|
| `rclpy` | Nœuds ROS 2 Python |
| `rclcpp` | Nœuds ROS 2 C++ |
| `numpy` | Calcul vectoriel |
| `sklearn` (scikit-learn) | DBSCAN, RANSAC fallback |
| `open3d` | RANSAC plan 3D (backend principal) |
| `do_mpc` | Contrôleur MPC non-linéaire |
| `casadi` | Optimisation symbolique (utilisé par do_mpc) |
| `robot_localization` | EKF (package ROS 2) |
| `ros_gz_bridge` | Pont Gazebo ↔ ROS 2 |
| `gz-waves` | Simulation de vagues |
| `mavros` | Interface BlueROV2 réel (optionnel) |

---

## 📝 Notes importantes

1. **Le fichier `Bluerov2_realistic_2D.urdf.xml`** est le URDF actif en mission — le Sonoptix y est configuré comme un LaserScan 2D (et non PointCloud2).

2. **La Phase 2 actuelle** (`net_full_inspection_true_auv.launch.py`) utilise :
   - `net_approach_2D_sono` (Sonoptix LaserScan Float32)
   - `phase3_inspection_2D_sono` (pipeline 2D)

3. **Le Ping360** est utilisé **uniquement** pour la phase de recherche globale (`GLOBAL_SEARCH`) et la détection du rayon de la cage (`ping360_circle_fitting`).

4. **Le Sonoptix** prend le relais en Phase 2 APPROACHING et assure tout le contrôle en Phase 3.

5. **Le spawn** est randomisé sur un cercle de rayon 3.5m autour du filet avec le nez pointé vers le filet.

6. **La détection de fin de tour** en Phase 3 utilise le yaw accumulé (`≥ 2π rad`) pour savoir quand une orbite complète est effectuée. La profondeur descend ensuite d'un pas de 0.5m jusqu'à -6m.
