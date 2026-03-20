# 🤖 Plan d'Amélioration de la Simulation Robotique

---

## 1. Amélioration de la Phase 4 (Inspection du Filet)
*Anciennement Point 3*

* **Problème actuel :** Le MPC manque de précision et de stabilité. Le robot ne parvient pas à rester face au filet ni à proximité immédiate.
* **Objectifs :**
    * Stabiliser l'asservissement pour maintenir une position fixe par rapport au filet.
    * Assurer que le robot reste orienté face à la structure.
    * Permettre une inspection visuelle ou sensorielle fiable.

---

## 2. Optimisation de la Phase 2 (Transitions & Vitesse)
*Anciennement Point 1*

* **Améliorations requises :**
    * Intégrer un **MPC (Model Predictive Control)** pour gérer de manière fluide les transitions entre les différentes étapes.
    * Optimiser les paramètres pour augmenter la vitesse globale d'exécution du robot.

---

## 3. Localisation et Initialisation (Recherche de Repères)
*Anciennement Point 2*

* **Concept de "Drop Aléatoire" :** Le robot est parachuté à un endroit inconnu dans le filet, sans connaître la position du centre.
* **Objectifs :**
    * Le robot doit utiliser ses **capteurs embarqués** pour se repérer de manière autonome.
    * Simuler une perte totale de connaissance de l'environnement au démarrage pour tester la robustesse de la localisation.

---

## 4. Phase Finale (Inspection de la Partie Conique)
*Anciennement Point 4*

* **Objectif :** Réaliser l'inspection détaillée de la section conique de la structure une fois que toutes les étapes de navigation et de stabilisation précédentes sont validées.