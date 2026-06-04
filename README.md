# 🚗 Système d'Aide au Maintien de Voie (LKA)
### Projet — Technologie de l'Automobile

[![Scilab](https://img.shields.io/badge/Scilab-76.5%25-blue?logo=data:image/png;base64,)](https://www.scilab.org/)
[![C](https://img.shields.io/badge/C-19.4%25-lightgrey?logo=c)](https://en.wikipedia.org/wiki/C_(programming_language))
[![C++](https://img.shields.io/badge/C%2B%2B-4.1%25-00599C?logo=c%2B%2B)](https://isocpp.org/)
[![License](https://img.shields.io/badge/License-Academic-green)]()

---

## 📋 Description

Ce projet porte sur la **modélisation, la simulation et l'implémentation d'un système LKA (*Lane Keeping Assist*)** — une technologie d'aide à la conduite permettant de maintenir automatiquement le véhicule dans sa voie de circulation. Il s'inscrit dans le cadre du cours de **Technologie de l'Automobile** à l'**ENSA Tétouan**, et suit une méthodologie de développement **itérative par sprints**.

Le système LKA détecte les marquages de voies sur la chaussée, estime la déviation latérale du véhicule, et génère des corrections de direction pour maintenir la trajectoire dans la voie.

---

## 🎯 Objectifs

- Comprendre le fonctionnement d'un système ADAS de type LKA
- Modéliser la dynamique latérale du véhicule
- Implémenter des algorithmes de détection de voie et de contrôle en boucle fermée
- Simuler le comportement du système sous **Scilab/Xcos**
- Valider les performances via des scénarios de test représentatifs

---

## 🗂️ Structure du Projet

Le projet est organisé en **6 sprints** correspondant aux différentes phases de développement :

```
Projet-technologie-de-l-automobile-LKA/
│
├── sprint 1/    # Analyse du cahier des charges & modélisation du véhicule
├── sprint 2/    # Modèle dynamique latéral & représentation d'état
├── sprint 3/    # Détection des marquages de voie (traitement de signal/image)
├── sprint 4/    # Conception du contrôleur (PID / LQR / autre)
├── sprint 5/    # Intégration & simulation complète du système
└── sprint 6/    # Validation, tests et rapport final
```

---

## 🔧 Technologies & Outils

| Outil | Utilisation |
|-------|-------------|
| **Scilab / Xcos** | Simulation et modélisation (76.5 % du code) |
| **C** | Algorithmes embarqués bas niveau (19.4 %) |
| **C++** | Modules de traitement supplémentaires (4.1 %) |
| **Git / GitHub** | Gestion de version & collaboration |

---

## ⚙️ Fonctionnement du Système LKA

```
┌──────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Capteurs   │────▶│  Détection voie  │────▶│   Contrôleur    │
│ (caméra/IRL) │     │ (traitement sig.)│     │ (correction dir)│
└──────────────┘     └──────────────────┘     └────────┬────────┘
                                                        │
                                               ┌────────▼────────┐
                                               │    Actionneur   │
                                               │ (direction EPS) │
                                               └─────────────────┘
```

1. **Perception** : détection des lignes de voie par traitement du signal / image
2. **Estimation** : calcul de la déviation latérale et de l'angle de cap
3. **Décision** : le contrôleur génère une consigne de correction
4. **Action** : envoi de la commande à la direction assistée électrique (EPS)

---

## 🚀 Démarrage Rapide

### Prérequis

- [Scilab 6.x ou supérieur](https://www.scilab.org/download/) (pour les simulations `.sce` / `.zcos`)
- Compilateur C/C++ (GCC recommandé pour les modules embarqués)
- Git

### Installation

```bash
# Cloner le dépôt
git clone https://github.com/MohamedReda2003/Projet-technologie-de-l-automobile-LKA.git
cd Projet-technologie-de-l-automobile-LKA
```

### Exécution (Scilab)

```scilab
// Depuis la console Scilab
cd("chemin/vers/sprint 5")
exec("main_simulation.sce")
```

---

## 📊 Résultats de Simulation

Les simulations réalisées sous Scilab/Xcos permettent d'observer :

- La **trajectoire latérale** du véhicule avec et sans LKA
- L'**erreur de centrage** dans la voie au cours du temps
- La **commande de braquage** générée par le contrôleur
- Les **marges de stabilité** du système en boucle fermée

---

## 👥 Équipe

Projet réalisé par des étudiants en **Génie Mécatronique** — ENSA Tétouan, Maroc.


*(voir [Contributors](https://github.com/MohamedReda2003/Projet-technologie-de-l-automobile-LKA/graphs/contributors))*

---

## 📚 Références

- ISO 11270 — *Intelligent Transport Systems — Lane Keeping Assist Systems*
- R. Rajamani, *Vehicle Dynamics and Control*, Springer, 2006
- Documentation Scilab — [https://help.scilab.org](https://help.scilab.org)
- Cours de Technologie de l'Automobile — ENSA Tétouan

---

## 📄 Licence

Ce projet est réalisé dans un cadre académique. Toute réutilisation doit mentionner la source.

---

> *ENSA Tétouan — Génie Mécatronique — Année académique 2025/2026*
