// ============================================================================
// PROJET LKA - DIAGRAMME XCOS COMPLET
// Système Line Keeping Assistant avec contrôleur LQR
// ============================================================================
// Ce fichier décrit le diagramme Xcos à construire pour le Sprint 3
// ============================================================================

// Pour créer le diagramme dans Xcos :
// 1. Ouvrir Scilab
// 2. Taper : xcos
// 3. Construire le diagramme selon les instructions ci-dessous
// 4. Sauvegarder sous : Sprint3_Diagramme_LKA.zcos

// ============================================================================
// INSTRUCTIONS DE CONSTRUCTION DU DIAGRAMME XCOS
// ============================================================================

/*

┌─────────────────────────────────────────────────────────────────────────────┐
│                    DIAGRAMME XCOS - SYSTÈME LKA COMPLET                      │
│                                                                             │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐                   │
│  │   STEP      │────→│   SUM       │────→│  INTEGRAL   │──→ y (écart)    │
│  │ (écart      │     │  (erreur)   │     │  (1/s)      │                   │
│  │  initial)   │     │             │     │             │                   │
│  └─────────────┘     └──────┬──────┘     └─────────────┘                   │
│                             ↑                                               │
│                             │                                               │
│  ┌─────────────┐     ┌────┴────┐                                          │
│  │   GAIN      │←────│   K     │←── [beta, r, y, psi]                     │
│  │  (K_y,      │     │  (LQR)  │                                          │
│  │   K_psi)    │     └────┬────┘                                          │
│  └──────┬──────┘          ↑                                               │
│         │                 │                                                 │
│         │            ┌────┴────┐                                          │
│         │            │  STATE-  │                                          │
│         │            │  SPACE   │                                          │
│         │            │  (A,B,   │                                          │
│         │            │   C,D)   │                                          │
│         │            └────┬────┘                                          │
│         │                 │                                                 │
│         │            ┌────┴────┐                                          │
│         └───────────→│   SUM    │←── PERTURBATION (vent)                   │
│                      │          │                                          │
│                      └────┬─────┘                                          │
│                           │                                                 │
│                      ┌────┴────┐                                          │
│                      │  SATUR   │  ←── Limitation ±5° (REQ-LKA-003)        │
│                      │ (±5°)    │                                          │
│                      └────┬────┘                                          │
│                           │                                                 │
│                      ┌────┴────┐                                          │
│                      │  SCOPE   │  ←── Visualisation delta(t)              │
│                      └─────────┘                                          │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘

*/

// ============================================================================
// MÉTHODE ALTERNATIVE : Création par script Scilab
// ============================================================================

// Cette méthode crée le diagramme Xcos par programmation

clear; clc;

// Charger la palette Xcos
loadXcosLibs();

// Créer un nouveau diagramme
diagram = scicos_diagram();

// ============================================================================
// PARAMÈTRES DU SYSTÈME (mêmes que dans le script principal)
// ============================================================================
m = 1800; L = 2.875; a = 1.15; b = 1.725;
Iz = 3500; Cf = 120000; Cr = 180000; V = 30;

a11 = -(Cf + Cr) / (m * V);
a12 = -1 + (b*Cr - a*Cf) / (m * V^2);
a21 = (b*Cr - a*Cf) / Iz;
a22 = -(a^2 * Cf + b^2 * Cr) / (Iz * V);

A = [a11, a12, 0, 0; a21, a22, 0, 0; V, 0, 0, V; 0, 1, 0, 0];
B = [Cf/(m*V); a*Cf/Iz; 0; 0];
C = [0, 0, 1, 0; 0, 0, 0, 1];
D = [0; 0];

// Gain LQR
Q = diag([1, 1, 100, 50]);
R = 10;
P = riccati(A, B*inv(R)*B', Q, 'c');
K = inv(R) * B' * P;

// ============================================================================
// BLOC 1 : Générateur de step (écart initial)
// ============================================================================
// Position : (50, 100)
// Paramètres : Step time = 0, Initial value = 0.5, Final value = 0.5

STEP = STEP_FUNCTION("define");
STEP.graphics.orig = [50, 100];
STEP.graphics.sz = [40, 40];
STEP.graphics.flip = %f;
STEP.model.rpar = [0; 0.5; 0.5];  // [step_time, initial, final]
STEP.model.ipar = [];

// ============================================================================
// BLOC 2 : Sommateur (comparateur)
// ============================================================================
// Position : (150, 100)
// Paramètres : [1; -1] (entrée +, entrée -)

SUM = SUMMATION("define");
SUM.graphics.orig = [150, 100];
SUM.graphics.sz = [40, 40];
SUM.graphics.flip = %f;
SUM.model.ipar = [1; -1];  // Signes des entrées

// ============================================================================
// BLOC 3 : Espace d'état du véhicule (CSS)
// ============================================================================
// Position : (250, 100)
// Paramètres : A, B, C, D, x0

CSS = CSS_BLOCK("define");
CSS.graphics.orig = [250, 80];
CSS.graphics.sz = [60, 60];
CSS.graphics.flip = %f;
CSS.model.rpar = [A(:); B(:); C(:); D(:)];
CSS.model.ipar = [4; 1; 2; 4];  // [nx, nu, ny, nz]

// ============================================================================
// BLOC 4 : Gain LQR (MATMUL)
// ============================================================================
// Position : (350, 50)
// Paramètres : Matrice K (1x4)

GAIN = GAINBLK_f("define");
GAIN.graphics.orig = [350, 50];
GAIN.graphics.sz = [40, 40];
GAIN.graphics.flip = %f;
GAIN.model.rpar = K(:);

// ============================================================================
// BLOC 5 : Saturation (±5°)
// ============================================================================
// Position : (200, 200)
// Paramètres : Upper limit = 5° en rad, Lower limit = -5° en rad

SAT = SATURATION("define");
SAT.graphics.orig = [200, 200];
SAT.graphics.sz = [40, 40];
SAT.graphics.flip = %f;
SAT.model.rpar = [-5*%pi/180; 5*%pi/180];

// ============================================================================
// BLOC 6 : Scope (visualisation)
// ============================================================================
// Position : (450, 100)

SCOPE = CMSCOPE("define");
SCOPE.graphics.orig = [450, 80];
SCOPE.graphics.sz = [60, 60];
SCOPE.graphics.flip = %f;
SCOPE.model.ipar = [1; 1; 1; 1; 1];  // 1 courbe, couleurs par défaut

// ============================================================================
// BLOC 7 : Perturbation vent (step)
// ============================================================================
// Position : (50, 250)
// Paramètres : Step time = 0.5, Initial = 0, Final = 500/(m*V)

WIND = STEP_FUNCTION("define");
WIND.graphics.orig = [50, 250];
WIND.graphics.sz = [40, 40];
WIND.graphics.flip = %f;
WIND.model.rpar = [0.5; 0; 500/(m*V)];

// ============================================================================
// BLOC 8 : Intégrateur pour l'écart latéral
// ============================================================================
// Position : (350, 150)

INTEG = INTEGRAL_f("define");
INTEG.graphics.orig = [350, 150];
INTEG.graphics.sz = [40, 40];
INTEG.graphics.flip = %f;

// ============================================================================
// CONNEXIONS ENTRE LES BLOCS
// ============================================================================

/*
Connexions à établir dans Xcos :

1. STEP (sortie) ──→ SUM (entrée 1)
2. SUM (sortie) ──→ CSS (entrée u)
3. CSS (sortie y) ──→ SCOPE (entrée 1)  [visualisation y]
4. CSS (sortie x) ──→ GAIN (entrée)     [état complet vers K]
5. GAIN (sortie) ──→ SAT (entrée)       [commande vers saturation]
6. SAT (sortie) ──→ SUM (entrée 2)      [retour négatif]
7. WIND (sortie) ──→ CSS (entrée perturbation) [si modèle augmenté]

Pour les signaux internes du CSS (x = [beta, r, y, psi]),
il faut utiliser un démultiplexeur pour extraire chaque état.
*/

// ============================================================================
// SAUVEGARDE DU DIAGRAMME
// ============================================================================

// Sauvegarder le diagramme
// save("Sprint3_Diagramme_LKA.zcos", diagram);

disp("============================================================");
disp("INSTRUCTIONS POUR CRÉER LE DIAGRAMME XCOS");
disp("============================================================");
disp("");
disp("MÉTHODE 1 - Manuelle (recommandée pour le projet) :");
disp("  1. Ouvrir Scilab");
disp("  2. Taper : xcos");
disp("  3. Dans la palette, ajouter les blocs suivants :");
disp("     • Sources → STEP_FUNCTION (écart initial)");
disp("     • Math Operations → SUMMATION (comparateur)");
disp("     • Continuous → CLR (ou CSS pour espace d'état)");
disp("     • Math Operations → GAINBLK (gain LQR)");
disp("     • Discontinuities → SATURATION (±5°)");
disp("     • Sinks → SCOPE (visualisation)");
disp("     • Signal Routing → DEMUX (démultiplexeur pour les états)");
disp("  4. Connecter les blocs selon le schéma ci-dessus");
disp("  5. Configurer les paramètres de chaque bloc");
disp("  6. Sauvegarder : Sprint3_Diagramme_LKA.zcos");
disp("");
disp("MÉTHODE 2 - Par script (avancée) :");
disp("  Exécuter ce fichier .sce dans Scilab");
disp("  Puis utiliser la commande : save('Sprint3_Diagramme_LKA.zcos', diagram)");
disp("");
disp("============================================================");

// ============================================================================
// PARAMÈTRES DES BLOCS XCOS (pour référence)
// ============================================================================

disp("");
disp("PARAMÈTRES DES BLOCS :");
disp("-" * 60);
disp("");
disp("BLOC STEP (Écart initial) :");
disp("  Step time    = 0");
disp("  Initial value = 0.5");
disp("  Final value   = 0.5");
disp("");
disp("BLOC CSS (Véhicule) :");
disp("  A = matrice 4x4 (voir script principal)");
disp("  B = matrice 4x1");
disp("  C = [0 0 1 0; 0 0 0 1]");
disp("  D = [0; 0]");
disp("  x0 = [0; 0; 0.5; 0]");
disp("");
disp("BLOC GAIN (LQR) :");
disp("  K = [" + string(K(1)) + ", " + string(K(2)) + ", " + string(K(3)) + ", " + string(K(4)) + "]");
disp("");
disp("BLOC SATURATION :");
disp("  Upper limit = " + string(5*%pi/180) + " rad (5°)");
disp("  Lower limit = " + string(-5*%pi/180) + " rad (-5°)");
disp("");
disp("BLOC STEP (Vent) :");
disp("  Step time    = 0.5");
disp("  Initial value = 0");
disp("  Final value   = " + string(500/(m*V)) + " (accélération latérale)");
disp("");
disp("============================================================");
