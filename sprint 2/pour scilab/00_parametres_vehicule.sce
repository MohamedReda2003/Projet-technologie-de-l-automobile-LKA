// =============================================================================
//  LKA-ADAS-2026 — Sprint 2 — Paramètres véhicule Tesla Model 3
//  Fichier : 00_parametres_vehicule.sce
//  Auteur  : Dev 2 (Paramétrage, Tests & Intégration)
//  Source  : Fiche paramètres Tesla Model 3 v1.0 — Sprint 1
//  Date    : 2026-06-09
// =============================================================================
//  Ce script centralise TOUS les paramètres du véhicule de référence.
//  Il est appelé par les autres scripts (modele_bicyclette.sce, controleur...).
//  Compatible Scilab 6.x (Scilab 2024+ recommandé pour le projet).
// =============================================================================

mode(0);    // Pas d'affichage pendant l'include
clc;

// -----------------------------------------------------------------------------
// 1. Identification véhicule
// -----------------------------------------------------------------------------
vehicule.marque        = "Tesla";
vehicule.modele        = "Model 3";
vehicule.version       = "Long Range AWD pre-Highland";
vehicule.annee         = 2024;
vehicule.motorisation  = "Dual Motor AWD (IPM/SynRM arrière + induction avant)";

// -----------------------------------------------------------------------------
// 2. Paramètres géométriques (sources : Tesla Owner's Manual, EVKX.net)
// -----------------------------------------------------------------------------
m  = 1823;     // Masse totale [kg]            (Tesla OM, version Long Range AWD)
L  = 2.875;    // Empattement [m]              (Tesla OM — constant)
a  = 1.423;    // Distance CG → essieu avant [m] (calcul m_r/m × L, voir §4.2)
b  = 1.452;    // Distance CG → essieu arrière [m](calcul m_f/m × L, voir §4.2)
lf = 1.584;    // Voie avant [m]                (Tesla OM)
lr = 1.584;    // Voie arrière [m]              (Tesla OM)
hG = 0.50;     // Hauteur du CG [m]             (MarkLines + EVKX, voir §6)

// -----------------------------------------------------------------------------
// 3. Inertie (sources : TU Eindhoven + Rajamani + Sigenthaler)
// -----------------------------------------------------------------------------
Iz = 3500;     // Moment d'inertie en lacet [kg·m²] (moyenne 3 méthodes)

// -----------------------------------------------------------------------------
// 4. Rigidité de dérive pneumatique (sources : Milliken, Pacejka)
// -----------------------------------------------------------------------------
Cf = 120000;   // Rigidité de dérive essieu avant [N/rad]
Cr = 180000;   // Rigidité de dérive essieu arrière [N/rad]

// -----------------------------------------------------------------------------
// 5. Conditions de simulation
// -----------------------------------------------------------------------------
V_ref = 25;            // Vitesse nominale [m/s] (~ 90 km/h)
V_min = 20;            // Vitesse basse [m/s]  (72 km/h)
V_max = 35;            // Vitesse haute [m/s]  (126 km/h)
g_acc = 9.81;          // Gravité [m/s²]

// -----------------------------------------------------------------------------
// 6. Limites de la commande (REQ-LKA-003)
// -----------------------------------------------------------------------------
delta_max_deg = 5;     // Braquage max REQ-LKA-003 [°]
delta_max     = %pi/180 * delta_max_deg;   // [rad] = 0.0873 rad
delta_dot_max = 0.6;   // Vitesse de braquage max [rad/s] (sécurité actionneur)

// -----------------------------------------------------------------------------
// 7. Critères de performance (REQ-LKA)
// -----------------------------------------------------------------------------
REQ.sigma_005   = 0.05;  // Écart final admissible [m] (5 cm)
REQ.t_react_002 = 2.0;   // Temps de correction REQ-LKA-002 [s]
REQ.epsilon_005 = 0.2;   // Robustesse vent REQ-LKA-005 [m]
REQ.epsilon_006 = 0.15;  // Performance virage REQ-LKA-006 [m]
REQ.R_006       = 250;   // Rayon virage REQ-LKA-006 [m]
REQ.V_006_ms    = 100/3.6;  // Vitesse REQ-LKA-006 [m/s]

// -----------------------------------------------------------------------------
// 8. Paramètres d'observateur (cf. document retour d'état)
// -----------------------------------------------------------------------------
L_pole_factor = 5;      // Facteur de séparation (pôles observateur = K × poles_ctrl)
L_psi_min     = 0.5;    // Pôle le plus lent de l'observateur [rad/s] (damping garanti)

// -----------------------------------------------------------------------------
// 9. Coefficients dérivés (équations du modèle bicycle, voir fiche Tesla §10.1)
// -----------------------------------------------------------------------------
function calcul_coefficients(V, a, b, m, Iz, Cf, Cr)
    // Coefficients matrice A
    a11 = -(Cf + Cr) / (m*V);
    a12 = -1 + (b*Cr - a*Cf) / (m*V^2);
    a21 =  (b*Cr - a*Cf) / Iz;
    a22 = -(a^2*Cf + b^2*Cr) / (Iz*V);
    a31 =  V;
    a34 =  V;
    a42 =  1;
    // Coefficients matrice B
    b1  =  Cf / (m*V);
    b2  =  a*Cf / Iz;
    b3  =  0;
    b4  =  0;
endfunction

// -----------------------------------------------------------------------------
// 10. Affichage (vérification)
// -----------------------------------------------------------------------------
disp("=================================================================");
disp("  LKA-ADAS-2026 — Paramètres véhicule Tesla Model 3 Long Range AWD");
disp("=================================================================");
disp(" ");
disp("GEOMÉTRIE :");
printf("  m  = %6.0f kg     (masse)\n", m);
printf("  L  = %6.3f m       (empattement)\n", L);
printf("  a  = %6.3f m       (CG → essieu avant)\n", a);
printf("  b  = %6.3f m       (CG → essieu arrière)\n", b);
printf("  Iz = %6.0f kg·m²  (inertie en lacet)\n", Iz);
disp(" ");
disp("PNEUMATIQUE :");
printf("  Cf = %6.0f N/rad  (essieu avant)\n", Cf);
printf("  Cr = %6.0f N/rad  (essieu arrière)\n", Cr);
disp(" ");
disp("CONDITIONS :");
printf("  V  = %6.1f m/s     (~ %.0f km/h)\n", V_ref, V_ref*3.6);
printf("  δ_max = %5.2f°    (REQ-LKA-003)\n", delta_max_deg);
disp(" ");

// Empactage dans une struct pour passage aux autres scripts
params.m  = m;    params.L  = L;    params.a  = a;    params.b  = b;
params.lf = lf;   params.lr = lr;   params.hG = hG;
params.Iz = Iz;   params.Cf = Cf;   params.Cr = Cr;
params.V  = V_ref;
params.g  = g_acc;
params.delta_max = delta_max;
params.delta_dot_max = delta_dot_max;
params.REQ = REQ;
params.vehicule = vehicule;

clear m L a b lf lr hG Iz Cf Cr V_ref V_min V_max g_acc delta_max delta_max_deg delta_dot_max;

disp("✓ Paramètres chargés dans la variable ''params''.");
disp("  → Exécuter ''01_modele_bicyclette.sce'' pour la suite.");
