// =============================================================================
//  LKA-ADAS-2026 — Sprint 2 — Modèle bicycle linéarisé
//  Fichier : 01_modele_bicyclette.sce
//  Auteur  : Dev 1 (Modélisation & Contrôleur) — revue Dev 2
//  Source  : Modèle bicycle 2-essieux, linéarisation petits angles
//  Date    : 2026-06-09
// =============================================================================
//  Hypothèses :
//    - Petit angle de braquage (|δ| < 5°)
//    - Pneumatique en régime linéaire (|α| < 4°)
//    - Route plate, vitesse V constante
//    - Modèle linéarisé autour de l'équilibre rectiligne
//
//  Variables d'état : x = [β, r, y, ψ]ᵀ
//    β = angle de dérive au CG [rad]
//    r = vitesse de lacet [rad/s]
//    y = écart latéral par rapport à la trajectoire de référence [m]
//    ψ = angle de cap [rad]
//
//  Entrée : u = δ (angle de braquage) [rad]
//  Sortie : y_out = [y, ψ]ᵀ (mesures typiquement disponibles sur un LKA)
// =============================================================================

clc;
mode(0);

// Charger les paramètres
exec('00_parametres_vehicule.sce');
mode(0);

// Récupération des variables
V  = params.V;
m  = params.m;
a  = params.a;
b  = params.b;
Iz = params.Iz;
Cf = params.Cf;
Cr = params.Cr;

// -----------------------------------------------------------------------------
// Construction des matrices A(4×4), B(4×1), C(2×4), D(2×1)
// -----------------------------------------------------------------------------
// Formulation linéarisée (Rajamani, Vehicle Dynamics and Control, 2012)

A = zeros(4,4);
A(1,1) = -(Cf + Cr)/(m*V);
A(1,2) = -1 + (b*Cr - a*Cf)/(m*V^2);
A(2,1) =  (b*Cr - a*Cf)/Iz;
A(2,2) = -(a^2*Cf + b^2*Cr)/(Iz*V);
A(3,1) =  V;
A(3,4) =  V;
A(4,2) =  1;

B = zeros(4,1);
B(1,1) =  Cf/(m*V);
B(2,1) =  a*Cf/Iz;
B(3,1) =  0;
B(4,1) =  0;

// Capteurs : on suppose disponibles y (écart latéral) et ψ (cap) — issus d'une caméra
// et d'un GPS/capteur de lacet. β et r sont mesurés par IMU + GPS (r) ou estimés (β).
C = zeros(2,4);
C(1,1) = 0;  C(1,2) = 0;  C(1,3) = 1;  C(1,4) = 0;   // mesure de y
C(2,1) = 0;  C(2,2) = 0;  C(2,3) = 0;  C(2,4) = 1;   // mesure de ψ

D = zeros(2,1);

// Système d'état Scilab
sys = syslin('c', A, B, C, D);

// -----------------------------------------------------------------------------
// Vérifications de cohérence
// -----------------------------------------------------------------------------
disp("=================================================================");
disp("  Modèle bicycle Tesla Model 3 — Matrices état");
disp("=================================================================");
disp(" ");
printf("  A (%dx%d) =\n", size(A,1), size(A,2)); disp(A);
printf("  B (%dx%d) =\n", size(B,1), size(B,2)); disp(B);
printf("  C (%dx%d) =\n", size(C,1), size(C,2)); disp(C);
printf("  D (%dx%d) =\n", size(D,1), size(D,2)); disp(D);
disp(" ");

// Pôles du système en boucle ouverte
poles_OL = spec(A);
disp("Pôles en boucle ouverte (stabilité naturelle) :");
disp(poles_OL);
disp(" ");
if min(real(poles_OL)) < 0 then
    disp("  → Système en BO naturellement STABLE (mode bicycle sous-amorti).");
else
    disp("  ⚠ Système en BO instable — vérifier paramètres.");
end
disp(" ");

// Controllabilité
Mc = cont_mat(A, B);
rango_Mc = rank(Mc);
disp("Matrice de controllabilité :");
printf("  rang(Mc) = %d / %d (ordre système)\n", rango_Mc, size(A,1));
if rango_Mc == size(A,1) then
    disp("  → Système COMPLÈTEMENT COMMANDABLE — retour état applicable.");
else
    disp("  ⚠ Système NON complètement commandable.");
end
disp(" ");

// Observabilité (avec mesure de y, ψ)
Mo = obsv_mat(A, C);
rango_Mo = rank(Mo);
disp("Matrice observabilité :");
printf("  rang(Mo) = %d / %d (ordre système)\n", rango_Mo, size(A,1));
if rango_Mo == size(A,1) then
    disp("  → Système COMPLÈTEMENT OBSERVABLE avec [y, ψ].");
else
    disp("  ⚠ Système NON complètement observable → observateur indispensable.");
end
disp(" ");

// Export dans le workspace
clear V m a b Iz Cf Cr rango_Mc rango_Mo;
disp("✓ Modèle construit dans la variable ''sys'' (A, B, C, D).");
disp("  → Exécuter ''02_controleur_retour_etat.sce'' pour la suite.");
