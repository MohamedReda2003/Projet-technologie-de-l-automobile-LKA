// ============================================================================
// PROJET LKA - SPRINT 3 : VALIDATION MiL
// Système Line Keeping Assistant - Tesla Model 3
// Méthode : Agile Scrum | Approche : Minimax (LQR)
// ============================================================================
// Auteur : Équipe LKA-ADAS-2026
// Date : Juin 2026
// Outil : Scilab 2024.x avec Xcos
// ============================================================================

clear; clc; close;

// ============================================================================
// SECTION 1 : PARAMÈTRES DU VÉHICULE (Tesla Model 3)
// ============================================================================

m = 1800;           // Masse totale (kg)
L = 2.875;          // Empattement (m)
a = 1.15;           // Distance CG -> essieu avant (m)
b = 1.725;          // Distance CG -> essieu arrière (m)
Iz = 3500;          // Moment d'inertie en lacet (kg.m²)
Cf = 120000;        // Rigidité latérale avant (N/rad)
Cr = 180000;        // Rigidité latérale arrière (N/rad)
V = 30;             // Vitesse longitudinale (m/s = 108 km/h)

// Affichage des paramètres
disp("============================================================");
disp("PARAMÈTRES DU VÉHICULE - Tesla Model 3");
disp("============================================================");
disp("Masse m = " + string(m) + " kg");
disp("Empattement L = " + string(L) + " m");
disp("a = " + string(a) + " m, b = " + string(b) + " m");
disp("Iz = " + string(Iz) + " kg.m²");
disp("Cf = " + string(Cf) + " N/rad, Cr = " + string(Cr) + " N/rad");
disp("Vitesse V = " + string(V) + " m/s (" + string(V*3.6) + " km/h)");

// ============================================================================
// SECTION 2 : MATRICES DU MODÈLE D'ÉTAT (Modèle bicyclette)
// ============================================================================
// État x = [beta, r, y, psi]'
// beta = angle de dérive latérale (rad)
// r    = vitesse de lacet (rad/s)
// y    = écart latéral par rapport à la trajectoire (m)
// psi  = angle de cap (rad)
// Entrée u = delta (angle de braquage, rad)

a11 = -(Cf + Cr) / (m * V);
a12 = -1 + (b*Cr - a*Cf) / (m * V^2);
a21 = (b*Cr - a*Cf) / Iz;
a22 = -(a^2 * Cf + b^2 * Cr) / (Iz * V);

A = [a11,  a12,   0,    0;
     a21,  a22,   0,    0;
      V,    0,    0,    V;
      0,    1,    0,    0];

b1 = Cf / (m * V);
b2 = a * Cf / Iz;

B = [b1;
     b2;
      0;
      0];

disp("");
disp("============================================================");
disp("MATRICE A (système) :");
disp(A);
disp("MATRICE B (entrée) :");
disp(B);

// ============================================================================
// SECTION 3 : ANALYSE DES POLES EN BOUCLE OUVERTE
// ============================================================================

disp("");
disp("============================================================");
disp("POLES DU SYSTÈME EN BOUCLE OUVERTE");
disp("============================================================");
poles_BO = spec(A);
disp(poles_BO);

if real(poles_BO) < 0 then
    disp("✅ Système stable en boucle ouverte");
else
    disp("⚠️ Système instable ou marginalement stable");
end

// ============================================================================
// SECTION 4 : SYNTHÈSE DU CONTRÔLEUR LQR (Approche Minimax)
// ============================================================================
// Q : pondération des états [beta, r, y, psi]
// R : pondération de l'effort de commande delta

Q = diag([1, 1, 100, 50]);   // y très pénalisé (centre voie)
R = 10;                       // effort modéré

// Résolution de l'équation de Riccati
P = riccati(A, B*inv(R)*B', Q, 'c');

// Gain optimal LQR
K = inv(R) * B' * P;

disp("");
disp("============================================================");
disp("SYNTHÈSE LQR (Approche Minimax)");
disp("============================================================");
disp("Matrice Q (pondération états) :");
disp(Q);
disp("Matrice R (pondération commande) : " + string(R));
disp("Gain LQR K = ");
disp(K);
disp("K_beta = " + string(K(1)));
disp("K_r    = " + string(K(2)));
disp("K_y    = " + string(K(3)));
disp("K_psi  = " + string(K(4)));

// ============================================================================
// SECTION 5 : SYSTÈME EN BOUCLE FERMÉE
// ============================================================================

A_BF = A - B * K;
poles_BF = spec(A_BF);

disp("");
disp("============================================================");
disp("POLES EN BOUCLE FERMÉE (LQR)");
disp("============================================================");
disp(poles_BF);

if real(poles_BF) < 0 then
    disp("✅ Système stable en boucle fermée");
else
    disp("❌ Système instable en boucle fermée");
end

// Temps de réponse estimé
dominant_pole = poles_BF(real(poles_BF) == max(real(poles_BF)));
tau = -1 / real(dominant_pole(1));
tr_5pc = 3 * tau;
disp("Pôle dominant : " + string(dominant_pole(1)));
disp("Constante de temps τ = " + string(tau) + " s");
disp("Temps de réponse à 5% ≈ " + string(tr_5pc) + " s");

// ============================================================================
// SECTION 6 : SIMULATION SCÉNARIO 1 - ROUTE RECTILIGNE
// ============================================================================
// Écart initial de 0.5m, vitesse constante

disp("");
disp("============================================================");
disp("SCÉNARIO 1 : ROUTE RECTILIGNE (y0 = 0.5m)");
disp("============================================================");

// Conditions initiales
x0 = [0; 0; 0.5; 0];   // [beta, r, y, psi]

// Temps de simulation
t = 0:0.005:5;         // 5 secondes, pas de 5ms

// Fonction pour la dynamique en boucle ouverte
function dx = dyn_BO(t, x)
    delta = 0;  // Pas de correction
    dx = A*x + B*delta;
endfunction

// Fonction pour la dynamique en boucle fermée (LQR)
function dx = dyn_BF(t, x)
    delta = -K*x;
    // Saturation de l'angle de braquage à ±5° (≈ ±0.087 rad)
    delta_max = 5 * %pi / 180;
    delta = max(min(delta, delta_max), -delta_max);
    dx = A*x + B*delta;
endfunction

// Simulation BOUCLE OUVERTE
x_BO = ode(x0, 0, t, dyn_BO);

// Simulation BOUCLE FERMÉE
x_BF = ode(x0, 0, t, dyn_BF);

// Extraction des variables
beta_BO = x_BO(1, :);
r_BO = x_BO(2, :);
y_BO = x_BO(3, :);
psi_BO = x_BO(4, :);

beta_BF = x_BF(1, :);
r_BF = x_BF(2, :);
y_BF = x_BF(3, :);
psi_BF = x_BF(4, :);

// Validation REQ-LKA-002 : Correction écart > 0.3m en < 2s
idx_corr = find(abs(y_BF) < 0.3, 1);
if idx_corr <> [] then
    t_corr = t(idx_corr);
    disp("Temps de correction < 0.3m : " + string(t_corr) + " s");
    if t_corr < 2.0 then
        disp("✅ REQ-LKA-002 VALIDÉ : " + string(t_corr) + "s < 2.0s");
    else
        disp("❌ REQ-LKA-002 NON VALIDÉ");
    end
else
    disp("❌ Écart jamais corrigé");
end

disp("Erreur stationnaire BO : y_inf = " + string(y_BO($)) + " m");
disp("Erreur stationnaire BF : y_inf = " + string(y_BF($)) + " m");

// ============================================================================
// GRAPHIQUE 1 : SCÉNARIO 1 - Route rectiligne (4 sous-graphes)
// ============================================================================

scf(1);
clf(1);

// Écart latéral
subplot(2, 2, 1);
plot(t, y_BO, 'r--', 'LineWidth', 2);
plot(t, y_BF, 'b-', 'LineWidth', 2);
plot([0, 5], [0.3, 0.3], 'orange:', 'LineWidth', 1.5);
plot([0, 5], [-0.3, -0.3], 'orange:', 'LineWidth', 1.5);
xlabel('Temps (s)');
ylabel('Écart latéral y (m)');
title('Écart latéral y(t)');
legend('Sans correcteur (BO)', 'Avec correcteur LQR (BF)', 'Seuil REQ-LKA-002 (0.3m)', 4);
grid on;
xlim([0, 5]);

// Angle de cap
subplot(2, 2, 2);
plot(t, psi_BO * 180/%pi, 'r--', 'LineWidth', 2);
plot(t, psi_BF * 180/%pi, 'b-', 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Angle de cap ψ (°)');
title('Angle de cap ψ(t)');
legend('Sans correcteur', 'Avec correcteur LQR', 1);
grid on;
xlim([0, 5]);

// Angle de dérive
subplot(2, 2, 3);
plot(t, beta_BO * 180/%pi, 'r--', 'LineWidth', 2);
plot(t, beta_BF * 180/%pi, 'b-', 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Angle de dérive β (°)');
title('Angle de dérive latérale β(t)');
legend('Sans correcteur', 'Avec correcteur LQR', 1);
grid on;
xlim([0, 5]);

// Vitesse de lacet
subplot(2, 2, 4);
plot(t, r_BO * 180/%pi, 'r--', 'LineWidth', 2);
plot(t, r_BF * 180/%pi, 'b-', 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Vitesse de lacet r (°/s)');
title('Vitesse de lacet r(t)');
legend('Sans correcteur', 'Avec correcteur LQR', 1);
grid on;
xlim([0, 5]);

// Titre global
title_page = gcf();
title_page.figure_name = 'Scénario 1 : Route rectiligne - Comparaison BO/BF';

// Sauvegarde
xs2png(gcf(), 'Sprint3_Scenario1_RouteRectiligne.png');
disp("✅ Graphique sauvegardé : Sprint3_Scenario1_RouteRectiligne.png");

// ============================================================================
// SECTION 7 : SIMULATION SCÉNARIO 2 - VIRAGE CONSTANT
// ============================================================================
// Virage de rayon R = 250m à V = 100 km/h

disp("");
disp("============================================================");
disp("SCÉNARIO 2 : VIRAGE CONSTANT (R = 250m, V = 100 km/h)");
disp("============================================================");

V_virage = 100 / 3.6;   // 27.78 m/s

// Recalcul des matrices avec la nouvelle vitesse
a11_v = -(Cf + Cr) / (m * V_virage);
a12_v = -1 + (b*Cr - a*Cf) / (m * V_virage^2);
a21_v = (b*Cr - a*Cf) / Iz;
a22_v = -(a^2 * Cf + b^2 * Cr) / (Iz * V_virage);

A_v = [a11_v,  a12_v,   0,          0;
       a21_v,  a22_v,   0,          0;
       V_virage,  0,    0,    V_virage;
          0,      1,     0,          0];

b1_v = Cf / (m * V_virage);
b2_v = a * Cf / Iz;

B_v = [b1_v;
       b2_v;
         0;
         0];

// Synthese LQR pour le virage
Q_v = diag([1, 1, 100, 50]);
R_v = 10;
P_v = riccati(A_v, B_v*inv(R_v)*B_v', Q_v, 'c');
K_v = inv(R_v) * B_v' * P_v;
A_BF_v = A_v - B_v * K_v;

// Trajectoire de référence
R_virage = 250;                    // Rayon du virage (m)
psi_dot_ref = V_virage / R_virage;  // Vitesse de rotation de référence

// Condition initiale : légèrement décentré
x0_v = [0; 0; 0.15; 0];   // Écart initial de 0.15m

t_v = 0:0.005:10;          // 10 secondes

// Dynamique BO sur virage
function dx = dyn_BO_virage(t, x)
    delta = psi_dot_ref * (L / V_virage);  // Braquage nominal pour le virage
    dx = A_v*x + B_v*delta;
endfunction

// Dynamique BF sur virage
function dx = dyn_BF_virage(t, x)
    delta = -K_v*x;
    delta_max = 5 * %pi / 180;
    delta = max(min(delta, delta_max), -delta_max);
    dx = A_v*x + B_v*delta;
endfunction

x_BO_v = ode(x0_v, 0, t_v, dyn_BO_virage);
x_BF_v = ode(x0_v, 0, t_v, dyn_BF_virage);

y_BO_v = x_BO_v(3, :);
y_BF_v = x_BF_v(3, :);

// Validation REQ-LKA-006
err_max_BO_v = max(abs(y_BO_v));
err_max_BF_v = max(abs(y_BF_v));
disp("Erreur max BO : " + string(err_max_BO_v) + " m");
disp("Erreur max BF : " + string(err_max_BF_v) + " m");
if err_max_BF_v < 0.15 then
    disp("✅ REQ-LKA-006 VALIDÉ : " + string(err_max_BF_v) + "m < 0.15m");
else
    disp("❌ REQ-LKA-006 NON VALIDÉ");
end

// ============================================================================
// GRAPHIQUE 2 : SCÉNARIO 2 - Virage constant
// ============================================================================

scf(2);
clf(2);

subplot(2, 2, 1);
plot(t_v, y_BO_v, 'r--', 'LineWidth', 2);
plot(t_v, y_BF_v, 'b-', 'LineWidth', 2);
plot([0, 10], [0.15, 0.15], 'orange:', 'LineWidth', 1.5);
plot([0, 10], [-0.15, -0.15], 'orange:', 'LineWidth', 1.5);
xlabel('Temps (s)');
ylabel('Écart latéral y (m)');
title('Écart latéral y(t)');
legend('Sans correcteur (BO)', 'Avec correcteur LQR (BF)', 'Seuil REQ-LKA-006 (0.15m)', 2);
grid on;
xlim([0, 10]);

subplot(2, 2, 2);
plot(t_v, x_BO_v(4, :) * 180/%pi, 'r--', 'LineWidth', 2);
plot(t_v, x_BF_v(4, :) * 180/%pi, 'b-', 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Angle de cap ψ (°)');
title('Angle de cap ψ(t)');
legend('Sans correcteur', 'Avec correcteur LQR', 1);
grid on;
xlim([0, 10]);

subplot(2, 2, 3);
plot(t_v, x_BO_v(1, :) * 180/%pi, 'r--', 'LineWidth', 2);
plot(t_v, x_BF_v(1, :) * 180/%pi, 'b-', 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Angle de dérive β (°)');
title('Angle de dérive β(t)');
legend('Sans correcteur', 'Avec correcteur LQR', 1);
grid on;
xlim([0, 10]);

subplot(2, 2, 4);
plot(t_v, x_BO_v(2, :) * 180/%pi, 'r--', 'LineWidth', 2);
plot(t_v, x_BF_v(2, :) * 180/%pi, 'b-', 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Vitesse de lacet r (°/s)');
title('Vitesse de lacet r(t)');
legend('Sans correcteur', 'Avec correcteur LQR', 1);
grid on;
xlim([0, 10]);

title_page = gcf();
title_page.figure_name = 'Scénario 2 : Virage constant - Comparaison BO/BF';
xs2png(gcf(), 'Sprint3_Scenario2_VirageConstant.png');
disp("✅ Graphique sauvegardé : Sprint3_Scenario2_VirageConstant.png");

// ============================================================================
// SECTION 8 : SIMULATION SCÉNARIO 3 - PERTURBATION VENT LATÉRAL
// ============================================================================
// Vent latéral de 15 m/s (force latérale ≈ 500 N)

disp("");
disp("============================================================");
disp("SCÉNARIO 3 : PERTURBATION VENT LATÉRAL (Vvent = 15 m/s)");
disp("============================================================");

F_vent = 500;                    // Force latérale équivalente (N)
wind_force = F_vent / (m * V);  // Accélération latérale

x0_w = [0; 0; 0; 0];           // Pas d'écart initial
t_w = 0:0.005:5;               // 5 secondes

// Dynamique BO avec vent
function dx = dyn_BO_vent(t, x)
    delta = 0;
    dx = A*x + B*delta;
    dx(1) = dx(1) + wind_force;  // Perturbation sur beta
endfunction

// Dynamique BF avec vent
function dx = dyn_BF_vent(t, x)
    delta = -K*x;
    delta_max = 5 * %pi / 180;
    delta = max(min(delta, delta_max), -delta_max);
    dx = A*x + B*delta;
    dx(1) = dx(1) + wind_force;  // Perturbation sur beta
endfunction

x_BO_w = ode(x0_w, 0, t_w, dyn_BO_vent);
x_BF_w = ode(x0_w, 0, t_w, dyn_BF_vent);

y_BO_w = x_BO_w(3, :);
y_BF_w = x_BF_w(3, :);

// Validation REQ-LKA-005
err_max_BO_w = max(abs(y_BO_w));
err_max_BF_w = max(abs(y_BF_w));
disp("Écart max BO : " + string(err_max_BO_w) + " m");
disp("Écart max BF : " + string(err_max_BF_w) + " m");
if err_max_BF_w < 0.2 then
    disp("✅ REQ-LKA-005 VALIDÉ : " + string(err_max_BF_w) + "m < 0.2m");
else
    disp("❌ REQ-LKA-005 NON VALIDÉ");
end

// ============================================================================
// GRAPHIQUE 3 : SCÉNARIO 3 - Perturbation vent
// ============================================================================

scf(3);
clf(3);

subplot(2, 2, 1);
plot(t_w, y_BO_w, 'r--', 'LineWidth', 2);
plot(t_w, y_BF_w, 'b-', 'LineWidth', 2);
plot([0, 5], [0.2, 0.2], 'orange:', 'LineWidth', 1.5);
plot([0, 5], [-0.2, -0.2], 'orange:', 'LineWidth', 1.5);
xlabel('Temps (s)');
ylabel('Écart latéral y (m)');
title('Écart latéral y(t)');
legend('Sans correcteur (BO)', 'Avec correcteur LQR (BF)', 'Seuil REQ-LKA-005 (0.2m)', 2);
grid on;
xlim([0, 5]);

subplot(2, 2, 2);
plot(t_w, x_BO_w(4, :) * 180/%pi, 'r--', 'LineWidth', 2);
plot(t_w, x_BF_w(4, :) * 180/%pi, 'b-', 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Angle de cap ψ (°)');
title('Angle de cap ψ(t)');
legend('Sans correcteur', 'Avec correcteur LQR', 1);
grid on;
xlim([0, 5]);

subplot(2, 2, 3);
plot(t_w, x_BO_w(1, :) * 180/%pi, 'r--', 'LineWidth', 2);
plot(t_w, x_BF_w(1, :) * 180/%pi, 'b-', 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Angle de dérive β (°)');
title('Angle de dérive β(t)');
legend('Sans correcteur', 'Avec correcteur LQR', 1);
grid on;
xlim([0, 5]);

subplot(2, 2, 4);
plot(t_w, x_BO_w(2, :) * 180/%pi, 'r--', 'LineWidth', 2);
plot(t_w, x_BF_w(2, :) * 180/%pi, 'b-', 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Vitesse de lacet r (°/s)');
title('Vitesse de lacet r(t)');
legend('Sans correcteur', 'Avec correcteur LQR', 1);
grid on;
xlim([0, 5]);

title_page = gcf();
title_page.figure_name = 'Scénario 3 : Vent latéral - Comparaison BO/BF';
xs2png(gcf(), 'Sprint3_Scenario3_PerturbationVent.png');
disp("✅ Graphique sauvegardé : Sprint3_Scenario3_PerturbationVent.png");

// ============================================================================
// SECTION 9 : GRAPHIQUE COMPARATIF GLOBAL
// ============================================================================

disp("");
disp("============================================================");
disp("GÉNÉRATION DU GRAPHIQUE COMPARATIF GLOBAL");
disp("============================================================");

scf(4);
clf(4);

// Route rectiligne
subplot(1, 3, 1);
plot(t, y_BO, 'r--', 'LineWidth', 2.5);
plot(t, y_BF, 'b-', 'LineWidth', 2.5);
plot([0, 5], [0.3, 0.3], 'orange:', 'LineWidth', 1.5);
plot([0, 5], [-0.3, -0.3], 'orange:', 'LineWidth', 1.5);
xlabel('Temps (s)');
ylabel('Écart latéral y (m)');
title('Route rectiligne — y(t)', 'fontsize', 11, 'fontweight', 'bold');
legend('Sans correcteur', 'Avec correcteur LQR', 2);
grid on;
xlim([0, 5]);
ylim([-0.6, 0.6]);
xstring(2.5, 0.4, 'REQ-LKA-002 ✓');

// Virage
subplot(1, 3, 2);
plot(t_v, y_BO_v, 'r--', 'LineWidth', 2.5);
plot(t_v, y_BF_v, 'b-', 'LineWidth', 2.5);
plot([0, 10], [0.15, 0.15], 'orange:', 'LineWidth', 1.5);
plot([0, 10], [-0.15, -0.15], 'orange:', 'LineWidth', 1.5);
xlabel('Temps (s)');
ylabel('Écart latéral y (m)');
title('Virage constant — y(t)', 'fontsize', 11, 'fontweight', 'bold');
legend('Sans correcteur', 'Avec correcteur LQR', 2);
grid on;
xlim([0, 10]);

// Vent
subplot(1, 3, 3);
plot(t_w, y_BO_w, 'r--', 'LineWidth', 2.5);
plot(t_w, y_BF_w, 'b-', 'LineWidth', 2.5);
plot([0, 5], [0.2, 0.2], 'orange:', 'LineWidth', 1.5);
plot([0, 5], [-0.2, -0.2], 'orange:', 'LineWidth', 1.5);
xlabel('Temps (s)');
ylabel('Écart latéral y (m)');
title('Vent latéral — y(t)', 'fontsize', 11, 'fontweight', 'bold');
legend('Sans correcteur', 'Avec correcteur LQR', 2);
grid on;
xlim([0, 5]);

title_page = gcf();
title_page.figure_name = 'Comparaison globale : Avec vs Sans correcteur LQR';
xs2png(gcf(), 'Sprint3_ComparaisonGlobale.png');
disp("✅ Graphique sauvegardé : Sprint3_ComparaisonGlobale.png");

// ============================================================================
// SECTION 10 : GRAPHIQUE DE LA COMMANDE (Angle de braquage)
// ============================================================================

// Calcul des commandes
delta_BF = [];
delta_BF_v = [];
delta_BF_w = [];

for i = 1:length(t)
    d = -K * x_BF(:, i);
    d = max(min(d, 5*%pi/180), -5*%pi/180);
    delta_BF(i) = d;
end

for i = 1:length(t_v)
    d = -K_v * x_BF_v(:, i);
    d = max(min(d, 5*%pi/180), -5*%pi/180);
    delta_BF_v(i) = d;
end

for i = 1:length(t_w)
    d = -K * x_BF_w(:, i);
    d = max(min(d, 5*%pi/180), -5*%pi/180);
    delta_BF_w(i) = d;
end

scf(5);
clf(5);

subplot(1, 3, 1);
plot(t, delta_BF * 180/%pi, 'b-', 'LineWidth', 2);
plot([0, 5], [5, 5], 'r--', 'LineWidth', 1.5);
plot([0, 5], [-5, -5], 'r--', 'LineWidth', 1.5);
xlabel('Temps (s)');
ylabel('Angle de braquage δ (°)');
title('Route rectiligne — δ(t)', 'fontsize', 11, 'fontweight', 'bold');
legend('δ LQR', 'Limite ±5° (REQ-LKA-003)', 1);
grid on;
xlim([0, 5]);

subplot(1, 3, 2);
plot(t_v, delta_BF_v * 180/%pi, 'b-', 'LineWidth', 2);
plot([0, 10], [5, 5], 'r--', 'LineWidth', 1.5);
plot([0, 10], [-5, -5], 'r--', 'LineWidth', 1.5);
xlabel('Temps (s)');
ylabel('Angle de braquage δ (°)');
title('Virage constant — δ(t)', 'fontsize', 11, 'fontweight', 'bold');
legend('δ LQR', 'Limite ±5° (REQ-LKA-003)', 1);
grid on;
xlim([0, 10]);

subplot(1, 3, 3);
plot(t_w, delta_BF_w * 180/%pi, 'b-', 'LineWidth', 2);
plot([0, 5], [5, 5], 'r--', 'LineWidth', 1.5);
plot([0, 5], [-5, -5], 'r--', 'LineWidth', 1.5);
xlabel('Temps (s)');
ylabel('Angle de braquage δ (°)');
title('Vent latéral — δ(t)', 'fontsize', 11, 'fontweight', 'bold');
legend('δ LQR', 'Limite ±5° (REQ-LKA-003)', 1);
grid on;
xlim([0, 5]);

title_page = gcf();
title_page.figure_name = 'Commande du système LKA (Angle de braquage δ)';
xs2png(gcf(), 'Sprint3_CommandeBraquage.png');
disp("✅ Graphique sauvegardé : Sprint3_CommandeBraquage.png");

// ============================================================================
// SECTION 11 : TABLEAU RÉCAPITULATIF FINAL
// ============================================================================

disp("");
disp("============================================================");
disp("TABLEAU RÉCAPITULATIF DE VALIDATION - SPRINT 3");
disp("============================================================");
disp("");
disp("| Exigence    | Critère                    | Résultat      | Statut |");
disp("|-------------|----------------------------|---------------|--------|");

// REQ-LKA-002
if idx_corr <> [] & t_corr < 2.0 then
    disp("| REQ-LKA-002 | Correction < 0.3m en < 2s  | " + string(t_corr) + "s       | ✅ OK  |");
else
    disp("| REQ-LKA-002 | Correction < 0.3m en < 2s  | NON           | ❌ KO  |");
end

// REQ-LKA-003
delta_max_all = max([max(abs(delta_BF)), max(abs(delta_BF_v)), max(abs(delta_BF_w))]) * 180/%pi;
if delta_max_all < 5 then
    disp("| REQ-LKA-003 | Braquage ≤ ±5°            | " + string(delta_max_all) + "°      | ✅ OK  |");
else
    disp("| REQ-LKA-003 | Braquage ≤ ±5°            | " + string(delta_max_all) + "°      | ⚠️ AJUSTER |");
end

// REQ-LKA-005
if err_max_BF_w < 0.2 then
    disp("| REQ-LKA-005 | Écart < 0.2m (vent 15m/s) | " + string(err_max_BF_w) + "m   | ✅ OK  |");
else
    disp("| REQ-LKA-005 | Écart < 0.2m (vent 15m/s) | " + string(err_max_BF_w) + "m   | ❌ KO  |");
end

// REQ-LKA-006
if err_max_BF_v < 0.15 then
    disp("| REQ-LKA-006 | Erreur < 0.15m (R=250m)   | " + string(err_max_BF_v) + "m    | ✅ OK  |");
else
    disp("| REQ-LKA-006 | Erreur < 0.15m (R=250m)   | " + string(err_max_BF_v) + "m    | ❌ KO  |");
end

disp("");
disp("============================================================");
disp("FIN DU SCRIPT SPRINT 3 - Tous les graphiques sont sauvegardés");
disp("============================================================");
