// =============================================================================
//  LKA-ADAS-2026 — Sprint 2 — Observateur de Luenberger
//  Fichier : 03_observateur_luenberger.sce
//  Auteur  : Dev 2 (Paramétrage, Tests & Intégration)
//  Date    : 2026-06-09
// =============================================================================
//  But : estimer les états NON mesurés (β et r) à partir des mesures [y, ψ]
//        et de la commande u = δ.
//
//  Équation de l'observateur :
//    x̂̇ = A·x̂ + B·u + L·(y_mes - C·x̂)
//         |__________|   |___________________|
//           modèle         correction (innovation)
//
//  avec L la matrice de gain de l'observateur (4×2) calculée par placement
//  de pôles pour (A - L·C), choisis 5× plus rapides que ceux du régulateur
//  (règle de séparation).
// =============================================================================

clc;
mode(0);

// Charger modèle + contrôleur
exec('02_controleur_retour_etat.sce');
mode(0);

// =============================================================================
//  Conception de l'observateur : placement de pôles pour (A - L·C)
// =============================================================================
//  Les pôles de l'observateur doivent être 3 à 10 fois plus rapides que ceux
//  du régulateur, mais rester dans des limites réalistes :
//    - trop rapides → amplification du bruit capteur
//    - trop lents → l'estimé diverge et le contrôle n'est plus efficace
//
//  Choix : 5× plus rapides (compromis standard pour les LKA académiques).
// =============================================================================

// Pôles du régulateur (cf. 02_...)
poles_ctrl = spec(A - B*K);
max_pole_ctrl = max(real(poles_ctrl));
disp("=================================================================");
disp("  Observateur de Luenberger — Conception");
disp("=================================================================");
disp(" ");
printf("  Pôles du régulateur (boucle fermée) :\n");
for i=1:4
    printf("    p%d = %.3f + %.3fj\n", i, real(poles_ctrl(i)), imag(poles_ctrl(i)));
end
disp(" ");

// Pôles de l'observateur : 5× plus rapides (décalés à gauche)
obs_factor = 5;
poles_obs = poles_ctrl * obs_factor;

// On garde les mêmes parties imaginaires relatives (mêmes modes) mais on
// multiplie la partie réelle par obs_factor.
poles_obs_fast = real(poles_ctrl) * obs_factor + %i * imag(poles_ctrl);

//disp("  Pôles de l'observateur (5× plus rapides) :");
for i=1:4
    printf("    p_obs%d = %.3f + %.3fj\n", i, real(poles_obs_fast(i)), imag(poles_obs_fast(i)));
end
disp(" ");

// Pour un système MIMO, le placement par Scilab passe par ppol.
// Le dual (A', C') → K → L = K' (méthode du système dual).
A_dual  = A';
B_dual  = C';
K_dual  = ppol(A_dual, B_dual, poles_obs_fast);
L_luen  = K_dual';

printf("  Matrice de gain de observateur L (4×2) =\n");
disp(L_luen);
disp(" ");

// Vérification : pôles de (A - L·C) doivent être ceux qu'on a choisis
poles_obs_check = spec(A - L_luen * C);
disp("  Pôles de (A - L·C) — vérification :");
for i=1:4
    printf("    p%d = %.3f + %.3fj\n", i, real(poles_obs_check(i)), imag(poles_obs_check(i)));
end
disp(" ");

// =============================================================================
//  Test de convergence de l'observateur
// =============================================================================
disp("  Test de convergence sur 2 s (estimation à partir un offset initial) :");

t_sim = 0:0.01:2;
x0_vrai  = [0.05; 0.1; 0.5; 0.05];   // β₀=3°, r₀=5°/s, y₀=0,5m, ψ₀=3°
x0_estim = [0;   0;   0.5; 0.05];   // On suppose β et r mal connus, y et ψ ok
u0       = 0;                        // Commande nulle pendant le test

// Intégration du système réel et de l'observateur
dt = 0.01;
N  = length(t_sim);
x_vrai  = zeros(4, N);
x_estim = zeros(4, N);
x_vrai(:,1)  = x0_vrai;
x_estim(:,1) = x0_estim;

for k = 1:N-1
    // Système réel (ici on triche un peu : pas de perturbation, u=0)
    xp = A*x_vrai(:,k) + B*u0;
    x_vrai(:,k+1) = x_vrai(:,k) + dt*xp;
    // Mesure (y, ψ)
    y_meas = C*x_vrai(:,k+1) + 0.001*rand(2,1);  // bruit de mesure 1 mm
    // Observateur
    xp_hat = A*x_estim(:,k) + B*u0 + L_luen*(y_meas - C*x_estim(:,k));
    x_estim(:,k+1) = x_estim(:,k) + dt*xp_hat;
end

// Erreur d'estimation finale
err_fin = x_vrai(:,N) - x_estim(:,N);
printf("    État final estimé vs réel :\n");
printf("      β  : estimé=%.4f rad  /  vrai=%.4f rad  (err=%.2e)\n", ...
       x_estim(1,N), x_vrai(1,N), abs(err_fin(1)));
printf("      r  : estimé=%.4f rad/s / vrai=%.4f rad/s (err=%.2e)\n", ...
       x_estim(2,N), x_vrai(2,N), abs(err_fin(2)));
printf("      y  : estimé=%.4f m     / vrai=%.4f m     (err=%.2e)\n", ...
       x_estim(3,N), x_vrai(3,N), abs(err_fin(3)));
printf("      ψ  : estimé=%.4f rad  /  vrai=%.4f rad  (err=%.2e)\n", ...
       x_estim(4,N), x_vrai(4,N), abs(err_fin(4)));
disp(" ");
if max(abs(err_fin)) < 0.01 then
    disp("  ✓ Observateur convergent (erreur < 1 cm en 2 s).");
else
    disp("  ⚠ Erreur estimation non négligeable — augmenter le facteur.");
end
disp(" ");

// =============================================================================
//  Sauvegarde pour les scripts aval
// =============================================================================
clear poles_ctrl poles_obs poles_obs_check max_pole_ctrl;
clear x0_vrai x0_estim u0 x_vrai x_estim xp xp_hat y_meas err_fin;
clear A_dual B_dual K_dual t_sim dt N obs_factor poles_obs_fast;

disp("✓ Observateur conçu : L_luen (4×2).");
disp("  → Exécuter ''04_simulation_rectiligne.sce'' pour la validation.");
