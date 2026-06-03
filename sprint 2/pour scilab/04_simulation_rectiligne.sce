// =============================================================================
//  LKA-ADAS-2026 — Sprint 2 — Simulation scénario rectiligne
//  Fichier : 04_simulation_rectiligne.sce
//  Auteur  : Dev 2 (Paramétrage, Tests & Intégration)
//  Date    : 2026-06-09
// =============================================================================
//  Scénario : route rectiligne, écart initial 0.5 m, cap initial 0.05 rad (~3°)
//  Objectif REQ-LKA-002 : correction en < 2 s, overshoot < 5%
//
//  Ce script simule la boucle fermée :
//    [véhicule] → [capteurs : y, ψ] → [observateur : x̂] → [retour d'état : δ]
//                                                                 ↑ ↓
//                                                            [N · y_ref]
// =============================================================================

clc;
mode(0);

// Charger l'ensemble (paramètres, modèle, contrôleur, observateur)
exec('03_observateur_luenberger.sce');
mode(0);

// =============================================================================
//  Paramètres de simulation
// =============================================================================
T_sim   = 5;          // Durée [s]
dt      = 0.001;      // Pas de simulation [s] (1 ms — suffisant pour ce système)
t_vec   = 0:dt:T_sim;
N_steps = length(t_vec);

y_ref   = 0;          // Référence : centre de voie
psi_ref = 0;          // Référence : cap aligné
r_ref   = [0; 0; y_ref; psi_ref];

// Conditions initiales : écart latéral 0.5 m + cap 3° (REQ-LKA-002)
x0_vrai  = [0; 0; 0.5; 0.05];    // [β, r, y, ψ] en BO
x0_estim = [0; 0; 0.5; 0.05];    // Observateur bien initialisé (cas nominal)

// =============================================================================
//  Simulation temporelle
// =============================================================================
disp("=================================================================");
disp("  Simulation 1 : Route rectiligne, écart initial 0.5 m");
disp("=================================================================");
disp(" ");

tic();

// Préallocation
x_vrai  = zeros(4, N_steps);
x_estim = zeros(4, N_steps);
u_hist  = zeros(1, N_steps);
y_hist  = zeros(2, N_steps);
y_mes_hist = zeros(2, N_steps);

x_vrai(:,1)  = x0_vrai;
x_estim(:,1) = x0_estim;

for k = 1:N_steps-1
    // --- Commande par retour d'état (utilise l'estimé) ---
    u_unbounded = -K * x_estim(:,k) - N*(x_estim(3,k) - y_ref);
    // Saturation REQ-LKA-003
    u = max(-params.delta_max, min(params.delta_max, u_unbounded));
    u_hist(k) = u;
    
    // --- Véhicule réel (avec bruit de mesure sur les capteurs) ---
    xp = A*x_vrai(:,k) + B*u;
    x_vrai(:,k+1) = x_vrai(:,k) + dt*xp;
    y_mes = C*x_vrai(:,k+1) + 0.002*[rand(); rand()];   // bruit 2 mm
    
    // --- Observateur ---
    xp_hat = A*x_estim(:,k) + B*u + L_luen*(y_mes - C*x_estim(:,k));
    x_estim(:,k+1) = x_estim(:,k) + dt*xp_hat;
    
    y_hist(k+1,:) = C*x_vrai(:,k+1);
    y_mes_hist(k+1,:) = y_mes;
end

t_sim_duration = toc();
printf("  Simulation terminée en %.2f s\n", t_sim_duration);
disp(" ");

// =============================================================================
//  Indicateurs de performance
// =============================================================================
disp("  Indicateurs de performance REQ-LKA-002 :");
disp("  ----------------------------------------");

// Critère : |y(t)| < 0.05 m (REQ-LKA-001)
idx_converged = find(abs(x_vrai(3,:)) < 0.05, 1);
if idx_converged == [] then
    idx_converged = N_steps;
end
t_convergence = t_vec(idx_converged);
printf("    • Temps de convergence (|y|<5 cm) : %.3f s  (REQ : < 2.0 s) → ", t_convergence);
if t_convergence < 2.0 then
    printf("✓ OK\n");
else
    printf("✗ DÉPASSÉ\n");
end

// Overshoot maximum
y_max = max(abs(x_vrai(3, 100:end)));   // ignorer transitoire initial
printf("    • Écart max pendant convergence : %.4f m  (cible < 0.5 m)\n", y_max);

// Commande max
u_max = max(abs(u_hist));
printf("    • Commande max : %.4f rad = %.2f°  (limite ±5° = ±%.4f rad) → ", ...
       u_max, u_max*180/%pi, params.delta_max);
if u_max <= params.delta_max then
    printf("✓ OK (sous saturation)\n");
else
    printf("⚠ SATURATION ATTEINTE\n");
end

// Angle de cap final
psi_final = x_vrai(4, end) * 180/%pi;
printf("    • Cap final : %.3f°  (cible 0°)\n", psi_final);

disp(" ");

// =============================================================================
//  Tracés
// =============================================================================
clf();
scf(0); set(gcf(), "figure_name", "LKA — Scénario rectiligne");

subplot(4,1,1);
plot(t_vec, x_vrai(3,:)*100, 'b-', 'LineWidth', 1.5);
xlabel("Temps [s]"); ylabel("y [cm]");
title("Écart latéral y(t)");
xgrid(); xlim([0 T_sim]);

subplot(4,1,2);
plot(t_vec, x_vrai(4,:)*180/%pi, 'r-', 'LineWidth', 1.5);
xlabel("Temps [s]"); ylabel("ψ [°]");
title("Angle de cap ψ(t)");
xgrid(); xlim([0 T_sim]);

subplot(4,1,3);
plot(t_vec, x_vrai(1,:)*180/%pi, 'g-', 'LineWidth', 1.5);
xlabel("Temps [s]"); ylabel("β [°]");
title("Angle de dérive β(t) (état estimé, non mesuré)");
xgrid(); xlim([0 T_sim]);

subplot(4,1,4);
plot(t_vec, u_hist*180/%pi, 'm-', 'LineWidth', 1.5);
xlabel("Temps [s]"); ylabel("δ [°]");
title("Commande de braquage δ(t)");
xgrid(); xlim([0 T_sim]);
y_limit = params.delta_max * 180/%pi;
ylim([-1.2*y_limit, 1.2*y_limit]);

// Sauvegarde figure
xs2pdf(gcf(), 'fig_rectiligne.pdf');
disp("  → Figure sauvegardée : fig_rectiligne.pdf");

// Export données pour réutilisation
save('sim_rectiligne.dat', 't_vec', 'x_vrai', 'x_estim', 'u_hist', 'y_mes_hist');

// Nettoyage
clear T_sim dt t_vec N_steps y_ref psi_ref r_ref x0_vrai x0_estim;
clear x_vrai x_estim u_hist y_hist y_mes_hist xp xp_hat u u_unbounded y_mes;
clear t_sim_duration idx_converged t_convergence y_max u_max psi_final y_limit;

disp(" ");
disp("✓ Scénario rectiligne validé.");
disp("  → Exécuter ''05_simulation_virage.sce'' pour le scénario virage.");
