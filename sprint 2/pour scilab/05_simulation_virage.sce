// =============================================================================
//  LKA-ADAS-2026 — Sprint 2 — Simulation scénario virage
//  Fichier : 05_simulation_virage.sce
//  Auteur  : Dev 2 (Paramétrage, Tests & Intégration)
//  Date    : 2026-06-09
// =============================================================================
//  Scénario REQ-LKA-006 : virage constant R = 250 m, V = 100 km/h = 27.78 m/s
//  L'entrée n'est plus δ=0 mais une rampe d'angle de braquage menant à un
//  virage stabilisé. La consigne est de maintenir l'écart y aussi proche de 0
//  que possible malgré la perturbation.
//
//  La référence ψ_ref est générée comme un virage à taux constant.
// =============================================================================

clc;
mode(0);

// Charger l'ensemble
exec('03_observateur_luenberger.sce');
mode(0);

// =============================================================================
//  Paramètres du scénario
// =============================================================================
T_sim    = 10;
dt       = 0.001;
t_vec    = 0:dt:T_sim;
N_steps  = length(t_vec);

V_ms     = 100/3.6;        // 100 km/h = 27.78 m/s
R_virage = 250;            // 250 m de rayon
psi_dot_ref = V_ms / R_virage;  // 0.111 rad/s = 6.36°/s — taux de lacet en virage

// Mise à jour des matrices pour V = 27.78 m/s
V  = V_ms;
a11 = -(Cf + Cr)/(m*V);
a12 = -1 + (b*Cr - a*Cf)/(m*V^2);
a21 =  (b*Cr - a*Cf)/Iz;
a22 = -(a^2*Cf + b^2*Cr)/(Iz*V);
a31 =  V;
a34 =  V;
a42 =  1;
A_v  = [a11, a12, 0, 0; a21, a22, 0, 0; a31, 0, 0, a34; 0, a42, 0, 0];
B_v  = [Cf/(m*V); a*Cf/Iz; 0; 0];
sys_v = syslin('c', A_v, B_v, C, D);

// Recalcul K pour ce V (LQR rechargé)
[K_v, S_v, P_v] = lqr(sys_v, Q_lqr, R_lqr);
N_v = -1 / (C(1,:) * inv(A_v - B_v*K_v) * B_v);

disp("=================================================================");
disp("  Simulation 2 : Virage R=250m, V=100 km/h (REQ-LKA-006)");
disp("=================================================================");
disp(" ");
printf("  V       = %.2f m/s  (%.0f km/h)\n", V_ms, V_ms*3.6);
printf("  R       = %.0f m\n", R_virage);
printf("  ψ̇_ref  = %.4f rad/s  (%.2f°/s)\n", psi_dot_ref, psi_dot_ref*180/%pi);
printf("  K_v     = [%.3f, %.3f, %.3f, %.3f]\n", K_v);
printf("  N_v     = %.3f\n", N_v);
disp(" ");

// Initialisation : véhicule en train de tourner à taux constant
x0_vrai  = [0; psi_dot_ref; 0; 0];
x0_estim = x0_vrai;

// =============================================================================
//  Simulation
// =============================================================================
x_vrai  = zeros(4, N_steps);
x_estim = zeros(4, N_steps);
u_hist  = zeros(1, N_steps);
y_err   = zeros(1, N_steps);

x_vrai(:,1)  = x0_vrai;
x_estim(:,1) = x0_estim;

tic();
for k = 1:N_steps-1
    // Référence pour cet instant
    psi_ref_k = psi_dot_ref * t_vec(k);
    y_ref_k   = 0;
    r_ref_k   = [0; 0; y_ref_k; psi_ref_k];
    
    // Commande
    u_unbounded = -K_v * x_estim(:,k) - N_v*(x_estim(3,k) - y_ref_k) + 0;  // pas de feedforward sur ψ dans cette version
    u = max(-params.delta_max, min(params.delta_max, u_unbounded));
    u_hist(k) = u;
    
    // Véhicule
    xp = A_v*x_vrai(:,k) + B_v*u;
    x_vrai(:,k+1) = x_vrai(:,k) + dt*xp;
    y_mes = C*x_vrai(:,k+1) + 0.002*[rand(); rand()];
    
    // Observateur
    xp_hat = A_v*x_estim(:,k) + B_v*u + L_luen*(y_mes - C*x_estim(:,k));
    x_estim(:,k+1) = x_estim(:,k) + dt*xp_hat;
    
    y_err(k) = x_vrai(3,k+1) - y_ref_k;
end
t_sim_duration = toc();
printf("  Simulation terminée en %.2f s\n", t_sim_duration);
disp(" ");

// Ignorer le transitoire initial (1 s)
idx_steady = find(t_vec > 1.0, 1);

// =============================================================================
//  Indicateurs REQ-LKA-006
// =============================================================================
disp("  Indicateurs de performance REQ-LKA-006 :");
disp("  ----------------------------------------");

y_rms = sqrt(mean(y_err(idx_steady:end).^2));
y_max_abs = max(abs(y_err(idx_steady:end)));
printf("    • Écart RMS en régime permanent : %.4f m\n", y_rms);
printf("    • Écart max en régime permanent  : %.4f m  (REQ : < 0.15 m) → ", y_max_abs);
if y_max_abs < 0.15 then
    printf("✓ OK\n");
else
    printf("✗ DÉPASSÉ\n");
end

u_mean = mean(u_hist(idx_steady:end));
u_rms  = sqrt(mean(u_hist(idx_steady:end).^2));
printf("    • Commande moyenne : %.4f rad  (%.2f°)\n", u_mean, u_mean*180/%pi);
printf("    • Commande RMS    : %.4f rad  (%.2f°)\n", u_rms, u_rms*180/%pi);

disp(" ");

// =============================================================================
//  Tracés
// =============================================================================
clf();
scf(0); set(gcf(), "figure_name", "LKA — Scénario virage R=250m");

subplot(3,1,1);
plot(t_vec, x_vrai(3,:)*100, 'b-', 'LineWidth', 1.5);
xlabel("Temps [s]"); ylabel("y [cm]");
title("Écart latéral y(t) en virage R=250m, V=100km/h");
xgrid(); xlim([0 T_sim]);
y_limit = 0.15 * 100;
yline(15, 'r--', 'REQ: 15 cm'); yline(-15, 'r--');

subplot(3,1,2);
plot(t_vec, x_vrai(4,:)*180/%pi, 'r-', 'LineWidth', 1.5);
xlabel("Temps [s]"); ylabel("ψ [°]");
title("Angle de cap ψ(t)");
xgrid(); xlim([0 T_sim]);

subplot(3,1,3);
plot(t_vec, u_hist*180/%pi, 'm-', 'LineWidth', 1.5);
xlabel("Temps [s]"); ylabel("δ [°]");
title("Commande de braquage δ(t)");
xgrid(); xlim([0 T_sim]);
y_limit = params.delta_max * 180/%pi;
yline(y_limit, 'k--', 'Saturation'); yline(-y_limit, 'k--');
ylim([-1.2*y_limit, 1.2*y_limit]);

xs2pdf(gcf(), 'fig_virage.pdf');
disp("  → Figure sauvegardée : fig_virage.pdf");

// Nettoyage
clear T_sim dt t_vec N_steps V_ms R_virage psi_dot_ref;
clear A_v B_v sys_v K_v S_v P_v N_v;
clear a11 a12 a21 a22 a31 a34 a42;
clear x0_vrai x0_estim x_vrai x_estim u_hist y_err;
clear idx_steady y_rms y_max_abs u_mean u_rms;
clear xp xp_hat u u_unbounded y_mes psi_ref_k y_ref_k r_ref_k;
clear t_sim_duration y_limit;

disp(" ");
disp("✓ Scénario virage validé.");
disp("  → Exécuter ''99_analyse_stabilite.sce'' pour l'analyse de stabilité.");
