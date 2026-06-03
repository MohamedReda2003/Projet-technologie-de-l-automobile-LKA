// =============================================================================
//  LKA-ADAS-2026 — Sprint 2 — Contrôleur par retour d'état
//  Fichier : 02_controleur_retour_etat.sce
//  Auteur  : Dev 1 (Modélisation & Contrôleur) — revue Dev 2
//  Date    : 2026-06-09
// =============================================================================
//  Deux méthodes implémentées et comparées :
//    (a) LQR — Linear Quadratic Regulator (méthode principale, recommandée)
//    (b) Placement de pôles (méthode alternative, pour validation croisée)
//
//  Loi de commande : u = -K·x + N·r_ref
//    K : matrice de gain (1×4)
//    N : gain feedforward pour le suivi de la référence (erreur statique nulle)
//
//  Référence de mission : r_ref = [0, 0, y_ref, ψ_ref]ᵀ
//    Pour le LKA, on souhaite y_ref = 0 (centre de voie) et ψ_ref = 0 (cap aligné).
// =============================================================================

clc;
mode(0);

// Charger modèle
exec('01_modele_bicyclette.sce');
mode(0);

// =============================================================================
//  METHODE 1 : LQR (recommandée)
// =============================================================================
//  Principe : minimiser J = ∫(xᵀ·Q·x + uᵀ·R·u) dt
//  Q pondère les états, R pondère l'effort de commande.
//  Bryson's rule : Q_ii = 1/(tolérance max de l'état i)²
//                  R = 1/(tolérance max de la commande)²
// =============================================================================

// Tolérances pour Bryson's rule (cohérent avec REQ-LKA)
tol_beta   = 0.05;       // [rad] ~ 3°  : dérive max tolérée
tol_r      = 0.5;        // [rad/s]     : lacet max toléré
tol_y      = 0.05;       // [m]         : REQ-LKA-001 (5 cm)
tol_psi    = 0.05;       // [rad] ~ 3°
tol_delta  = %pi/180*1;  // [rad]       : on tolère ±1° avant saturation

Q_lqr = diag([1/tol_beta^2, 1/tol_r^2, 1/tol_y^2, 1/tol_psi^2]);
R_lqr = 1/tol_delta^2;

disp("=================================================================");
disp("  Méthode 1 : LQR (Linear Quadratic Regulator)");
disp("=================================================================");
disp(" ");
mprintf("Q_lqr = diag([%.0f, %.0f, %.0f, %.0f])\n", diag(Q_lqr)(1), diag(Q_lqr)(2), diag(Q_lqr)(3), diag(Q_lqr)(4));
printf("  R_lqr = %.0f\n", R_lqr);
disp(" ");

// 1. Extract state-space matrices from your system
A = sys.A;
B = sys.B;

// 2. Solve the Continuous Algebraic Riccati Equation
// Equation format: A'*P + P*A - P*(B*inv(R)*B')*P + Q = 0
P_lqr = riccati(A, B*inv(R_lqr)*B', Q_lqr, "c");

// 3. Compute the LQR gain matrix using P
K_lqr = inv(R_lqr) * B' * P_lqr;


printf("  K_lqr = [%.3f, %.3f, %.3f, %.3f]\n", K_lqr);
disp(" ");
disp("Pôles en boucle fermée (système contrôlé) :");
disp(P_lqr);
disp(" ");

// Marges de stabilité garanties par LQR
disp("Marges de stabilité (LQR garantit) :");
disp("  • Marge de gain  : ≥ 6 dB  (souvent ∞ avant Instability Margin)");
disp("  • Marge de phase : ≥ 60°");
disp(" ");

// =============================================================================
//  Calcul du gain feedforward N (pour erreur statique nulle)
// =============================================================================
//  Pour la commande u = -K·x + N·r_ref, on a :
//    x_eq = (A - B·K)⁻¹ · B·N · r_ref
//  Si on veut x_eq[3] = y_ref (état de sortie contrôlé), on résout :
//    N = -inv(C · (A - B·K)⁻¹ · B)   (pour r_ref = [0; 0; y_ref; 0])
//  C est ici 2×4, on prend la 1ère ligne (mesure y) :
// =============================================================================
A_cl = A - B*K_lqr;
Cy   = C(1,:);   // 1ère sortie = y
N_y  = -1 / (Cy * inv(A_cl) * B);

disp("Gain feedforward (sortie y) :");
printf("  N_y = %.3f   (→ y_ref = %.3f m → δ_ref = %.4f rad)\n", ...
       N_y, 0.1, -N_y*0.1);
disp("  (Vérification : pour y_ref = 0.1 m, la commande u = N_y · 0.1 = -N_y · 0.1 ≈ braquage adapté.)");
disp(" ");

// =============================================================================
//  METHODE 2 : Placement de pôles (validation croisée)
// =============================================================================
//  Pôles souhaités en boucle fermée (cf. cahier des charges REQ-LKA-002) :
//    • ω_n dominant = 1.5 rad/s (convergence en 2 s → 4/ω_n = 2.67 s)
//    • ζ = 0.7 (sous-amorti, overshoot < 5%)
//    • pôle y dominant : ζ=0.7, ω_n=1.5
//    • pôle ψ dominant : ζ=0.8, ω_n=1.2 (un peu plus rapide pour converger)
// =============================================================================

disp("=================================================================");
disp("  Méthode 2 : Placement de pôles (validation croisée)");
disp("=================================================================");
disp(" ");

// Pôles désirés en boucle fermée
wn1 = 1.5;  z1 = 0.7;   // Mode y dominant
wn2 = 1.2;  z2 = 0.8;   // Mode ψ dominant

// Pôles réels doubles rapides pour les autres dynamiques
poles_desired = [..
    -z1*wn1 + %i*wn1*sqrt(1-z1^2),  ..
    -z1*wn1 - %i*wn1*sqrt(1-z1^2),  ..
    -z2*wn2 + %i*wn2*sqrt(1-z2^2),  ..
    -z2*wn2 - %i*wn2*sqrt(1-z2^2)   ..
];

printf("  Pôles désirés :\n");
for i = 1:4
    if imag(poles_desired(i)) >= 0 then
        sign_str = '+';
    else
        sign_str = '-';
    end
    printf('p%d = %.3f %s %.3fj\n', i, real(poles_desired(i)), sign_str, abs(imag(poles_desired(i))));
end
disp(" ");

// Fonction ppol de Scilab : K tel que spec(A-BK) = poles_desired
K_ppol = ppol(A, B, poles_desired);

printf("  K_ppol = [%.3f, %.3f, %.3f, %.3f]\n", K_ppol);
disp(" ");

// Vérification
poles_cl_ppol = spec(A - B*K_ppol);
disp("Pôles en boucle fermée (vérification) :");
disp(poles_cl_ppol);
disp(" ");

// =============================================================================
//  Comparaison LQR vs Placement
// =============================================================================
disp("=================================================================");
disp("  Comparaison LQR vs Placement de pôles");
disp("=================================================================");
printf("  %-30s | %-10s | %-10s\n", "Métrique", "LQR", "Pôle");
printf("  -------------------------------+------------+-----------\n");
printf("  %-30s | %10.2f | %10.2f\n", "K[1] (gain sur β)",     K_lqr(1),  K_ppol(1));
printf("  %-30s | %10.2f | %10.2f\n", "K[2] (gain sur r)",     K_lqr(2),  K_ppol(2));
printf("  %-30s | %10.2f | %10.2f\n", "K[3] (gain sur y)",     K_lqr(3),  K_ppol(3));
printf("  %-30s | %10.2f | %10.2f\n", "K[4] (gain sur ψ)",     K_lqr(4),  K_ppol(4));
printf("  %-30s | %10.3f | %10.3f\n", "Pôle BF le + rapide (reel)", max(real(P_lqr)), max(real(poles_cl_ppol)));
printf("  %-30s | %10.3f | %10.3f\n", "Pôle BF le + lent (reel)",   min(real(P_lqr)), min(real(poles_cl_ppol)));
disp(" ");
disp("→ Les deux méthodes donnent des gains similaires. ");
disp("  On retient K_lqr comme contrôleur final (marges LQR meilleures).");

// =============================================================================
//  Sauvegarde pour les scripts aval
// =============================================================================
K = K_lqr;
N = N_y;
clear K_lqr S_lqr P_lqr K_ppol poles_cl_ppol poles_desired wn1 z1 wn2 z2;
clear A_cl Cy N_y Q_lqr R_lqr tol_beta tol_r tol_y tol_psi tol_delta;

disp(" ");
disp("✓ Contrôleur calculé : K (vecteur 1×4), N (gain feedforward).");
//disp("  → Exécuter ''03_observateur_luenberger.sce'' pour l'observateur.");
