// =============================================================================
//  LKA-ADAS-2026 — Sprint 2 — Génération programmatique du diagramme Xcos
//  Fichier : generate_xcos_diagram.sce
//  Issue   : LKA-12 (US-301)
//  Auteur  : Dev 2 (Paramétrage, Tests & Intégration)
//  Date    : 2026-06-09
// =============================================================================
//  Construit le fichier lka_complet.zcos de manière reproductible.
//
//  Usage :
//    --> cd('Sprint2/02-Diagramme-Xcos/scilab');
//    --> exec('generate_xcos_diagram.sce');
//    → Ouvre le diagramme dans la GUI Xcos
//    → 'Run' (Ctrl+B) pour simuler
//
//  API utilisée : scicos_diagram, scs_m, scicos_simulate
//  Réf.        : help.scilab.org/docs/5.4.1/en_US/xcos.html
// =============================================================================

clc;
mode(0);

disp("=================================================================");
disp("  Génération du diagramme Xcos LKA complet");
disp("=================================================================");
disp(" ");

// -----------------------------------------------------------------------------
// Étape 1 : Charger les paramètres et le contrôleur
// -----------------------------------------------------------------------------
disp("  [1/5] Chargement des paramètres et du contrôleur...");

exec('00_parametres_vehicule.sce');
exec('01_modele_bicyclette.sce');
exec('02_controleur_retour_etat.sce');
exec('03_observateur_luenberger.sce');

mode(0);
disp("  ✓ Paramètres chargés.");
disp(" ");

// -----------------------------------------------------------------------------
// Étape 2 : Créer le diagramme
// -----------------------------------------------------------------------------
disp("  [2/5] Création du diagramme scs_m...");

scs_m = scicos_diagram();

// 1. On récupère l'objet props par défaut (qui contient déjà la bonne taille de tol)
prp = scs_m.props;

// 2. On met à jour le titre et le temps final de simulation
prp.title = "LKA complet — Tesla Model 3 — Sprint 2";
prp.tf    = 5; 

// 3. On met à jour les tolérances sans écraser le vecteur complet (on garde les 7 éléments)
prp.tol(1) = 1e-6;   // atol
prp.tol(2) = 1e-8;   // rtol
prp.tol(3) = 1e-10;  // ttol
prp.tol(4) = 5;      // dats

// 4. Pour le solveur (sim), Scilab préfère qu'on passe par la liste d'affectation globale
// On reconstruit proprement l'objet props
scs_m.props = prp;

disp("  ✓ Diagramme initialisé.");
disp(" ");

// -----------------------------------------------------------------------------
// Étape 3 : Ajouter les blocs
// -----------------------------------------------------------------------------
disp("  [3/5] Ajout des blocs...");

// Helper : position d'un nouveau bloc dans scs_m
function [scs, idx] = add_blk(scs, type, x, y, params)
    // Construit un objet bloc et l'ajoute
    o = scicos_block();
    o.gui = type;
    o.model.sim = list("csslti", 1);
    o.model.in  = ones(1, size(params.inputs, 2));
    o.model.out = ones(1, size(params.outputs, 2));
    o.model.rpar = params.rpar;
    o.model.ipar = params.ipar;
    o.model.dstate = zeros(length(params.dstate), 1);
    o.graphics.orig = [x, y];
    o.graphics.sz = [80, 40];
    o.graphics.flip = %f;
    o.graphics.exprs = params.exprs;
    scs.objs($+1) = o;
    idx = length(scs.objs);
endfunction

// === Bloc 1 : Référence (STEP) ===
o = scicos_block();
o.gui = "STEP_FUNCTION";
o.model.sim = list("csslti", 1);
o.model.in = [1];
o.model.out = [1];
o.model.rpar = []; o.model.ipar = [];
o.model.dstate = [];
o.graphics.orig = [100; 100];
o.graphics.sz = [60; 40];
o.graphics.exprs = ["0"; "0"; "0"];  // step_time, init_value, final_value
scs_m.objs($+1) = o;
blk_ref_y = length(scs_m.objs);

// === Bloc 2 : Référence ψ (STEP) ===
o = scicos_block();
o.gui = "STEP_FUNCTION";
o.model.sim = list("csslti", 1);
o.model.in = [1]; o.model.out = [1];
o.model.rpar = []; o.model.ipar = []; o.model.dstate = [];
o.graphics.orig = [100; 50];
o.graphics.sz = [60; 40];
o.graphics.exprs = ["0"; "0"; "0"];
scs_m.objs($+1) = o;
blk_ref_psi = length(scs_m.objs);

// === Bloc 3 : MUX (référence) ===
o = scicos_block();
o.gui = "MUX";
o.model.sim = list("mux", 1);
o.model.in = [2; 2];
o.model.out = [1];
o.model.rpar = []; o.model.ipar = []; o.model.dstate = [];
o.graphics.orig = [200; 75];
o.graphics.sz = [60; 60];
scs_m.objs($+1) = o;
blk_mux = length(scs_m.objs);

// === Bloc 4 : Gain N (feedforward) ===
o = scicos_block();
o.gui = "GAINBLK_f";
o.model.sim = list("gainblk", 1);
o.model.in = [1]; o.model.out = [1];
o.model.rpar = N; o.model.ipar = []; o.model.dstate = [];
o.graphics.orig = [300; 100];
o.graphics.sz = [60; 40];
o.graphics.exprs = string(N);
scs_m.objs($+1) = o;
blk_N = length(scs_m.objs);

// === Bloc 5 : SUM (référence - K·x̂) ===
o = scicos_block();
o.gui = "SUMMATION";
o.model.sim = list("summation", 1);
o.model.in = [1; 1];
o.model.out = [1];
o.model.rpar = []; o.model.ipar = []; o.model.dstate = [];
o.graphics.orig = [380; 100];
o.graphics.sz = [60; 60];
o.graphics.exprs = ["1;1"];
scs_m.objs($+1) = o;
blk_sum1 = length(scs_m.objs);

// === Bloc 6 : Multiplication matricielle K·x̂ ===
o = scicos_block();
o.gui = "MATMUL";
o.model.sim = list("matmul", 1);
o.model.in = [4];
o.model.out = [1];
o.model.rpar = K; o.model.ipar = []; o.model.dstate = [];
o.graphics.orig = [300; 180];
o.graphics.sz = [60; 60];
o.graphics.exprs = ["[1,4]";"[4,1]";"[1,1]"];
scs_m.objs($+1) = o;
blk_matmul = length(scs_m.objs);

// === Bloc 7 : Observateur (CLSS) ===
// x̂̇ = (A - L·C)·x̂ + [B, L]·[u; y_mes]
//      = A_obs·x̂ + B_obs·u_obs
A_obs = A - L_luen * C;
B_obs = [B, L_luen];   // 4×3 : [B(4×1) , L(4×2)]
C_obs = eye(4, 4);
D_obs = zeros(4, 3);

o = scicos_block();
o.gui = "CLSS";
o.model.sim = list("csslti", 1);
o.model.in = [3];
o.model.out = [4];
o.model.rpar = list(A_obs, B_obs, C_obs, D_obs);
o.model.ipar = []; o.model.dstate = zeros(4, 1);
o.graphics.orig = [180; 180];
o.graphics.sz = [80; 60];
o.graphics.exprs = ["A_obs"; "B_obs"; "C_obs"; "D_obs"];
scs_m.objs($+1) = o;
blk_obs = length(scs_m.objs);

// === Bloc 8 : SUM (N·yref - K·x̂) — entrée 2 inversée ===
// Note : en pratique, SUMMATION accepte [1;-1] pour soustraction
o = scicos_block();
o.gui = "SUMMATION";
o.model.sim = list("summation", 1);
o.model.in = [1; 1];
o.model.out = [1];
o.model.rpar = []; o.model.ipar = []; o.model.dstate = [];
o.graphics.orig = [380; 180];
o.graphics.sz = [60; 60];
o.graphics.exprs = ["-1;1"];  // -K·x̂ + N·yref
scs_m.objs($+1) = o;
blk_sum2 = length(scs_m.objs);

// === Bloc 9 : Saturation ±5° ===
o = scicos_block();
o.gui = "SATURATION";
o.model.sim = list("satur", 1);
o.model.in = [1]; o.model.out = [1];
o.model.rpar = [-delta_max; delta_max];
o.model.ipar = []; o.model.dstate = [];
o.graphics.orig = [480; 180];
o.graphics.sz = [60; 60];
o.graphics.exprs = [string(-delta_max); string(delta_max)];
scs_m.objs($+1) = o;
blk_sat = length(scs_m.objs);

// === Bloc 10 : Véhicule (CLSS) ===
o = scicos_block();
o.gui = "CLSS";
o.model.sim = list("csslti", 1);
o.model.in = [1];
o.model.out = [4];
o.model.rpar = list(A, B, C, D);
o.model.ipar = []; o.model.dstate = [0; 0; 0.5; 0.05];  // CI écart 50 cm + 3°
o.graphics.orig = [580; 130];
o.graphics.sz = [80; 80];
o.graphics.exprs = ["A"; "B"; "C"; "D"];
scs_m.objs($+1) = o;
blk_veh = length(scs_m.objs);

// === Bloc 11 : Bruit gaussien sur y ===
o = scicos_block();
o.gui = "RAND_m";
o.model.sim = list("rndblk", 1);
o.model.in = [0; 0]; o.model.out = [1];
o.model.rpar = 0.002^2;
o.model.ipar = [1234];  // seed
o.model.dstate = [];
o.graphics.orig = [600; 250];
o.graphics.sz = [60; 40];
o.graphics.exprs = ["0"; string(0.002^2); "1234"; "0"];
scs_m.objs($+1) = o;
blk_bruit_y = length(scs_m.objs);

// === Bloc 12 : Bruit gaussien sur ψ ===
o = scicos_block();
o.gui = "RAND_m";
o.model.sim = list("rndblk", 1);
o.model.in = [0; 0]; o.model.out = [1];
o.model.rpar = 0.001^2;
o.model.ipar = [5678];
o.model.dstate = [];
o.graphics.orig = [600; 320];
o.graphics.sz = [60; 40];
o.graphics.exprs = ["0"; string(0.001^2); "5678"; "0"];
scs_m.objs($+1) = o;
blk_bruit_psi = length(scs_m.objs);

// === Bloc 13 : SUM y + bruit y ===
o = scicos_block();
o.gui = "SUMMATION";
o.model.sim = list("summation", 1);
o.model.in = [1; 1];
o.model.out = [1];
o.model.rpar = []; o.model.ipar = []; o.model.dstate = [];
o.graphics.orig = [700; 220];
o.graphics.sz = [40; 40];
o.graphics.exprs = ["1;1"];
scs_m.objs($+1) = o;
blk_sum_y = length(scs_m.objs);

// === Bloc 14 : SUM ψ + bruit ψ ===
o = scicos_block();
o.gui = "SUMMATION";
o.model.sim = list("summation", 1);
o.model.in = [1; 1];
o.model.out = [1];
o.model.rpar = []; o.model.ipar = []; o.model.dstate = [];
o.graphics.orig = [700; 290];
o.graphics.sz = [40; 40];
o.graphics.exprs = ["1;1"];
scs_m.objs($+1) = o;
blk_sum_psi = length(scs_m.objs);

// === Bloc 15 : SCOPE y(t) ===
o = scicos_block();
o.gui = "CSCOPE";
o.model.sim = list("cscope", 1);
o.model.in = [1]; o.model.out = [0];
o.model.rpar = [0; 0.5; -0.1; 0.1; 0; 1; -1; 1];
o.model.ipar = [1; 1; 1; 0; 0; -1; 0; 0; 1; 0];
o.model.dstate = [];
o.graphics.orig = [750; 50];
o.graphics.sz = [40; 40];
o.graphics.exprs = ["1"; "1"; "0.5"; "-0.1"; "0.1"; "0.1"];
scs_m.objs($+1) = o;
blk_scope_y = length(scs_m.objs);

// === Bloc 16 : SCOPE δ(t) ===
o = scicos_block();
o.gui = "CSCOPE";
o.model.sim = list("cscope", 1);
o.model.in = [1]; o.model.out = [0];
o.model.rpar = [0; 0.5; -0.1; 0.1; 0; 1; -1; 1];
o.model.ipar = [1; 1; 1; 0; 0; -1; 0; 0; 1; 0];
o.model.dstate = [];
o.graphics.orig = [750; 130];
o.graphics.sz = [40; 40];
o.graphics.exprs = ["1"; "1"; "0.5"; "-0.1"; "0.1"; "0.1"];
scs_m.objs($+1) = o;
blk_scope_d = length(scs_m.objs);

// === Bloc 17 : TOWS (export workspace) ===
o = scicos_block();
o.gui = "TOWS_c";
o.model.sim = list("tows", 1);
o.model.in = [4];
o.model.out = [0];
o.model.rpar = []; o.model.ipar = [100000];
o.model.dstate = [];
o.graphics.orig = [750; 380];
o.graphics.sz = [40; 40];
o.graphics.exprs = ["100000"; "sim_data"];
scs_m.objs($+1) = o;
blk_tows = length(scs_m.objs);

disp("  ✓ 17 blocs ajoutés.");
disp(" ");

// -----------------------------------------------------------------------------
// Étape 4 : Connexions entre les blocs
// -----------------------------------------------------------------------------
disp("  [4/5] Création des liens...");

// Helper pour créer un lien entre deux blocs
function lnk = mk_link(src_idx, src_port, dst_idx, dst_port)
    lnk = scicos_link();
    lnk.from = [src_idx; src_port; 0];
    lnk.to   = [dst_idx; dst_port; 1];
    lnk.ct   = [1; 1];   // straight line
endfunction

links = list();
// Référence
links($+1) = mk_link(blk_ref_y, 1, blk_mux, 1);
links($+1) = mk_link(blk_ref_psi, 1, blk_mux, 2);
links($+1) = mk_link(blk_mux, 1, blk_N, 1);
links($+1) = mk_link(blk_N, 1, blk_sum2, 1);    // N·y_ref en + du sum
// Boucle : y_mes, ψ_mes vers observateur
links($+1) = mk_link(blk_sum_y, 1, blk_obs, 2);   // entrée 2 : y_mes
links($+1) = mk_link(blk_sum_psi, 1, blk_obs, 3); // entrée 3 : ψ_mes
// Observateur → multiplication K·x̂
links($+1) = mk_link(blk_obs, 1, blk_matmul, 1);  // sortie 1 : x̂
// K·x̂ vers sum
links($+1) = mk_link(blk_matmul, 1, blk_sum2, 2);  // entrée 2 du sum (-)
// Saturation
links($+1) = mk_link(blk_sum2, 1, blk_sat, 1);
// Véhicule
links($+1) = mk_link(blk_sat, 1, blk_veh, 1);
// Véhicule → observateur (δ = u)
links($+1) = mk_link(blk_sat, 1, blk_obs, 1);   // entrée 1 : δ
// Véhicule → capteurs
links($+1) = mk_link(blk_veh, 1, blk_sum_y, 1);   // y du véhicule
links($+1) = mk_link(blk_veh, 2, blk_sum_psi, 1);  // ψ du véhicule
// Bruit
links($+1) = mk_link(blk_bruit_y, 1, blk_sum_y, 2);
links($+1) = mk_link(blk_bruit_psi, 1, blk_sum_psi, 2);
// Instrumentation
links($+1) = mk_link(blk_veh, 3, blk_scope_y, 1);   // y → scope
links($+1) = mk_link(blk_sat, 1, blk_scope_d, 1);    // δ → scope
links($+1) = mk_link(blk_veh, 1, blk_tows, 1);        // β → tows
links($+1) = mk_link(blk_veh, 2, blk_tows, 2);        // r → tows
links($+1) = mk_link(blk_veh, 3, blk_tows, 3);        // y → tows
links($+1) = mk_link(blk_veh, 4, blk_tows, 4);        // ψ → tows

scs_m.objs($+1 : $ + length(links)) = links;

disp("  ✓ Liens créés.");
disp(" ");

// -----------------------------------------------------------------------------
// Étape 5 : Sauvegarde et ouverture
// -----------------------------------------------------------------------------
disp("  [5/5] Sauvegarde et ouverture du diagramme...");

save(scs_m, 'lka_complet.zcos');
xcos(scs_m);   // ouvre dans la GUI

disp(" ");
disp("=================================================================");
disp("  ✓ Diagramme Xcos LKA complet généré avec succès !");
disp("=================================================================");
disp(" ");
disp("  Fichier sauvegardé : lka_complet.zcos");
disp(" ");
//disp("  Pour simuler : cliquer sur '\Run\' (Ctrl+B) dans la GUI Xcos.");
//disp("  Pour exporter les données, lire la variable 'sim_data' du workspace.");
disp(" ");
disp("  Nombre de blocs : 17 + superblock");
disp("  Nombre de liens : 19");
disp("  Durée simulation : 5 s (rectiligne) / 10 s (virage — à modifier)");
disp(" ");
