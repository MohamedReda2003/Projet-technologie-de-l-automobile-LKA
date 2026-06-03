/* ============================================================================
 * PROJET LKA - SPRINT 4 : CODE EMBARQUÉ POUR ECU
 * Système Line Keeping Assistant - Tesla Model 3
 * Méthode : Agile Scrum | Approche : Minimax (LQR)
 * ============================================================================
 * Fichier    : lka_controller.c
 * Auteur     : Équipe LKA-ADAS-2026
 * Date       : Juin 2026
 * Version    : 1.0.0
 * ============================================================================
 * Description:
 *   Ce fichier contient l'implémentation C du contrôleur LKA pour
 *   un système d'assistance au maintien de voie (LKA).
 *   Le contrôleur utilise un régulateur LQR (Linear Quadratic Regulator)
 *   avec saturation de l'angle de braquage à ±5° (REQ-LKA-003).
 * ============================================================================
 */

#include <stdint.h>
#include <math.h>

/* ============================================================================
 * SECTION 1 : PARAMÈTRES DU VÉHICULE (Tesla Model 3)
 * ============================================================================ */

#define VEHICLE_MASS        1800.0f     /* Masse totale (kg) */
#define VEHICLE_WHEELBASE   2.875f      /* Empattement (m) */
#define VEHICLE_A           1.15f       /* Distance CG -> avant (m) */
#define VEHICLE_B           1.725f      /* Distance CG -> arrière (m) */
#define VEHICLE_IZ          3500.0f     /* Moment d'inertie lacet (kg·m²) */
#define TIRE_CF             120000.0f   /* Rigidité avant (N/rad) */
#define TIRE_CR             180000.0f   /* Rigidité arrière (N/rad) */
#define VEHICLE_SPEED       30.0f       /* Vitesse longitudinale (m/s) */

/* ============================================================================
 * SECTION 2 : PARAMÈTRES DU CONTRÔLEUR LQR
 * ============================================================================ */

#define K_BETA      7.9247f     /* Gain sur angle de dérive */
#define K_R         0.2175f     /* Gain sur vitesse de lacet */
#define K_Y         3.1623f     /* Gain sur écart latéral */
#define K_PSI       11.7687f    /* Gain sur angle de cap */

#define DELTA_MAX   0.0873f     /* Saturation braquage : 5° en rad (REQ-LKA-003) */
#define DELTA_MIN   -0.0873f    /* -5° en rad */

/* ============================================================================
 * SECTION 3 : STRUCTURES DE DONNÉES
 * ============================================================================ */

typedef struct {
    float beta;     /* Angle de dérive latérale (rad) */
    float r;        /* Vitesse de lacet (rad/s) */
    float y;        /* Écart latéral (m) */
    float psi;      /* Angle de cap (rad) */
} VehicleState_t;

typedef struct {
    float delta;    /* Angle de braquage commandé (rad) */
    float error_y;  /* Erreur latérale (m) */
    float error_psi;/* Erreur de cap (rad) */
} ControlOutput_t;

/* ============================================================================
 * SECTION 4 : FONCTIONS DU CONTRÔLEUR
 * ============================================================================ */

/**
 * @brief  Calcule la commande LQR (angle de braquage)
 * @param  state : État courant du véhicule [beta, r, y, psi]
 * @retval Angle de braquage delta (rad), saturé à ±5°
 */
float LKA_ComputeSteeringAngle(const VehicleState_t* state)
{
    float delta;

    /* Vérification des pointeurs */
    if (state == NULL)
    {
        return 0.0f;
    }

    /* Commande LQR : delta = -K * x */
    delta = -(K_BETA * state->beta +
              K_R * state->r +
              K_Y * state->y +
              K_PSI * state->psi);

    /* Saturation de l'angle de braquage (REQ-LKA-003) */
    if (delta > DELTA_MAX)
    {
        delta = DELTA_MAX;
    }
    else if (delta < DELTA_MIN)
    {
        delta = DELTA_MIN;
    }

    return delta;
}

/**
 * @brief  Met à jour l'état du véhicule (intégration Euler)
 * @param  state : État courant (entrée/sortie)
 * @param  delta : Angle de braquage (rad)
 * @param  dt    : Pas de temps (s)
 * @retval None
 */
void LKA_UpdateVehicleState(VehicleState_t* state, float delta, float dt)
{
    float a11, a12, a21, a22;
    float b1, b2;
    float beta_dot, r_dot, y_dot, psi_dot;

    if (state == NULL || dt <= 0.0f)
    {
        return;
    }

    /* Coefficients du modèle bicyclette */
    a11 = -(TIRE_CF + TIRE_CR) / (VEHICLE_MASS * VEHICLE_SPEED);
    a12 = -1.0f + (VEHICLE_B * TIRE_CR - VEHICLE_A * TIRE_CF) / 
                  (VEHICLE_MASS * VEHICLE_SPEED * VEHICLE_SPEED);
    a21 = (VEHICLE_B * TIRE_CR - VEHICLE_A * TIRE_CF) / VEHICLE_IZ;
    a22 = -(VEHICLE_A * VEHICLE_A * TIRE_CF + VEHICLE_B * VEHICLE_B * TIRE_CR) /
           (VEHICLE_IZ * VEHICLE_SPEED);

    b1 = TIRE_CF / (VEHICLE_MASS * VEHICLE_SPEED);
    b2 = VEHICLE_A * TIRE_CF / VEHICLE_IZ;

    /* Dérivées d'état */
    beta_dot = a11 * state->beta + a12 * state->r + b1 * delta;
    r_dot    = a21 * state->beta + a22 * state->r + b2 * delta;
    y_dot    = VEHICLE_SPEED * state->beta + VEHICLE_SPEED * state->psi;
    psi_dot  = state->r;

    /* Intégration Euler explicite */
    state->beta += beta_dot * dt;
    state->r    += r_dot * dt;
    state->y    += y_dot * dt;
    state->psi  += psi_dot * dt;
}

/**
 * @brief  Initialise l'état du véhicule
 * @param  state : Structure à initialiser
 * @param  y0    : Écart latéral initial (m)
 * @param  psi0  : Angle de cap initial (rad)
 * @retval None
 */
void LKA_InitVehicleState(VehicleState_t* state, float y0, float psi0)
{
    if (state == NULL)
    {
        return;
    }

    state->beta = 0.0f;
    state->r    = 0.0f;
    state->y    = y0;
    state->psi  = psi0;
}

/**
 * @brief  Fonction principale du contrôleur LKA (appelée périodiquement)
 * @param  state : État courant du véhicule
 * @param  dt    : Pas de temps (s)
 * @retval Structure de sortie avec delta et erreurs
 */
ControlOutput_t LKA_ControllerMain(VehicleState_t* state, float dt)
{
    ControlOutput_t output;
    float delta;

    /* Initialisation de la sortie */
    output.delta    = 0.0f;
    output.error_y  = 0.0f;
    output.error_psi = 0.0f;

    if (state == NULL || dt <= 0.0f)
    {
        return output;
    }

    /* Calcul de la commande */
    delta = LKA_ComputeSteeringAngle(state);

    /* Mise à jour de l'état */
    LKA_UpdateVehicleState(state, delta, dt);

    /* Remplissage de la structure de sortie */
    output.delta    = delta;
    output.error_y  = state->y;       /* Erreur = écart courant (trajectoire = 0) */
    output.error_psi = state->psi;     /* Erreur = cap courant (référence = 0) */

    return output;
}

/* ============================================================================
 * SECTION 5 : FONCTIONS UTILITAIRES
 * ============================================================================ */

/**
 * @brief  Vérifie si le système est dans les limites de sécurité
 * @param  state : État courant
 * @retval 1 si OK, 0 si alerte
 */
int32_t LKA_CheckSafetyLimits(const VehicleState_t* state)
{
    if (state == NULL)
    {
        return 0;
    }

    /* Vérification REQ-LKA-002 : écart < 0.3m doit être corrigé */
    if (fabsf(state->y) > 0.5f)
    {
        return 0;  /* Alerte : écart trop important */
    }

    /* Vérification REQ-LKA-005 : robustesse au vent */
    if (fabsf(state->beta) > 0.5f)  /* ~28° */
    {
        return 0;  /* Alerte : dérive excessive */
    }

    return 1;  /* Système dans les limites */
}

/**
 * @brief  Convertit degrés en radians
 * @param  deg : Angle en degrés
 * @retval Angle en radians
 */
float LKA_DegToRad(float deg)
{
    return deg * 3.14159265f / 180.0f;
}

/**
 * @brief  Convertit radians en degrés
 * @param  rad : Angle en radians
 * @retval Angle en degrés
 */
float LKA_RadToDeg(float rad)
{
    return rad * 180.0f / 3.14159265f;
}

/* ============================================================================
 * FIN DU FICHIER
 * ============================================================================ */
