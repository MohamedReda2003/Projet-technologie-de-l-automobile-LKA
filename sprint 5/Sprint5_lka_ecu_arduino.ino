/* ============================================================================
 * PROJET LKA - SPRINT 5 : CODE ECU ARDUINO POUR HiL
 * Système Line Keeping Assistant - Tesla Model 3
 * Méthode : Agile Scrum | Plateforme : Arduino Mega 2560
 * ============================================================================
 * Fichier    : lka_ecu_arduino.ino
 * Auteur     : Équipe LKA-ADAS-2026
 * Date       : Juin 2026
 * Version    : 1.0.0
 * ============================================================================
 * Description:
 *   Ce code s'exécute sur l'Arduino Mega 2560 (ECU) et communique
 *   avec le simulateur Scilab/Xcos via liaison Serial USB.
 *   Protocole : 115200 baud, format JSON-like simplifié
 * ============================================================================
 */

#include <Arduino.h>

/* ============================================================================
 * SECTION 1 : PARAMÈTRES DU VÉHICULE
 * ============================================================================ */

const float VEHICLE_MASS = 1800.0f;
const float VEHICLE_SPEED = 30.0f;
const float TIRE_CF = 120000.0f;
const float TIRE_CR = 180000.0f;

/* ============================================================================
 * SECTION 2 : PARAMÈTRES LQR
 * ============================================================================ */

const float K_BETA = 7.9247f;
const float K_R = 0.2175f;
const float K_Y = 3.1623f;
const float K_PSI = 11.7687f;

const float DELTA_MAX = 0.0873f;   // 5° en radians
const float DELTA_MIN = -0.0873f;  // -5° en radians

/* ============================================================================
 * SECTION 3 : VARIABLES GLOBALES
 * ============================================================================ */

float beta = 0.0f;
float r = 0.0f;
float y = 0.0f;
float psi = 0.0f;
float delta = 0.0f;

unsigned long lastCycleTime = 0;
const unsigned long CYCLE_PERIOD_US = 1000;  // 1 kHz = 1000 µs

/* ============================================================================
 * SECTION 4 : FONCTIONS
 * ============================================================================ */

/**
 * @brief Calcule l'angle de braquage LQR
 */
float computeSteeringAngle(float b, float rv, float yv, float p)
{
    float d = -(K_BETA * b + K_R * rv + K_Y * yv + K_PSI * p);

    // Saturation REQ-LKA-003
    if (d > DELTA_MAX) d = DELTA_MAX;
    else if (d < DELTA_MIN) d = DELTA_MIN;

    return d;
}

/**
 * @brief Parse les données reçues du simulateur
 * Format attendu : "BETA,R,Y,PSI\n"
 */
bool parseStateFromSerial(float* b, float* rv, float* yv, float* p)
{
    if (Serial.available() > 0)
    {
        String data = Serial.readStringUntil('\n');
        int comma1 = data.indexOf(',');
        int comma2 = data.indexOf(',', comma1 + 1);
        int comma3 = data.indexOf(',', comma2 + 1);

        if (comma1 > 0 && comma2 > comma1 && comma3 > comma2)
        {
            *b = data.substring(0, comma1).toFloat();
            *rv = data.substring(comma1 + 1, comma2).toFloat();
            *yv = data.substring(comma2 + 1, comma3).toFloat();
            *p = data.substring(comma3 + 1).toFloat();
            return true;
        }
    }
    return false;
}

/**
 * @brief Envoie la commande vers le simulateur
 * Format : "DELTA\n"
 */
void sendCommandToSerial(float d)
{
    Serial.print(d, 6);
    Serial.print("\n");
}

/**
 * @brief Vérifie les limites de sécurité
 */
bool checkSafetyLimits(float yv, float bv)
{
    if (abs(yv) > 0.5f) return false;   // REQ-LKA-002
    if (abs(bv) > 0.5f) return false;   // REQ-LKA-005
    return true;
}

/* ============================================================================
 * SECTION 5 : SETUP ET LOOP ARDUINO
 * ============================================================================ */

void setup()
{
    Serial.begin(115200);
    while (!Serial) { ; }  // Attente connexion

    Serial.println("LKA_ECU_READY");

    lastCycleTime = micros();
}

void loop()
{
    unsigned long currentTime = micros();

    // Boucle temps réel à 1 kHz
    if (currentTime - lastCycleTime >= CYCLE_PERIOD_US)
    {
        lastCycleTime = currentTime;

        // 1. Réception état du simulateur
        if (parseStateFromSerial(&beta, &r, &y, &psi))
        {
            // 2. Vérification sécurité
            if (checkSafetyLimits(y, beta))
            {
                // 3. Calcul commande LQR
                delta = computeSteeringAngle(beta, r, y, psi);

                // 4. Envoi commande
                sendCommandToSerial(delta);
            }
            else
            {
                // Mode dégradé : désactivation
                sendCommandToSerial(0.0f);
                Serial.println("LKA_SAFETY_ALERT");
            }
        }
    }
}

/* ============================================================================
 * FIN DU FICHIER
 * ============================================================================ */
