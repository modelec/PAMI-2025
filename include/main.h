#ifndef MAIN_H
#define MAIN_H

enum Direction
{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP
};

enum StepType
{
    STEP_FORWARD,
    STEP_ROTATE,
    STEP_FORWARD_UNTIL_END
};

struct Step
{
    StepType type;
    float value; // cm pour STEP_FORWARD, deg pour ROTATE, ignoré pour UNTIL_FALL
};

// UART
#define TX_PIN 17
#define RX_PIN 16

// PINs réels
#ifndef SIMULATOR

// Moteur 1 - Gauche
#define M1_DIR_PIN 19
#define M1_STEP_PIN 23
#define M1_ENABLE_PIN 26

// Moteur 2 - Droite
#define M2_DIR_PIN 21
#define M2_STEP_PIN 25
#define M2_ENABLE_PIN 27

// Capteur à ultrasons
#define TRIG_PIN 15
#define ECHO_PIN 14

// Capteur de chute
#define FALL_PIN 13

#endif

// Le moteur 1 est en 32 microsteps, le moteur 2 est en 64 microsteps
#define MOTOR_MULTIPLIER 2 // Pour ajuster la vitesse des moteurs car ils ont des microsteps différents

// PINs pour le simulateur
#ifdef SIMULATOR

// Moteur 1 - Gauche
#define M1_DIR_PIN 4
#define M1_STEP_PIN 33
#define M1_ENABLE_PIN 14

// Moteur 2 - Droite
#define M2_DIR_PIN 16
#define M2_STEP_PIN 32
#define M2_ENABLE_PIN 27

// Capteur à ultrasons
#define TRIG_PIN 18
#define ECHO_PIN 5

// Capteur de chute
#define FALL_PIN 17

#endif

// Paramètres moteurs
#define R_SENSE 0.11f
#define CURRENT 800 // mA
// Adresses des drivers TMC2209
#define DRIVER1_ADDR 0b01
#define DRIVER2_ADDR 0b10

// Constantes
#define WHEEL_DIAMETER 5.5 // cm
#define WHEEL_BASE 7.2     // cm
#define STEPS_PER_REV 200.0
#define MICROSTEPPING 8.0

#endif // MAIN_H