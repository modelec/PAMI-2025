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

// UART
#define TX_PIN 17
#define RX_PIN 16

// PINs réels
#ifndef SIMULATOR

// Moteur 1 - Gauche
#define M1_DIR_PIN 18
#define M1_STEP_PIN 22
#define M1_ENABLE_PIN 32

// Moteur 2 - Droite
#define M2_DIR_PIN 19
#define M2_STEP_PIN 23
#define M2_ENABLE_PIN 26

// Capteur à ultrasons
#define TRIG_PIN 14
#define ECHO_PIN 15

// Capteur de chute
#define FALL_TRIG_PIN 12
#define FALL_ECHO_PIN 13

#endif

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
#define FALL_TRIG_PIN 17
#define FALL_ECHO_PIN 15

#endif

// Paramètres moteurs
#define R_SENSE 0.11f
#define CURRENT 800 // mA
// Adresses des drivers TMC2209
#define DRIVER1_ADDR 0b00
#define DRIVER2_ADDR 0b01

// Constantes pour les roues
#define WHEEL_DIAMETER 5.5     // cm
#define STEPS_PER_REV 200 * 16 // microsteps
#define WHEEL_BASE 7.2         // cm

#endif // MAIN_H