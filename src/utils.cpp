#include "main.h"
#include "utils.h"
#include <Arduino.h>

void enableDrivers()
{
    digitalWrite(M1_ENABLE_PIN, LOW); // LOW pour activer le driver
    digitalWrite(M2_ENABLE_PIN, LOW);
}

void disableDrivers()
{
    digitalWrite(M1_ENABLE_PIN, HIGH);
    digitalWrite(M2_ENABLE_PIN, HIGH);
}

int getStepsForDistance(float cm)
{
    /*
    float circumference = 3.1416 * WHEEL_DIAMETER;
    float steps_per_cm = (STEPS_PER_REV * 16) / circumference;
    */
    return cm * 740; // Valeur calculée à la main pour une vitesse de 5000 :)
}