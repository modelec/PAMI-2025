#include <Arduino.h>

#include <TMCStepper.h>

// Courant max moteur (en mA)
#define R_SENSE 0.11f // Valeur typique pour TMC2209
#define CURRENT 800   // Courant RMS souhaité

// UART
#define TX_PIN1 17
#define RX_PIN1 16

// Moteur 1
#define DIR_PIN1 4
#define STEP_PIN1 33

// Moteur 2
#define DIR_PIN2 16
#define STEP_PIN2 32

// Moteur 3
#define DIR_PIN3 2
#define STEP_PIN3 13

// Instances TMC2209
TMC2209Stepper driver1(&Serial1, R_SENSE, 0b00);
TMC2209Stepper driver2(&Serial1, R_SENSE, 0b01);
TMC2209Stepper driver3(&Serial1, R_SENSE, 0b10);

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN1, TX_PIN1);

  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STEP_PIN3, OUTPUT);
  pinMode(DIR_PIN3, OUTPUT);

  // Initialisation des drivers
  driver1.begin();
  driver1.rms_current(CURRENT);
  driver1.microsteps(16);
  driver1.en_spreadCycle(false); // Utilise StealthChop
  driver1.pdn_disable(true);     // UART uniquement

  driver2.begin();
  driver2.rms_current(CURRENT);
  driver2.microsteps(16);
  driver2.en_spreadCycle(false);
  driver2.pdn_disable(true);

  driver3.begin();
  driver3.rms_current(CURRENT);
  driver3.microsteps(16);
  driver3.en_spreadCycle(false);
  driver3.pdn_disable(true);

  Serial.println("Drivers initialisés.");
}

void stepMotor(int stepPin, int dirPin, bool direction, int steps, int delay_us)
{
  digitalWrite(dirPin, direction);
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_us);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_us);
  }
}

void loop()
{
  // Exemple : chaque moteur fait 200 pas dans un sens, puis dans l'autre
  stepMotor(STEP_PIN1, DIR_PIN1, true, 200, 1000);
  delay(500);
  stepMotor(STEP_PIN1, DIR_PIN1, false, 200, 1000);

  stepMotor(STEP_PIN2, DIR_PIN2, true, 200, 1000);
  delay(500);
  stepMotor(STEP_PIN2, DIR_PIN2, false, 200, 1000);

  stepMotor(STEP_PIN3, DIR_PIN3, true, 200, 1000);
  delay(500);
  stepMotor(STEP_PIN3, DIR_PIN3, false, 200, 1000);

  delay(2000);
}
