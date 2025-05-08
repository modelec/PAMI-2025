#include <Arduino.h>
#include <TMCStepper.h>
#include "main.h"

// UART (bus unique)
HardwareSerial TMCSerial(1);

TMC2209Stepper driver1(&TMCSerial, R_SENSE, DRIVER1_ADDR);
TMC2209Stepper driver2(&TMCSerial, R_SENSE, DRIVER2_ADDR);

volatile int32_t speed1_steps_per_sec = 0; // target speed (signed)
volatile int32_t speed2_steps_per_sec = 0;
uint32_t last_step_time1 = 0;
uint32_t last_step_time2 = 0;

// Pour le redémarrage des moteurs après capteur obstacle
Direction lastDirection = STOP;
volatile int32_t lastSpeed = 0;

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

// Non-blocking stepper update function
void updateSteppers()
{
  uint32_t now = micros();
  // Moteur 1
  if (speed1_steps_per_sec != 0)
  {
    uint32_t interval = 1000000UL / abs(speed1_steps_per_sec);
    if (now - last_step_time1 >= interval)
    {
      digitalWrite(M1_STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(M1_STEP_PIN, LOW);
      last_step_time1 = now;
    }
  }
  // Moteur 2
  if (speed2_steps_per_sec != 0)
  {
    uint32_t interval = 1000000UL / abs(speed2_steps_per_sec);
    if (now - last_step_time2 >= interval)
    {
      digitalWrite(M2_STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(M2_STEP_PIN, LOW);
      last_step_time2 = now;
    }
  }
}

// En avant
void forward(int32_t speed)
{
  digitalWrite(M1_DIR_PIN, HIGH);
  digitalWrite(M2_DIR_PIN, HIGH);
  lastDirection = FORWARD;
  speed1_steps_per_sec = speed;
  speed2_steps_per_sec = speed;
}

// En arrière
void backward(int32_t speed)
{
  digitalWrite(M1_DIR_PIN, LOW);
  digitalWrite(M2_DIR_PIN, LOW);
  lastDirection = BACKWARD;
  speed1_steps_per_sec = speed;
  speed2_steps_per_sec = speed;
}

// Rotation à gauche
void turnLeft(int32_t speed)
{
  digitalWrite(M1_DIR_PIN, LOW);
  digitalWrite(M2_DIR_PIN, HIGH);
  lastDirection = LEFT;
  speed1_steps_per_sec = speed;
  speed2_steps_per_sec = speed;
}

// Rotation à droite
void turnRight(int32_t speed)
{
  digitalWrite(M1_DIR_PIN, HIGH);
  digitalWrite(M2_DIR_PIN, LOW);
  lastDirection = RIGHT;
  speed1_steps_per_sec = speed;
  speed2_steps_per_sec = speed;
}

// Arrêt des moteurs
void stopMotors()
{
  speed1_steps_per_sec = 0;
  speed2_steps_per_sec = 0;
}

// Vérifie si les moteurs sont en mouvement
bool isMoving()
{
  return (speed1_steps_per_sec != 0 || speed2_steps_per_sec != 0);
}

// Capteur à ultrasons
const int obstacleCheckInterval = 100; // ms between distance checks
unsigned long lastObstacleCheckTime = 0;
const int obstacleThresholdCM = 20; // stop if closer than 20 cm

long readDistanceCM()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 20000); // timeout 20 ms
  long distance = duration * 0.034 / 2;
  return distance;
}

void detectObstacles()
{
  // On vérifie les obstacles
  unsigned long now = millis();
  if (now - lastObstacleCheckTime >= obstacleCheckInterval)
  {
    lastObstacleCheckTime = now;

    // Si le robot tourne sur lui-même, on ne vérifie pas les obstacles
    if (lastDirection == RIGHT || lastDirection == LEFT)
    {
      return;
    }

    long distance = readDistanceCM();

    if (distance > 0 && distance < obstacleThresholdCM)
    {
      if (isMoving())
      {
        lastSpeed = speed1_steps_per_sec;
        stopMotors();
        Serial.println("Obstacle detected: Stopped");
      }
    }
    else
    {
      if (!isMoving())
      {
        switch (lastDirection)
        {
        case FORWARD:
          forward(lastSpeed);
          break;
        case BACKWARD:
          backward(lastSpeed);
          break;
        case LEFT:
          turnLeft(lastSpeed);
          break;
        case RIGHT:
          turnRight(lastSpeed);
          break;
        default:
          break;
        }
        Serial.println("Path clear: Resuming movement");
      }
    }
  }
}

void setup()
{
  Serial.begin(115200);
  TMCSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_ENABLE_PIN, OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_ENABLE_PIN, OUTPUT);
  // Capteur à ultrasons
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(M1_ENABLE_PIN, LOW);
  digitalWrite(M2_ENABLE_PIN, LOW);

  driver1.begin();
  driver2.begin();

  driver1.rms_current(CURRENT);
  driver2.rms_current(CURRENT);

  driver1.microsteps(16);
  driver2.microsteps(16);

  driver1.en_spreadCycle(false);
  driver2.en_spreadCycle(false);

  stopMotors();
  turnRight(200); // Move forward at 200 steps/sec
  Serial.println("Setup complete");
}

void loop()
{
  updateSteppers(); // Mise à jour des moteurs asynchrone

  detectObstacles(); // Vérification des obstacles
}
