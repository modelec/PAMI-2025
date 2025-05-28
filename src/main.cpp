#include <Arduino.h>
// #define SIMULATOR // Commenter cette ligne pour utiliser le matériel réel
//   Définitions des constantes dans main.h
#include "main.h"
#include "utils.h"

volatile int32_t speed_steps_per_sec = 0; // target speed (signed)
uint32_t last_step_time = 0;

// Pour le redémarrage des moteurs après capteur obstacle
Direction lastDirection = STOP;
volatile int32_t lastSpeed = 0;

volatile int32_t steps_target = 0;
volatile int32_t steps_done = 0;
bool movementInProgress = false;

// Chaque étape du scénario
Step scenario[] = {
    {STEP_FORWARD, 20},
    // {STEP_ROTATE, 90},
    // {STEP_FORWARD_UNTIL_FALL, 0}
};
const int scenarioLength = sizeof(scenario) / sizeof(Step);
int currentScenarioStep = 0;
bool scenarioInProgress = false;

// Arrêt des moteurs
void stopMotors()
{
  speed_steps_per_sec = 0;
}

// Fonction d'initialisation
void setup()
{
  Serial.begin(115200);

  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_ENABLE_PIN, OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_ENABLE_PIN, OUTPUT);
  // Capteur à ultrasons
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // Capteur de chute
  pinMode(FALL_PIN, INPUT);

  enableDrivers();
  stopMotors();

  scenarioInProgress = true;
  currentScenarioStep = 0;

  Serial.println("Setup complete");
}

void moveAsyncSteps(int32_t steps, int32_t speed, bool forwardDir)
{
  // Le nombre de pas à faire
  steps_target = steps;
  steps_done = 0;

  // La direction du moteur
  digitalWrite(M1_DIR_PIN, forwardDir ? HIGH : LOW);
  digitalWrite(M2_DIR_PIN, forwardDir ? HIGH : LOW);

  speed_steps_per_sec = speed;
  movementInProgress = true;
  lastDirection = forwardDir ? FORWARD : BACKWARD;
}

void rotateAsync(float angleDeg, int32_t speed, bool toRight)
{
  // Informations sur les roues
  const float wheelCirc = PI * WHEEL_DIAMETER;

  float rotationCirc = PI * WHEEL_BASE;
  float distPerWheel = (angleDeg / 360.0) * rotationCirc;
  int32_t steps = (distPerWheel / wheelCirc) * STEPS_PER_REV;

  // Le nombre de pas à faire
  steps_target = steps;
  steps_done = 0;

  // La direction du moteur
  digitalWrite(M1_DIR_PIN, toRight ? HIGH : LOW);
  digitalWrite(M2_DIR_PIN, toRight ? LOW : HIGH);

  speed_steps_per_sec = speed;
  movementInProgress = true;
  lastDirection = toRight ? RIGHT : LEFT;
}

// Non-blocking stepper update function
void updateSteppers()
{
  uint32_t now = micros();

  // Moteur 1
  if (speed_steps_per_sec != 0 && (steps_done < steps_target))
  {
    uint32_t interval1 = 1000000UL / abs(speed_steps_per_sec);
    static uint32_t last_step_time1 = 0;
    if (now - last_step_time1 >= interval1)
    {

      digitalWrite(M1_STEP_PIN, HIGH);
      digitalWrite(M2_STEP_PIN, HIGH);
      delayMicroseconds(2); // Short pulse width for step signal
      digitalWrite(M1_STEP_PIN, LOW);
      digitalWrite(M2_STEP_PIN, LOW);
      last_step_time1 = now;
      steps_done++;
    }
  }

  // Comptage des pas (on considère le moteur le plus lent pour terminer l'étape)
  if (movementInProgress && steps_done >= steps_target)
  {

    stopMotors();
    movementInProgress = false;
    Serial.println("Etape terminé");
  }
}

// Vérifie si les moteurs sont en mouvement
bool isMoving()
{
  return speed_steps_per_sec != 0;
}

// Capteur à ultrasons
const int obstacleCheckInterval = 100; // ms between distance checks
unsigned long lastObstacleCheckTime = 0;
const int obstacleThresholdCM = 20; // stop if closer than 20 cm

long readDistanceCM(uint8_t trigPin, uint8_t echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000); // timeout 20 ms
  long distance = duration * 0.034 / 2;
  return distance;
}

// Vérification de la distance du sol pour éviter les chutes
void checkFall()
{
  // On a fini notre chemin si on arrive en bout de table (détection de distance du sol)
  if (digitalRead(FALL_PIN) == HIGH && movementInProgress)
  {
    stopMotors();
    movementInProgress = false;
    Serial.println("Bout de table détecté : Arrêt");
  }
}

void detectObstacles()
{
  // On vérifie les obstacles
  unsigned long now = millis();
  if (now - lastObstacleCheckTime >= obstacleCheckInterval)
  {
    lastObstacleCheckTime = now;

#ifndef SIMULATOR
    // On regarde la distance du sol
    checkFall();
#endif

    // Si le robot tourne sur lui-même, on ne vérifie pas les obstacles
    if (lastDirection == RIGHT || lastDirection == LEFT)
    {
      return;
    }

    long distance = readDistanceCM(TRIG_PIN, ECHO_PIN);

    if (distance > 0 && distance < obstacleThresholdCM)
    {
      // Si un obstacle est détecté, on arrête le robot
      if (isMoving())
      {
        lastSpeed = speed_steps_per_sec;
        stopMotors();
        Serial.println("Obstacle détecté : Arrêt");
      }
    }
    else
    {
      // Si le robot est arrêté et qu'il n'y a pas d'obstacle, on reprend le mouvement
      if (!isMoving() && movementInProgress)
      {
        switch (lastDirection)
        {
        case FORWARD:
          moveAsyncSteps(steps_target - steps_done, lastSpeed, true);
          break;
        case BACKWARD:
          moveAsyncSteps(steps_target - steps_done, lastSpeed, false);
          break;
        default:
          break;
        }
        Serial.println("Plus d'obstacle : Reprise du mouvement");
      }
    }
  }
}

// On gère le scénario du robot
void processScenario()
{
  if (!scenarioInProgress)
    return;
  if (movementInProgress)
    return; // On attend la fin du mouvement

  if (currentScenarioStep >= scenarioLength)
  {
    Serial.println("Scénario terminé !");
    scenarioInProgress = false;
    return;
  }

  Step &step = scenario[currentScenarioStep];
  switch (step.type)
  {
  case STEP_FORWARD:
    moveAsyncSteps(getStepsForDistance(step.value), 5000, true);
    break;
  case STEP_ROTATE:
    rotateAsync(step.value, 1000, true);
    break;
  case STEP_FORWARD_UNTIL_FALL:
    // On lance un mouvement très long, on s'arrêtera à la détection de chute
    moveAsyncSteps(getStepsForDistance(200), 5000, true); // 200cm = "infini"
    break;
  }
  currentScenarioStep++;
}

void loop()
{

  updateSteppers(); // Mise à jour des moteurs asynchrone

  detectObstacles(); // Vérification des obstacles

  processScenario(); // Gestion du scénario
}
