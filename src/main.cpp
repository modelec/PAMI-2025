#include <Arduino.h>
#include <TMCStepper.h>

// Paramètres moteurs
#define R_SENSE 0.11f
#define CURRENT 800

// UART
#define TX_PIN1 17
#define RX_PIN1 16

// Moteur 1 - Avant gauche
#define DIR_PIN1 4
#define STEP_PIN1 33

// Moteur 2 - Avant droite
#define DIR_PIN2 16
#define STEP_PIN2 32

// Moteur 3 - Arrière
#define DIR_PIN3 2
#define STEP_PIN3 13

TMC2209Stepper driver1(&Serial1, R_SENSE, 0b00);
TMC2209Stepper driver2(&Serial1, R_SENSE, 0b01);
TMC2209Stepper driver3(&Serial1, R_SENSE, 0b10);

// Moteur state
enum MovementState
{
  STOP,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
};

MovementState currentState = STOP;

unsigned long lastStepTime = 0;
const int stepDelay = 800; // microsecondes entre les pas
int stepsRemaining = 0;

struct Motor
{
  int stepPin;
  int dirPin;
  bool dir;
  bool enabled;
  Motor(int s, int d, bool direction, bool en = false)
      : stepPin(s), dirPin(d), dir(direction), enabled(en) {}
};

Motor motors[3] = {
    Motor(STEP_PIN1, DIR_PIN1, true),
    Motor(STEP_PIN2, DIR_PIN2, true),
    Motor(STEP_PIN3, DIR_PIN3, true),
};

void setupMotors()
{
  for (int i = 0; i < 3; i++)
  {
    pinMode(motors[i].stepPin, OUTPUT);
    pinMode(motors[i].dirPin, OUTPUT);
  }

  Serial1.begin(115200, SERIAL_8N1, RX_PIN1, TX_PIN1);

  driver1.begin();
  driver1.rms_current(CURRENT);
  driver1.microsteps(16);
  driver1.en_spreadCycle(false);
  driver1.pdn_disable(true);
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
}

void applyDirection()
{
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(motors[i].dirPin, motors[i].dir);
  }
}

void setMovement(MovementState state, int steps = 400)
{
  currentState = state;
  stepsRemaining = steps;

  switch (state)
  {
  case FORWARD:
    motors[0].dir = true;
    motors[1].dir = true;
    motors[2].dir = true;
    motors[0].enabled = motors[1].enabled = motors[2].enabled = true;
    break;
  case BACKWARD:
    motors[0].dir = false;
    motors[1].dir = false;
    motors[2].dir = false;
    motors[0].enabled = motors[1].enabled = motors[2].enabled = true;
    break;
  case LEFT:
    motors[0].dir = false;
    motors[1].dir = true;
    motors[2].enabled = false;
    motors[0].enabled = motors[1].enabled = true;
    break;
  case RIGHT:
    motors[0].dir = true;
    motors[1].dir = false;
    motors[2].enabled = false;
    motors[0].enabled = motors[1].enabled = true;
    break;
  case STOP:
  default:
    motors[0].enabled = motors[1].enabled = motors[2].enabled = false;
    break;
  }

  applyDirection();
}

void updateMotors()
{
  static bool stepState = false;

  if (stepsRemaining <= 0)
  {
    setMovement(STOP);
    return;
  }

  unsigned long now = micros();
  if (now - lastStepTime >= stepDelay)
  {
    stepState = !stepState;
    lastStepTime = now;

    for (int i = 0; i < 3; i++)
    {
      if (motors[i].enabled)
      {
        digitalWrite(motors[i].stepPin, stepState ? HIGH : LOW);
      }
    }

    if (!stepState)
      stepsRemaining--; // Compte les pas complets
  }
}

void setup()
{
  Serial.begin(115200);
  setupMotors();
  Serial.println("Moteurs prêts.");
}

void loop()
{
  updateMotors();

  // Exemple : déclencher des mouvements toutes les 4 secondes
  static unsigned long lastAction = 0;
  static int actionIndex = 0;
  unsigned long now = millis();

  if (now - lastAction > 4000 && currentState == STOP)
  {
    switch (actionIndex)
    {
    case 0:
      setMovement(FORWARD);
      break;
    case 1:
      setMovement(BACKWARD);
      break;
    case 2:
      setMovement(LEFT);
      break;
    case 3:
      setMovement(RIGHT);
      break;
    }
    actionIndex = (actionIndex + 1) % 4;
    lastAction = now;
  }
}
