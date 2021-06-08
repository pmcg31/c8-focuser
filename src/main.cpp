#include <Arduino.h>

#include "ELSC8FocuserMotor.hpp"

const int drvEnablePin = 25;
const int drvStepPin = 26;
const int drvDirPin = 27;
const int drvMs1Pin = 33;
const int drvMs2Pin = 32;

bool enabled = true;

ELS::C8FocuserMotor *focuserMotor = 0;

// FreeRTOS task function sig
void taskFunc(void *param);
TaskFunction_t taskFuncPtr = taskFunc;
double ticksPerMS = 1 / portTICK_PERIOD_MS;
void setup()
{
  delay(1500);

  Serial2.begin(115200);
  Serial2.println();
  Serial2.printf("c8-focuser %s %s\r\n", __DATE__, __TIME__);

  focuserMotor = new ELS::C8FocuserMotor(drvEnablePin,
                                         drvStepPin,
                                         drvDirPin,
                                         drvMs1Pin,
                                         drvMs2Pin);

  if (!focuserMotor->startComms())
  {
    Serial2.println("Failed to start comms!");
    return;
  }

  // focuserMotor->enableMotor(true);

  // focuserMotor->testMotor(1);

  // delay(40000);

  // focuserMotor->enableMotor(false);
}

void loop()
{
  // End the built-in task for loop
  vTaskDelete(NULL);
}
