#include <MicroArduino.h>
#include <StepperMotor.h>

DCMotor coil1;
DCMotor coil2;

StepperMotor myStepper;

long posMS = 0;


void setup()
{
  Serial.begin(115200);
  Serial.println("StepperMotor @ Micro Arduino");

  coil1 = *micro.getDCMotor(1);
  coil2 = *micro.getDCMotor(2);

  myStepper.setHalfBridge(coil1, coil2);
  myStepper.motorConfig(200, 80, 280, 400);

  delay(500);
}

void loop() {
  while (!myStepper.stepping(posMS, 1800)) {}
  myStepper.release();
  delay(5000);
}
