#include <MegaDueShield.h>
#include <HalfBridgeStepper.h>

const int stepsPerRevolution = 2038;

void setup() {

  MegaDueShield shield {};

  DC_Motor *coil1 = shield.getDCMotor(M5);
  DC_Motor *coil2 = shield.getDCMotor(M6);

  // not best practice on the MegaDueShield, but decent for demonstration.
  StepperMotor *mot = new HalfBridgeStepper(coil1, coil2, 2038);

  mot->speedConfig(0, 30, 30);

  mot->setDistance(stepsPerRevolution);
  while (!mot->run()) {}

  delay(1000);

  mot->setDistance(-2038*2);
  while (!mot->run()) {}

  delay(1000);

  mot->setDistance(2038);
  while (!mot->run()) {}

  mot->release();
}

void loop() {

}
