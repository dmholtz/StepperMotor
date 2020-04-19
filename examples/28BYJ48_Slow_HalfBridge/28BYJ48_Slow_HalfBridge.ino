#include <MegaDueShield.h>
#include <HalfBridgeStepper.h>

void setup() {

  MegaDueShield shield {};

  DC_Motor *coil1 = shield.getDCMotor(M1);
  DC_Motor *coil2 = shield.getDCMotor(M2);

  // not best practice on the MegaDueShield, but decent for demonstration.
  StepperMotor *mot = new HalfBridgeStepper(coil1, coil2, 2038);

  mot->speedConfig(0, 30, 30);

  mot->setDistance(2038*2);
  while (!mot->run()) {}

  delay(1000);

  mot->setDistance(-2038*2);
  while (!mot->run()) {}

  mot->release();
}

void loop() {

}
