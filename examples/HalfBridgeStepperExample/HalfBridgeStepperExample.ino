#include <MegaDueShield.h>
#include <HalfBridgeStepper.h>

void setup() {

  MegaDueShield shield {};

  DC_Motor *coil1 = shield.getDCMotor(M3);
  DC_Motor *coil2 = shield.getDCMotor(M4);

  // not best practice on the MegaDueShield, but decent for demonstration.
  StepperMotor *mot = new HalfBridgeStepper(coil1, coil2, 200);

  mot->speedConfig(0, 300, 100);

  mot->setDistance(4000);
  while (!mot->run()) {}

  delay(1000);

  mot->setDistance(-4000);
  while (!mot->run()) {}

  mot->release();
}

void loop() {

}
