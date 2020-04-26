#include <MegaDueShield.h>
#include <A4988.h>

void setup() {

  MegaDueShield shield {};
  Serial.begin(115200);

  // get the shield's second stepper and config 200 steps per revolution
  StepperMotor *mot = shield.getStepper(EXT_STEPPER_2, 200);

  mot->speedConfig(1, 150, 100);

  Serial.println("Start....");

  mot->setDistance(400);
  while (!mot->run()) {}

  Serial.println("run 1....");
  delay(1000);


  mot->setDistance(-400);
  while (!mot->run()) {}

  Serial.println("run 2 finished....");

  mot->release();
}

void loop() {

}
