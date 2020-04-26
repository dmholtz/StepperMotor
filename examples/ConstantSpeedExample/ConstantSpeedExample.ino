#include <MegaDueShield.h>
#include <HalfBridgeStepper.h>

void setup() {

  MegaDueShield shield {};

  StepperMotor *mot = shield.getStepper(M7, M8, 200);

  mot->speedConfig(60, 120, 100);

  // Run the motor at constant speed for one second in each direction

  unsigned long timeStamp = millis();
  while(millis() < (timeStamp + 1000))
  {
    mot->runConstSpeed(Direction::CLOCKWISE);
  }
  delay(1000);
  timeStamp = millis();
  while(millis() < (timeStamp + 1000))
  {
    mot->runConstSpeed(Direction::COUNTERCLOCKWISE);
  }

  mot->release();
}

void loop() {

}
