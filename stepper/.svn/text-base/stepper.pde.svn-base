#include <Stepper.h>

#define STEP_CONTROL01 9
#define STEP_CONTROL02 10
#define STEP_CONTROL03 11
#define STEP_CONTROL04 12
#define STEPPS_PER_REVOLUTION 200
#define STEPPER_SPEED 60
Stepper stepper = Stepper(STEPPS_PER_REVOLUTION,STEP_CONTROL01, STEP_CONTROL02, STEP_CONTROL03, STEP_CONTROL04);

void setup(){
  stepper.setSpeed(STEPPER_SPEED);
}

void loop(){
  stepper.step(10);
  delay(1);
}
