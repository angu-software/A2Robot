#include "WProgram.h"
void setup();
void loop();
#include <Stepper.h>

#define STEP_CONTROL01 12
#define STEP_CONTROL02 11
#define STEP_CONTROL03 10
#define STEP_CONTROL04 9
#define STEPPS_PER_REVOLUTION 200
#define STEPPER_SPEED 500
Stepper stepper = Stepper(STEPPS_PER_REVOLUTION,STEP_CONTROL01, STEP_CONTROL02, STEP_CONTROL03, STEP_CONTROL04);

void setup(){
	stepper.setSpeed(STEPPER_SPEED);
}

void loop(){
	stepper.step(100);
		delay(1000);
}


