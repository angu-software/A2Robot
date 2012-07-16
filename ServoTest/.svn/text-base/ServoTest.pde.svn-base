#include <SoftwareServo.h>

#define SERVOPIN 3
#define MAX_SPEED 180

SoftwareServo servo;

void setup(){
  servo.attach(SERVOPIN);
}


void loop(){

  servo.write(MAX_SPEED);
  SoftwareServo::refresh();
  
}
