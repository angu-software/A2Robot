#include <IRremote.h>
#include <IRremoteInt.h>
#include <Stepper.h>
#include <SoftwareServo.h>
#include "Data.h"

#define DEBUG
//#define DEBUG_INTERUPT

//== Weel Definitions
#define PIN_LEFT_WHEEL 6
#define PIN_RIGHT_WHEEL 5

#define WHEEL_LEFT_MAX_FORWARD    30
#define WHEEL_RIGHT_MAX_FORWARD   150

#define WHEEL_LEFT_MAX_BACKWARD   180
#define WHEEL_RIGHT_MAX_BACKWARD  0

#define WHEEL_LEFT_HALF_BACKWARD  110
#define WHEEL_RIGHT_HALF_BACKWARD 70

#define WHEEL_POWER_CONTROL 7
#define WHEEL_POWER_ON HIGH
#define WHEEL_POWER_OFF LOW
bool wheelOn = false;

SoftwareServo leftWheel;
SoftwareServo rightWheel;

Speed mSpeed;
Movement mMovement;
Direction mDirection;

//== Stepper definitions
#define STEP_CONTROL01 9
#define STEP_CONTROL02 10
#define STEP_CONTROL03 11
#define STEP_CONTROL04 12
#define STEPPS_PER_REVOLUTION 200
#define STEPPER_SPEED 60
#define STEPS_PER_LOOP 1
Stepper stepper = Stepper(STEPPS_PER_REVOLUTION, STEP_CONTROL01, STEP_CONTROL02, STEP_CONTROL03, STEP_CONTROL04);
Movement stepperDirection = backwards;
int mStepperAngle;

//== IR-Friend communication definitions
#define INTERUPT 0 //digital pin 2
#define REC_SEND_DELAY 10
bool sendingIR = false;
bool friendSeen = false;
IRsend irsend;
unsigned int friendSignal[] = {100, 100, 100, 100, 100, 100};
unsigned int mFriendLastSeenAngle;

//== proximity
#define PROXIMITY_INPUT 5
#define PROXIMITY_TRESHOLD 170
#define PROXIMITY_FRIEND_TRESHOLD 300

unsigned int mCurrentDistance;

//=== states
#define STATE_DRIVE_FORWARD  0
#define STATE_AVOID_OBSTACLE 1
#define STATE_FOLLOW_FRIEND  2

unsigned int mState = STATE_DRIVE_FORWARD;
bool mTimerRuns = false;

#define AVOID_OBSTACLE_DELAY 500

// timestamp
unsigned long mCurrentTs;

void setup() {
    mCurrentTs = millis();
    randomSeed(analogRead(0));

    // wheels
    pinMode(WHEEL_POWER_CONTROL, OUTPUT);
    setupWheels();
    stepper.setSpeed(STEPPER_SPEED);

    //friend detection
    attachInterrupt(INTERUPT, friendDetected, LOW);
    
    // stepper config
    mStepperAngle = 100; // 0 means behind the robot, start position!

    Serial.begin(115200);
}

void loop() {
    sendFriendSignal();
    delay(5);

    if (mState == STATE_DRIVE_FORWARD) {
        moveSensorTurret();
        handleSensorData();
    } else if (   mState == STATE_AVOID_OBSTACLE
               || mState == STATE_FOLLOW_FRIEND) {
        if (!mTimerRuns) {
            startTimer();
        } else {
            if (delayOver(AVOID_OBSTACLE_DELAY)) {
                handleSensorData();
            }
        }
    }

    SoftwareServo::refresh();
}

void startTimer() {
    mCurrentTs = millis();
    mTimerRuns = true;
}

bool delayOver(unsigned int delay) {
    if (mCurrentTs + delay > millis()) {
        return false;
    } else {
        mTimerRuns = false;
        return true;
    }
}

void handleSensorData() {
    mState = STATE_DRIVE_FORWARD; // default state

    friendDetection();
    measureDistanceAndDecide();

    if (mState == STATE_AVOID_OBSTACLE) {
        #ifdef DEBUG
        Serial.println("avoidance behavior");
        #endif
        if (mStepperAngle >= 0 && mStepperAngle <= 40) {
            #ifdef DEBUG
            Serial.println("something is left");
            #endif
            driveRobot(full, forwards, right);
        } else if (mStepperAngle >= 60 && mStepperAngle <= 100) {
            #ifdef DEBUG
            Serial.println("something is right");
            #endif
            driveRobot(full, forwards, left);
        } else if (mStepperAngle > 40 && mStepperAngle < 60) {
            #ifdef DEBUG
            Serial.println("something in front");
            #endif
            driveRobot(full, backwards, left);
        } else {
            #ifdef DEBUG
            Serial.println("false alarm!");
            #endif
            mState = STATE_DRIVE_FORWARD;
            driveRobot(full, forwards, none);
        }
    } else if (mState == STATE_FOLLOW_FRIEND) {
        if (mFriendLastSeenAngle >= 0 && mFriendLastSeenAngle <= 45) {
            #ifdef DEBUG
            Serial.println("friend is left");
            #endif
            driveRobot(full, forwards, left);
        } else if (mFriendLastSeenAngle >= 55 && mFriendLastSeenAngle <= 100) {
            #ifdef DEBUG
            Serial.println("friend is right");
            #endif
            driveRobot(full, forwards, right);
        } else if (mFriendLastSeenAngle > 45 && mFriendLastSeenAngle < 55) {
            #ifdef DEBUG
            Serial.println("friend is in front");
            #endif
            driveRobot(full, forwards, none);
        } else {
            #ifdef DEBUG
            Serial.println("friend is behind me, ignore");
            #endif
            mState = STATE_DRIVE_FORWARD;
            driveRobot(full, forwards, none);
        }
    } else {
        driveRobot(full, forwards, none);
    }
}

void measureDistanceAndDecide() {
    mCurrentDistance = getProximity();

    // if we see a friend, use another threshold
    if (mState == STATE_FOLLOW_FRIEND) {
        if (mCurrentDistance > PROXIMITY_FRIEND_TRESHOLD) {
            #ifdef DEBUG
            Serial.println("the friend is to near!");
            #endif
            mState = STATE_AVOID_OBSTACLE;
        }
    } else {
        if (mCurrentDistance > PROXIMITY_TRESHOLD) {
            #ifdef DEBUG
            Serial.println("something is near!");
            #endif
            mState = STATE_AVOID_OBSTACLE;
        }
    }
}

void moveSensorTurret(){
    // move turret
    if(mStepperAngle >= (STEPPS_PER_REVOLUTION / 2)) {
        stepperDirection = backwards;
    } else if (mStepperAngle == 0){
        stepperDirection = forwards;
    }
    stepper.step(STEPS_PER_LOOP * stepperDirection);

    if (stepperDirection == forwards) {
        mStepperAngle += STEPS_PER_LOOP;
        if (mStepperAngle > (STEPPS_PER_REVOLUTION / 2)) {
            mStepperAngle = (STEPPS_PER_REVOLUTION / 2);    
        }
    } else {
        mStepperAngle -= STEPS_PER_LOOP;
        if (mStepperAngle < 0) {
            mStepperAngle = 0;    
        }
    }
}

int getProximity(){
    int prox = analogRead(PROXIMITY_INPUT);

    #ifdef DEBUG
    Serial.print("Proximity (");
    Serial.print(mStepperAngle);
    Serial.print(" deg) :");
    Serial.println(prox);
    #endif

    return prox;
}

//=== Wheel methods

void setupWheels() {
    leftWheel.attach(PIN_LEFT_WHEEL);
    rightWheel.attach(PIN_RIGHT_WHEEL);

    mSpeed = stop;
    mMovement = forwards;
    mDirection = none;
}

void stopWheels(bool stopIt){
    if(!stopIt){
        digitalWrite(WHEEL_POWER_CONTROL,WHEEL_POWER_ON);
    } else {
        digitalWrite(WHEEL_POWER_CONTROL,WHEEL_POWER_OFF);
    }
}

void driveRobot(Speed speed, Movement movement, Direction direction) {
    mSpeed = speed;
    mMovement = movement;
    mDirection = direction;

    adjustWheels();
}

void adjustWheels() {
    int speedLeft, speedRight;
    
    if (mMovement == forwards) {
        speedLeft = WHEEL_LEFT_MAX_FORWARD;
        speedRight = WHEEL_RIGHT_MAX_FORWARD;

        if (mDirection == right) {
            speedRight = WHEEL_RIGHT_MAX_BACKWARD;
        } else if (mDirection == left) {
            speedLeft = WHEEL_LEFT_MAX_BACKWARD;
        }
    } else {
        speedLeft = WHEEL_LEFT_MAX_BACKWARD;
        speedRight = WHEEL_RIGHT_MAX_BACKWARD;

        if (mDirection == right) {
            speedLeft = WHEEL_LEFT_HALF_BACKWARD;
        } else if (mDirection == left) {
            speedRight = WHEEL_RIGHT_HALF_BACKWARD;
        }
    }

        
    if(mSpeed != stop && !wheelOn){
        wheelOn = true;
        stopWheels(!wheelOn);
    }else if(mSpeed == stop && wheelOn){
        wheelOn = false;
        stopWheels(!wheelOn);
    }

    leftWheel.write(speedLeft);
    rightWheel.write(speedRight);
}

//== friend communication

void sendFriendSignal(){
    sendingIR = true;

    irsend.sendRaw(friendSignal, 6, 40);

    sendingIR = false;
}

void friendDetected(){
    if(!sendingIR){
        friendSeen = true;
        mFriendLastSeenAngle = mStepperAngle;
    }

    #ifdef DEBUG_INTERUPT
    Serial.println("INTERUPT");
    #endif
}

void friendDetection() {
    if (friendSeen) {
        mState = STATE_FOLLOW_FRIEND;
        friendSeen = false;
        #ifdef DEBUG
        Serial.println("i can see friends!");
        #endif
    }
}