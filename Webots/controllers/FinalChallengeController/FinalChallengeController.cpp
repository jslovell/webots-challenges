#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/TouchSensor.hpp>

#include <iostream>

#define TIME_STEP 64
#define MAX_SPEED .5
#define MAX_CAMERA_SPEED 2.0
#define MOVE_TIME 85

// All the webots classes are defined in the "webots" namespace
using namespace webots;

//All components of the robot
static Motor* leftMotor;
static Motor* rightMotor;
static TouchSensor* leftSensor;
static TouchSensor* rightSensor;
static Camera* camera;
static Motor* cameraMotor;

//High level functions
static void MoveForward() {
    leftMotor->setVelocity(MAX_SPEED);
    rightMotor->setVelocity(MAX_SPEED);
}

static void MoveBack() {
    leftMotor->setVelocity(-MAX_SPEED);
    rightMotor->setVelocity(-MAX_SPEED);
}

static void TurnLeft() {
    leftMotor->setVelocity(-MAX_SPEED);
    rightMotor->setVelocity(MAX_SPEED);
}

static void TurnRight() {
    leftMotor->setVelocity(MAX_SPEED);
    rightMotor->setVelocity(-MAX_SPEED);
}

static void ResetMotors() {
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    cameraMotor->setVelocity(0);
}

static bool IsObject(){
return true;
}

static void ThereAndBackAgain(Robot* robot){
    MoveForward();
    robot->step(TIME_STEP*MOVE_TIME);
    MoveBack();
    robot->step(TIME_STEP*MOVE_TIME);
    ResetMotors();
}

int main(int argc, char** argv) {
    //Create the robot instance.
    Robot* robot = new Robot();

    //Set up motor/track wheels
    leftMotor = robot->getMotor("leftMotor");
    rightMotor = robot->getMotor("rightMotor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    //Set up touch sensors
    leftSensor = robot->getTouchSensor("left sensor");
    rightSensor = robot->getTouchSensor("right sensor");
    leftSensor->enable(TIME_STEP);
    rightSensor->enable(TIME_STEP);

    //Set up camera and camera motor
    camera = robot->getCamera("camera");
    camera->enable(TIME_STEP);
    cameraMotor = robot->getMotor("camera motor");
    cameraMotor->setPosition(INFINITY);
    cameraMotor->setVelocity(0.0);


    //Local variables
    //int examlpeNum = 0;
    int currAngle = 0;

    //*********Robot AI goes here*********
    
    if(IsObject()){
      ThereAndBackAgain(robot);
    }
    while (currAngle < 375){
      TurnLeft();
      robot->step(TIME_STEP*3);
      ResetMotors();
      if(IsObject()){
      ThereAndBackAgain(robot);
      }
      
        currAngle = currAngle + 15;
    }
    
    //End of AI
    ResetMotors();
    delete robot;
    return 0;  // EXIT_SUCCESS
}