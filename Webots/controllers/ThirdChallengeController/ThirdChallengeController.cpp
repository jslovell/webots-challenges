#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/TouchSensor.hpp>

#define TIME_STEP 64
#define MAX_SPEED .5
#define MAX_CAMERA_SPEED 2.0

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// All components of the robot
static Motor* leftMotor;
static Motor* rightMotor;
static TouchSensor* leftSensor;
static TouchSensor* rightSensor;
static Camera* camera;
static Motor* cameraMotor;

// High level functions
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

static void DriveForward(double angle, bool inRadians = false) {
    if (inRadians) {
        angle *= 57.295;//180/3.14 = 57
    }
    if (angle > 90 || angle < -90) {
        return;
    }
    else if (angle >= 0) {
        leftMotor->setVelocity(MAX_SPEED);
        rightMotor->setVelocity(MAX_SPEED * sin((angle + 45) / 28.648) );
    }
    else {
        leftMotor->setVelocity(MAX_SPEED * sin((-angle + 45) / 28.648) );
        rightMotor->setVelocity(MAX_SPEED);
    }
}

static void TurnCameraUp() {
    cameraMotor->setVelocity(-MAX_CAMERA_SPEED);
}

static void TurnCameraDown() {
    cameraMotor->setVelocity(MAX_CAMERA_SPEED);
}

static void ResetMotors() {
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    cameraMotor->setVelocity(0);
}

int* GetRGB(int width, int height, int vertOffset) {
    int* array = new int[3];
    array[0] = camera->imageGetRed(camera->getImage(), width, width / 2, height-1-vertOffset);
    array[1] = camera->imageGetGreen(camera->getImage(), width, width / 2, height-1-vertOffset);
    array[2] = camera->imageGetBlue(camera->getImage(), width, width / 2, height-1-vertOffset);
    return array;
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
    ResetMotors();

    //Local variables
    int width = camera->getWidth();
    int height = camera->getWidth();

    //Robot AI goes here
    
    DriveForward(-30);
    int turnTime = 0;
    while (robot->step(TIME_STEP) != -1) {
        //Gets the RGB of the bottom middle pixel on the camera
        int* rgb = GetRGB(width, height, 10);
        //Print any test messages here
        std::cout << "Red: " << rgb[0] << ", Green: " << rgb[1] << ", Blue: " << rgb[2] << std::endl;
        turnTime++;
        if (rgb[2] < 100) {
            delete[] rgb;
            ResetMotors();
            break;
        } else {
            delete[] rgb;
        }
    }
    robot->step(TIME_STEP * 5);
    
    DriveForward(30);
    while (robot->step(TIME_STEP) != -1 && turnTime > 0) {
        turnTime--;
    }
    ResetMotors();
    robot->step(TIME_STEP * 5);
    
    DriveForward(30);
    while (robot->step(TIME_STEP) != -1) {
        //Gets the RGB of the bottom middle pixel on the camera
        int* rgb = GetRGB(width, height, 10);
        //Print any test messages here
        std::cout << "Red: " << rgb[0] << ", Green: " << rgb[1] << ", Blue: " << rgb[2] << std::endl;
        turnTime++;
        if (rgb[2] < 100) {
            delete[] rgb;
            break;
        } else {
            delete[] rgb;
        }
    }
    robot->step(TIME_STEP * 2);
    ResetMotors();
    robot->step(TIME_STEP * 5);
    
    DriveForward(0);
    robot->step(TIME_STEP * 50);
    

    //End of AI
    ResetMotors();
    delete robot;
    return 0;  // EXIT_SUCCESS
}