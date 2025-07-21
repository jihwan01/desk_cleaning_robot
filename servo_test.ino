#include <Servo.h>

#define DEBUG true // print debug msg or not

#define DELAY 500

// pin configurations which related to the servo
#define ROT_PIN 8
#define BASE_PIN 7

// constant values for servo operation
#define BASE_MIN_ANGLE 0
#define BASE_MAX_ANGLE 180

#define ROT_INIT_ANGLE 90
#define ROT_STEP 15
#define BASE_INIT_ANGLE 0
#define BASE_STEP 30

Servo baseServo, rotServo;

int curBaseAngle = BASE_INIT_ANGLE;
int curRotAngle = ROT_INIT_ANGLE;

void setup()
{
    Serial.begin(9600); // Set rate

    // Initialize Status
    initAll();
}

// Initialize All
void initAll()
{
    Serial.begin(9600);
    initServo();
}

// Initialize servo status
void initServo()
{
    // Binding pin and object
    baseServo.attach(BASE_PIN);
    rotServo.attach(ROT_PIN);

    // Initialize global variable
    curBaseAngle = BASE_INIT_ANGLE;
    curRotAngle = ROT_INIT_ANGLE;

    // Initialize Servo Location
    baseServo.write(BASE_INIT_ANGLE);
    rotServo.write(ROT_INIT_ANGLE);
}

// For Easy Delay
void commonDelay()
{
    delay(DELAY);
}

void loop()
{
    baseServo.write(BASE_INIT_ANGLE);
    rotServo.write(ROT_INIT_ANGLE);
}