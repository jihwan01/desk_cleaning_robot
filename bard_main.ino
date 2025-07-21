#include <Servo.h>
#include <Goldelox_Serial_4DLib.h>
#include <Goldelox_Const4D.h>
#include "HX711.h"

// global flag
bool binCondDisplayed = false;

#define DEBUG true              // print debug msg or not

#define DELAY 700               // common delay

#define INFDELAY 1000           // interference delay (to prevent interference betwen sonar sensor)

/* =============== Servo =============== */

// Servo pin configuration
#define ROT_PIN 8
#define BASE_PIN 7

// Constant values for servo
#define BASE_MIN_ANGLE 0
#define BASE_MAX_ANGLE 180
#define BASE_STEP 30

#define ROT_INIT_ANGLE 99
#define ROT_STEP 48
#define BASE_INIT_ANGLE 0

// define servo variables
Servo baseServo, rotServo;

int curBaseAngle = BASE_INIT_ANGLE;
int curRotAngle = ROT_INIT_ANGLE;

/* =============== Sonar =============== */

// Sonar pin configuration
#define TRIG_PIN1 13
#define ECHO_PIN1 12
#define TRIG_PIN2 11
#define ECHO_PIN2 10

#define NUM_DIST 360 / BASE_STEP        // # of distance that we measure
#define CLIFF_THRESHOLD 70              // threshold distance for cliff detection
#define OBSTACLE_THREADHOLD 15          // threshold distance for obstacle detection

float dist[NUM_DIST];                   // 0~6 : left angle, 7~13 : right angle

// measured in nodding phase
float frontDist;
float backDist;

int movingStatus = 1;                   // 1 : can go forward 2 : can't go forward, can go backward, 3 : can't go forward and backward

volatile bool IsEdgeDetected = false;   // flag for edge deteced

/* =============== DC Motor & Motor Drive =============== */

// Motor Pin configuration
#define LEFT_SPEED_PIN 6
#define LEFT_DIR_PIN1 22
#define LEFT_DIR_PIN2 23

#define RIGHT_SPEED_PIN 9
#define RIGHT_DIR_PIN1 24
#define RIGHT_DIR_PIN2 25

#define STBY_PIN 29                     // LOW : OFF Motor Driver, HIGH : ON Motor Driver

// constant for motor
#define MOTOR_SPEED 250
#define ROTATION_TIME_PER_DEGREE 10     // motor operate on 10 * degree [ms]

#define MOVING_TIME_F 750               // motor operating time to move forward
#define MOVING_TIME_B 1250              // motor operating time to move backward


/* =============== uLCD =============== */
#define DisplaySerial Serial1           // uLCD
#define LCD_RESET_PIN 4                 // uLCD pin

Goldelox_Serial_4DLib Display(&DisplaySerial);

/* =============== Load Cell & HX711(Amp) =============== */
#define DOUT 3                          // HX711 data output pin
#define CLK 5                           // HX711 clock pin
#define FULL_THRESHOLD 5                // bin full threshold

const float CALIBRATION_FACTOR = 5000;
HX711 loadCellAmp;

/* =============== Vacuum =============== */
#define VACUUM_POL_PIN 2                // Polling pin for motor
volatile bool isVacuumStopped = false;  // flag for motor off

void setup()
{
    Serial.begin(9600); // Set Serial Rate
    
    initAll();
}

/*
    Initialize all peripherals
*/
void initAll()
{
    initServo();
    initSonar();
    initLoadCell();
    initLCD();
    initMotor();
    initVC();
}

/*
    Initialize Servo
*/
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


/*
    Initialize Sonar Sensor
*/
void initSonar()
{
    // Set PinMode Correctly
    pinMode(TRIG_PIN1, OUTPUT);
    pinMode(ECHO_PIN1, INPUT);
    pinMode(TRIG_PIN2, OUTPUT);
    pinMode(ECHO_PIN2, INPUT);
}

/*
    Initialize Load Cell & HX711
*/
void initLoadCell()
{
    // Set Pin
    loadCellAmp.begin(DOUT, CLK);
    
    // Set scale factor
    loadCellAmp.set_scale(CALIBRATION_FACTOR);

    // Reset the load cell reading to zero (tare calibration)
    loadCellAmp.tare();
}

/*
    Initialize uLCD
*/
void initLCD()
{
    // reset LCD
    pinMode(LCD_RESET_PIN, OUTPUT);
    
    digitalWrite(LCD_RESET_PIN, LOW);
    delay(100);
    digitalWrite(LCD_RESET_PIN, HIGH);

    // initial setting
    DisplaySerial.begin(9600);
    delay(5000);
    Display.gfx_Cls();

    // draw smile
    SmileFace();
}

/*
    Initialize Motor
*/
void initMotor()
{
    // Set PinMode Correctly
    pinMode(LEFT_SPEED_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN1, OUTPUT);
    pinMode(LEFT_DIR_PIN2, OUTPUT);

    pinMode(RIGHT_SPEED_PIN, OUTPUT);
    pinMode(RIGHT_DIR_PIN1, OUTPUT);
    pinMode(RIGHT_DIR_PIN2, OUTPUT);

    pinMode(STBY_PIN, OUTPUT);

    // Set initial value of each pin
    digitalWrite(STBY_PIN, HIGH);

    analogWrite(LEFT_SPEED_PIN, 0);
    analogWrite(RIGHT_SPEED_PIN, 0);

    digitalWrite(LEFT_DIR_PIN1, LOW);
    digitalWrite(LEFT_DIR_PIN2, LOW);

    digitalWrite(RIGHT_DIR_PIN1, LOW);
    digitalWrite(RIGHT_DIR_PIN2, LOW);
}

/*
    Initialize Vacuum
*/
void initVC()
{
    // Set pinMode
    pinMode(VACUUM_POL_PIN, INPUT);
}

// For Easy Delay
void commonDelay()
{
    delay(DELAY);
}

/*
    Measure the distance by using the specified sonar sensor
    input : type of sonar
    output : measured distance

*/
float getDist(int type)
{
    long duration;
    float distance;
    if (type == 1)
    {
        digitalWrite(TRIG_PIN1, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN1, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN1, LOW);
        duration = pulseIn(ECHO_PIN1, HIGH); // TimeOut 30ms, MAX dist 5m
        distance = duration * 0.0343 / 2.0;  // Get Distance (cm)
    }
    else if (type == 2)
    {
        digitalWrite(TRIG_PIN2, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN2, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN2, LOW);
        duration = pulseIn(ECHO_PIN2, HIGH); // TimeOut 30ms, MAX dist 5m
        distance = duration * 0.0343 / 2.0;  // Get Distance (cm)
    }
    return distance;
}

/*
    Nodding the bard's head
    While nodding its head, it measures the front and back distance to detect edge (cliff and obstacle)
*/
void nodding()
{
    // To ensure stability, it moves in three distinct steps
    rotServo.write(ROT_INIT_ANGLE);
    commonDelay();
    rotServo.write(ROT_INIT_ANGLE - ROT_STEP / 3);
    commonDelay();
    rotServo.write(ROT_INIT_ANGLE - 2 * ROT_STEP / 3);
    commonDelay();
    rotServo.write(ROT_INIT_ANGLE - ROT_STEP);
    commonDelay();
    commonDelay();

    // get front distance
    frontDist = getDist(1);
    Serial.print("Front Dist : ");
    Serial.println(frontDist);

    // To ensure stability, it moves in three distinct steps
    commonDelay();
    rotServo.write(ROT_INIT_ANGLE - 2 * ROT_STEP / 3);
    commonDelay();
    rotServo.write(ROT_INIT_ANGLE - ROT_STEP / 3);
    commonDelay();
    rotServo.write(ROT_INIT_ANGLE);
    commonDelay();

    // To ensure stability, it moves in three distinct steps
    rotServo.write(ROT_INIT_ANGLE + ROT_STEP / 3);
    commonDelay();
    rotServo.write(ROT_INIT_ANGLE + 2 * ROT_STEP / 3);
    commonDelay();
    rotServo.write(ROT_INIT_ANGLE + ROT_STEP);
    commonDelay();
    commonDelay();

    // get back distance
    backDist = getDist(2);
    Serial.print("Back Dist : ");
    Serial.println(backDist);

    // To ensure stability, it moves in three distinct steps
    rotServo.write(ROT_INIT_ANGLE + 2 * ROT_STEP / 3);
    commonDelay();
    rotServo.write(ROT_INIT_ANGLE + ROT_STEP / 3);
    commonDelay();
    rotServo.write(ROT_INIT_ANGLE);
    commonDelay();
    
    // update movingStatus according to the measured distances
    if (OBSTACLE_THREADHOLD < frontDist && frontDist < CLIFF_THRESHOLD && backDist < CLIFF_THRESHOLD)
    {
        movingStatus = 1;   // can go forward
    }
    else if ((OBSTACLE_THREADHOLD > frontDist || frontDist >= CLIFF_THRESHOLD) && (backDist < CLIFF_THRESHOLD && backDist > OBSTACLE_THREADHOLD))
    {
        movingStatus = 2;   // can't go forward, can go backward
    }
    else
    {
        movingStatus = 3;   // can't go forward and backward
    }

    if (DEBUG)
    {
        Serial.print("Moving Status : ");
        Serial.println(movingStatus);
    }
}

/*
    Given the distances for each angle, return the angle that has the median distance.
*/
int findMedianIndex()
{
    // copy distance array
    float copyArr[NUM_DIST];
    for (int i = 0; i < NUM_DIST; i++)
    {
        copyArr[i] = dist[i];
    }

    // bubble sort
    for (int i = 0; i < NUM_DIST - 1; i++)
    {
        for (int j = 0; j < NUM_DIST - i - 1; j++)
        {
            if (copyArr[j] > copyArr[j + 1])
            {
                float temp = copyArr[j];
                copyArr[j] = copyArr[j + 1];
                copyArr[j + 1] = temp;
            }
        }
    }
    float medianValue = copyArr[NUM_DIST / 2];

    // get the angle that has median distance
    for (int i = 0; i < NUM_DIST; i++)
    {
        if (dist[i] == medianValue)
        {
            return i;
        }
    }

    return -1;
}

/*
    Get Around Distance Information
    Measure the distance from BASE_MIN_ANGLE to BASE_MAX_ANGLE for each BASE_STEP angle.
*/
void getAroundDist()
{
    for (curBaseAngle = BASE_MIN_ANGLE; curBaseAngle < BASE_MAX_ANGLE; curBaseAngle += BASE_STEP)
    {
        // move base servo motor
        Serial.println(curBaseAngle);
        int idx = curBaseAngle / BASE_STEP;
        baseServo.write(curBaseAngle);
        commonDelay();

        dist[idx] = getDist(1);
        
        // print value
        Serial.print("Angle ");
        Serial.print(curBaseAngle);
        Serial.print(": ");
        Serial.println(dist[idx]);

        delay(INFDELAY);                            // delay to prevent interference issue

        dist[idx + (NUM_DIST / 2)] = getDist(2);    // type 2 sonar sensor measures the distance in the opposite direction.

        // print value
        Serial.print("Angle ");
        Serial.print(curBaseAngle + 180);
        Serial.print(": ");
        Serial.println(dist[idx + (NUM_DIST / 2)]);
        
        commonDelay();
    }

    // Return to base angle
    baseServo.write(BASE_INIT_ANGLE);
    commonDelay();
}

/*
    Stop motor
*/
void stopMotor()
{
    analogWrite(LEFT_SPEED_PIN, 0);
    analogWrite(RIGHT_SPEED_PIN, 0);
    digitalWrite(LEFT_DIR_PIN1, LOW);
    digitalWrite(LEFT_DIR_PIN2, LOW);
    digitalWrite(RIGHT_DIR_PIN1, LOW);
    digitalWrite(RIGHT_DIR_PIN2, LOW);
}

/*
    Rotate the robot according to the angleIdx
    Actual Rotation Angle = angleIdx * BASE_STEP
*/
void rotation(int angleIdx)
{
    int desiredAngle = angleIdx * BASE_STEP;
    if (desiredAngle > 180)
    {
        desiredAngle -= 360;
    }
    
    int rotationTime = abs(desiredAngle) * ROTATION_TIME_PER_DEGREE;
    
    if (DEBUG)
    {
        Serial.print("Rotating by ");
        Serial.print(desiredAngle);
        Serial.print(" degrees (time: ");
        Serial.print(rotationTime);
        Serial.println(" ms)");
    }

    if (desiredAngle > 0)
    {
        // clockwise, left forward / right backward
        digitalWrite(LEFT_DIR_PIN1, HIGH);
        digitalWrite(LEFT_DIR_PIN2, LOW);
        analogWrite(LEFT_SPEED_PIN, MOTOR_SPEED);

        digitalWrite(RIGHT_DIR_PIN1, LOW);
        digitalWrite(RIGHT_DIR_PIN2, HIGH);
        analogWrite(RIGHT_SPEED_PIN, MOTOR_SPEED);
    }
    else if (desiredAngle < 0)
    {
        // counter-clockwise, left backward / right forward
        digitalWrite(LEFT_DIR_PIN1, LOW);
        digitalWrite(LEFT_DIR_PIN2, HIGH);
        analogWrite(LEFT_SPEED_PIN, MOTOR_SPEED);

        digitalWrite(RIGHT_DIR_PIN1, HIGH);
        digitalWrite(RIGHT_DIR_PIN2, LOW);
        analogWrite(RIGHT_SPEED_PIN, MOTOR_SPEED);
    }

    // rotate!
    delay(rotationTime);

    stopMotor();
    commonDelay();
}

/*
    Move forward for time (ms)
*/
void moveForward(int time)
{
    // moving forward
    digitalWrite(LEFT_DIR_PIN1, HIGH);
    digitalWrite(LEFT_DIR_PIN2, LOW);
    analogWrite(LEFT_SPEED_PIN, MOTOR_SPEED);

    digitalWrite(RIGHT_DIR_PIN1, HIGH);
    digitalWrite(RIGHT_DIR_PIN2, LOW);
    analogWrite(RIGHT_SPEED_PIN, MOTOR_SPEED);

    delay(time);

    stopMotor();
    commonDelay();
}

/*
    Move backward for time (ms)
*/
void moveBackward(int time)
{

    // moving backward
    digitalWrite(LEFT_DIR_PIN1, LOW);
    digitalWrite(LEFT_DIR_PIN2, HIGH);
    analogWrite(LEFT_SPEED_PIN, MOTOR_SPEED);

    digitalWrite(RIGHT_DIR_PIN1, LOW);
    digitalWrite(RIGHT_DIR_PIN2, HIGH);
    analogWrite(RIGHT_SPEED_PIN, MOTOR_SPEED);

    delay(time);

    stopMotor();
    commonDelay();
}

/*
    Display smile face
*/
void SmileFace()
{
    Serial.println("Smile :)");
    Display.gfx_Cls(); // clears (background black by default)

    // Face shape (circle)
    Display.gfx_Circle(64, 64, 30, YELLOW);

    // Eyes
    Display.gfx_CircleFilled(54, 58, 4, YELLOW);
    Display.gfx_CircleFilled(74, 58, 4, YELLOW);

    int x_start = 54;
    int x_end = 74;
    int y = 75;

    for (int i = 0; i < 6; i++)
    { // creates 6-line thick smile
        Display.gfx_Line(x_start + i, y + i, x_end - i, y + i, YELLOW);
    }
}

/*
    Show edge detected sentence on uLCD
*/
void DrawEdgeDetected()
{
    Display.gfx_Cls();
    Display.gfx_MoveTo(20, 90);
    Serial.println("Draw Edge Detected");
    Display.putstr("Edge Detected!");
}

/*
    Show bin status
*/
void showBinIsFull(bool isFull)
{
    if (isFull)
    {
        Serial.print("Bin is Full!");
        char sen[30];
        sprintf(sen, "Bin : Full");
        Display.gfx_Cls();
        Display.gfx_MoveTo(15, 90);
        Display.putstr(sen);
    }
    else
    {
        Serial.print("Bin is not full!");
        char sen[35];
        sprintf(sen, "Bin : clean");
        Display.gfx_Cls();
        Display.gfx_MoveTo(15, 90);
        Display.putstr(sen);
    }
}

void loop()
{
    if (digitalRead(VACUUM_POL_PIN))                // Check show bin condition signal
    {
        if (!binCondDisplayed)                       // Display bin condition only once
        {
            binCondDisplayed = true;
            Serial.println("Show Weight");
            delay(2000);                            // Wait for 2s to wait until the bard be stable
            float weight = 0;

            // get 5 values to get the average (stability)
            for (int i = 0; i < 5; i++)
            {
                weight += loadCellAmp.get_units();
                delay(50);
            }

            if (DEBUG)
            {
                // Testing purpose
                Serial.print("Motor stopped. Weight reading: ");
                Serial.print(weight / 5, 2);
                Serial.println(" g");
            }

            // display bin condition
            bool isFull = (weight / 5) > FULL_THRESHOLD;
            showBinIsFull(isFull);
        }
    }
    else                                            // moving phase
    {
        SmileFace();                                // draw smile :)
        
        binCondDisplayed = false;                   // clear bin condition displayed signal

        getAroundDist();                            // 1. Get around distance

        rotation(findMedianIndex());                // 2. Get Median index and rotate

        commonDelay();

        movingStatus = 1;
        nodding();                                  // 3. Check whether the robot can go forward or not (moving status is updated in nodding)
        if (movingStatus == 1)                      // 4. Move properly according to the moving status
        {
            // Go forward
            Serial.println("Go Forward----------");
            moveForward(MOVING_TIME_F);
        }
        else
        {
            // Edge detected
            Serial.println("Cliff Detected------");
            DrawEdgeDetected();
            if (movingStatus == 2)
            {
                // Go backward
                Serial.println("Go Backward---------");
                moveBackward(MOVING_TIME_B);
            }
            else if (movingStatus == 3)
            {
                // Rotate 90 degree
                Serial.println("Rotate to 90 degree-");
                rotation(NUM_DIST / 4);
            }
        }
        commonDelay();
    }
}