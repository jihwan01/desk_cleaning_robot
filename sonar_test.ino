// global flag
bool error = false;

#define DEBUG true // print debug msg or not

#define DELAY 500

// pin configuration for two sonar sensors
#define TRIG_PIN1 13
#define ECHO_PIN1 12
#define TRIG_PIN2 11
#define ECHO_PIN2 10

/*
    input : type of sonar
    output : measured distance
    behavior : Measure the distance by using the specified sonar sensor
*/
float getDist(int type)
{
    long duration;
    float distance;

    // use first sonar sensor
    if (type == 1)
    {
        digitalWrite(TRIG_PIN1, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN1, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN1, LOW);
        duration = pulseIn(ECHO_PIN1, HIGH, 30000); // timeout 30ms
        distance = duration * 0.0343 / 2.0;         // unit : cm
    }
    else if (type == 2) // use second sonar sensor
    {
        digitalWrite(TRIG_PIN2, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN2, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN2, LOW);
        duration = pulseIn(ECHO_PIN2, HIGH, 30000);
        distance = duration * 0.0343 / 2.0;
    }
    else
    {
        Serial.println("Error!");
        distance = -1;
    }

    return distance;
}

void setup()
{
    Serial.begin(9600);

    // Initialize Sonar Sensor 1
    pinMode(TRIG_PIN1, OUTPUT);
    pinMode(ECHO_PIN1, INPUT);

    // Initialize Sonar Sensor 2
    pinMode(TRIG_PIN2, OUTPUT);
    pinMode(ECHO_PIN2, INPUT);
}

void commonDelay()
{
    delay(DELAY);
}

void loop()
{
    float distance1 = getDist(1);
    float distance2 = getDist(2);

    Serial.print("Sensor 1 dist: ");
    Serial.print(distance1);
    Serial.println(" cm");

    Serial.print("Sensor 2 dist: ");
    Serial.print(distance2);
    Serial.println(" cm");

    delay(1000);
}