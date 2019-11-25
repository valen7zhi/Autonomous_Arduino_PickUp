#include <Servo.h>

int servoPin = 6;
int xjoyPin = A1;
int switchPin = 2;
int switchState = 0;
int ledPin = 5;
bool swLogic = false;

// #define XJOY_PIN A1       //X axis reading from joystick will go into analog pin A1
Servo myservo;

void setup()
{
    Serial.begin(9600);

    myservo.attach(servoPin);
    myservo.write(0);

    pinMode(ledPin, OUTPUT);
    pinMode(switchPin, INPUT);
    digitalWrite(switchPin, HIGH);
}

void loop()
{
    delay(200);
    int joystickXVal = analogRead(xjoyPin); //read joystick input on pin A1
    joystickXVal = map(joystickXVal, 0, 1023, 0, 180);
    Serial.print(joystickXVal);               //print the value from A1
    Serial.println(" = input from joystick"); //print "=input from joystick" next to the value
    // Serial.print((joystickXVal + 520) / 10);  //print a from A1 calculated, scaled value
    Serial.print(joystickXVal);
    Serial.println(" = output to servo"); //print "=output to servo" next to the value
    Serial.println();
    // myservo.write((joystickXVal + 520) / 10); //write the calculated value to the servo
    myservo.write(joystickXVal);

    switchState = digitalRead(switchPin);
    Serial.print(switchState);
    Serial.println(" = switchState");

    if (switchState == 0 && swLogic == false)
    {
        swLogic = true;
    }
    else if (switchState == 0 && swLogic == true)
    {
        swLogic = false;
    }

    if (swLogic == true)
    {
        digitalWrite(ledPin, HIGH);
    }
    else if (swLogic == false)
    {
        digitalWrite(ledPin, LOW);
    }
}