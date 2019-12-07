/*
  BN: Arduino/Genuino Uno
  VID: 0x2341
  PID: 0x0043
  SN: 85735313832351812091
  Autonomous Arduino Pick-Up Truck on-vehicle ECU
  @Author: Zhibo Wang
  ISE511/AME504 Mechatronic Systems Engineering '19
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <Servo.h>
#include <Ultrasonic.h>


RF24 radio(9, 10); // CE, CSN Pins
Ultrasonic urLeftCorner (A0, A1); // Ultrasonic Radar left corner A0 Trig, A1 Echo
Ultrasonic urRightCorner (A2, A3); // Ultrasonic Radar right corner A2 Trig, A3 Echo

const byte addresses[][6] = {"00001", "00002"}; // read 00002, write 00001 for transmitter

const int servoPin = 2;
const int ledPin = 8;

int dataArray [5];
/*
  [0] -> xjoy
  [1] -> yjoy
  [2] -> joyswitch
  [3:5] -> extra array space for further dev
*/

//bool swLogic = false;

const int enablePin = 3; // L293D PWM pin
const int in1Pin = 4; // L293D logic pin 1
const int in2Pin = 5; // L293D logic pin 2

Servo servoSteering;

void setup()
{
  Serial.begin(9600);

  servoSteering.attach(servoPin);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.setPALevel(RF24_PA_MIN); // power saving signal strength


}

void loop()
{

  //  if (radio.available())
  //  {
  //    char text[32] = "";
  //    radio.read(&text, sizeof(text));
  //    Serial.println(text);
  //  }

  delay(10);
  radio.startListening();

  if (radio.available())
  {
    while (radio.available())
    {

      radio.read(&dataArray, sizeof(dataArray));

      int xAxis = dataArray[0];
      int yAxis = dataArray[1];
      int switchState = dataArray[2]; // 0 -> manual, 1-> autonomous

      if (switchState == 0)
      {
        digitalWrite(ledPin, LOW);
        // xaxis for steering servo to change direction
        // yaxis for drive motor moving forward and backward
        int steeringAngle = xAxis;
        int motorSpeed = yAxis;
        driveMotor(motorSpeed);
        steering(xAxis);
      }
      else if (switchState == 1)
      {
        digitalWrite(ledPin, HIGH);
        //        int distLeftCorner = urLeftCorner.Ranging(CM);
        //        int distRightCorner = urRightCorner.Ranging(CM);
        autonomousMode();
      }

      //      Serial.print("Object found at: ");
      //      Serial.print(distLeftCorner);
      //      Serial.println("cm");

    }

  }


}

/*
  Motor speed (forward & backward) control function
*/
void driveMotor(int motorSpeed)
{

  if (motorSpeed > 30 ) // forward case
  {
    analogWrite(enablePin, motorSpeed);
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  }
  else if (motorSpeed < -30) // reverse case
  {
    analogWrite(enablePin, -motorSpeed);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
  else // stop case
  {
    analogWrite(enablePin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}

/*
  Steering servo control function
*/
void steering(int steeringAngle)
{

  servoSteering.write(steeringAngle);

}

/*
  autonomous driving algorithm function
*/

void autonomousMode()
{
  //  int dLC = distLeftCorner;
  //  int dRC = distRightCorner;

  int distLeftCorner = urLeftCorner.Ranging(CM);
  int distRightCorner = urRightCorner.Ranging(CM);

  if (distLeftCorner >= 50)
  {
    while (distLeftCorner >= 50)
    {
      Serial.print("Object found at: ");
      Serial.print(distLeftCorner);
      Serial.println("cm");
      // keeps follwoing until distance gets to threhold value
      //    analogWrite(enablePin, 100);
      //    digitalWrite(in1Pin, HIGH);
      //    digitalWrite(in2Pin, LOW);
      driveMotor(255);
//      delay(1000);
      distLeftCorner = urLeftCorner.Ranging(CM);
    }
  }
  else
  {
    Serial.print("Object found at: ");
    Serial.print(distLeftCorner);
    Serial.println("cm");
    driveMotor(0);

  }




}
