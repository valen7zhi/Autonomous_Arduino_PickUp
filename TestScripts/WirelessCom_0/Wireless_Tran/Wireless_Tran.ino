/*
  BN: Arduino/Genuino Uno
  VID: 0x2341
  PID: 0x0043
  SN: 75735323030351804122
  Autonomous Arduino Pick-Up Truck Remote Controller
  @Author: Zhibo Wang
  ISE511/AME504 Mechatronic Systems Engineering '19
*/
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>


RF24 radio(9, 10); // CE, CSN Pins

// #define XJOY_PIN A1 //X axis reading from joystick will go into analog pin A1

const byte addresses[][6] = {"00001", "00002"}; // read 00001, write 00002 for transmitter

const int servoPin = 6;
const int ledPin = 8;

const int xjoyPin = A0;
const int yjoyPin = A1;

const int switchPin = 2;
int switchState = 0;
bool swLogic = false;

int dataArray [5];
/*
  [0] -> xjoy
  [1] -> yjoy
  [2] -> joyswitch
  [3:5] -> extra array space for further dev
*/

Servo servoSteering;

void setup()
{
  Serial.begin(9600);

  servoSteering.attach(servoPin);
  servoSteering.write(0);

  pinMode(ledPin, OUTPUT);
  pinMode(switchPin, INPUT);
  digitalWrite(switchPin, HIGH);

  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_MIN); // power saving signal strength
  //    radio.stopListening();

}

void loop()
{
  delay(150); // sweet spot time for the joystick switch
  radio.stopListening();

  int joystickXVal = analogRead(xjoyPin); //read joystick xaxis
  joystickXVal = map(joystickXVal, 0, 1023, 0, 180);

  int joystickYVal = analogRead(yjoyPin); //read joystick yaxis
  joystickYVal = map(joystickYVal, 0, 1023, -255, 255);

  dataArray[0] = joystickXVal;
  dataArray[1] = joystickYVal;

  switchState = digitalRead(switchPin);

  /*
    switch state for manual/autonomous mode
  */
  if (switchState == 0 && swLogic == false)
  {
    swLogic = true;
    digitalWrite(ledPin, HIGH);
    dataArray[2] = 1;
  }
  else if (switchState == 0 && swLogic == true)
  {
    swLogic = false;
    digitalWrite(ledPin, LOW);
    dataArray[2] = 0;
  }

  Serial.println("-------------");
  Serial.print("xAxis: ");
  Serial.println(joystickXVal);
  Serial.print("yAxis: ");
  Serial.println(joystickYVal);
  Serial.println();


  radio.write(&dataArray, sizeof(dataArray));

  //  const char text[] = "Hello World!";
  //  radio.write(&text, sizeof(text));
  //  delay(1000);
}
