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
#include <LedControl.h>

RF24 radio(9, 10); // CE, CSN Pins

// #define XJOY_PIN A1 //X axis reading from joystick will go into analog pin A1

const byte addresses[][6] = {"00001", "00002"}; // read 00001, write 00002 for transmitter

const int servoPin = 6;
const int ledPin = 8;
const int statusledPin = 3;

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

int feedbackArray [3]; //data generated from receiver
/*
  [0] -> ODS (obstacle detection System status)
  [1:2] -> extra array space for future development
*/

int feedBack;

const int DIN = 7;
const int CS = 6;
const int CLK = 5;
LedControl ledMatrix = LedControl(DIN, CLK, CS, 0);
byte A[8] = {0x00, 0x18, 0x24, 0x42, 0x7e, 0x42, 0x42, 0x42};
byte M[8] = {0x00, 0x42, 0x66, 0x5A, 0x42, 0x42, 0x42, 0x42};
byte smileyface[8] = {0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C};

Servo servoSteering;

void setup()
{
  Serial.begin(9600);
//  pinMode(LED_BUILTIN, OUTPUT); // built-in led for operating indication

  servoSteering.attach(servoPin);
  servoSteering.write(0);

  pinMode(ledPin, OUTPUT);
  pinMode(switchPin, INPUT);
  digitalWrite(switchPin, HIGH);

  pinMode(statusledPin, OUTPUT);
  digitalWrite(statusledPin, LOW);

  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_MIN); // power saving signal strength
  radio.setDataRate(RF24_250KBPS);//set data rate to 250kbps
  radio.setChannel(77);//set frequency to channel 77
  //    radio.stopListening();

  ledMatrix.shutdown(0, false);
  ledMatrix.setIntensity(0, 15);
  ledMatrix.clearDisplay(0);

  //  ledMatrix.clearDisplay(0);
  printByte(smileyface);
  delay(3000);
  printByte(M);
  
}

void loop()
{
  digitalWrite(statusledPin, LOW);
  delay(150); // sweet spot time for the joystick switch
  digitalWrite(statusledPin, HIGH);
  
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

    ledMatrix.clearDisplay(0);
    printByte(A);
  }
  else if (switchState == 0 && swLogic == true)
  {
    swLogic = false;
    digitalWrite(ledPin, LOW);
    dataArray[2] = 0;

    ledMatrix.clearDisplay(0);
    printByte(M);
  }

  Serial.println("-------------");
  Serial.print("xAxis: ");
  Serial.println(joystickXVal);
  Serial.print("yAxis: ");
  Serial.println(joystickYVal);

  radio.write(&dataArray, sizeof(dataArray));
  
  
  //  delay(20);
  //  radio.startListening();
  //  if (radio.available())
  //  {
  //    radio.read(&feedBack,sizeof(feedBack));
  ////    radio.read(&feedbackArray, sizeof(feedbackArray));
  //  }
  //  radio.stopListening();


  //  Serial.print("ODS val: ");
  //  Serial.println(feedBack);

  //  const char text[] = "Hello World!";
  //  radio.write(&text, sizeof(text));
  //  delay(1000);
}

/*
  LED Matrix red 8x8 64 Led MAX7219 print function
*/
void printByte(byte character [])
{
  int i = 0;
  for (i = 0; i < 8; i++)
  {
    ledMatrix.setRow(0, i, character[i]);
  }
}
