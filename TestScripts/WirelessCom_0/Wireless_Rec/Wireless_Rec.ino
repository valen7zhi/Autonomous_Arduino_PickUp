/*
  BN: Arduino/Genuino Uno
  VID: 0x2341
  PID: 0x0043
  SN: 85735313832351812091
  Autonomous Arduino Pick-Up Truck on-vehicle ECU
  @Author: Zhibo Wang
  ISE511/AME504 Mechatronic Systems Engineering '19
  Super Mario Theme Song by: Dipto Pratyaksa
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <Servo.h>
#include <Ultrasonic.h>
#include <LedControl.h>


RF24 radio(9, 10); // CE, CSN Pins

const byte addresses[][6] = {"00001", "00002"}; // read 00002, write 00001 for transmitter

const int servoPin = 2;
const int ledPin = 8;

int dataArray [5]; //data sent from transmitter
/*
  [0] -> xjoy
  [1] -> yjoy
  [2] -> joyswitch
  [3:4] -> extra array space for future development
*/
int feedbackArray [3]; //data generated from receiver
/*
  [0] -> ODS (obstacle detection System status)
  [1:2] -> extra array space for future development
*/
int feedBack;
//bool swLogic = false;

const int enablePin = 3; // L293D PWM pin
const int in1Pin = 4; // L293D logic pin 1
const int in2Pin = 5; // L293D logic pin 2

Servo servoSteering;

Ultrasonic urLeftCorner (A4, A5); // Ultrasonic Radar left corner A0 Trig, A1 Echo
Ultrasonic urRightCorner (A0, A1); // Ultrasonic Radar right corner A2 Trig, A3 Echo

const int irRightCorner = 7; // IR reads 1 when nothing is detected, 0 when object is ahead
//const int irLeftCorner = 6;

const int buzzer = 6;

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
//Mario main them
int melody[] = {
  NOTE_E7, NOTE_E7, 0, NOTE_E7,
  0, NOTE_C7, NOTE_E7, 0,
  NOTE_G7, 0, 0,  0,
  NOTE_G6, 0, 0, 0,

  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0,

  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0
};
//Mario main them tempo
int tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};

void setup()
{
  Serial.begin(9600);

  servoSteering.attach(servoPin);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(irRightCorner, INPUT);

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.setPALevel(RF24_PA_MIN); // power saving signal strength
  radio.setDataRate(RF24_250KBPS);//set data rate to 250kbps
  radio.setChannel(77);//set frequency to channel 77

  pinMode(buzzer, OUTPUT);

}

void loop()
{

  //  if (radio.available())
  //  {
  //    char text[32] = "";
  //    radio.read(&text, sizeof(text));
  //    Serial.println(text);
  //  }

  delay(20);
  radio.startListening();

  //  if (radio.available())
  //  {
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

      manualMode(steeringAngle, motorSpeed);

      //        obstacleDection();
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

  //  }


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
  int minDist = 35; // min distance between user and the car
  int maxDist = 150; // can't track user beyond this distance

  int leftDegree = 180;
  int rightDegree = 0;

  int distLeftCorner = urLeftCorner.Ranging(CM);
  int distRightCorner = urRightCorner.Ranging(CM);

  /*
    go straight - Left and Right within range & larger than threshold
    left turn - Left within range & larger than threshold, Right beyond range
    right turn - Right within range & larger than threshold, Left beyond range
    stall - Left & Right beyond range
  */

  if (distLeftCorner >= minDist && distLeftCorner < maxDist && distRightCorner >= minDist && distRightCorner < maxDist ) // go straight
  {
    while (distLeftCorner >= 35 && distLeftCorner < 100 && distRightCorner >= 35 && distRightCorner < 100)
    {
      //      Serial.print("Object found at: ");
      //      Serial.print(distLeftCorner);
      //      Serial.println("cm");
      Serial.println("---Autonomous User Following (straight) Activated---");
      // keeps follwoing until distance gets to threhold value

      driveMotor(255);
      //      delay(1000);
      distLeftCorner = urLeftCorner.Ranging(CM);
      distRightCorner = urRightCorner.Ranging(CM);
    }
  }
  else if (distLeftCorner >= minDist && distLeftCorner < maxDist && distRightCorner > maxDist ) // left turn
  {
    while (distLeftCorner >= minDist && distLeftCorner < maxDist && distRightCorner > maxDist )
    {
      Serial.println("---Autonomous User Following (left) Activated---");
      steering(leftDegree);
      driveMotor(255);
      distLeftCorner = urLeftCorner.Ranging(CM);
      distRightCorner = urRightCorner.Ranging(CM);
    }

  }
  else if (distLeftCorner > maxDist && distRightCorner >= minDist && distRightCorner < maxDist ) // right turn
  {
    while (distLeftCorner > maxDist && distRightCorner >= minDist && distRightCorner < maxDist  )
    {
      Serial.println("---Autonomous User Following (right) Activated---");
      steering(rightDegree);
      driveMotor(255);
      distLeftCorner = urLeftCorner.Ranging(CM);
      distRightCorner = urRightCorner.Ranging(CM);
    }

  }
  else
  {
    Serial.print("Object found at: (left)");
    Serial.print(distLeftCorner);
    Serial.print(" --- (right)");
    Serial.print(distRightCorner);
    Serial.println();
    driveMotor(0);
    steering(90);
  }

}

/*
  manual mode function
  IR reads 1 when nothing is detected, 0 when object is ahead
*/

void manualMode(int steeringAngle, int motorSpeed)
{
  int irRightRead  = digitalRead(irRightCorner);
  //  int irLeftRead = digitalRead(irLeftCorner);
  Serial.print("IR Status: (left)");
  //  Serial.print(irLeftRead);
  Serial.print("--- (right)");
  Serial.println(irRightRead);

  if (irRightRead == 0)
  {
    steering(90);
    driveMotor(0);
    //    feedbackArray[0] = 1;
    singMario();

    //    tone(buzzer, 1000); // Send 1KHz sound signal...
    //    delay(1000);        // ...for 1 sec
    //    noTone(buzzer);     // Stop sound...

  }
  else
  {
    steering(steeringAngle);
    driveMotor(motorSpeed);
    //    feedbackArray[0] = 0;
    //    delay(5);
    //    radio.stopListening();
    //    radio.write(&feedbackArray, sizeof(feedbackArray));
  }

}

/*
  Super Mario Theme functions
  singMario()
  buzz()
*/

void singMario() {
  Serial.println(" 'Mario Theme'");

  int size = sizeof(melody) / sizeof(int);
  for (int thisNote = 0; thisNote < size; thisNote++)
  {
    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / tempo[thisNote];

    buzz(buzzer, melody[thisNote], noteDuration);
    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;

    delay(pauseBetweenNotes);
    // stop the tone playing:
    buzz(buzzer, 0, noteDuration);
  }
}

void buzz(int targetPin, long frequency, long length) {
  long delayValue = 1000000 / frequency / 2; // calculate the delay value between transitions
  //// 1 second's worth of microseconds, divided by the frequency, then split in half since
  //// there are two phases to each cycle
  long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing
  //// multiply frequency, which is really cycles per second, by the number of seconds to
  //// get the total number of cycles to produce
  for (long i = 0; i < numCycles; i++) { // for the calculated length of time...
    digitalWrite(targetPin, HIGH); // write the buzzer pin high to push out the diaphram
    delayMicroseconds(delayValue); // wait for the calculated delay value
    digitalWrite(targetPin, LOW); // write the buzzer pin low to pull back the diaphram
    delayMicroseconds(delayValue); // wait again or the calculated delay value
  }
}
