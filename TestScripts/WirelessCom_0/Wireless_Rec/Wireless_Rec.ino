#include <SPI.h>

#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

#include <Servo.h>

RF24 radio(9, 10); // CE, CSN

const byte address[6] = "00001";

// #define XJOY_PIN A1       //X axis reading from joystick will go into analog pin A1
Servo myservo;

void setup()
{
  Serial.begin(9600);


  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop()
{

  if (radio.available())
  {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }

}
