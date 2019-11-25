/*  

 */

int photodiode = 2;

int LED = 8;
int senRead = A0;
int limit = 850;

void setup()
{
    // opens serial port, sets data rate to 9600 bps
    Serial.begin(9600);

    pinMode(photodiode, OUTPUT);
    pinMode(LED, OUTPUT);

    digitalWrite(photodiode, HIGH);
    digitalWrite(LED, LOW);
}

void loop()
{
    int val = analogRead(senRead);

    Serial.println(val);

    if (val <= limit)
    {
        digitalWrite(LED, HIGH);
        delay(200);
    }
}