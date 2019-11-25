/*  

 */

int irEm = 2;
int RLED = 4;

void setup()
{
    // opens serial port, sets data rate to 9600 bps
    Serial.begin(9600);

    // pinMode(sense, INPUT);
    // pinMode(YLED, OUTPUT);
    pinMode(irEm, OUTPUT);
    digitalWrite(irEm, HIGH);

    pinMode(RLED, OUTPUT);
    digitalWrite(RLED, HIGH);
}

void loop()
{
}
