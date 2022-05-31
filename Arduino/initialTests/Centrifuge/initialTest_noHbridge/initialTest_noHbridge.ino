
int potPin = A0;
int ledPin = 9;
int val;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  val = analogRead(potPin); //(0-1023)
  PWMval
  analogWrite(ledPin, val / 4); //(0-255)
}
