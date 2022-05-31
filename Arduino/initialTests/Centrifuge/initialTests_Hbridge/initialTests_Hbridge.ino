//Initial DC motor test

int potPin = A0;
int val, PWMval;

//motor1
int ENA = 5;
int IN1 = 6;
int IN2 = 7;

/*motor2
  int ENB = 9;
  int IN3 = 10;
  int IN4 = 11;
*/
void setup() {

  Serial.begin(9600);
  
  //motor setup
  pinMode(potPin, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  /*
    pinMode(ENB,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);
  */

}

void loop() {
  //speed
  val = analogRead(potPin); 
  PWMval = map(val, 0, 1024, 0, 255);
  analogWrite(ENA, PWMval);
  Serial.println(PWMval);

  //direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

}
