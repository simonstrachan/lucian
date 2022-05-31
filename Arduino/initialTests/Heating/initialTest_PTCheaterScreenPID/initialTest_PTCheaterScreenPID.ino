//LCD initialisation
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 2, 3, 4, 5);
#define contrast 9
#define bright 10

//LAMP values
const double targetTemp = 65;
const double minTemp = 63;
const double maxTemp = 68;
const double timeMin = 45;

//PID initialisation
const double kp = 2;
const double ki = 5;
const double kd = 1;

unsigned long currentTime, previousTime;
double elapsedTime, error, lastError, addError, rateError;

//Initialising Pins
#define thermoPin A0
#define fetPin 6
#define greenLED 7
#define redLED 8

//NTC Variables
#define thermoOhm 1000000             //NTC nominal resistance      
#define thermoOhmTemp 25              //NTC temperature for nominal resistance 
#define thermoBeta 3950               //Beta for NTC

//Other Variables
#define seriesResistor 1000000        //Value of R2
#define heatAdjust 10                 //Adjustment of voltage to correct temp
#define numSamples 50
int samples[numSamples];

double timeActual = timeMin * 60000;
uint8_t i;
int set, timeSeconds;
double adcvalue, thermoValue, steinhart;

void setup(void) {
  Serial.begin(9600);
  analogReference(EXTERNAL);          //3V3 AREF

  //LCD setup
  lcd.begin(16, 2);
  pinMode(contrast, OUTPUT);
  pinMode(bright, OUTPUT);
  digitalWrite(contrast, LOW);
  analogWrite(bright, 255);

  lcd.print("Temp:");
  lcd.setCursor(0, 1);
  lcd.print("Time:");

  pinMode(fetPin, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
}

void loop(void) {
  //Taking samples
  for (i = 0; i < numSamples; i++) {
    samples[i] = analogRead(thermoPin);
    delay(10);
  }
  adcvalue = 0;

  //averaging samples
  for (i = 0; i < numSamples; i++) {
    adcvalue += samples[i];
  }
  adcvalue /= numSamples;


  /*            Math
     Voltage divider: Vo=Vcc(R2/R1+R2)
     ADC value = vo*1023/Varef
     ADC value = Vcc(R2/R1+R2)*1023/Varef
     Varef=Vcc
     ADC value= (R2/R1+R2)*1023
     R2 = - (R1 * ADC value)/(ADC value - 1023)
  */
  thermoValue = -(seriesResistor * adcvalue) / (adcvalue - 1023);

  /*   Steinhart-Hart Equation
     1/T = (1/To) + 1/B * ln(R/Ro)
  */
  steinhart = thermoValue / thermoOhm;            // (R/Ro)
  steinhart = log(steinhart);                     // ln(R/Ro)
  steinhart /= thermoBeta;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (thermoOhmTemp + 273.15);    // + (1/To)
  steinhart = 1.0 / steinhart;                    // Invert
  steinhart -= 273.15;                            // convert absolute temp to C

  //Serial.print("Temp:");
  //Serial.print(steinhart);
  //Serial.print("°C ");
  lcd.setCursor(5, 0);
  lcd.print(steinhart);
  lcd.print("C");
  
  set = PID(steinhart);
  analogWrite(fetPin, set);
  
  //Serial.print("PWM:");
  //Serial.println(set);

  //Turn on red LED if out of temp zone, green if in temp zone and both if the time limit has elapsed
  if (steinhart > minTemp && steinhart < maxTemp) {
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
  } else {
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, HIGH);
  }

  if (millis() > timeActual) {
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, HIGH);
  }

  //Print time elapsed
  lcd.setCursor(5, 1);
  timeSeconds = millis() / 1000;
  lcd.print(timeSeconds);
  lcd.print("s");

  delay(500);
}

//Using PID to get a more accurate result
int PID(int currentTemp){
  currentTime = millis();
  Serial.print("currentTime:");
  Serial.println(currentTime);
  elapsedTime = (double)(currentTime - previousTime);
  Serial.print("elapsedTime:");
  Serial.println(elapsedTime);

  error = targetTemp - currentTemp;
  Serial.print("error:");
  Serial.println(error);
  addError += error * elapsedTime;                  //Integral
  //rateError = (error - lastError)/elapsedTime;      //Derivative
  Serial.print("addError:");
  Serial.println(addError);
  //PID variable = proportionalGain*errorValue + integralGain*integralOfError + derivativeGain*rateError
  double output = kp*error + ki*addError;      //PI
  Serial.print("output:");
  Serial.println(output);
  lastError = error;
  previousTime = currentTime;

  return output;
}

/*                      Links
   Source: https://learn.adafruit.com/thermistor/overview
   Steinhart-Hart: https://en.wikipedia.org/wiki/Thermistor
   PID: https://www.teachmemicro.com/arduino-pid-control-tutorial/#:~:text=values%20are%20achieved.-,Implementing%20PID%20in%20Code,value%20and%20set%20point%20value.&text=output%20%3D%20Kp%20*%20error%20%2B%20Ki%20*%20cumError,-%2B%20Kd%20*%20rateError%3B&text=Here%2C%20the%20Kp%2C%20Ki%20and%20Kd%20are%20the%20predetermined%20constants
*/
