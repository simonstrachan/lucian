#include <PID_v1.h>

//LCD initalisation
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 2, 3, 4, 5);
#define contrast 9
#define bright 10

//LAMP values
const double targetTemp = 65;
const double minTemp = 63;
const double maxTemp = 67;
const double timeMin = 45;

//Initialising Pins
#define thermoPin A0
#define extraThermo A1
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
int extrasamples[numSamples];

double timeActual = timeMin * 60000;
uint8_t i;
double set;
uint32_t elapsedTime, timeSeconds, timeMinutes;
double adcvalue, extraAdcValue, thermoValue, extraThermoValue, steinhart, extraSteinhart, currentTemp;
char timeline[20];

//PID initialisation
const double Kp = 1;
const double Ki = 1;
const double Kd = 1;
PID myPID(&steinhart, &set, &targetTemp, Kp, Ki, Kd, DIRECT);

void setup(void) {
  Serial.begin(9600);
  analogReference(EXTERNAL);          //3V3 AREF

  //LCD setup
  lcd.begin(16, 2);
  pinMode(contrast, OUTPUT);
  pinMode(bright, OUTPUT);
  digitalWrite(contrast, LOW);
  analogWrite(bright, 255);

  lcd.print("Temp:         11");
  lcd.setCursor(0, 1);
  lcd.print("Time:          1");

  myPID.SetMode(AUTOMATIC);

  pinMode(fetPin, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);

}

void loop(void) {
  steinhartCalc();

  myPID.Compute();
  analogWrite(fetPin, set);

  switchingLEDs();
  
  printTime();
  
  serialPrint();

  delay(500);
}

void steinhartCalc() {
for (i = 0; i < numSamples; i++) {
    samples[i] = analogRead(thermoPin);
    extrasamples[i] = analogRead(extraThermo);
    delay(10);
  }
  adcvalue = 0;

  //averaging samples
  for (i = 0; i < numSamples; i++) {
    adcvalue += samples[i];
    extraAdcValue += extrasamples[i];
  }
  adcvalue /= numSamples;
  extraAdcValue /= numSamples;

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

  //Extra Thermo calculation
  extraThermoValue = -(seriesResistor * extraAdcValue) / (extraAdcValue - 1023);

  extraSteinhart = extraThermoValue / thermoOhm;            // (R/Ro)
  extraSteinhart = log(extraSteinhart);                     // ln(R/Ro)
  extraSteinhart /= thermoBeta;                        // 1/B * ln(R/Ro)
  extraSteinhart += 1.0 / (thermoOhmTemp + 273.15);    // + (1/To)
  extraSteinhart = 1.0 / extraSteinhart;                    // Invert
  extraSteinhart -= 273.15;                            // convert absolute temp to C

  lcd.setCursor(5, 0);
  lcd.print(steinhart);
  lcd.print("C");
}


void switchingLEDs() {
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
}

void printTime() {
  //Print time elapsed
  elapsedTime = millis() / 1000;
  timeMinutes = elapsedTime / 60;
  timeSeconds = elapsedTime % 60;

  lcd.setCursor(5, 1);
  lcd.print(timeMinutes);
  lcd.print("m ");
  lcd.print(timeSeconds);
  lcd.print("s ");
}

void serialPrint() {
  Serial.print(steinhart);
  Serial.print(", ");
  Serial.print(extraSteinhart);
  Serial.print(", ");
  Serial.println(set);
}

/*                      Links
   Source: https://learn.adafruit.com/thermistor/overview
   Steinhart-Hart: https://en.wikipedia.org/wiki/Thermistor
   PIDLibrary: https://github.com/br3ttb/Arduino-PID-Library
*/
