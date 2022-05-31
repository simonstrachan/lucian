//LCD initalisation
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 2, 3, 4, 5);
#define contrast 9
#define bright 10

//LAMP values
const float targetTemp = 30;
const float minTemp = 28;
const float maxTemp = 32;
const float timeMin = 45;

//Initialising Pins
#define thermoPin A0
#define fetPin 6
#define greenLED 7
#define redLED 8

//NTC Variables
#define thermoOhm 1000000  //NTC nominal resistance
#define thermoOhmTemp 25   //NTC temperature for nominal resistance
#define thermoBeta 3950    //Beta for NTC

//Other Variables
#define seriesResistor 1000000  //Value of R2
#define heatAdjust 10           //Adjustment of voltage to correct temp
#define numSamples 50
int samples[numSamples];

float timeActual = timeMin * 60000;
uint8_t i;
int set, timeSeconds;
float adcvalue, thermoValue, steinhart, currentTemp;

void setup(void) {
  Serial.begin(9600);
  analogReference(EXTERNAL);  //3V3 AREF

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
  steinhart = thermoValue / thermoOhm;          // (R/Ro)
  steinhart = log(steinhart);                   // ln(R/Ro)
  steinhart /= thermoBeta;                      // 1/B * ln(R/Ro)
  steinhart += 1.0 / (thermoOhmTemp + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;                  // Invert
  steinhart -= 273.15;                          // convert absolute temp to C

  Serial.print("Temp:");
  Serial.print(steinhart);
  Serial.print("°C ");
  lcd.setCursor(5, 0);
  lcd.print(steinhart);
  lcd.print("C");

  currentTemp = steinhart;
  if (currentTemp <= targetTemp) {
    set = 255;
  } else {
    if (currentTemp >= targetTemp) {
      set = 0;
    }
  }
  analogWrite(fetPin, set);
  Serial.print("PWM:");
  Serial.println(set);

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

  lcd.setCursor(5, 1);
  timeSeconds = millis() / 1000;
  lcd.print(timeSeconds);
  lcd.print("s");

  delay(500);
}

/*                      Links
   Source: https://learn.adafruit.com/thermistor/overview
   Steinhart-Hart: https://en.wikipedia.org/wiki/Thermistor
*/
