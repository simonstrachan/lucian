//LAMP values
const float targetTemp = 65;
const float minTemp = 63;
const float maxTemp = 67;
const float timeMin = 10;

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
float timeActual = timeMin * 60000;
uint8_t i,count;
int set;
float adcvalue, extraAdcValue, thermoValue, extraThermoValue, steinhart, extraSteinhart, fiveMoreDeg;

void setup(void) {
  Serial.begin(9600);
  analogReference(EXTERNAL);          //3V3 AREF

  pinMode(fetPin, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
}

void loop(void) {
  //Taking samples
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

  if(count <= 1){
    fiveMoreDeg = steinhart + 45;
    count++;
  }
  if (steinhart <= fiveMoreDeg) {
    set = 255;
  } else {
    if (steinhart >= fiveMoreDeg) {
      set = 0;
    }
  }
  /*
  if (steinhart <= targetTemp) {
    set = 255;
  } else {
    if (steinhart >= targetTemp) {
      set = 0;
    }
  }*/
  
  analogWrite(fetPin, set);
  //Serial.print("PWM:");
  Serial.print(steinhart);
  Serial.print(", ");
  Serial.print(extraSteinhart);
  Serial.print(", ");
  //Serial.println(set);

  Serial.print(set);
  Serial.print(", ");
  Serial.println(fiveMoreDeg);

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
  delay(500);
  //delay(1000);
}

/*                      Links
   Source: https://learn.adafruit.com/thermistor/overview
   Steinhart-Hart: https://en.wikipedia.org/wiki/Thermistor
*/
