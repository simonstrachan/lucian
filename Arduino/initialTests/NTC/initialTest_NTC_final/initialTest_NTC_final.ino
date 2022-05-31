//Initialising Pins
#define thermoPin A0   

//NTC Variables
#define thermoOhm 1000000             //NTC nominal resistance      
#define thermoOhmTemp 25              //NTC temperature for nominal resistance 
#define thermoBeta 3950               //Beta for NTC

//Other Variables
#define seriesResistor 1000000        //Value of R2
#define numSamples 50               
int samples[numSamples];
uint8_t i;
float adcvalue, thermoValue, steinhart;

void setup(void) {
  Serial.begin(9600);
  analogReference(EXTERNAL);          //3V3 AREF
}

void loop(void) {
  //Taking samples
  for (i=0; i< numSamples; i++) {
   samples[i] = analogRead(thermoPin);
   delay(10);
  }
  adcvalue = 0;
  
  //averaging samples
  for (i=0; i< numSamples; i++) {
     adcvalue += samples[i];
  }
  adcvalue /= numSamples;


  /*            Math
   * Voltage divider: Vo=Vcc(R2/R1+R2)
   * ADC value = vo*1023/Varef 
   * ADC value = Vcc(R2/R1+R2)*1023/Varef
   * Varef=Vcc
   * ADC value= (R2/R1+R2)*1023
   * R2 = - (R1 * ADC value)/(ADC value - 1023)
   */
  thermoValue = -(seriesResistor * adcvalue) / (adcvalue - 1023);

  /*   Steinhart-Hart Equation
   * 1/T = (1/To) + 1/B * ln(R/Ro)
   */
  steinhart = thermoValue / thermoOhm;            // (R/Ro)
  steinhart = log(steinhart);                     // ln(R/Ro)
  steinhart /= thermoBeta;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (thermoOhmTemp + 273.15);    // + (1/To)
  steinhart = 1.0 / steinhart;                    // Invert
  steinhart -= 273.15;                            // convert absolute temp to C
  
  Serial.print("Temperature: "); 
  Serial.print(steinhart);
  Serial.println("°C");
  
  delay(1000);
}

/*                      Links
 * Source: https://learn.adafruit.com/thermistor/overview
 * Steinhart-Hart: https://en.wikipedia.org/wiki/Thermistor
 */
