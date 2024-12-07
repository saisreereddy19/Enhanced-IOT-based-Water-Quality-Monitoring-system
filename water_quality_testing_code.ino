#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4         // DS18B20 data pin connected to GPIO 4 (D2)
#define TDS_SENSOR_PIN A0  // TDS sensor is connected to A0 pin
#define TURBIDITY_SENSOR_PIN 2 // Turbidity sensor connected to A0
#define VREF 3.3          // Reference voltage for ESP8266 (usually 3.3V)
#define SCOUNT 30          // Number of samples to average for TDS

int count;
int analogBuffer[SCOUNT]; // Buffer to store analog readings
int analogBufferIndex = 0;
float voltage, tdsValue=0;
const float TurbidityFactor = 1.0; // Calibration factor for turbidity


// Safety thresholds
const int TDS_THRESHOLD = 50; // TDS threshold in ppm
const float TURBIDITY_THRESHOLD = 5.0; // Turbidity threshold in NTU
const float TEMPERATURE_THRESHOLD = 30.0; // Temperature threshold in °C


// DS18B20 temperature sensor setup

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

float temperature;

void setup() {
  Serial.begin(115200);
  sensors.begin();
}


void loop() {
  count = 0;
while(1)
{
 static unsigned long analogSampleTimepoint = millis();
 
  if (millis() - analogSampleTimepoint > 1U) {  // Read sensor every 200ms
    analogSampleTimepoint = millis();
   
    // Store the analog reading from the TDS sensor in the buffer
    analogBuffer[analogBufferIndex] = analogRead(TDS_SENSOR_PIN);
    analogBufferIndex++;
   
    // When we have filled the buffer, calculate TDS
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0; // Reset index to avoid out of bounds
     
      // Calculate average voltage from the readings in the buffer
      int total = 0;
      for (int i = 0; i < SCOUNT; i++) {
        total += analogBuffer[i];
      }
      count++;
      voltage = (total / SCOUNT) * (VREF / 1024.0); // Convert to voltage (10-bit ADC)
      tdsValue = calculateTDS(voltage);  // Calculate TDS from the voltage
      Serial.print("TDS (ppm): ");
      Serial.println(tdsValue);
      
    }
  }
 if(count == 1){break;}
 }
  sensors.requestTemperatures();

  temperature = sensors.getTempCByIndex(0);

  Serial.print("Temperature (°C): ");

  Serial.println(temperature);


  // Read turbidity sensor value

  int turbidityValue = analogRead(TURBIDITY_SENSOR_PIN);

  float turbidityVoltage = turbidityValue * (VREF / 1024.0);  // Convert to voltage

  float turbidity = turbidityVoltage * TurbidityFactor;       // Apply calibration

  Serial.print("Turbidity (V): ");

  Serial.println(turbidityVoltage);

  Serial.print("Turbidity (NTU, approximate): ");

  Serial.println(turbidity);  // NTU approximation

   // Simple predictive model for water safety

  if (predictWaterSafety(tdsValue, turbidity, temperature)) {

    Serial.println("Water Status: SAFE");

  } else {

    Serial.println("Water Status: NOT SAFE");

  }


  //delay(1000);  // Delay 1 secondss

}
// Function to calculate TDS from the voltage
float calculateTDS(float voltage) {
  const float TdsFactor = 0.5;  // Calibration factor for TDS sensor
  // Using a typical formula for TDS, you can modify this if needed based on calibration
  return (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage) * TdsFactor;
}

// Function to determine water safety using thresholds (simulating ML model)

bool predictWaterSafety(float tds, float turbidity, float temp) {

  // Simple logic simulating machine learning decision

  return (tds < TDS_THRESHOLD && turbidity < TURBIDITY_THRESHOLD && temp < TEMPERATURE_THRESHOLD);

}


// Function to calculate median of sensor values for TDS

int getMedianValue(int *buffer) {

  int sorted[SCOUNT];

  for (int i = 0; i < SCOUNT; i++) {

    sorted[i] = buffer[i];

  }

  // Simple bubble sort for median calculation

  for (int i = 0; i < SCOUNT - 1; i++) {

    for (int j = i + 1; j < SCOUNT; j++) {

      if (sorted[i] > sorted[j]) {

        int temp = sorted[i];

        sorted[i] = sorted[j];

        sorted[j] = temp;

      }

    }

  }

  // Return the median value

  return sorted[SCOUNT / 2];

}