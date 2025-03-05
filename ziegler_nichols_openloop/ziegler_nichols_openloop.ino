#include <Arduino.h>

// Pin for LM35
#define PIN_LM35 A0

// Pin PWM output
#define PIN_PWM 9

// Duty cycle variables
const int dutyCycle = 50; // Stay on 50%
float temperature = 0.0;
float previousTemperature = 0.0;
unsigned long lastTime = 0;
unsigned long currentTime = 0;
unsigned long startTime = 0;

// Logging data variable
bool logData = false;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_LM35, INPUT); // LM35 as input
  pinMode(PIN_PWM, OUTPUT); // PWM pin as output

  // Tampilkan pesan awal
  Serial.println("=== Ziegler-Nichols PWM and Temperature Measurement ===");
  Serial.print("Duty cycle set to: ");
  Serial.print(dutyCycle);
  Serial.println("%");

  // Atur nilai PWM sesuai duty cycle
  analogWrite(PIN_PWM, map(dutyCycle, 0, 100, 0, 255));

  // Mulai waktu logging data
  startTime = millis();
  logData = true;
}

void loop() {
  // Current time set
  currentTime = millis();

  // Read analog value from LM35
  int adcValue = analogRead(PIN_LM35);

  // Analog value -> temperature in Celcius
  previousTemperature = temperature;
  temperature = adcValue * 4.887585533 / 10;

  // Show time, duty cycle, and temperature on serial monitor
  //Serial.print("Time (ms): ");
  //Serial.print(currentTime - startTime);
  //Serial.print(", Duty Cycle: ");
  //Serial.print(dutyCycle); // Menampilkan duty cycle dalam persen
  Serial.print("%, Temperature: ");
  Serial.println(temperature, 2); // 2 decimal points

  // Serial Plotter Output
  if (logData) {
    Serial.print(temperature, 2); //  2 decimal points
    Serial.print(",");
    Serial.println(currentTime - startTime); // Time miliseconds on X-axis
  }

  delay(1000); // One second delay
}