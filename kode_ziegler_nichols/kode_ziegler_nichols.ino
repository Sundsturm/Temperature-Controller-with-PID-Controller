#include <Arduino.h>

// Pin untuk sensor LM35
#define PIN_LM35 A0

// Pin PWM output
#define PIN_PWM 9

// Variabel untuk duty cycle
const int dutyCycle = 50; // Duty cycle tetap pada 50%
float temperature = 0.0;
float previousTemperature = 0.0;
unsigned long lastTime = 0;
unsigned long currentTime = 0;
unsigned long startTime = 0;

// Variabel untuk logging data
bool logData = false;

void setup() {
  Serial.begin(9600); // Inisialisasi komunikasi Serial dengan baud rate 9600

  pinMode(PIN_LM35, INPUT); // Set pin LM35 sebagai input
  pinMode(PIN_PWM, OUTPUT); // Set pin PWM sebagai output

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
  // Catat waktu saat ini
  currentTime = millis();

  // Baca nilai ADC dari LM35
  int adcValue = analogRead(PIN_LM35);

  // Konversi nilai ADC menjadi suhu dalam derajat Celcius
  previousTemperature = temperature;
  temperature = adcValue * 4.887585533 / 10;

  // Tampilkan suhu pada Serial Monitor
  //Serial.print("Time (ms): ");
  //Serial.print(currentTime - startTime);
  //Serial.print(", Duty Cycle: ");
  //Serial.print(dutyCycle); // Menampilkan duty cycle dalam persen
  Serial.print("%, Temperature: ");
  Serial.println(temperature, 2); // Tampilkan dengan 2 desimal

  // Serial Plotter Output
  if (logData) {
    Serial.print(temperature, 2); // Suhu dengan 2 desimal (y-axis)
    Serial.print(",");
    Serial.println(currentTime - startTime); // Waktu dalam ms (x-axis)
  }

  delay(1000); // Delay 1 detik sebelum pembacaan berikutnya
}