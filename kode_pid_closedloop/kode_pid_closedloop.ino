// Define pins
const int tempSensorPin = A0; // Temperature sensor pin
const int pwmPin = 9;        // PWM output pin

// PID parameters
double Setpoint, Input, Output;
double Kp = 10.8694, Ki = 0.4165, Kd = 70.9120; // PID tuning parameters

// Variables for PID calculations
double integral = 0, previousError = 0;

// Variables for averaging temperature
double tempTotal = 0;
int tempCount = 0;

// Control mode (DIRECT or REVERSE)
const bool REVERSE = true;
bool PIDMode = REVERSE;

// Timing variables
unsigned long lastUpdateTime = 0; // For PID updates
unsigned long lastSerialPrintTime = 0; // For Serial updates
const unsigned long pidUpdateInterval = 50; // PID update interval (ms)
const unsigned long serialPrintInterval = 500; // Serial print interval (ms)

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Set the PWM pin as output
  pinMode(pwmPin, OUTPUT);

  // Set the desired temperature (setpoint)
  Setpoint = 60.0;

  // Initialize the output limits
  Output = 0;
}

void loop() {
  // Read the temperature from the LM35 sensor
  int adcValue = analogRead(tempSensorPin);

  // Convert ADC value to temperature in Celsius
  double temperature = adcValue * 4.887585533 / 10;

  // Accumulate temperature values
  tempTotal += temperature;
  tempCount++;

  // Ensure exactly 10 data points are collected
  if (tempCount == 10) {
    // Calculate the average temperature
    Input = tempTotal / tempCount;

    // Reset averaging variables
    tempTotal = 0;
    tempCount = 0;

    // Get the current time
    unsigned long currentTime = millis();

    // Check if it's time to update the PID (every 50 ms)
    if (currentTime - lastUpdateTime >= pidUpdateInterval) {
      // Calculate error
      double error = Setpoint - Input;

      // Proportional term
      double proportional = Kp * error;

      // Integral term
      integral += error * (pidUpdateInterval / 1000.0);
      double integralTerm = Ki * integral;

      // Derivative term
      double derivative = (error - previousError) / (pidUpdateInterval / 1000.0);
      double derivativeTerm = Kd * derivative;

      // Compute PID output
      Output = proportional + integralTerm + derivativeTerm;

      // Apply PID mode
      if (PIDMode == REVERSE) {
        Output = -Output;
      }

      // Constrain output to valid PWM range (0-255)
      Output = constrain(Output, 0, 255);

      // Anti-windup logic
      if (Output >= 255 || Output <= 0) {
        integral -= error * (pidUpdateInterval / 1000.0);
      }

      // Apply the PID output to the PWM pin
      analogWrite(pwmPin, (int)Output);

      // Save current error and time
      previousError = error;
      lastUpdateTime = currentTime;
    }

    // Check if it's time to print to the Serial Monitor (every 500 ms)
    if (currentTime - lastSerialPrintTime >= serialPrintInterval) {
      // Print values to Serial Monitor
      Serial.print(Setpoint);
      Serial.print(", ");
      Serial.print(Input);
      Serial.print(", ");
      Serial.println(Output);

      // Update the last serial print time
      lastSerialPrintTime = currentTime;
    }
  }
}