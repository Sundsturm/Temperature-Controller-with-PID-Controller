// Define pins
const int tempSensorPin = A0; // Temperature sensor pin
const int pwmPin = 9;        // PWM output pin

// PID parameters
double Setpoint, Input, Output;
double Kp = 10.8694, Ki = 0.4165, Kd = 70.9120; // PID tuning parameters

// Variables for PID calculations
double integral = 0, previousError = 0;
unsigned long lastTime = 0;

// Control mode (DIRECT or REVERSE)
const bool REVERSE = true;
const bool DIRECT = false;
bool PIDMode = REVERSE;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Set the PWM pin as output
  pinMode(pwmPin, OUTPUT);

  // Set the desired temperature (setpoint)
  Setpoint = 80.0;

  // Initialize the output limits
  Output = 0;
}

void loop() {
  // Read the temperature from the LM35 sensor
  int adcValue = analogRead(tempSensorPin);

  // Convert ADC value to temperature in Celsius
  double temperature = adcValue * 4.887585533 / 10;

  // Assign temperature to Input
  Input = temperature;

  // Calculate time elapsed since last PID calculation
  unsigned long now = millis();
  double timeChange = (now - lastTime) / 1000.0; // Convert to seconds

  if (timeChange > 0) {
    // Calculate error
    double error = Setpoint - Input;

    // Proportional term
    double proportional = Kp * error;

    // Integral term
    integral += error * timeChange;
    double integralTerm = Ki * integral;

    // Derivative term
    double derivative = (error - previousError) / timeChange;
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
    if (Output >= 255) {
        integral -= error * timeChange;
        Output = 255;
    } else if (Output <= 0) {
        integral -= error * timeChange;
        Output = 0;
    }

    // Apply the PID output to the PWM pin
    analogWrite(pwmPin, (int)Output);

    // Save current time and error for the next loop
    lastTime = now;
    previousError = error;

    // Debugging: print values to Serial Monitor
    Serial.print(Setpoint);
    Serial.print(", ");
    Serial.print(Input);
    Serial.print(", ");
    Serial.println(Output);
  }

  delay(500); // Adjust delay as needed
}