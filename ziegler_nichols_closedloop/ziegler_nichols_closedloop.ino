// Define pins
const int tempSensorPin = A0; // Temperature sensor pin
const int pwmPin = 9;        // PWM output pin

// P controller parameters
double Setpoint, Input, Output;
double Kp = 256; // Initial guess for critical gain (adjust during testing)

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Set the PWM pin as output
  pinMode(pwmPin, OUTPUT);

  // Set the desired temperature (setpoint)
  Setpoint = 50.0;
}

void loop() {
  // Read the temperature from the LM35 sensor
  int adcValue = analogRead(tempSensorPin);

  // Convert ADC value to temperature in Celsius
  double temperature = adcValue * 4.887585533 / 10;

  // Assign temperature to Input
  Input = temperature;

  // Compute P controller output
  Output = Kp * (Input - Setpoint); // Mode REVERSE

  // Clamp the output to PWM range (0-255)
  if (Output > 255) {
    Output = 255;
  } else if (Output < 0) {
    Output = 0;
  }

  // Apply the P controller output to the PWM pin
  analogWrite(pwmPin, Output);

  // Debugging: print values to Serial Monitor
  Serial.print(Setpoint);
  Serial.print(" °C, ");
  Serial.print(Input);
  Serial.print(" °C, ");
  Serial.print(Output);
  Serial.print(", ");
  Serial.println(Input - Setpoint);

  // Wait before next loop
  delay(500);
}
