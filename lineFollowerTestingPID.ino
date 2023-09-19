#define ENA 5
#define ENB 6

#define in1 11
#define in2 10
#define in3 9
#define in4 8

const int NUM_SENSORS = 5;            
const int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4}; 
const int sensorThresholds[NUM_SENSORS] = {500, 500, 500, 500, 500}; 

// PID constants
const float Kp = 0.2;
const float Ki = 0.0;
const float Kd = 0.0;

// PID variables
float prevError = 0;
float integral = 0;

// Motor control pins
const int leftMotorPin = 3; // Example pins; adjust as needed
const int rightMotorPin = 4;

void setup() {
  Serial.begin(9600);
  
  pinMode(ENA, OUTPUT);    /*leftMotorPin*/ 
  pinMode(ENB, OUTPUT);    /*rightMotorPin*/ 

  pinMode(in2, OUTPUT);    /*leftMotorPin*/ 
  pinMode(in3, OUTPUT);    /*rightMotorPin*/ 
  
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  delay(5000);
}

void loop() {
  int sensorValues[NUM_SENSORS];

  // Read sensor values
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  // Calculate the position of the line
  int weightedSum = 0;
  int total = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] > sensorThresholds[i]) {
      weightedSum += i;
      total++;
    }
  }

  int position = (total > 0) ? (weightedSum * 1000 / total) : 0;

  // Define the desired position (usually the middle sensor)
  int desiredPosition = 2000;

  // Calculate the error
  int error = position - desiredPosition;

  // Calculate PID control output
  float output = Kp * error + Ki * integral + Kd * (error - prevError);

  // Update PID variables for the next iteration
  integral += error;
  prevError = error;

  // Motor control code based on PID output
  int leftMotorSpeed = 150; // Base motor speed
  int rightMotorSpeed = 150;

  // Adjust motor speeds based on PID output
  leftMotorSpeed += output;
  rightMotorSpeed -= output;

  // Ensure motor speeds are within limits (0-255)
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 200);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 200);

  // Apply motor speeds
  analogWrite(/*leftMotorPin*/ in3, leftMotorSpeed);
  analogWrite(/*rightMotorPin*/ in2, rightMotorSpeed);

  // Print sensor values and PID output for debugging
  Serial.print("Position: ");
  Serial.print(position);
  Serial.print("   Desired Position: ");
  Serial.print(desiredPosition);
  Serial.print("   Error: ");
  Serial.print(error);
  Serial.print("   PID Output: ");
  Serial.println(output);
}
