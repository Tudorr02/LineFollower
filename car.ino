// Define the pins for the motors
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

// Initialize the motor speeds
int m1Speed = 0;
int m2Speed = 0;

// PID controller constants
float kp = 6.75;
float ki = 0;
float kd = 6.3;

// PID variables
int p = 1;
int i = 0;
int d = 0;
int error = 0;
int lastError = 0;

// Speed limits
const int maxSpeed = 255;
const int minSpeed = -255;

// Base speed for the motors
int baseSpeed = 255;

// Speed for calibration
int calibrationSpeed = 200;

// QTR sensor object
QTRSensors qtr;

// Number of sensors
const int sensorCount = 6;

// Array to hold sensor values
int sensorValues[sensorCount];

// Initialize sensor array
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };

void setup() {
  // Begin serial communication
  Serial.begin(9600);

  // Set up the motor pins
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);

  // Set up the QTR sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);

  // Wait for sensor to stabilize
  delay(500);

  // Turn on built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  

  // Set initial motor speed
  setMotorSpeed(baseSpeed, 0);

  // Calibrate the sensors
  calibrate();

  // Turn off built-in LED
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // Calculate error for PID controller
  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);

  // Update PID variables
  p = error;
  i = i + error;
  d = error - lastError;

  // Calculate motor speed based on PID controller
  int motorSpeed = kp * p + ki * i + kd * d; // = error in this case

  // Set base speeds
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  // Adjust motor speeds based on error
  if (error < -15) {
    m1Speed += motorSpeed;
  }
  else if (error > 15) {
    m2Speed -= motorSpeed;
  }

  // Constrain motor speeds to min and max values
  m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, minSpeed, maxSpeed);

  // Set motor speeds
  setMotorSpeed(m1Speed, m2Speed);

  // Print debugging information
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("M1 speed: ");
  Serial.println(m1Speed);
  Serial.print("M2 speed: ");
  Serial.println(m2Speed);

  // Delay for stability
  delay(250);
}
 
// Function to calibrate the sensors
void calibrate() {
  int delay = 16;
  for (int i = 0; i < 240; i++) {
    qtr.calibrate();
    if (i % delay == 0)
     calibrationSpeed= -1 * calibrationSpeed;
    setMotorSpeed(calibrationSpeed, 0);
  }
}
 

// Function to set the speed of the motors
void setMotorSpeed(int motor1Speed, int motor2Speed) {

  // If motor speed is 0, stop the motor
  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    // If motor speed is positive, move forward
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    // If motor speed is negative, move backward
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }

  // Repeat for the second motor
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
     if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
} 
