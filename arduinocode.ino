// Pin definitions for motors
const int rightFwd = 9;
const int rightRev = 10;
const int leftFwd = 5;
const int leftRev = 6;

// Pin definitions for ultrasonic sensor
const int trigPin = 3;
const int echoPin = 4;

// Motor control variables
int rightPwm = 0;
int leftPwm = 0;

// Function to update PWM values
void updatePwm(int rightPwm, int leftPwm) {
  analogWrite(rightFwd, rightPwm);
  analogWrite(leftFwd, leftPwm);
  analogWrite(rightRev, 0);
  analogWrite(leftRev, 0);
}

// Function to stop motors
void pwmStop() {
  analogWrite(rightFwd, 0);
  analogWrite(rightRev, 0);
  analogWrite(leftFwd, 0);
  analogWrite(leftRev, 0);
}

// Function to get distance from ultrasonic sensor
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = (duration / 2) / 29.1;  // Convert to cm
  return distance;
}

void setup() {
  // Initialize motor pins
  pinMode(rightFwd, OUTPUT);
  pinMode(rightRev, OUTPUT);
  pinMode(leftFwd, OUTPUT);
  pinMode(leftRev, OUTPUT);

  // Initialize pins for ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize serial communication
  Serial.begin(9600);

  // Stop motors at startup
  pwmStop();
}

void loop() {
  // Read distance from ultrasonic sensor
  long distance = getDistance();
  
  // If distance is less than 10 cm, stop the motors
  if (distance < 10) {
    pwmStop();
  } else {
    // Check for incoming serial data
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      
      // Process motor command
      if (command.startsWith("MOTOR")) {
        int separatorIndex = command.indexOf(' ', 6);
        rightPwm = command.substring(6, separatorIndex).toInt();
        leftPwm = command.substring(separatorIndex + 1).toInt();
        updatePwm(rightPwm, leftPwm);
      } else if (command.startsWith("STOP")) {
        pwmStop();
      }
    }
  }
}
