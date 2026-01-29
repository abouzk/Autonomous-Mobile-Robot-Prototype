//ENCODER ITERATION
#include <avr/interrupt.h>

//Motor pins
#define IN1 12  // Right motor direction
#define IN2 13
#define IN3 7  // Left motor direction
#define IN4 8
#define ENA 11  // Right motor PWM
#define ENB 6   // Left motor PWM

//Encoder test
#define ENCA1 2                  //right
#define ENCB1 4                  //
#define ENCA2 3                  //left
#define ENCB2 5                  //
#define readA1 bitRead(PIND, 2)  //faster than digitalRead()
#define readB1 bitRead(PIND, 3)

//Button
#define BUTTON 22

// Ultrasonic sensor pins
const int echoPin = 9;
const int trigPin = 10;
float duration, distance;


int r_pos = 0;
int l_pos = 0;
const unsigned int encoder_ticks_per_meter = 9000; //1000;
int fullturndegrees = 2900;

void setup() {
  Serial.begin(9600);
  //Motor initialize
  pinMode(IN1, OUTPUT);  //right
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);  //left
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  //Encoder
  pinMode(ENCA1, INPUT);  //Right
  pinMode(ENCB1, INPUT);
  pinMode(ENCA2, INPUT);  //Left
  pinMode(ENCB2, INPUT);

  //encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENCA1), rightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), leftEncoder, CHANGE);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //Initialize button
  pinMode(BUTTON, INPUT_PULLUP);
}

void loop() {
  // main
  bool buttonState = digitalRead(BUTTON);  // Read the button state

  // if (buttonState == LOW) {              // LOW when pressed (inverted logic)
  //   Serial.println("Button Pressed");
  // } else {
  //   Serial.println("Button Released");
  //}
  Serial.print("Right: ");
  Serial.print(r_pos);
  Serial.print("      Left: ");
  Serial.print(l_pos);
  Serial.print("      Distance: ");
  Serial.println(measureDistance());
  Serial.println();
  //driveForward(1.0);
  turn(90);
  // driveForward(0.5);
  // turn(180);
  // driveForward(0.5);
  // turn(-90);
  // driveForward(1.0);
  // turn(180);
  Serial.println("END, LOOPING");
  delay(3000);
}

void rightEncoder() {
  if (digitalRead(ENCA1) == digitalRead(ENCB1)) {
    r_pos++;  // Forward
  } else {
    r_pos--;  // Reverse
  }
}

// Interrupt routine for the left encoder
void leftEncoder() {
  if (digitalRead(ENCA2) == digitalRead(ENCB2)) {
    l_pos++;  // Forward
  } else {
    l_pos--;  // Reverse
  }
}

void driveForward(float distance) {
  // Convert meters to encoder ticks
  long target_ticks = distance * encoder_ticks_per_meter; 
  
  // Reset counters before starting a new segment to avoid accumulated drift errors
  r_pos = 0; 
  l_pos = 0;

  // Control Constants
  float Kp = 5.0;       // Proportional Gain (Tune this: Higher = Stronger Correction)
  int baseSpeed = 200;  // Base PWM speed (0-255)

  // Loop until we reach the target distance
  // (Using average of both wheels for distance termination)
  while ((abs(r_pos) + abs(l_pos)) / 2 < target_ticks) {
    
    // SAFETY CHECK: Subsumption Architecture
    // Check obstacle BEFORE driving. If blocked, this function pauses motors inside checkObstacle()
    checkObstacle(); 

    // PROPORTIONAL CONTROLLER
    // Calculate error: Positive means Right is ahead, Negative means Left is ahead
    int error = abs(r_pos) - abs(l_pos); 

    // Adjust speeds: If Right is ahead (error > 0), Right slows down, Left speeds up
    int rightSpeed = baseSpeed - (Kp * error);
    int leftSpeed = baseSpeed + (Kp * error);

    // Clamp values to valid PWM range (0-255)
    rightSpeed = constrain(rightSpeed, 0, 255);
    leftSpeed = constrain(leftSpeed, 0, 255);

    // Apply to motors
    digitalWrite(IN1, HIGH);  // Forward Right
    digitalWrite(IN2, LOW);
    analogWrite(ENA, rightSpeed);

    digitalWrite(IN3, HIGH);  // Forward Left
    digitalWrite(IN4, LOW);
    analogWrite(ENB, leftSpeed);

    // Debugging Telemetry
    Serial.print("Target: "); Serial.print(target_ticks);
    Serial.print(" | R: "); Serial.print(r_pos);
    Serial.print(" | L: "); Serial.print(l_pos);
    Serial.print(" | Err: "); Serial.println(error);
  }

  stopMotors(); // Done
}

void turn(int degrees) {
  Serial.print("TURN : ");
  Serial.println(degrees);
  delay(1000);  //pause for 1 second
  reset_encoders();
  int r_fac, l_fac;
  if (degrees == 90) { //RIGHT TURN
    digitalWrite(IN1, LOW);  // Backwards right
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 255);
    r_fac = -4;

    digitalWrite(IN3, HIGH);  // Forward left
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 255);
    l_fac = 4;
  } else if (degrees == -90) { //LEFT TURN
    digitalWrite(IN1, HIGH);  // Forwards right
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 255);
    r_fac = 4;

    digitalWrite(IN3, LOW);  // Backwards left
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, 255);
    l_fac = -4;
  } else if (degrees == 180) { //180 TURN AROUND
    digitalWrite(IN1, LOW);  // Backwards right
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 255);
    r_fac = -2;

    digitalWrite(IN3, HIGH);  // Forward left
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 255);
    l_fac = 2;
  }else if (degrees == 360) { //360 CIRCLE AROUND
    digitalWrite(IN1, LOW);  // Backwards right
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 250);
    r_fac = -1;

    digitalWrite(IN3, HIGH);  // Forward left
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 250);
    l_fac = 1;
  }

  Serial.println("Checkpoint in turn function");
  Serial.print("r_fac: ");
  Serial.print(r_fac);
  Serial.print(" l_fac: ");
  Serial.println(l_fac);

  Serial.print("r_pos: ");
  Serial.print(r_pos);
  Serial.print(" l_pos: ");
  Serial.println(l_pos);

  Serial.print("fullturndegrees/r_fac: ");
  Serial.print(fullturndegrees/r_fac);
  Serial.print(" fullturndegrees/l_fac: ");
  Serial.println(fullturndegrees/l_fac);

  while (abs(r_pos) < abs(fullturndegrees/r_fac) || abs(l_pos) < abs(fullturndegrees/l_fac)) {
    if(abs(r_pos) >= abs(fullturndegrees/r_fac)){
      analogWrite(ENA, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
    if(abs(l_pos) >= abs(fullturndegrees/l_fac)){
      analogWrite(ENB, 0);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
    Serial.print("Right: ");
    Serial.print(r_pos);
    Serial.print("      Left: ");
    Serial.println(l_pos);
  }

  stopMotors();
  Serial.print("Right: ");
  Serial.print(r_pos);
  Serial.print("      Left: ");
  Serial.println(l_pos);
  reset_encoders();
}

void checkObstacle() {
  float distanceCm = measureDistance();
  Serial.print("    DISTANCE:   ");
  Serial.println(distanceCm);
  if ((distanceCm < 20) && (distanceCm > 0)) {
    Serial.println("distance check determined we need to call stopMotors(), stopping");
    stopMotors();
  }else{
    digitalWrite(IN1, HIGH);  // Forward right
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 236);
    digitalWrite(IN3, HIGH);  // Forward left
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 240);
  }
}

float measureDistance() {
  long duration;
  float distanceCm;

  // Clear the trigPin by setting it LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by setting the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH, 30000);  // Timeout after 30ms (for max distance)

  // Calculate the distance in centimeters
  distanceCm = (duration / 2.0) * 0.0343;

  // Handle out-of-range values
  if (duration == 0) {
    distanceCm = 0;  // Indicates out of range or no object detected
  }

  return distanceCm;
}

void reset_encoders(){
  detachInterrupt(digitalPinToInterrupt(ENCA1));
  detachInterrupt(digitalPinToInterrupt(ENCA2));
  r_pos = 0; l_pos = 0;
  attachInterrupt(digitalPinToInterrupt(ENCA1), rightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), leftEncoder, CHANGE);
  Serial.print("Reset, r_pos: ");
  Serial.print(r_pos);
  Serial.print(" l_pos: ");
  Serial.println(l_pos);
}

void stopMotors() {
  Serial.println("StopMotors() function entered.");
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(2000);
}
