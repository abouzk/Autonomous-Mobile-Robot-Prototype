/**
 * Autonomous Mobile Robot Controller
 * Controls a two-tracked robot with encoder-based odometry and ultrasonic obstacle detection
 * 
 * Hardware:
 * - 2x DC motors with quadrature encoders
 * - HC-SR04 ultrasonic sensor for obstacle detection
 * - Arduino with AVR interrupts for encoder counting
 */

#include <avr/interrupt.h>

// ===== MOTOR CONFIGURATION =====
#define IN1 12  // Right motor direction control (HIGH = forward)
#define IN2 13  // Right motor direction control (LOW = forward)
#define IN3 7   // Left motor direction control (HIGH = forward)
#define IN4 8   // Left motor direction control (LOW = forward)
#define ENA 11  // Right motor PWM speed (0-255)
#define ENB 6   // Left motor PWM speed (0-255)

// ===== ENCODER CONFIGURATION =====
// Quadrature encoder pins for motor speed/direction feedback
#define ENCA1 2      // Right encoder channel A (interrupt pin)
#define ENCB1 4      // Right encoder channel B
#define ENCA2 3      // Left encoder channel A (interrupt pin)
#define ENCB2 5      // Left encoder channel B

// ===== ULTRASONIC SENSOR CONFIGURATION =====
const int echoPin = 9;   // Receives echo pulse from HC-SR04
const int trigPin = 10;  // Triggers distance measurement on HC-SR04

// ===== BUTTON CONFIGURATION =====
#define BUTTON 22  // Button input (pulled high, LOW when pressed)

// ===== CALIBRATION CONSTANTS =====
// These values must be calibrated for one's specific robot hardware
volatile int r_pos = 0;  // Right encoder position (ticks)
volatile int l_pos = 0;  // Left encoder position (ticks)
const unsigned int encoder_ticks_per_meter = 9000;  // Encoder ticks per meter traveled
const int fullturndegrees = 2900;  // Encoder ticks for a full 360-degree rotation

void setup() {
  Serial.begin(9600);
  
  // Initialize motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize encoder pins as inputs
  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);

  // Attach interrupts to encoder pins for high-speed position tracking
  // CHANGE trigger fires on both rising and falling edges for better resolution
  attachInterrupt(digitalPinToInterrupt(ENCA1), rightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), leftEncoder, CHANGE);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize button with internal pull-up (reads LOW when pressed)
  pinMode(BUTTON, INPUT_PULLUP);
}

void loop() {
  // Read button state (INPUT_PULLUP: LOW when pressed)
  bool buttonState = digitalRead(BUTTON);
  
  // TODO: Implement button-triggered modes (autonomous, manual, etc.)
  // if (buttonState == LOW) {
  //   Serial.println("Button Pressed");
  // }
  
  // Telemetry output
  Serial.print("Right: ");
  Serial.print(r_pos);
  Serial.print("  |  Left: ");
  Serial.print(l_pos);
  Serial.print("  |  Distance: ");
  Serial.print(measureDistance());
  Serial.println(" cm");
  
  // TEST SEQUENCE: Comment out unused movements
  turn(90);
  // driveForward(1.0);
  // turn(180);
  // driveForward(0.5);
  // turn(-90);
  // driveForward(1.0);
  // turn(180);
  
  Serial.println("---END LOOP---");
  delay(3000);
}

/**
 * Right encoder interrupt handler
 * Triggered on CHANGE (rising and falling edges) of ENCA1
 * Direction determined by comparing ENCA1 and ENCB1 states
 */
void rightEncoder() {
  // If channel A and B are in same state, robot moving forward
  // If different state, robot moving backward
  if (digitalRead(ENCA1) == digitalRead(ENCB1)) {
    r_pos++;  // Forward rotation
  } else {
    r_pos--;  // Backward rotation
  }
}

/**
 * Left encoder interrupt handler
 * Triggered on CHANGE (rising and falling edges) of ENCA2
 * Direction determined by comparing ENCA2 and ENCB2 states
 */
void leftEncoder() {
  if (digitalRead(ENCA2) == digitalRead(ENCB2)) {
    l_pos++;  // Forward rotation
  } else {
    l_pos--;  // Backward rotation
  }
}

/**
 * Drive forward a specified distance using proportional feedback control
 * Uses encoder-based odometry to track actual distance traveled
 * Subsumption architecture: obstacle checking takes priority over motion
 * 
 * @param distance Distance to travel in meters
 */
void driveForward(float distance) {
  // Convert meters to encoder ticks based on wheel circumference
  long target_ticks = distance * encoder_ticks_per_meter; 
  
  // Reset position counters to measure distance for this segment only
  r_pos = 0; 
  l_pos = 0;

  // Proportional control parameters (tune these for your robot)
  const float Kp = 5.0;       // Proportional gain - higher = stronger wheel sync correction
  const int baseSpeed = 200;  // Base PWM speed (0-255)

  // Drive until average distance traveled reaches target
  while ((abs(r_pos) + abs(l_pos)) / 2 < target_ticks) {
    
    // SUBSUMPTION: Obstacle detection takes priority - stops motors if blocked
    checkObstacle(); 

    // PROPORTIONAL CONTROLLER: Keep wheels synchronized
    // Error is positive when right wheel is ahead (slipping)
    int error = abs(r_pos) - abs(l_pos); 

    // Adjust PWM: Slow down the fast wheel, speed up the slow wheel
    int rightSpeed = baseSpeed - (Kp * error);
    int leftSpeed = baseSpeed + (Kp * error);

    // Constrain to valid PWM range
    rightSpeed = constrain(rightSpeed, 0, 255);
    leftSpeed = constrain(leftSpeed, 0, 255);

    // Apply speeds to motors (both driving forward)
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, rightSpeed);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, leftSpeed);

    // Telemetry for tuning and debugging
    Serial.print("Target: "); Serial.print(target_ticks);
    Serial.print(" | R: "); Serial.print(r_pos);
    Serial.print(" | L: "); Serial.print(l_pos);
    Serial.print(" | Error: "); Serial.println(error);
  }

  stopMotors();
}

/**
 * Rotate in place by a specified angle
 * Currently supports: 90°, -90°, 180°, 360° (extend for arbitrary angles)
 * Uses encoder ticks and pre-calibrated turn factors
 * 
 * @param degrees Rotation angle (positive = right/clockwise, negative = left/counter-clockwise)
 */
void turn(int degrees) {
  Serial.print("TURN: ");
  Serial.print(degrees);
  Serial.println(" degrees");
  delay(1000);  // Brief pause before starting turn
  
  reset_encoders();
  
  // Determine motor directions and encoder tick scaling based on turn angle
  int r_fac, l_fac;
  
  if (degrees == 90) {
    // Right turn: right motor backward, left motor forward
    digitalWrite(IN1, LOW);   // Backward right
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 255);
    r_fac = -4;  // Right wheel ticks scaled (negative = backward)

    digitalWrite(IN3, HIGH);  // Forward left
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 255);
    l_fac = 4;   // Left wheel ticks scaled
    
  } else if (degrees == -90) {
    // Left turn: right motor forward, left motor backward
    digitalWrite(IN1, HIGH);  // Forward right
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 255);
    r_fac = 4;

    digitalWrite(IN3, LOW);   // Backward left
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, 255);
    l_fac = -4;
    
  } else if (degrees == 180) {
    // 180° turn: right backward, left forward
    digitalWrite(IN1, LOW);   // Backward right
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 255);
    r_fac = -2;

    digitalWrite(IN3, HIGH);  // Forward left
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 255);
    l_fac = 2;
    
  } else if (degrees == 360) {
    // 360° circle: right backward, left forward
    digitalWrite(IN1, LOW);   // Backward right
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 250);
    r_fac = -1;

    digitalWrite(IN3, HIGH);  // Forward left
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 250);
    l_fac = 1;
    
  } else {
    Serial.println("ERROR: Unsupported turn angle. Supported: ±90, 180, 360");
    return;
  }

  // Debug: Show calculated tick targets
  Serial.print("Target ticks - Right: ");
  Serial.print(fullturndegrees / r_fac);
  Serial.print(" | Left: ");
  Serial.println(fullturndegrees / l_fac);

  // Rotate until both wheels reach their target tick counts
  while (abs(r_pos) < abs(fullturndegrees / r_fac) && abs(l_pos) < abs(fullturndegrees / l_fac)) {
    // Once right wheel reaches target, stop it (let left wheel catch up)
    if (abs(r_pos) >= abs(fullturndegrees / r_fac)) {
      analogWrite(ENA, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
    // Once left wheel reaches target, stop it
    if (abs(l_pos) >= abs(fullturndegrees / l_fac)) {
      analogWrite(ENB, 0);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
    
    // Telemetry
    Serial.print("Right: ");
    Serial.print(r_pos);
    Serial.print("  |  Left: ");
    Serial.println(l_pos);
  }

  stopMotors();
  Serial.println("Turn complete.");
}

/**
 * Measure distance to nearest obstacle using HC-SR04 ultrasonic sensor
 * 
 * @return Distance to obstacle in centimeters (0 = out of range or no object)
 */
float measureDistance() {
  // Ensure trigger pin is low before pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send 10µs pulse to trigger sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo pulse width (proportional to distance)
  // Timeout after 30ms (prevents hanging on sensor failure)
  long duration = pulseIn(echoPin, HIGH, 30000);

  // Convert time to distance: speed of sound ≈ 343 m/s
  // Distance = (duration / 2) * 0.0343 cm/µs
  float distanceCm = (duration / 2.0) * 0.0343;

  // Sensor returns 0 on timeout (out of range)
  if (duration == 0) {
    distanceCm = 0;
  }

  return distanceCm;
}

/**
 * Check for obstacles and stop if too close
 * Called during forward motion to implement subsumption architecture (obstacle checking = highest priority)
 * 
 * @note This function has a design issue: it should only update motor state based on sensor input,
 *       not re-drive motors. The calling function should maintain motor control.
 */
void checkObstacle() {
  float distanceCm = measureDistance();
  
  const int OBSTACLE_THRESHOLD = 20;  // Minimum safe distance in cm
  
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");
  
  // If obstacle detected, stop motors immediately
  if ((distanceCm < OBSTACLE_THRESHOLD) && (distanceCm > 0)) {
    Serial.println("OBSTACLE DETECTED - STOPPING");
    stopMotors();
  }
}

/**
 * Reset encoder position counters and re-attach interrupts
 * Used between movement segments to measure distance for each segment independently
 */
void reset_encoders() {
  // Temporarily disable interrupts to safely reset counters
  detachInterrupt(digitalPinToInterrupt(ENCA1));
  detachInterrupt(digitalPinToInterrupt(ENCA2));
  
  r_pos = 0;
  l_pos = 0;
  
  // Re-enable interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA1), rightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), leftEncoder, CHANGE);
  
  Serial.println("Encoders reset to 0");
}

/**
 * Stop both motors and hold them stationary
 * Includes 2-second delay to ensure motors fully decelerate
 */
void stopMotors() {
  Serial.println("STOPPING MOTORS");
  
  // Cut all motor power
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  // Hold pins low (removes any residual force)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  delay(2000);  // Wait for motors to fully stop before next command
}
