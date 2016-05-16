/**
 * SpoolHolder.ino
 * 
 * 3D filament spool holder, software to control the Arduino board for the automatic filament dispenser.\n
 * This is the software to control the Arduino board with DC motor controller to manage the motorized filament
 * dispenser.
 * 
 * @date May 2016
 * @author Enrico Miglino, Balearic Dynamics <balearicdynamics@gmail.com>
 * @version 1.0
 */


#define E1 10  ///< Enable Pin for motor 1

#define I1 8  ///< Control pin 1 for motor 1
#define I2 9  ///< Control pin 2 for motor 1

#define SENSOR_PIN 2 ///< Microswitch sensor

#define STOP_MOTOR 0 ///< motor speed
#define MIN_SPEED 64 ///< Minimum PWM motor speed
#define MAX_SPEED 255 ///< Max PWM motor speed
#define ACCEL_SPEED 15 ///< msec delay between acceleration increment steps
#define MANUAL_SPEED 192 ///< Max PWM motor speed in manual motion
#define POSITIVE_DIR 1 ///< Positive direction flag
#define NEGATIVE_DIR 0 ///< Negative direction flag

#define MODE_SET 5 ///< Mode pin Running/Manual control
#define MANUAL_LEFT 4 ///< Left direction in manual mode pin
#define MANUAL_RIGHT 3 ///< Right direction in manual mode pin

#define LED_MAN 12 ///< Manual mode LED pin
#define LED_RUN 11 ///< Running mode LED pin

// Replica of the LEDs pins for the signal usage.
// It is not needed to assign these digital pins twice
#define LED_ACTION_RUN 12 ///< Action LED active when the motor is active in running mode
#define LED_ACTION_MAN 11 ///< Action LED active when the motor is active in manual mode

// Activity status flags
#define RUNNING_MODE 1 ///< Running mode flag status
#define MANUAL_MODE 0 ///< Manual mode flag status

int speedMotor; ///< Current motor speed
int directionMotor; ///< positive or negative value determining the current direction
int directionLogic[2]; ///< Logic values of the current direction set to the corresponding digital pins
boolean runningStatus; ///< Status of the controls (manual or running)

#define __DEBUG__
#define SERIAL_SPEED 9600

/**
 * Initialisation
 */
void setup() {

#ifdef __DEBUG__
  Serial.begin(SERIAL_SPEED);
#endif

  runningStatus = RUNNING_MODE;

  pinMode(LED_MAN, OUTPUT);
  pinMode(LED_RUN, OUTPUT);

  pinMode(E1, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  
  pinMode(SENSOR_PIN, INPUT);

  pinMode(MODE_SET, INPUT);
  pinMode(MANUAL_LEFT, INPUT);
  pinMode(MANUAL_RIGHT, INPUT);
  
  // Initial motor speed
  speedMotor = STOP_MOTOR;
  // Initialize direction parameters
  directionMotor = POSITIVE_DIR;
  directionLogic[0] = HIGH;
  directionLogic[1] = LOW;
  runningStatus = RUNNING_MODE;
  
  stopMotor();
  updateLED();
}

/**
 * Enable the action LED status depending on the current running status
 * 
 * @param isEnabled If HIGH, enable the LED else if is LOW disable it
 */
 void enableActionLED(int isEnabled) {
  if(runningStatus == RUNNING_MODE) {
    digitalWrite(LED_ACTION_RUN, isEnabled);
  }
  else {
    digitalWrite(LED_ACTION_MAN, isEnabled);
  }
 }

/**
 * Manually moves the motor clockwise direction
 * 
 * If the previous direction is set to the opposite, the motor is stopped and 
 * accelerated in the desired direction.
 */
void runMotorClockwise() {
  if(directionMotor == NEGATIVE_DIR) {
      swapDirection();  
    }
  // Rise the desired speed
  startMotor();
  accelMotor(MANUAL_SPEED);
}

/**
 * Manually moves the motor counter clockwise direction
 * 
 * If the previous direction is set to the opposite, the motor is stopped and 
 * accelerated in the desired direction.
 */
void runMotorCounterClockwise() {
  if(directionMotor == POSITIVE_DIR) {
    swapDirection();  
  }
  // Rise the desired speed
  startMotor();
  accelMotor(MANUAL_SPEED);
}

/**
 * Update the running/manual LED status
 */
void updateLED() {
  if(runningStatus == RUNNING_MODE) {
    digitalWrite(LED_RUN, HIGH);
    digitalWrite(LED_MAN, LOW);
  }
  else {
    digitalWrite(LED_RUN, LOW);
    digitalWrite(LED_MAN, HIGH);
  }
 }

/**
 *Swap the current status between running and manual mode
 */
 void swapMode() {
  if(runningStatus == RUNNING_MODE)
    runningStatus = MANUAL_MODE;
  else
    runningStatus = RUNNING_MODE;

  updateLED();
 }

/**
 * Stop the motor and set the speed to 0. The motor is stopped immediately 
 * without deceleration
 */
void stopMotor() {

  if(speedMotor != STOP_MOTOR) {
    analogWrite(E1, STOP_MOTOR);
    digitalWrite(I1, LOW);
    digitalWrite(I2, LOW);
    speedMotor = STOP_MOTOR;
  }
}

/**
 * Start the current motor direction at the current speed
 */
void startMotor() {
  int currSpeed;

  // Save the current motor speed
  currSpeed = speedMotor;
  // Start from minimum speed accelerating to the desired speed
  accelMotor(MIN_SPEED);
  digitalWrite(I1, directionLogic[0]);
  digitalWrite(I2, directionLogic[1]);
  accelMotor(currSpeed);
}

/**
 * Progressively increase the pwm from the current speed value to the target speed value.
 * 
 * If the target value is lower than the speed, the steps decelerate the motor speed.
 * If the speed value is out of the speed limits it is forced to the limit.
 * 
 * @param target The taget speed value
 */
void accelMotor(int target) {
  int dir, j;  ///< Increment direction

  // Check for the speed limits
  if(target > MAX_SPEED)
    target = MAX_SPEED;

  if(target < MIN_SPEED)
    target = MIN_SPEED;
  
  // Check for the direction
  if(target > speedMotor)
    dir = 1;
  else
    dir = -1;

  // Acceleration loop
  for(j = speedMotor; j != target; j += dir) {
    analogWrite(E1, j);
    delay(ACCEL_SPEED);
  }

  // Update the current speed
  speedMotor = j - dir;

}

/**
 * Invert the direction of the motor deceleratin the current direction to the min
 * value then rising the the speed again in the opposite direction.
 */
 void reverseDirection() {
  int currSpeed;

  // Save the current motor speed
  currSpeed = speedMotor;
  // Reduce the motor speed to the minimum
  accelMotor(MIN_SPEED);
  // Invert the motor direction
  swapDirection();
  // Accelerate the motor to the speed
  accelMotor(currSpeed);
 }

 /**
  * Swap the direction settings. This funciton has no effect on movement, it only affcts the
  * flags. This method is used by the reverseDirection() function
  */
  void swapDirection() {

    if(directionMotor == POSITIVE_DIR) {
      directionMotor = NEGATIVE_DIR;
      directionLogic[0] = LOW;
      directionLogic[1] = HIGH;
    }
    else {
      directionMotor = POSITIVE_DIR;
      directionLogic[0] = HIGH;
      directionLogic[1] = LOW;
    }

    digitalWrite(I1, directionLogic[0]);
    digitalWrite(I2, directionLogic[1]);
}

/**
 * Exectues a sequence enabled by the sensor
 */

/**
 * Main application loop
 */
void loop() {
  // Check for sensor pin
  if(digitalRead(SENSOR_PIN) == HIGH) {
    if(runningStatus == RUNNING_MODE) {
      // Enable the action LED
      enableActionLED(HIGH);
      // Start the motor
      startMotor();
      // Rise the desired speed
      accelMotor(MAX_SPEED);
//      // Invert the direction
//      reverseDirection();
//      // check for the sensor status and continue rotation until sensor is active
//      while(digitalRead(SENSOR_PIN) == HIGH)
//        delay(50);
//
      stopMotor();
      // Enable the action LED
      enableActionLED(LOW);
    } // Running mode
  } // Sensor pin

  // Check for mode button pressed
  if(digitalRead(MODE_SET) == HIGH) {
#ifdef __DEBUG__
      Serial.println("swapMode()");
#endif
    swapMode();
  } // Mode button pressed

  // Check for the manual controls, if it is in manual mode
  // If the sensor pin is active, the movements are locked
  if(runningStatus == MANUAL_MODE) {
    if(digitalRead(SENSOR_PIN) == LOW) {
      if(digitalRead(MANUAL_LEFT) == HIGH) {
        // Enable the action LED
        enableActionLED(HIGH);
        runMotorClockwise();
      } // Manual left button pressed
      if(digitalRead(MANUAL_RIGHT) == HIGH) {
        // Enable the action LED
        enableActionLED(HIGH);
        runMotorCounterClockwise();
      } // Manual right button pressed
      if( (digitalRead(MANUAL_RIGHT) == LOW) && (digitalRead(MANUAL_LEFT) == LOW) ) {
          if (speedMotor != STOP_MOTOR) {
            stopMotor();
          } // Stop motor after maual running
        // Disable the action LED
        enableActionLED(LOW);
      } // No buttons pressed
    } // Sensor pin is not active
  } // Manual mode set and sensor not set
  else {
    if(speedMotor != STOP_MOTOR) {
      stopMotor();
      // Disable the action LED
      enableActionLED(LOW);
    }
  }

  delay(125);
}


