/*
  KAA109 Engineering Problem Solving and Data Analysis
  Assignment 3: Arduino Autonomous Vehicle
  Team Number: 17
  Team Members: Jack Connell, Caleb Reynolds, Romy Di Ubaldo
  
  This programme implements the following funcitonality:
   * 
   *  
   * 
*/

// Enable debugging:
// If defined then output will be printed through Serial.print
#define DEBUG

// Tinkercad or physical system selection:
// If TINKERCAD is defined, then the RGB LEDs will use different
// values for enabling and setting their red, green and blue outputs.
//#define TINKERCAD

#ifdef DEBUG
#define log(x) Serial.print(x)
#define logln(x) Serial.println(x)
#else
#define log(x)
#define logln(x)
#endif

// Drive pin definitions
#define DRIVE_RIGHT_PIN_1 9   // PWM/analogWrite(0-255)
#define DRIVE_RIGHT_PIN_2 8
#define DRIVE_LEFT_PIN_1  10  // PWM/analogWrite(0-255)
#define DRIVE_LEFT_PIN_2  12

// Encoder pin definitions:
#define LEFT_ENCODER_PIN  2
#define RIGHT_ENCODER_PIN 3

// Ultrasonic sensor pin and value definitions:
#define LEFT_TRIG_PIN     A1
#define LEFT_ECHO_PIN     A0

#define RIGHT_TRIG_PIN    A2
#define RIGHT_ECHO_PIN    A3

// RGB LED pin definitions:
#define RED_LED_PIN           5
#define GREEN_LED_PIN         11
#define BLUE_LED_PIN          6
#define LED_LEFT_EN_PIN       4
#define LED_RIGHT_EN_PIN      7

// 'On' brightness of RGB LEDs (max 255):
#ifdef TINKERCAD
#define BRIGHT_PWM        255
#else
#define BRIGHT_PWM        16
#endif

// Left and right enable output values:
// Hint: Make sure you enable the left and right LEDs
//       if you want them to be on.
// Hint: To set red, green and blue outputs use the
//       LED_PWM_VALUE(x) macro to get their brightness.
#ifdef TINKERCAD
#define LED_LEFT_ENABLED   LOW
#define LED_RIGHT_ENABLED  LOW
#define LED_LEFT_DISABLED  HIGH
#define LED_RIGHT_DISABLED HIGH
#define LED_PWM_VALUE(pwm) pwm
#else
#define LED_LEFT_ENABLED   HIGH
#define LED_RIGHT_ENABLED  HIGH
#define LED_LEFT_DISABLED  LOW
#define LED_RIGHT_DISABLED LOW
#define LED_PWM_VALUE(pwm) (255-pwm)
#endif

//START OF CODE:

//Romy- Sensors and Input Code:
/* 
JC needs:
	   x position of the robot
       y position of the robot
       the robots angle of travel, defined from every change of direction
       ultrasonic distances of LHS and RHS
       and if ultrasonic senors is detecting something (ultrasonic valid or non valid)
*/
  
//Caleb- Movement and Hardware Control Code:

//JC- Navigation and Obstacle Detection Code:

// Group setup component
// set up Millis components
unsigned long startStartTime = 0;

unsigned long flashStartTime = 0;

unsigned long obstacleClearTime = 0;
const unsigned long CLEAR_CONFIRM_MS = 200; // obstacle must be clear for this long
int lockedTurnDirection = 1; // 1 = right, -1 = left

// turning timing variables
const unsigned long TURN_TIME_MS = 400;

const unsigned long STOP_PAUSE_MS = 100;

// Flash rate for LED's
const unsigned long FLASH_INTERVAL_MS = 250;

//JC setup
// define the individual states of the robot with a number 0-6 to determine which case the robot it changing to.
enum ROBOTSTATE {
 STATE_START,
 STATE_REORIENT,
 STATE_MOVE_FORWARD,
 STATE_AVOID_OBSTACLE,
 STATE_TURN_LEFT,
 STATE_TURN_RIGHT,
 STATE_BYPASS_DRIVE,
 STATE_STATIONARY,
 STATE_COMPLETED,
};

ROBOTSTATE CURRENT_STATE = STATE_START;

ROBOTSTATE newState = STATE_START;

// Distance parameters

const float pi = 3.14;
const float object_threshold = 20.0; // cm this is the acceptable distance from the obstacle that the robot can be, this means after this point the robot will stop.
const float target_x = 200.0; //cm position of the robot in x
const float target_y = 0.0; //cm position of the robot in y
const float target_tolerance = 20.0; // cm

// Timing parameters
long stateStartTime = 0;

//Setup CR

//Restrict wheel speeds (0 to 255)
#define DRIVE_SPEED 160
#define TURN_SPEED 120
#define REVERSE_SPEED 160


//Define the pin modes (output or input) for each component 
void setup(){
  pinMode(DRIVE_RIGHT_PIN_1, OUTPUT);
  pinMode(DRIVE_RIGHT_PIN_2, OUTPUT);
  pinMode(DRIVE_LEFT_PIN_1, OUTPUT);
  pinMode(DRIVE_LEFT_PIN_2, OUTPUT);
  
  pinMode(LED_LEFT_EN_PIN, OUTPUT);
  pinMode(LED_RIGHT_EN_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), LeftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), RightEncoderISR, RISING);

  StopDriving();
  DisableBothLEDs();
}

//Setup RD

const float CM_PER_PULSE = (pi * 6.7) / 20;

const float turn_radius = 13.0 / 2.0;

const float circumference = pi * 13;

const float DEGREES_PER_PULSE = (CM_PER_PULSE / circumference) * 360;

float left_distance = 0; 

float right_distance = 0; 

float x_position = 0; 

float y_position = 0; 

float headingDeg = 0; 

bool obstacle_left = false; 

bool obstacle_right = false; 

volatile long leftEncoderCount = 0; 

volatile long rightEncoderCount = 0; 

long previousLeftEncoderCount = 0; 

long previousRightEncoderCount = 0; 

const unsigned long REORIENT_OBSTACLE_SUPPRESS_MS = 600;

float avoidTargetHeading = 0;

//JC: functions for changing/representing robot states

//uses the wheel encoder knowledge to find the displacement of the robot from the end point(0,200)
float distance_to_target () 
{
  float dx = target_x - x_position; // compare the point in space the robot is in refernece to the target in the x-axis
  float dy = target_y - y_position; // compare the point in space the robot is in refernece to the target in the y-axis
  return sqrt(dx * dx + dy * dy); // use pythagoras theorem to determine the distance in a straight line to the target
}

// true or false statement switch about the position of the robot in space compared to end point.
bool target_reached() {
  return distance_to_target() < target_tolerance;
}

// detects if the sensor data about the left and right of the robot is less then the target tolerance (distance allowed from an object) 
bool obstacle_detected()
{
  return (left_distance > 0 && left_distance < object_threshold) ||
         (right_distance > 0 && right_distance < object_threshold);
}

// chooses based upon the distance from the sensor to dictate how the robot should act
int direction_of_turning()
{
  // obstacle only on right side, turn left
  if (obstacle_right && !obstacle_left) return -1;
  
  // obstacle only on left side, turn right
  if (obstacle_left && !obstacle_right) return 1;
  
  // both sensors or neither — default turn right
  return 1;
}

// State change aiding the final loop
void Changestate(ROBOTSTATE newState) {
  CURRENT_STATE = newState;
  stateStartTime = millis ();
}

// CR Functions
//Set the left and right drive pins to high/low 
// to make the car drive forward
void DriveForwards(){
  digitalWrite(DRIVE_RIGHT_PIN_1, LOW);
  analogWrite(DRIVE_RIGHT_PIN_2, DRIVE_SPEED);
  digitalWrite(DRIVE_LEFT_PIN_1, LOW);
  analogWrite(DRIVE_LEFT_PIN_2, DRIVE_SPEED);
}

//Set the left and right drive pins to high/low
// to make the car drive backwards
void DriveBackwards(){
  analogWrite(DRIVE_RIGHT_PIN_1, REVERSE_SPEED);
  digitalWrite(DRIVE_RIGHT_PIN_2, LOW);
  analogWrite(DRIVE_LEFT_PIN_1, REVERSE_SPEED);
  digitalWrite(DRIVE_LEFT_PIN_2, LOW);
}

//Set all drive pins to low to stop the car
void StopDriving(){
  analogWrite(DRIVE_RIGHT_PIN_1, 0);
  analogWrite(DRIVE_RIGHT_PIN_2, 0);
  analogWrite(DRIVE_LEFT_PIN_1, 0);
  analogWrite(DRIVE_LEFT_PIN_2, 0);
}

//Set the right drive pins to rotate forwards
//Set the left drive pins to rotate in reverse
void TurnLeft(){
  analogWrite(DRIVE_RIGHT_PIN_1, TURN_SPEED);
  digitalWrite(DRIVE_RIGHT_PIN_2, LOW);
  digitalWrite(DRIVE_LEFT_PIN_1, LOW);
  analogWrite(DRIVE_LEFT_PIN_2, TURN_SPEED);
}

//Set the right drive pins to rotate in reverse
//Set the left dive pins to rotate forwards
void TurnRight(){
  digitalWrite(DRIVE_RIGHT_PIN_1, LOW);
  analogWrite(DRIVE_RIGHT_PIN_2, TURN_SPEED);
  analogWrite(DRIVE_LEFT_PIN_1, TURN_SPEED);
  digitalWrite(DRIVE_LEFT_PIN_2, LOW);
}

void EnableLeftLED(){
  digitalWrite(LED_LEFT_EN_PIN, LED_LEFT_ENABLED);
}

void DisableLeftLED(){
  digitalWrite(LED_LEFT_EN_PIN, LED_LEFT_DISABLED);
}

void EnableRightLED(){
  digitalWrite(LED_RIGHT_EN_PIN, LED_RIGHT_ENABLED);
}

void DisableRightLED(){
  digitalWrite(LED_RIGHT_EN_PIN, LED_RIGHT_DISABLED);
}

void EnableBothLEDs(){
  EnableLeftLED();
  EnableRightLED();
}

void DisableBothLEDs(){
  DisableLeftLED();
  DisableRightLED();
}

void SetLEDColour(int red, int green, int blue){
  analogWrite(RED_LED_PIN, LED_PWM_VALUE(red));
  analogWrite(GREEN_LED_PIN, LED_PWM_VALUE(green));
  analogWrite(BLUE_LED_PIN, LED_PWM_VALUE(blue));
}

void DrivingLEDs(){
  EnableBothLEDs();
  SetLEDColour(0, BRIGHT_PWM, 0);
}

void ShowLeftTurnLED(){
  EnableLeftLED();
  DisableRightLED();
  SetLEDColour(0, 0, BRIGHT_PWM);
}

void ShowRightTurnLED(){
  EnableRightLED();
  DisableLeftLED();
  SetLEDColour(0, 0, BRIGHT_PWM);
}

void RedLED(){
  EnableBothLEDs();
  SetLEDColour(BRIGHT_PWM, 0, 0);
}

void DriveForwards(unsigned long timeMs){
  DriveForwards();
  delay(timeMs);
  StopDriving();
}

void TurnLeftFor(unsigned long timeMs){
  TurnLeft();
  delay(timeMs);
  StopDriving();
}

void TurnRightFor(unsigned long timeMs){
  TurnRight();
  delay(timeMs);
  StopDriving();
}

void Stop(){
  StopDriving();
  DisableBothLEDs();
}

void UpdateHeadingFromEncoders(int turnSign) {
  long leftDelta  = leftEncoderCount  - previousLeftEncoderCount;
  long rightDelta = rightEncoderCount - previousRightEncoderCount;
  previousLeftEncoderCount  = leftEncoderCount;
  previousRightEncoderCount = rightEncoderCount;
  float avgDelta = (leftDelta + rightDelta) / 2.0;
  headingDeg += turnSign * avgDelta * DEGREES_PER_PULSE;
  while (headingDeg < -180) headingDeg += 360;
  while (headingDeg >  180) headingDeg -= 360;
}

// RD Functions
// Reads one ultrasonic sensor and returns distance in cm 

float ReadUltrasonicDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000); // timeout after 30ms

  if (duration == 0)
  {
    return -1;  // Invalid reading, return error value
  }

  float distance = duration / 58.0;  // Convert pulse duration to cm

  // Check if the distance is within a reasonable range (0 - 400 cm)
  if (distance <= 0 || distance > 400)
  {
    return -1;  // Invalid distance, return error
  }

  return distance;  // Return valid distance in cm
}

 

// Updates left and right distance values 

void UpdateDistanceReadings() 

{ 

 left_distance = ReadUltrasonicDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN); 

 right_distance = ReadUltrasonicDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN); 

} 

 

// Updates obstacle booleans 

void UpdateObstacleFlags() 

{ 

 if(left_distance > 0 && left_distance < object_threshold) 

 { 

 obstacle_left = true; 

 } 

 else 

 { 

 obstacle_left = false; 

 } 

 

 if(right_distance > 0 && right_distance < object_threshold) 

 { 

 obstacle_right = true; 

 } 

 else 

 { 

 obstacle_right = false; 

 } 

} 

 

// Counts left encoder pulses 

void LeftEncoderISR() 

{ 

 leftEncoderCount++; 

} 

 

// Counts right encoder pulses 

void RightEncoderISR() 

{ 

 rightEncoderCount++; 

} 

 

// Simple position update 

void UpdatePosition() 

{ 

 long leftChange = leftEncoderCount - previousLeftEncoderCount; 
 long rightChange = rightEncoderCount - previousRightEncoderCount; 

 previousLeftEncoderCount = leftEncoderCount; 
 previousRightEncoderCount = rightEncoderCount; 

 float avgChange = (leftChange + rightChange) / 2.0;  // use 2.0 to avoid integer division
 float distance_Cm = avgChange * CM_PER_PULSE;

 float headingRad = headingDeg * pi / 180.0;

 x_position += distance_Cm * cos(headingRad); 
 y_position += distance_Cm * sin(headingRad);

} 

float angle_to_target () 

{
float dx = target_x - x_position;
float dy = target_y - y_position;
return atan2(dy, dx) *(180 / pi);
}

 

// Runs all sensor/input updates 

void UpdateSensorsAndInputs() 

{ 

 UpdateDistanceReadings(); 

 UpdateObstacleFlags(); 

} 

// Start of Main Loop

void loop () {
  
if (CURRENT_STATE == STATE_COMPLETED) {
  StopDriving();
  EnableBothLEDs();
  SetLEDColour(0, 0, BRIGHT_PWM);
  return; // exit loop() immediately, nothing below runs
}

  // update sensor values and input values
  UpdateSensorsAndInputs ();
  
  // define current time
  unsigned long currentTime = millis ();
  
   switch (CURRENT_STATE) {
    
    case STATE_START:
    	StopDriving (); 
    	DisableBothLEDs ();

headingDeg = angle_to_target();

    	if (target_reached ()) {
          
     	 Changestate(STATE_COMPLETED);
         
    	}
    
    	else {
          
     	 Changestate (STATE_MOVE_FORWARD);
          
    	}
   	 	break;
    
    // This case is for driving to the target
    case STATE_MOVE_FORWARD:
   { 	
     	UpdatePosition(); 
      DrivingLEDs (); // turns on LEDs to green
     
    	if (target_reached ()) 
        {
          StopDriving (); 
          Changestate(STATE_COMPLETED);
        }

    	else if (obstacle_detected()) 
        { 
          StopDriving (); 
          flashStartTime = currentTime;
          Changestate(STATE_AVOID_OBSTACLE);
        }
      // checks to see if the robot is heading off course
    else 
    {
          float targetAngle = angle_to_target ();
          float angleDiff = targetAngle - headingDeg;
          while (angleDiff < -180) angleDiff += 360;
          while (angleDiff > 180)  angleDiff -= 360;
        
          if (abs(angleDiff) > 15) 
        {
          StopDriving ();
          Changestate (STATE_REORIENT);
        }

          else 
        {
          DriveForwards ();
        }
    }
   }
    	break;
    
    case STATE_TURN_LEFT:
{
  ShowLeftTurnLED();
  float angleDiff = avoidTargetHeading - headingDeg;
  while (angleDiff < -180) angleDiff += 360;
  while (angleDiff >  180) angleDiff -= 360;

  if (abs(angleDiff) > 5) {
    TurnLeft();
    UpdateHeadingFromEncoders(-1);
  }
  else {
    StopDriving();
    Changestate(STATE_BYPASS_DRIVE);
  }
  break;
}

case STATE_TURN_RIGHT:
{
  ShowRightTurnLED();
  float angleDiff = avoidTargetHeading - headingDeg;
  while (angleDiff < -180) angleDiff += 360;
  while (angleDiff >  180) angleDiff -= 360;

  if (abs(angleDiff) > 5) {
    TurnRight();
    UpdateHeadingFromEncoders(+1);
  }
  else {
    StopDriving();
    Changestate(STATE_BYPASS_DRIVE);
  }
  break;
}

  case STATE_REORIENT:
  {
    unsigned long timeInState = currentTime - stateStartTime;

    // briefly pause so encoders reset
  if (timeInState < 200) {
    StopDriving();
    previousLeftEncoderCount  = leftEncoderCount;
    previousRightEncoderCount = rightEncoderCount;
    break;
  }


  if (obstacle_detected() && timeInState > REORIENT_OBSTACLE_SUPPRESS_MS) {
    StopDriving();
    Changestate(STATE_AVOID_OBSTACLE);
    break;
}
  DrivingLEDs ();

  float targetAngle = angle_to_target ();
  float angleDiff = targetAngle - headingDeg;

  // normalise the angles to -180 and +180 degrees values
  while (angleDiff < -180) angleDiff += 360;
  while (angleDiff > 180) angleDiff -= 360;

  //checks if the robot is within an acceptable degree dispacement from the target.
  if (abs(angleDiff) < 5) {
    StopDriving ();
    Changestate (STATE_MOVE_FORWARD);
  }
  else if (angleDiff > 0) {
  TurnRight();
  UpdateHeadingFromEncoders(+1);
}
else {
  TurnLeft();
  UpdateHeadingFromEncoders(-1);
}
  }
  break;

  case STATE_AVOID_OBSTACLE:
  StopDriving();

  if ((currentTime / FLASH_INTERVAL_MS) % 2 == 0) RedLED();
  else DisableBothLEDs();

  if (currentTime - stateStartTime >= FLASH_INTERVAL_MS * 4) {
    lockedTurnDirection = direction_of_turning();
    avoidTargetHeading = headingDeg + (lockedTurnDirection * 90.0);
    while (avoidTargetHeading < -180) avoidTargetHeading += 360;
    while (avoidTargetHeading >  180) avoidTargetHeading -= 360;
    if (lockedTurnDirection < 0) Changestate(STATE_TURN_LEFT);
    else                         Changestate(STATE_TURN_RIGHT);
  }
  break;

  case STATE_BYPASS_DRIVE:
{
  DrivingLEDs();

  unsigned long timeInState = currentTime - stateStartTime;
  const unsigned long MIN_BYPASS_MS =670;

  if (timeInState < MIN_BYPASS_MS) {
    DriveForwards();
  }
  else if (!obstacle_detected()) {
    StopDriving ();
    Changestate (STATE_REORIENT);
  }
  else {
    // obstacle is not detected any longer
    DriveForwards ();
  }
  break;
}

    case STATE_COMPLETED:
    
    	StopDriving ();
     
     	EnableBothLEDs ();
     
     	SetLEDColour (0, 0, BRIGHT_PWM);
    
    	break;
  }
}
