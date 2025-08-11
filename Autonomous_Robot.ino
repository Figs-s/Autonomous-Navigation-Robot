#include <Encoder.h>   // Encoder library
#include <PID_v1.h>    // PID library
#include <SimpleKalmanFilter.h>
#include <Servo.h>
Servo servo1;
Servo servo2;
// Define sensor pins
const byte SENSOR_EN1 = 22;    // Sensor GPIO
const byte SENSOR_RIGHT = A0;  // Sensor Vout for right
const byte SENSOR_EN2 = 24;    // Sensor GPIO
const byte SENSOR_FRONT = A1;  // Sensor Vout for front
const byte SENSOR_EN3 = 26;    // Sensor GPIO
const byte SENSOR_LEFT = A2;   // Sensor Vout for left

SimpleKalmanFilter kalmanFilterRight(0.1, 0.1, 0.01);
SimpleKalmanFilter kalmanFilterFront(0.1, 0.1, 0.01);
SimpleKalmanFilter kalmanFilterLeft(0.1, 0.1, 0.01);

const int motorPin1 = 5;       // PWM pin for left motor
const int motorPin2 = 6;       // PWM pin for right motor
const int direction1 = 7;      // Direction pin for left motor
const int direction2 = 4;      // Direction pin for right motor
const int leftEncoderPinA = 2; // Encoder A for left motor
const int leftEncoderPinB = 3; // Encoder B for left motor
const int rightEncoderPinA = 8;// Encoder A for right motor
const int rightEncoderPinB = 9;// Encoder B for right motor

// Encoder counts
volatile long leftMotorCount = 0;
volatile long rightMotorCount = 0;

// PID variables
double Setpointw, Inputw, Outputw;
double Kpw = 1.2, Kiw = 1.5, Kdw = 1.5;
PID myPIDwall(&Inputw, &Outputw, &Setpointw, Kpw, Kiw, Kdw, DIRECT);

double Setpoint, Input, Output;
  double Kp=2, Ki=0.65, Kd=0.5;
  PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


const int targetTicksmaze = 850;//1700 This is extremely important !!!
const int targetTicksA=1450;//Extremly important x2 !!!
int turnscounts=0;

// Time variables for movement
long previousMillis = 0;
long currentMillis = 0;

// Filtered sensor values
float filteredValueRight, filteredValueFront, filteredValueLeft;
float rawValueLeft;

// Encoder interrupt routines
void leftEncoderISR() {
  int b = digitalRead(leftEncoderPinB);
  if (b > 0) {
    leftMotorCount++;
  } else {
    leftMotorCount--;
  }
}

void rightEncoderISR() {
  int b = digitalRead(rightEncoderPinB);
  if (b > 0) {
    rightMotorCount++;
  } else {
    rightMotorCount--;
  }
}

void setup() 
{
  // Turn the PID on
  myPIDwall.SetMode(AUTOMATIC);
  myPIDwall.SetOutputLimits(-50, 50);   // Limit the output to control turning strength
  myPIDwall.SetSampleTime(10);

  // Set motor and encoder pins as inputs/outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(direction1, OUTPUT);
  pinMode(direction2, OUTPUT);
  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);
  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);
    //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-250,250);
  myPID.SetSampleTime(10);
  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);
  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), rightEncoderISR, RISING);

  // Set sensor enable pins as outputs and activate them
  pinMode(SENSOR_EN1, OUTPUT);
  pinMode(SENSOR_EN2, OUTPUT);
  pinMode(SENSOR_EN3, OUTPUT);
  digitalWrite(SENSOR_EN1, HIGH);
  digitalWrite(SENSOR_EN2, HIGH);
  digitalWrite(SENSOR_EN3, HIGH);

  servo1.attach(10);
  servo2.attach(12);

  Serial.begin(9600);
}

void readSensors() {
  // Read and filter sensor values
  int rawValueRight = analogRead(SENSOR_RIGHT);
  int rawValueFront = analogRead(SENSOR_FRONT);
  int rawValueLeft = analogRead(SENSOR_LEFT);

  filteredValueRight = kalmanFilterRight.updateEstimate(rawValueRight);
  filteredValueFront = kalmanFilterFront.updateEstimate(rawValueFront);
  filteredValueLeft = kalmanFilterLeft.updateEstimate(rawValueLeft);

}

void ForwardWithWallFollowing() {
 readSensors();
      // Calculate the error for left wall following
      Inputw = filteredValueLeft;

      // Compute PID correction
       Setpointw = 680 ;  
      myPIDwall.Compute();  // Calculate correction (Output)

        // Base speed for left motor
       int baseSpeed = 200;

       // Adjust motor speeds
        int leftMotorSpeed = baseSpeed + Outputw;                // Left motor stays constant
        int rightMotorSpeed = baseSpeed - Outputw;      // Right motor adjusts based on Output

      // Ensure motor speeds are within valid range (0–255)
        leftMotorSpeed = constrain(leftMotorSpeed, 5, 250);
        rightMotorSpeed = constrain(rightMotorSpeed, 5, 250);
        digitalWrite(direction1, LOW);  // Left motor forward
        digitalWrite(direction2, LOW);  // Right motor forward
        analogWrite(motorPin1, leftMotorSpeed);  // Left motor PWM
        analogWrite(motorPin2, rightMotorSpeed); // Right motor PWM
        if (filteredValueLeft<150)
  {
    while(filteredValueLeft<150)
    {
        readSensors();
        analogWrite(motorPin1, 100);  // Left motor PWM
        analogWrite(motorPin2, 250); // Right motor PWM
        delay(500);
     }
    }
  
}
void forward(){
Input=fabs(leftMotorCount)-rightMotorCount;
  Setpoint = 0;  
  myPID.Compute();

  digitalWrite(direction1,LOW);
  digitalWrite(direction2,LOW);
 int motorSpeed1 = constrain(250 + Output, 0, 255);
analogWrite(motorPin1, motorSpeed1);
  analogWrite(motorPin2, 250 - Output);
  
  // Serial.print("Input = ");
  // Serial.println(Input);

  // Serial.print("Output = ");
  // Serial.println(Output);

//   Serial.print(Output); Serial.print(",");
//   Serial.println(Input);
//   Serial.print("Left Motor Count: ");
// Serial.println(leftMotorCount);

}
void rightTurnA() {
  // Reset encoder counts
  leftMotorCount = 0;
  rightMotorCount = 0;  

  // Set motor directions for a right turn (one motor forward, one backward)
  digitalWrite(direction1, LOW);   // Left motor forward
  digitalWrite(direction2, HIGH);  // Right motor backward
  analogWrite(motorPin1, 200);     // Left motor speed
  analogWrite(motorPin2, 200);     // Right motor speed

  // Rotate until the encoder count reaches target ticks
  while (rightMotorCount < targetTicksA) {
  }

  // Stop both motors after completing the turn
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  delay(500);  // Small delay to stabilize after turn
}

void rightTurnmaze() {
  // Reset encoder counts
  leftMotorCount = 0;
  rightMotorCount = 0;  

  // Set motor directions for a right turn (one motor forward, one backward)
  digitalWrite(direction1, LOW);   // Left motor forward
  digitalWrite(direction2, HIGH);  // Right motor backward
  analogWrite(motorPin1, 200);     // Left motor speed
  analogWrite(motorPin2, 200);     // Right motor speed

  // Rotate until the encoder count reaches target ticks
  while (rightMotorCount < targetTicksmaze) {
  }

  // Stop both motors after completing the turn
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  delay(500);  // Small delay to stabilize after turn
}
void leftTurnmaze() {
  // Reset encoder counts
  leftMotorCount = 0;
  rightMotorCount = 0;  

  // Set motor directions for a right turn (one motor forward, one backward)
  digitalWrite(direction1, HIGH);   // Left motor forward
  digitalWrite(direction2, LOW);  // Right motor backward
  analogWrite(motorPin1, 200);     // Left motor speed
  analogWrite(motorPin2, 200);     // Right motor speed

  // Rotate until the encoder count reaches target ticks
  while (rightMotorCount > -targetTicksmaze) {
  }

  // Stop both motors after completing the turn
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  delay(500);  // Small delay to stabilize after turn
}

int mazecount=0;

void loop() {    
   servo2.write(-20);
   servo1.write(18);
   servo2.detach();
    readSensors();
    if (turnscounts<1)
    {
      analogWrite(motorPin1, 250);     // Left motor speed
      analogWrite(motorPin2, 250);     // Right motor speed 
      if(filteredValueFront>=685)
      {
        readSensors();
        rightTurnA();
        analogWrite(motorPin1, 0);     // Left motor speed
        analogWrite(motorPin2, 0);  
        turnscounts=+1;  
      }
    }  
    else if (turnscounts>=1&&turnscounts<40)
    {
      readSensors();
      ForwardWithWallFollowing();
      mazecount+=1;
      readSensors();
      if(filteredValueFront>=680)
       {
         rightTurnmaze();
       }
       //Here enters maze
       readSensors();
       if (filteredValueLeft>=670 && filteredValueRight<=200 && mazecount>=100)
       {
          rightTurnmaze();
          turnscounts=45;  
       }
    }
    //go straight
  else if (turnscounts==45)
  {
    do{
      readSensors();
      digitalWrite(direction1, LOW);   // Left motor forward
      digitalWrite(direction2, LOW);  // Right motor backward
      analogWrite(motorPin1, 250);     // Left motor speed
      analogWrite(motorPin2, 250);     // Right motor speed
      if(filteredValueLeft<=600)
      {
        ForwardWithWallFollowing();
      }
    }while(filteredValueFront<=488);
    rightTurnmaze();
    //check for dropping block
    do{
      readSensors();
      digitalWrite(direction1, LOW);   // Left motor forward
      digitalWrite(direction2, LOW);  // Right motor backward
      analogWrite(motorPin1, 250);     // Left motor speed
      analogWrite(motorPin2, 250);     // Right motor speed
    }while(filteredValueFront<=470);
    servo2.attach(12);
    analogWrite(motorPin1, 0);     // Left motor speed
    analogWrite(motorPin2, 0);     // Right motor speed
    servo2.write(120);
    delay(700);
    servo1.write(140);
    delay(700);
    servo2.write(0);
    delay(700);
    servo1.write(10);
    servo2.detach(); // Set pin 9 to LOW
    delay(1000);
    //moving backwards after dropping
    do{
      readSensors();
      digitalWrite(direction1, HIGH);   // Left motor forward
      digitalWrite(direction2, HIGH);  // Right motor backward
      analogWrite(motorPin1, 250);     // Left motor speed
      analogWrite(motorPin2, 250);     // Right motor speed
    }while(filteredValueFront>=488);
    leftTurnmaze();
    turnscounts=46;
  }
  //package dropped already
  else if(turnscounts==46)
  {
    mazecount=0;
    do {
      readSensors();
      digitalWrite(direction1, HIGH);   // Left motor forward
      digitalWrite(direction2, HIGH);  // Right motor backward
      analogWrite(motorPin1, 250);     // Left motor speed
      analogWrite(motorPin2, 250);     // Right motor speed
    }while(filteredValueFront<=275);
    leftTurnmaze();
    //forward to enter second line
    do{
      readSensors();
      digitalWrite(direction1, LOW);   // Left motor forward
      digitalWrite(direction2, LOW);  // Right motor backward
      analogWrite(motorPin1, 250);     // Left motor speed
      analogWrite(motorPin2, 250);     // Right motor speed
    }while(filteredValueFront>=680);
    leftTurnmaze();
    //in second line
    do {
      readSensors();
      digitalWrite(direction1, HIGH);   // Left motor forward
      digitalWrite(direction2, HIGH);  // Right motor backward
      analogWrite(motorPin1, 250);     // Left motor speed
      analogWrite(motorPin2, 250);     // Right motor speed
      if(filteredValueLeft<=600)
      {
        ForwardWithWallFollowing();
      }
    }while(filteredValueFront<=470);
    rightTurnmaze();
    //To pick up package
    do{
      readSensors();
      digitalWrite(direction1, LOW);   // Left motor forward
      digitalWrite(direction2, LOW);  // Right motor backward
      analogWrite(motorPin1, 250);     // Left motor speed
      analogWrite(motorPin2, 250);     // Right motor speed
    }while(filteredValueFront<=470);
    //Time to pick up the package
    servo2.attach(12);
    analogWrite(motorPin1, 0);     // Left motor speed
    analogWrite(motorPin2, 0);     // Right motor speed
    servo2.write(120);
    delay(700);
    servo1.write(140);
    delay(700);
    servo2.write(0);
    delay(700);
    servo1.write(10);
    servo2.detach(); // Set pin 9 to LOW
    delay(1000);
    //going back after picking up 
    do{
      readSensors();
      digitalWrite(direction1, HIGH);   // Left motor forward
      digitalWrite(direction2, HIGH);  // Right motor backward
      analogWrite(motorPin1, 250);     // Left motor speed
      analogWrite(motorPin2, 250);     // Right motor speed
    }while(filteredValueFront>=488);
    leftTurnmaze();
    turnscounts=47;
  }

  else if(turnscounts==47)
  {
    //straight with package
    do 
    {
      ForwardWithWallFollowing();
    }while(filteredValueFront<=685);
    leftTurnmaze();
    //almost at exit 
    do{
      readSensors();
      digitalWrite(direction1, LOW);   // Left motor forward
      digitalWrite(direction2, LOW);  // Right motor backward
      analogWrite(motorPin1, 250);     // Left motor speed
      analogWrite(motorPin2, 250);     // Right motor speed
    }while(filteredValueRight>=408);
    //Turning to get out 
    analogWrite(motorPin1, 0);     // Left motor speed
    analogWrite(motorPin2, 0);     // Right motor speed
    rightTurnmaze();
    //Straight for out
     do{
      readSensors();
      digitalWrite(direction1, LOW);   // Left motor forward
      digitalWrite(direction2, LOW);  // Right motor backward
      analogWrite(motorPin1, 250);     // Left motor speed
      analogWrite(motorPin2, 250);     // Right motor speed
    }while(filteredValueFront<=680);
    leftTurnmaze();
    do 
    {
      ForwardWithWallFollowing();
      mazecount+=1;

    }while(filteredValueFront<=685);
    readSensors();
    if(filteredValueLeft<=200 &&mazecount==100)
    {
      analogWrite(motorPin1, 0);     // Left motor speed
      analogWrite(motorPin2, 0);     // Right motor speed
    }
  }
  }

