/*
   7MRI0060 - Applied Medical Robotics Module
   October 2024
   Author: Alejandro Granados and Harry Robertshaw
 
   Purpose: Run a PID controller
 
   Tasks:
    1. Set pins to code
    2. Compute degrees and bound them to [-360,360]
    3. Compute elapsed time (deltaT) and control the frequency of printing
    4. Set demand position in degrees and compute errors (compared to previous, accumulation, and differential).
       Hint: make sure you update previous error at the end.
    5. Compute PID
    6. Send voltage to motors
    7. Plot demand versus current position in degrees
*/
 
// Define constants for pulses per revolution (PPR) and gear ratio (GR)
const float PPR = 3575.0855;
const float GR = 297.924;
const float CPR = 3; //encoder counts per revolution ie pulses per revolution
 
// Variables for tracking encoder positions
volatile long counter_m1 = 0;
volatile long counter_m2 = 0;
int aLastState_m1; //Last state of encoders from pin A motor 1(pin 2) --> Used to identify whether there is movement of the motor
int aLastState_m2; //Last state of encoders from pin A motor 2(pin 3) --> Used to identify whether there is movement of the motor
 
// Pins for reading encoders of motor 1 and 2
const int encoderPinA_m1 = 2;
const int encoderPinB_m1 = 10;
const int encoderPinA_m2 = 3;
const int encoderPinB_m2 = 11;
 
// Pins for setting the direction of motor 1 and 2
const int motorPin1_m1 = 4;//#4; //Comeback here and revise positions of the motorpins
const int motorPin2_m1 = 5;//5;
const int motorPin1_m2 = 7;
const int motorPin2_m2 = 8;
 
// Pins for setting the speed of rotation (Enable pin) of motors 1 and 2
const int enablePin_m1 = 6; //Comeback to verify the wirings
const int enablePin_m2 = 9;

//Declare
int i = 0;                // counter
String matlabStr = "";    // receives the string from matlab, it is empty at first
bool readyToSend = false; // flag to indicate a command was received and now ready to send back to matlab
bool logs = false;

char c;                   // characters received from matlab
float val1 = 0.0;         // input1 from matlab
float val2 = 0.0;         // input2 from matlab

long currentPosition_m1 = 0; //Current position of motor 1 according to counter
long currentPosition_m2 = 0; //Current position of motor 2 according to counter
float currentPositionInDegrees_m1; //Current position of motor 1 in degrees based on currentPosition_m1 variable
float currentPositionInDegrees_m2; //Current position of motor 2 in degrees based on currentPosition_m2 variable
  
// Time parameters to calculate speed of program execution
unsigned long currentTime;
unsigned long previousTime = 0;
unsigned long deltaT;
  
// PID gains to optimise accuracy and reduce error
float Kp_m1 = 20, Kd_m1 = 0, Ki_m1 = 0;
float Kp_m2 = 20, Kd_m2 = 0, Ki_m2 = 0;
  
// Error values --> Initialised for use later on in the program
float errorPositionInDegrees_prev_m1 = 0, errorPositionInDegrees_sum_m1 = 0;
float errorPositionInDegrees_prev_m2 = 0, errorPositionInDegrees_sum_m2 = 0;

void setup() {
  
  Serial.begin(9600);
  
  // Task 1: Initialize the pins using pinMode and attachInterrupt functions
  pinMode(encoderPinA_m1, INPUT_PULLUP);
  pinMode(encoderPinB_m1, INPUT_PULLUP);
  pinMode(encoderPinA_m2, INPUT_PULLUP);
  pinMode(encoderPinB_m2, INPUT_PULLUP);
  
  pinMode(motorPin1_m1, OUTPUT);
  pinMode(motorPin2_m1, OUTPUT);
  pinMode(enablePin_m1, OUTPUT);
  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);
  pinMode(enablePin_m2, OUTPUT);
  
  
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m1), updateEncoder_m1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m2), updateEncoder_m2, CHANGE);
  
  aLastState_m1 = digitalRead(encoderPinA_m1);
  aLastState_m2 = digitalRead(encoderPinA_m2);
  
  delay(3000);
  previousTime = micros();
    
}

void loop() {
  // First wait to receive data from matlab to arduino
  if (readyToSend == false) {
    if (Serial.available()>0)       // is there anything received?
    {
      c = Serial.read();            // read characters
      matlabStr = matlabStr + c;    // append characters to string as these are received
      
      if (matlabStr.indexOf(";") != -1) // have we received a semi-colon (indicates end of command from matlab)?
      {
        readyToSend = true;         // then set flag to true since we have received the full command

        // parse incomming data, e.g. C40.0,3.5;
        int posComma1 = matlabStr.indexOf(",");                     // position of comma in string
        val1 = matlabStr.substring(1, posComma1).toFloat();         // float from substring from character 1 to comma position
        int posEnd = matlabStr.indexOf(";");                        // position of last character
        val2 = matlabStr.substring(posComma1+1, posEnd).toFloat();  // float from substring from comma+1 to end-1
      } 
    } //end of inner 
  } //end of larger loop
  
  // Task 2: Compute the current position in degrees and bound it to [-360,360]
  float demandPositionInDegrees_m1 = val1; //Position that we are inputting into motor 1 in degrees
  float demandPositionInDegrees_m2 = val2; //Position that we are inputting into motor 2 in degrees
  
  currentPositionInDegrees_m1 = ((counter_m1 * 360) / (CPR * GR * 2));
  
  if (currentPositionInDegrees_m1 >= 360.0 || currentPositionInDegrees_m1 <= -360.0) {
    counter_m1 -= ((GR * CPR * 2) * ((int)(currentPositionInDegrees_m1 / 360)));
  }
  
  currentPositionInDegrees_m2 = ((counter_m2 * 360) / (CPR * GR * 2));
  
  if (currentPositionInDegrees_m2 >= 360.0 || currentPositionInDegrees_m2 <= -360.0){
    counter_m2 -= ((GR * CPR * 2) * ((int)(currentPositionInDegrees_m2 / 360)));
  }
  
  // Task 3: Compute elapsed time (deltaT) and control the frequency of printing
  
  currentTime = micros();
  deltaT = currentTime - previousTime;
  previousTime = currentTime;
  
  if (logs == true) {
  Serial.println(" ");
  }
  // digitalWrite(motorPin1_m1, HIGH);
  
  if (deltaT > 400) {
    // Task 4: Compute error (P,I,D), and ensure that the previous error is updated to minimize error and optimize accuracy
    float errorPositionInDegrees_m1 = currentPositionInDegrees_m1 - demandPositionInDegrees_m1;
    float errorPositionInDegrees_diff_m1 = (errorPositionInDegrees_m1 - errorPositionInDegrees_prev_m1) / deltaT;
    errorPositionInDegrees_sum_m1 += errorPositionInDegrees_m1;
    errorPositionInDegrees_prev_m1 = errorPositionInDegrees_m1;
  
    float errorPositionInDegrees_m2 = currentPositionInDegrees_m2 - demandPositionInDegrees_m2;
    float errorPositionInDegrees_diff_m2 = (errorPositionInDegrees_m2 - errorPositionInDegrees_prev_m2) / deltaT;
    errorPositionInDegrees_sum_m2 += errorPositionInDegrees_m2;
    errorPositionInDegrees_prev_m2 = errorPositionInDegrees_m2;
  
  
    // Task 5: Compute the PID output
    float controllerOutput_m1 = errorPositionInDegrees_m1 * Kp_m1 + errorPositionInDegrees_diff_m1 * Kd_m1 + errorPositionInDegrees_sum_m1 * Ki_m1 * deltaT;
    controllerOutput_m1 = constrain(controllerOutput_m1, -255, 255); //Restricting the amount of voltage that can be sent to the motors
  
    float controllerOutput_m2 = errorPositionInDegrees_m2 * Kp_m2 + errorPositionInDegrees_diff_m2 * Kd_m2 + errorPositionInDegrees_sum_m2 * Ki_m2 * deltaT;
    controllerOutput_m2 = constrain(controllerOutput_m2, -255, 255); //Restricting the amount og voltage that can be sent to the motors
      
    // Serial.print(controllerOutput_m1);
    // Serial.println(controllerOutput_m2);
    // Task 6: Send voltage to motors
    if (controllerOutput_m1 > 0){
      digitalWrite(motorPin1_m1, HIGH);
      digitalWrite(motorPin2_m1, LOW);
      analogWrite(enablePin_m1, controllerOutput_m1);
    } else {
      digitalWrite(motorPin1_m1, LOW);
      digitalWrite(motorPin2_m1, HIGH);
      analogWrite(enablePin_m1, -controllerOutput_m1);
    }
  
    if (controllerOutput_m2 > 0){
      digitalWrite(motorPin1_m2, HIGH);
      digitalWrite(motorPin2_m2, LOW);
      analogWrite(enablePin_m2, controllerOutput_m2);
    } else {
      digitalWrite(motorPin1_m2, LOW);
      digitalWrite(motorPin2_m2, HIGH);
      analogWrite(enablePin_m2, -controllerOutput_m2);
    }
  
    // Task 7: Print the current position and demanded position in degrees for plotting
    if (logs == true) {
    Serial.print(currentPositionInDegrees_m1);
    Serial.print(" ");
    Serial.println(demandPositionInDegrees_m1);
  
    Serial.print(currentPositionInDegrees_m2);
    Serial.print(" ");
    Serial.println(demandPositionInDegrees_m2);
    }
      
  }
  // Then, start sending data from Ardiono to matlab
  if (readyToSend)  // arduino has received command form matlab and now is ready to send
  {
    // e.g. c1,100
    Serial.print("c");                    // command
    Serial.print(val1);                      // series 1
    Serial.print(",");                    // delimiter
    Serial.print(val2); // series 2
    Serial.write(13);                     // carriage return (CR)
    Serial.write(10);                     // new line (NL)
    i += 1;
  }
}

  
  // Interrupt functions for tracking the encoder positions
void updateEncoder_m1() {
  // Code to update counter_m1 based on the state of the encoder pins
  aLastState_m1 = counter_m1;
  int aState_m1 = digitalRead(encoderPinA_m1);
  int bState_m1 = digitalRead(encoderPinB_m1);
  if (aState_m1 != aLastState_m1){
    if (bState_m1 != aState_m1){
      counter_m1 += 1;
    } else {
      counter_m1 -= 1;
    }
    aLastState_m1 = aState_m1;
  }    
}
  
void updateEncoder_m2() {
  // Code to update counter_m2 based on the state of the encoder pins
  aLastState_m2 = counter_m2;
  int aState_m2 = digitalRead(encoderPinA_m2);
  int bState_m2 = digitalRead(encoderPinB_m2);
  if (aState_m2 != aLastState_m2){
    if (bState_m2 != aState_m2){
      counter_m2 += 1;
    } else {
      counter_m2 -= 1;
    }
    aLastState_m2 = aState_m2;
  }
}



 

 