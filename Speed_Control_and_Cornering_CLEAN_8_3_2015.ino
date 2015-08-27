// *********************
// Libraries
// *********************
#include <QTRSensors.h> //for Sensor Array
#include <SPI.h> //for motor encoders
#include <Sabertooth.h>// for the motor controllers
#include <SoftwareSerial.h> //for serial communications

// *********************
// Define hardware pins
// *********************
#define sabertoothEstop 4 // Used in allStop() command, basically a fail-safe emergency break.

//Declaring the sensor info
#define NUM_SENSORS   8     // number of sensors used by pololu
#define TIMEOUT       2500  // waits for 2500 milliseconds for sensor outputs to go low
#define EMITTER_PIN   12    // emitter is controlled by digital pin 12


// SENSOR STUFF - adapated from code written by Pololu, -------------------------
//available HERE: https://www.pololu.com/product/961/resources

// sensors 0 through 7 are connected to digital pins 22-36 EVEN, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  22, 24, 26, 28, 30, 32, 34, 36
}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

//ENCODER STUFF - adapted from code written by SuperDroid,-------------------
//link above
// Slave Select pins for encoders 1 and 2
// Feel free to reallocate these pins to best suit your circuit
const int slaveSelectEnc1 = 7;
const int slaveSelectEnc2 = 8;

// These hold the current encoder count.
signed long encoder1count = 0;
signed long encoder2count = 0;
//accounting for variation between the two encoders, we'll pray that an average is good enough
signed long encoderCountAvg = 0;


//Motor stuff - adapted from code written by SuperDroid Robots ------------------
//link above
// Declaration of the software serial UART and motor controller objects
SoftwareSerial SWSerial(2, 3); // RX on pin 2 (unused), TX on pin 3 (to S1).
//PIN 2 IS UNUSED IN THAT IT DOES NOT RECEIVE A SIGNAL BACK FROM THE MOTOR CONTROLLERS. THE ARDUINO ONLY TRANSMITS

Sabertooth frontSaber(128, SWSerial); // Address 128, front motors, 1 = left, 2 = right
Sabertooth rearSaber(129, SWSerial); // Address 129, rear motors, 1 = left, 2 = right
// check that ours is not switched -eb

//movement variables...
int movementDelay = 15; // Delay in between movement commands
int speedDelta = 64;
int topForwardSpeed = 127 + speedDelta; //"top" speeds here are arbitrary. speedDelta could be as high as
int topBackwardSpeed = 127 - speedDelta; // 127, making the topForward = 255 and topBackward = 0. Just sayin.


//---------------------------------
//variables for PID
float Kp = 1.7;  //Proportional Constant
float Ki = 0.04;   //Integral constant
int offset = 35;    //desired position
int Tp = 127;      //target power - if at desired position, z/x should = 127
float turnVariable = 0;
int error = 0;
float spin = 0;
float integral = 0;

//motor control vars------------------------
int idealYSpeed = 160; //straight forward speed.
int turnYSpeed = idealYSpeed - 20; //turning forward speed - slower as to not lose line
int ySpeed = idealYSpeed; //initial speed
int xSpeed = 127;
int zSpeed = 127;


//*********************************
//SETUP
//*********************************
void setup()
{
  delay(3000);    // Short delay to powerup the motor controllers

  Serial.begin(9600);    // Serial for the debug output
  SWSerial.begin(9600);  // Serial for the motor controllers
  //-eb can we change this to a faster baud rate?
  //BAUD RATE IS THE NUMBER OF SIGNALING EVENTS THAT OCCURS PER SECOND. IN OTHER WORDS, IT IS THE NUMBER OF CHANGES IN BINARY STATE PER SECOND

  frontSaber.autobaud(); // Allows the motor controllers to detect the baud rate
  rearSaber.autobaud();  // we're using and adjust appropriately

  // Initialize GPIO inputs and outputs
  //sets pin 4 to output so we can stop the motors
  pinMode(sabertoothEstop, OUTPUT);

  allStop();		// Make sure all motors are stopped for safety

  //getting the encoders up and running
  initEncoders();       Serial.println("Encoders Initialized...");
  clearEncoderCount();  Serial.println("Encoders Cleared...");

  //calibrating
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

  for (int j = 0; j < 3; j++)  // move back and forth 3 times
  {
    //start calibrating before we move
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    Serial.println("calibrating: Start offline on robot's left side");

    for (int i = 127; i < 167; i++) // using the for() loop allows for a smooth ramp up of speed
    { qtrrc.calibrate();           //start calibration - will be called 40 times, 10 reads per call = 400 reads
      commandMotors(127, i, 127, 1);  //move right
    }
    allStop();
    for (int i = 127; i > 87;  i--)
    { qtrrc.calibrate();           //more calibration - another 400 reads
      commandMotors(127, i, 127, 1);   //move left
    }
    allStop();
    qtrrc.calibrate();
  }                      //that's ~800 reads, 3 times, so 2400 readings...
  digitalWrite(13, LOW); // turn on Arduino's LED to indicate we are in calibration mode


  //printing calibration figures
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print('\t');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print('\t');
  }

  Serial.println();

  //This loop smooths out the beginning...
  //slowly vector towards the center of the line
  //while slowly accelerating to idealYSpeed.
  for (int i = 127; i < 160; i++) {
    unsigned int position = qtrrc.readLine(sensorValues);  //will this work here?!
    if (position > 3500) {
      xSpeed = xSpeed - 1;
    }
    if (position < 3500) {
      xSpeed = xSpeed + 1;
    } else {
      xSpeed = 127;
    }
    commandMotors(i, xSpeed, 127, 1);
  }

}

//*********************************
//*********************************
//LOOOOOP
//*********************************
//*********************************
void loop() {

  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000
  unsigned int position = qtrrc.readLine(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  Serial.println(position); // comment this line out if you are using raw values

  /** SPEEDOMETER -- DOES NOT WORK YET
  //calculate the encoder values
      encoder1count = readEncoder(1);
      encoder2count = readEncoder(2);
    int encoderAvg = ((encoder1count*-1)+encoder2count)/2;

    float feetTravelled;
    float ftPerSec;
    //calculate speed... 1920 encoder units= 1 ft
    feetTravelled = encoderAvg/1920;
    ftPerSec = feetTravelled/(millis()*1000);
    Serial.println(ftPerSec);
    **/

  //PID SHIT
  int location = position / 100; //getting the position and error down to a managable number
  error = offset - location; //calculate the error

  if (error < 5 && error > -5) { //reset the integral when on the line
    integral = 0;
  } else {
    integral = integral + error;   //calulate the integral
  }

  //calculate the angle to get back to line (spin)
  turnVariable = Kp * error + (Ki * integral); //get the PI-derived turnVariable
  spin = Tp + int(turnVariable); //adjust the spin based on Tp(127) and the turnVariable

  //*****************************************************************
  //MOVEMENT************
  //*****************************************************************

  //x,y, and z speed control--------------
  String movement;
  if (position < 7000 && position > 0) { //if we are on the line AT ALL
    if (abs(error) > 5) { //if we're turning really sharply, engage z-axis spin and slow down a little bit
      xSpeed = 127;
      zSpeed = spin;
      ySpeed = turnYSpeed;
      movement = "SPINNING";
    }
    xSpeed = spin;
    zSpeed = 127;
    ySpeed = idealYSpeed;
    movement = "VECTORING";
  } else if (position == 0 || position == 7000) { //if we've lost the line completely
    movement = "TURNING";
    if (position == 0) { //if line lost and last known position was to the right, turn right
      xSpeed = 127;   
      zSpeed = 191;  //191 = HARD RIGHT turn
      ySpeed = 127;
    } else {          //if line lost and last known position was to the left, turn left
      xSpeed = 127; 
      zSpeed = 64;   //64 = HARD RIGHT turn
      ySpeed = 127;
    }
  }
  Serial.println(movement);
  commandMotors(ySpeed, xSpeed, zSpeed, 1); //THE ONLY LINE OF CODE IN THIS WHOLE PROGRAM THAT REALLY DOES ANYTHING

  delay(250);

}//end loop

