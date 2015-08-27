
/**
All of the folloring was written by SuperDroidRobots, obtained by us as part of
a package with the pre-fabricated robot base, as seen HERE: http://www.superdroidrobots.com/shop/item.aspx/programmable-mecanum-wheel-vectoring-robot-ig32-sb/1713/
**/


//******************************************************************************
// Sets the speed of all motor controllers to zero and sets all ESTOPs
// RETURNS: NONE
//******************************************************************************
void allStop()
{
  digitalWrite(sabertoothEstop, LOW);

  frontSaber.motor(1, 0);
  frontSaber.motor(2, 0);
  rearSaber.motor(1, 0);
  rearSaber.motor(2, 0);
}



//******************************************************************************
//commandMotors
// Processes all motor commands.
// The function expects three values;
//    yAxis is our forward and back movement
//    xAxis is our left and right movement
//    spin is our third axis
//    mode is the type of movement we're sending (tank versus vectoring)
//
// To understand the input of this function you need to envision a grid with 0,0
// in the bottom left, 127,127 in the center and 255,255 in the top right. The location
// of the data point passed into this function relative to the 127,127 center point
// is the direction that the robot is commanded to vector
//
// RETURNS: NONE
//******************************************************************************
void commandMotors(int yAxis, int xAxis, int spin, int mode)
{
  // Initialize our local variables
  int leftFrontPower = 0;
  int leftRearPower = 0;
  int rightFrontPower = 0;
  int rightRearPower = 0;
  int maxMotorPower = 0;
  double tempScale = 0;

  // Motor Constants
  int motorValueMax = 255;
  int motorValueMin = 0;
  int motorZero = 127;

  // Bound our incoming data to a safe and expected range
  if (yAxis > motorValueMax) {
    yAxis = motorValueMax;
  }
  else  if (yAxis < motorValueMin) {
    yAxis = motorValueMin;
  }
  if (xAxis > motorValueMax) {
    xAxis = motorValueMax;
  }
  else  if (xAxis < motorValueMin) {
    xAxis = motorValueMin;
  }
  if (spin > motorValueMax) {
    spin = motorValueMax;
  }
  else  if (spin < motorValueMin) {
    spin = motorValueMin;
  }

  // Shift incoming data to straddle 0
  yAxis = yAxis - 127;
  xAxis = xAxis - 127;
  spin = spin - 127;

  // A mode value of 1 passed into this function changes the motor mixing to
  // vectoring mode
  if (mode == 1)
  {
    // *************************
    // Front and Back Motion
    //all wheels equal power forward
    // *************************
    leftFrontPower = leftFrontPower + yAxis;
    leftRearPower = leftRearPower + yAxis;
    rightFrontPower = rightFrontPower + yAxis;
    rightRearPower = rightRearPower + yAxis;
    //    Serial.println("y calculation");
    //    Serial.print("LFP: "); Serial.println(leftFrontPower);
    //    Serial.print("LRP: "); Serial.println(leftRearPower);
    //      Serial.print("RFP: "); Serial.println(rightFrontPower);
    //        Serial.print("RRP: "); Serial.println(rightRearPower);
    // *************************
    // Left and Right Motion
    //front and rear wheels opposite sign
    //left and right opposite sign
    // *************************
    leftFrontPower = leftFrontPower + xAxis;
    leftRearPower = leftRearPower - xAxis;
    rightFrontPower = rightFrontPower - xAxis;
    rightRearPower = rightRearPower + xAxis;
    //        Serial.println("x calculation");
    //    Serial.print("LFP: "); Serial.println(leftFrontPower);
    //    Serial.print("LRP: "); Serial.println(leftRearPower);
    //      Serial.print("RFP: "); Serial.println(rightFrontPower);
    //        Serial.print("RRP: "); Serial.println(rightRearPower);

    // *************************
    // Spin
    // *************************
    leftFrontPower = leftFrontPower + spin;
    leftRearPower = leftRearPower + spin;
    rightFrontPower = rightFrontPower - spin;
    rightRearPower = rightRearPower - spin;
    //        Serial.println("spin calculation");
    //    Serial.print("LFP: "); Serial.println(leftFrontPower);
    //    Serial.print("LRP: "); Serial.println(leftRearPower);
    //      Serial.print("RFP: "); Serial.println(rightFrontPower);
    //        Serial.print("RRP: "); Serial.println(rightRearPower);

    // After our mixing above our motor powers are most likely going to exceed
    // our maximum values. We need to find our maximum and scale everything down
    // to values that our motor controller can understand
    maxMotorPower = max(abs(leftFrontPower), abs(leftRearPower));
    maxMotorPower = max(maxMotorPower, abs(rightFrontPower));
    maxMotorPower = max(maxMotorPower, abs(rightRearPower));

    // Scale down by the maximum value if we exceed 127
    if (maxMotorPower > 127)
    {
      tempScale = (double)127 / (double)maxMotorPower;
      leftFrontPower = tempScale * (double)leftFrontPower;
      leftRearPower = tempScale * (double)leftRearPower;
      rightFrontPower = tempScale * (double)rightFrontPower;
      rightRearPower = tempScale * (double)rightRearPower;
    }

    // Cleans up our output data
    leftFrontPower = boundAndDeadband(leftFrontPower);
    leftRearPower = boundAndDeadband(leftRearPower);
    rightFrontPower = boundAndDeadband(rightFrontPower);
    rightRearPower = boundAndDeadband(rightRearPower);

    // Raises the ESTOP lines before commanding the motors
    digitalWrite(sabertoothEstop, HIGH);

    //        Serial.println("FInal Values");
    //    Serial.print("LFP: "); Serial.println(leftFrontPower);
    //    Serial.print("LRP: "); Serial.println(leftRearPower);
    //      Serial.print("RFP: "); Serial.println(rightFrontPower);
    //        Serial.print("RRP: "); Serial.println(rightRearPower);
    // Applies our calculated and bounded values to our drive motor controllers
    frontSaber.motor(1, rightFrontPower);
    frontSaber.motor(2, leftFrontPower);
    rearSaber.motor(1, rightRearPower);
    rearSaber.motor(2, leftRearPower);
  }

  // If the mode value is not "1" then we are in tank mode
  else
  {
    // Applies our calculated and bounded values to our drive motor controllers
    frontSaber.drive(yAxis);
    frontSaber.turn(xAxis);

    rearSaber.drive(yAxis);
    rearSaber.turn(xAxis);
  }
}

//******************************************************************************
// Cleans up our values for the motor controllers
// The motor controllers only accept a value range of -127 to 127. We also apply
// a deadband so the robot doesn't drift when idle
//
// RETURNS: Cleaned up value
//******************************************************************************
int boundAndDeadband (int inputValue)
{
  if (inputValue < -127)  {
    inputValue = -127;
  }
  if (inputValue > 127)   {
    inputValue = 127;
  }
  if ((inputValue < 5) && (inputValue > -5)) {
    inputValue = 0;
  }

  return inputValue;
}
