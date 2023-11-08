/* Demo code for reading the encoders of the robot.

  Note: this code requires the following libraries (install them through the library manager):
     - SparkFun I2C Mux Arduino Library
     - AS5600 library
*/
#define TELEMETRY true

// Include the current library
#include "MecatroUtils.h"

// Include the AS5600 library (for the encoders) and Sparkfun I2C Mux (for multiplexer)
#include "AS5600.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"
#include "sensorbar.h"
// Header for I2C communication
#include "Wire.h"

#include "LibLog.h"

#define BAUDRATE 230400
// Define the control loop period, in ms.
#define CONTROL_LOOP_PERIOD 5

// Define the Multiplexer pins corresponding to each encoder
#define LEFT_ENCODER_PIN 7
#define RIGHT_ENCODER_PIN 6
#define LINE_FOLLOWER_PIN 1

QWIICMUX multiplexer;
AS5600 rightEncoder, leftEncoder;

// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

SensorBar mySensorBar(SX1509_ADDRESS);

#include "LibDerivative.h"
Buffer leftBufCumul(10);
Buffer leftBufRaw(10);
Buffer leftBuf2Derive(20);

void setup()
{
  // Setup serial communication with the PC - for debugging and logging.
  Serial.begin(BAUDRATE);
  // Start I2C communication
  Wire.begin();
  // Set I2C clock speed to 400kHz (fast mode)
  Wire.setClock(400000);

  // Init multiplexer
  if (!multiplexer.begin())
  {
    println("Error: I2C multiplexer not found. Check wiring.");
  }
  else
  {
    bool isInit = true;

    // Set multiplexer to use port LEFT_ENCODER_PIN to talk to left encoder.
    multiplexer.setPort(LEFT_ENCODER_PIN);
    leftEncoder.begin();
    if (!leftEncoder.isConnected())
    {
      println("Error: could not connect to left encoder. Check wiring.");
      isInit = false;
    }
    
    // Set multiplexer to use port RIGHT_ENCODER_PIN to talk to right encoder.
    multiplexer.setPort(RIGHT_ENCODER_PIN);
    rightEncoder.begin();
    if (!rightEncoder.isConnected())
    {
      println("Error: could not connect to right encoder. Check wiring.");
      isInit = false;
    }

    // Set multiplexer to use port LINE_FOLLOWER_PIN to talk to lines follower.
    multiplexer.setPort(LINE_FOLLOWER_PIN);

    //For this demo, the IR will only be turned on during reads.
    mySensorBar.setBarStrobe();
    //Other option: Command to run all the time
    //mySensorBar.clearBarStrobe();

    //Default dark on light
    mySensorBar.clearInvertBits();
    //Other option: light line on dark
    //mySensorBar.setInvertBits();
    
    //Don't forget to call .begin() to get the bar ready.  This configures HW.
    uint8_t returnStatus = mySensorBar.begin();
    if(returnStatus)
    {
      println("sx1509 IC communication OK");
    }
    else
    {
      println("sx1509 IC communication FAILED!");
      while(1);
    }
    println();

  // Initialize telemetry
  unsigned int const nVariables = 3;
  String variableNames[nVariables] = {"rRawAngle" , "rCumulativePos", "rAngularSpeed"};
  mecatro::initTelemetry(nVariables, variableNames);
  
  // Configure motor control and feedback loop call.
  mecatro::configureArduino(CONTROL_LOOP_PERIOD);
  }
}


void loop()
{
  // Don't forget to call this, otherwise nothing will happen !
  // This function never returns, put all your code inside mecatro::controlLoop.
  mecatro::run();
}

/*double abs(double v)
{
  if(v < 0.0) return -v;
  return v;
}*/

double cap(double v){
  if(v < 0.0) return 0.0;
  if(v > 1.0) return 1.0;
  return v;
}
// This function is called periodically, every CONTROL_LOOP_PERIOD ms.
// Put all your code here.
void mecatro::controlLoop()
{
  // Set multiplexer to use port 0 to talk to right encoder.
  multiplexer.setPort(RIGHT_ENCODER_PIN);
  println();
  print("Right encoder: raw angle ");
  // Raw encoder measurement - from 0 to 360 degrees
  print(rightEncoder.rawAngle() * AS5600_RAW_TO_DEGREES);

  // Software feature: the encoder itself does not measure multi-turn information nor rotation speed.
  // These features are thus implemented as software, taking two consequtive measurements and computing
  // their difference.
  // This of course assumes that the encoder has performed less than half of a turn between two calls (otherwise there
  // is no way to know how many turns were performed, or in which direction).
  // This is not a problem here: with a typical update rate of 5ms in this function, the maximum speed would be 60000rpm !
  print("°, cumulative position ");
  print(rightEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES);
  //print("° speed ");
  //print(rightEncoder.getAngularSpeed());
  //print("°/s ");

  // Check magnet positioning - this is for debug purposes only and is not required in normal operation.
  if (rightEncoder.magnetTooStrong())
  {
    print(" ; warning: magnet too close.");
  }
  if (rightEncoder.magnetTooWeak())
  {
    print(" ; warning: magnet too far.");
  }
  println();

  // Set multiplexer to use port LEFT_ENCODER_PIN to talk to left encoder.
  multiplexer.setPort(LEFT_ENCODER_PIN);
  leftBufCumul.push(leftEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES);
  leftBufRaw.push(leftEncoder.rawAngle() * AS5600_RAW_TO_DEGREES);

  print("Left encoder: ");
  print(leftBufRaw.last());
  print("°, cumulative position ");
  print(leftBufCumul.last());
  //print("° speed ");
  //print(leftEncoder.getAngularSpeed());
  //print("°/s ");

  // Check magnet positioning - this is for debug purposes only and is not required in normal operation.
  if (leftEncoder.magnetTooStrong())
  {
    print(" ; warning: magnet too close.");
  }
  if (leftEncoder.magnetTooWeak())
  {
    print(" ; warning: magnet too far.");
  }
  println();

  leftBuf2Derive.push(leftBufCumul.derivative(CONTROL_LOOP_PERIOD));
  // The first argument is the variable (column) id ; recall that in C++, numbering starts at 0.
  mecatro::log(0,  leftEncoder.rawAngle() * AS5600_RAW_TO_DEGREES * 12.0/360); 
  mecatro::log(1,  0.0);//leftBuf.last()); 
  mecatro::log(2, leftBuf2Derive.mean());
  //mecatro::log(2,  leftEncoder.getAngularSpeed()); 

  //Get the data from the sensor bar and load it into the class members
  multiplexer.setPort(LINE_FOLLOWER_PIN);
  uint8_t rawValue = mySensorBar.getRaw();
  
  //Print the binary value to the serial buffer.
  print("Bin value of input: ");
  for( int i = 7; i >= 0; i-- )
  {
    print((rawValue >> i) & 0x01);
  }
  println("b");

  //Print the hex value to the serial buffer.  
  print("Hex value of bar: 0x");
  if(rawValue < 0x10) //Serial.print( , HEX) doesn't pad zeros. Do it here
  {
	  //Pad a 0;
	  print("0");
  }
  //println(rawValue, HEX);
  
  //Print the position and density quantities
  print("Position (-127 to 127): ");
  println(mySensorBar.getPosition());
  print("Density, bits detected (of 8): ");
  println(mySensorBar.getDensity());
  println("");
  
  //Wait 2/3 of a second
  //delay(666); //do not do that


  // Keep the motor off, i.e. at 0 duty cycle (1 is full forward, -1 full reverse)
  double averageSpeed = 0.3;
  double delta = mySensorBar.getPosition();
  delta *= 2.0*averageSpeed/127.0;
  double diffLeft = -1.0;
  double diffRight = 1.0; //opposed signs
  

  mecatro::setMotorDutyCycle(cap(averageSpeed + diffLeft*delta), cap(averageSpeed + diffRight*delta));
}