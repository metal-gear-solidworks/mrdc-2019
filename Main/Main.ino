//#include <Arduino.h>
//#include <Arm.h>
//#include <DriveBase.h>
//#include <Intake.h>
#include <Servo.h>
#define baudrate 9600   // the baudrate for comms, has to match the baudrate of the driverstation
#define time_out 500    // the number of milliseconds to wait after recieving signal before calling failsafe

byte feedback[10];
byte controller[8];
byte data[8];

boolean connection;
boolean badPacket;
byte x;
byte packet_index;
byte i;
byte size1;
byte checkSumTX;    // check sum for transmitting data
byte checkSumRX;    // check sum for recieving data

byte driveThrottle; //drive variables
byte revThrottle;
byte driveHeading;

unsigned int _rStickX;

static double headingMod = 1.6;
bool quickTurn;

bool doorTrigger;   //intake variables
bool releaseTrigger;
bool intakeTrigger;
bool reverseTrigger;
bool prevState = false;
bool releasePrev = false;
bool releaseState = false;
bool piston_out = false;
bool firstTime = true;

int _DoorOne;
int _DoorTwo;
int _ReleaseOne;
int _ReleaseTwo;

byte armVal;    //arm variables

unsigned long read_time;

#define DriveML1 2
#define DriveML2 3
#define DriveMR1 4
#define DriveMR2 5
#define rollers 6
#define lift 8
#define armMotor 7

#define doorSolenoid 16
#define releaseSolenoid 17

//unused
#define doorSolenoidRev 18
#define releaseSolenoidRev 19
//#define compressorPin 30

  Servo DriveL1;
  Servo DriveL2;
  Servo DriveR1;
  Servo DriveR2;

  Servo intake1;
  Servo intake2;

  Servo ArmMotor;

void failsafe(){
    // write the code below that you want to run
    // when the robot loses a signal here
    firstTime = false;
    driveBaseFailsafe();
    IntakeFailsafe();
    armFailsafe();
    connection = false;
}

void setup(){
    //declare the Serial1 port for comms
    //the paramater of the begin function is the baudrate
    Serial.begin(9600);
    Serial1.begin(9600);
    // initialize the variables to 0
    memset(controller,0,sizeof(controller));
    memset(feedback,0,sizeof(feedback));
    connection = true;

    //subsystem initialization
    DriveBaseInit(DriveML1, DriveML2, DriveMR1, DriveMR2);
    IntakeInit(rollers, lift, doorSolenoid, doorSolenoidRev, releaseSolenoid, releaseSolenoidRev);
    ArmInit(armMotor);
    failsafe();
    read_time = millis();
    checkSumRX = 0;
    x = 0;
    packet_index = 0;
}

void loop(){
    // this while block of code might not need the "packet_index == 0" condition
    // it causes the robot to be more tolerant of old data which can be bad
    // you might want to deleting that condition
    connection = false;
    while(packet_index == 0 && Serial1.available() >= 22){
        Serial1.read();
    }

    size1 = Serial1.available();
    while(size1 > 0){
        if(packet_index == 0){
            if(Serial1.read()==255){
//              Serial.println("Valid lead");
                packet_index++;
            }
//            else Serial.println("Invalid lead");
        }
        else if(packet_index < 9){
            data[packet_index-1] = Serial1.read();
            checkSumRX += data[packet_index-1];
            packet_index++;
        }
        else if(packet_index == 9){
            if(Serial1.read() == checkSumRX){
                packet_index++;
            }else{
                packet_index=0;
            }
            checkSumRX = 0;
        }
        else if(packet_index == 10){
            if(Serial1.read() == 240){
//              Serial.println("Valid end packet");
                for(i=0; i<8; i++){
                    controller[i] = data[i];
                }
                connection = true;
                 read_time = millis();
                 firstTime = true;
                 mainCode();
            }
//            else Serial.println("Invalid end packet");
            packet_index=0;
        }
        size1--;
    }
    if((firstTime && millis() - read_time >= time_out)){
        failsafe();
    }
}
void mainCode()
{
          // write the code below that you want to run
        // when the robot recieves valid data of the xbox controller
        // basically all the motor control stuff
        
        driveThrottle = controller[7];
        revThrottle = controller[6];
        driveHeading = controller[2];
        updateDrive(driveThrottle, revThrottle, driveHeading);

        intakeTrigger = (B1 == ((controller[0] & B1)));
        reverseTrigger = (B1 == ((controller[0] & B10) >> 1));
        doorTrigger = (B1 == ((controller[0] & B1000) >> 3));
        releaseTrigger = (B1 == ((controller[0] & B100) >> 2));
        
        runIntake(reverseTrigger, intakeTrigger, doorTrigger, releaseTrigger);
        
        armVal = controller[5];

        setArm(armVal);

        // below is the code for sending feedback to the driver station

        Serial1.write(255);
        checkSumTX = 0;
        for(i=0; i<10; i++){
            Serial1.write(feedback[i]);
            checkSumTX += feedback[i];
        }
        Serial1.write(checkSumTX);
        Serial1.write(240);
}

//DriveBase subsystem
void DriveBaseInit(int leftPWM1, int leftPWM2, int rightPWM1, int rightPWM2)
{
//subsystem setup, assigns pin number and initializes pin
  DriveL1.attach(leftPWM1);
  DriveL2.attach(leftPWM2);
  DriveR1.attach(rightPWM1);
  DriveR2.attach(rightPWM2);
}

void setQuickTurn(double throttle, double heading)
{
  if((fabs(throttle) < 5) && (fabs(heading) > 5)) quickTurn = true;
  else quickTurn = false;  
}

void setThrottle(int _lStickY, int _rStickX)
{
   int lSpeed;
   int lSpeedMag;
   int rSpeed;
   double throttle = abs(_lStickY-90) * (_lStickY < 90 ? -1 : 1);    //// = magnitude & direction of throttle
   double heading = abs(_rStickX-90) * (_rStickX < 90 ? -1 : 1);    //// = magnitude and direction of heading

   double angularPower = fabs(throttle/90)*(heading)*(headingMod);

   setQuickTurn(throttle, heading);

   if(quickTurn) 
   {
   lSpeed = throttle + heading + 90;
   rSpeed = throttle - heading + 90;
   }
   else
   {
   lSpeed = throttle + angularPower + 90;
   rSpeed = throttle - angularPower + 90; 
   }
//constrains motor values to within the PWM limits
if(lSpeed > 180) lSpeed = 180;
if(rSpeed > 180) rSpeed = 180;
if(lSpeed < 0) lSpeed = 0;
if(rSpeed < 0) rSpeed = 0;
//sends value to speed controller
lSpeedMag = abs(90 - lSpeed);
lSpeedMag *= 13;
lSpeedMag /= 20;
if(lSpeed > 90) lSpeed = 90 + lSpeedMag;
else lSpeed = 90 - lSpeedMag;

  feedback[0] = lSpeed;
  feedback[1] = rSpeed;
  lSpeed = 180 - lSpeed;
  rSpeed = 180 - rSpeed;
  
  DriveL1.write(lSpeed);
  DriveL2.write(lSpeed);
  DriveR1.write(rSpeed);
  DriveR2.write(rSpeed);
}

void updateDrive(byte throttle, byte reverse, byte rStickX)
{
//run every cycle to set drive motor value and piston state
//l/r triggers assigns throttle, rStickX assigns rotation,
int _throttle = (throttle >> 1);
_throttle *= 180;
_throttle /= 100;

int _reverse = (reverse >> 1);
_reverse *= 180;
_reverse /= 100;

int throttleMag = _throttle - _reverse;
  throttleMag += 90;

_rStickX = rStickX;

_rStickX *= 180;
_rStickX /= 200;

//deadband control to prevent non-significant power output
  if(throttleMag > 85 && throttleMag < 95) throttleMag = 90;
  if(_rStickX > 85 && _rStickX < 95) _rStickX = 90;
  setThrottle(throttleMag, _rStickX);
}

void driveBaseFailsafe()
{
  DriveL1.write(90);
  DriveL2.write(90);
  DriveR1.write(90);
  DriveR2.write(90);
}


//intake subsystem
void IntakeInit(int PWM, int PWMTwo, int pistonOne, int pistonTwo, int pistonThree, int pistonFour)
{
//subsystem setup, assigns pin number and initializes pin
  
  int _PWM = PWM;
  int _PWM2 = PWMTwo;
  _DoorOne = pistonOne;
  _DoorTwo = pistonTwo;
  _ReleaseOne = pistonThree;
  _ReleaseTwo = pistonFour;
  pinMode(_DoorOne, OUTPUT);//pin for piston control
  pinMode(_DoorTwo, OUTPUT);
  pinMode(_ReleaseOne, OUTPUT);
  pinMode(_ReleaseTwo, OUTPUT);
  intake1.attach(_PWM);
  intake2.attach(_PWM2);
}

void setIntakeSpeed(int state)
{
//for input -1, 0, 1, sets speed to full forward, full reverse, or off

  int IntakeSpeed1 = (70 * state) + 90;
  int IntakeSpeed2 = 90 - (40*state);

//sends value to speed controller
  feedback[2] = IntakeSpeed1;
  intake1.write(IntakeSpeed1);
  intake2.write(IntakeSpeed2);
}

void runDoorPiston(bool piston)//accepts piston as state of button
{
  if(piston) feedback[3] = 1;
  else feedback[3] = 0;
    digitalWrite(_DoorOne, piston?HIGH:LOW);//send 'pressed' to _PISTON
    digitalWrite(_DoorTwo, piston?LOW:HIGH);//opposite of paired solenoid
}

void runReleasePiston(bool piston)
{
  if(piston) feedback[5] = 1;
  else feedback[5] = 0;
    digitalWrite(_ReleaseOne, piston?HIGH:LOW);//send 'pressed' to _PISTON
    digitalWrite(_ReleaseTwo, piston?LOW:HIGH);//opposite of paired solenoid
}

void runIntake(bool lTrigger, bool rTrigger, bool doorPiston, bool releasePiston)
{
//if left trigger held, run intake out
//if right trigger held, run intake in
//right trigger takes priority
if(rTrigger)
{
  setIntakeSpeed(1);//run intake in
}
else if(lTrigger)
{
  setIntakeSpeed(-1);//run intake out
}
else setIntakeSpeed(0);//do nothing

if(doorPiston && doorPiston != prevState)
{
  runDoorPiston(!piston_out);
  piston_out = !piston_out;
  prevState = doorPiston;
}
else if(!doorPiston)
{
  prevState = doorPiston;
}

if(releasePiston && releasePiston != releasePrev)
{
  runReleasePiston(!releaseState);
  releaseState = !releaseState;
  releasePrev = releasePiston;
}
else if(!doorPiston)
{
  releasePrev = releasePiston;
}
}

void IntakeFailsafe()
{
  intake1.write(90);
  intake2.write(90);
}


//arm subsystem
void ArmInit(int armPWM)
{
//subsystem setup, assigns pin number and initializes pin

  int _PWM1 = armPWM;

  ArmMotor.attach(_PWM1);
}

void setArmSpeed(byte _speed)
{
//for input -1, 0, 1, sets speed to half forward, half reverse, or off
long motorSpeed = _speed;

motorSpeed *= 180;
motorSpeed /= 200;

  if(abs(90 - motorSpeed) < 5) motorSpeed = 90;

  int magnitude = abs(90 - motorSpeed);
  magnitude *=3;
  magnitude /= 10;
  if(motorSpeed > 90) motorSpeed = 90 + magnitude;
  else motorSpeed = 90 - magnitude;

//sends value to speed controller
  feedback[4] = motorSpeed;
  ArmMotor.write(motorSpeed);
}

void setArm(byte _speed)
{
//setArmSpeed((lTrigger >> 1), (rTrigger >> 1));
setArmSpeed(_speed);
}

void armFailsafe()
{
  ArmMotor.write(90);
}
