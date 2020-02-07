//#include <Arduino.h>
//#include <Arm.h>
//#include <DriveBase.h>
//#include <Intake.h>
#include <Servo.h>
#define baudrate 9600   // the baudrate for comms, has to match the baudrate of the driverstation
#define time_out 500    // the number of milliseconds to wait after recieving signal before calling failsafe
#define SPEED 0.75 // 1 - 5

byte feedback[10];
byte controller[8];
byte data[8];

// pin macros, make sure to set these
#define frontLeftPWM 2
#define frontRightPWM 3
#define rearLeftPWM 4
#define rearRightPWM 5

#define intakePWM 6

#define shooterPWM1 7
#define shooterPWM2 8

#define linXPin 14
#define linYPin 15
#define linBPin 16
#define linAPin 17

// wireless communication variables
boolean connection;
boolean badPacket;
byte x;
byte packet_index;
byte i;
byte size1;
byte checkSumTX;    // check sum for transmitting data
byte checkSumRX;    // check sum for recieving data
bool firstTime = true;
unsigned long read_time;

// drive motors
Servo driveFrontLeft; // front left wheel
Servo driveFrontRight; // front right wheel
Servo driveRearLeft; // rear left wheel
Servo driveRearRight; // rear right wheel

// shooter motors
Servo shooterLeft;
Servo shooterRight;

// intake motor
Servo intake;

// drive variables
int leftThrottle = 1500; // throttle for left side
int rightThrottle = 1500; // throttle for right side

// intake variable
bool intakeTrigger = false;

// shooter variable
bool shooterTrigger = false;

// linear actuator variables
bool linX = false; // actuator bound to X
bool linY = false; // actuator bound to Y
bool linB = false; // actuator bound to B
bool linA = false; // actuator bound to A

void failsafe(){
    // write the code below that you want to run
    // when the robot loses a signal here
    firstTime = false;
    driveFailsafe();
    intakeFailsafe();
    linearActuatorFailsafe();
    connection = false;
}

void driveFailsafe() {
    driveFrontLeft.writeMicroseconds(1500);
    driveRearLeft.writeMicroseconds(1500);
    driveFrontRight.writeMicroseconds(1500);
    driveRearRight.writeMicroseconds(1500);
}

void linearActuatorFailsafe() {
    digitalWrite(linAPin, LOW);
    digitalWrite(linBPin, LOW);
    digitalWrite(linXPin, LOW);
    digitalWrite(linYPin, LOW);
}

void intakeFailsafe() {
    intake.write(90);
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
    initDrive(frontLeftPWM, rearLeftPWM, frontRightPWM, rearRightPWM);
    initShooter(shooterPWM1, shooterPWM2);
    initIntake(intakePWM);
    initLinearActuators(linAPin, linBPin, linXPin, linYPin);
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

// main code executed upon successful I/O
void mainCode() {
    updateDrive(controller[3], controller[5]);
    //updateShooter(B1 == ((controller[0] & B100000) >> 5)); // right bumper, set via janky bitwise stuff
    updateIntake(B1 == ((controller[0] & B10000) >> 4), B1 == ((controller[0] & B100000) >> 5)); // left bumper, through similar jank
    updateLinearActuators(B1 == ((controller[0] & B1)),
        B1 == ((controller[0] & B10) >> 1),
        B1 == ((controller[0] & B100) >> 2),
        B1 == ((controller[0] & B1000) >> 3));
}

// updates state of drive motors
void updateDrive(byte leftY, byte rightY) {
    // this might not work
    int leftProp = (int)leftY;
    int rightProp = (int)rightY;

    int rightPropInverted = (int)rightY;
    int leftPropInverted = (int)leftY;

    leftProp = map(leftProp, 0, 200, 1500 - 100*SPEED, 1500 + 100*SPEED);
    rightProp = map(rightProp, 0, 200, 1500 - 100*SPEED, 1500 + 100*SPEED);

    leftPropInverted = map(leftPropInverted, 0, 200, 1500 + 100*SPEED, 1500 - 100*SPEED);
    rightPropInverted = map(rightPropInverted, 0, 200, 1500 + 100*SPEED, 1500 - 100*SPEED);


   // Serial.println(leftProp);

    driveFrontLeft.writeMicroseconds(leftPropInverted);
    driveRearLeft.writeMicroseconds(leftProp);
    driveFrontRight.writeMicroseconds(rightPropInverted);
    driveRearRight.writeMicroseconds(rightProp);
}

// drive init function
void initDrive(int leftPWM1, int leftPWM2, int rightPWM1, int rightPWM2) {
    driveFrontLeft.attach(leftPWM1);
    driveRearLeft.attach(leftPWM2);
    driveFrontRight.attach(rightPWM1);
    driveRearRight.attach(rightPWM2);

    driveFrontLeft.writeMicroseconds(1500);
    driveRearLeft.writeMicroseconds(1500);
    driveFrontRight.writeMicroseconds(1500);
    driveRearRight.writeMicroseconds(1500);
    
    delay(2000);
}

// updates state of shooter motors
void updateShooter(bool rightBumper) {
    intake.writeMicroseconds(rightBumper ? 2000 : 1500);
}

// shooter init function
void initShooter(int leftPWM, int rightPWM) {
    shooterLeft.attach(leftPWM);
    shooterRight.attach(rightPWM);
}

// updates state of intake motors
void updateIntake(bool leftBumper, bool rightBumper) {
  if (leftBumper ^ rightBumper) {
    if (leftBumper) {
      intake.writeMicroseconds(1000);
    }
    else if(rightBumper) {
      intake.writeMicroseconds(2000);
    }
  }
  else {
    intake.writeMicroseconds(1500);
  }
}

// intake init function
void initIntake(int PWM) {
    intake.attach(PWM);
}

// updates state of linear actuators
void updateLinearActuators(bool A, bool B, bool X, bool Y) {
    digitalWrite(linAPin, A ? HIGH : LOW);
    digitalWrite(linBPin, B ? HIGH : LOW);
    digitalWrite(linXPin, X ? HIGH : LOW);
    digitalWrite(linYPin, Y ? HIGH : LOW);
}

// linear actuator init function
void initLinearActuators(int A, int B, int X, int Y) {
    pinMode(A, OUTPUT);
    pinMode(B, OUTPUT);
    pinMode(X, OUTPUT);
    pinMode(Y, OUTPUT);
}