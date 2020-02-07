//#include <Arduino.h>
//#include <Arm.h>
//#include <DriveBase.h>
//#include <Intake.h>
#include <Servo.h>
#define BAUDRATE 9600   // the BAUDRATE for comms, has to match the BAUDRATE of the driverstation
#define TIME_OUT 500    // the number of milliseconds to wait after recieving signal before calling FailSafe
#define SPEED 0.75      // 1 - 5
#define MOTOR_OFF 1500   // experimental off value for motors

byte feedback[10];
byte controller[8];
byte data[8];

// pin macros, make sure to set these
#define FRONT_LEFT_PWM 2
#define FRONT_RIGHT_PWM 3
#define REAR_LEFT_PWM 4
#define REAR_RIGHT_PWM 5

#define INTAKE_PWM 6

#define SHOOTER_PWM1 7
#define SHOOTER_PWM2 8

#define LIN_X_PIN 14
#define LIN_Y_PIN 15
#define LIN_B_PIN 16
#define LIN_A_PIN 17

// controller macros
#define 

// wireless communication constants
#define STARTING_PACKET_IDX 0

// wireless communication variables
boolean is_connected;
boolean bad_packet;
byte x;
byte packet_index;
byte i;
byte size1;
byte check_sum_tx;    // check sum for transmitting data
byte check_sum_rx;    // check sum for recieving data
bool first_time = true;
unsigned long read_time;

// drive motors
Servo drive_front_left; // front left wheel
Servo drive_front_right; // front right wheel
Servo drive_rear_left; // rear left wheel
Servo drive_rear_right; // rear right wheel

// shooter motors
Servo shooter_left;
Servo shooter_right;

// intake motor
Servo intake;

// drive variables
int left_throttle = 1500; // throttle for left side
int right_throttle = 1500; // throttle for right side

// intake variable
bool is_intake_trigger = false;

// shooter variable
bool is_shooter_trigger = false;

// linear actuator variables
bool is_lin_x = false; // actuator bound to X
bool is_lin_y = false; // actuator bound to Y
bool is_lin_b = false; // actuator bound to B
bool is_lin_a = false; // actuator bound to A

void FailSafe(){
    // write the code below that you want to run
    // when the robot loses a signal here
    first_time = false;
    DriveFailSafe();
    IntakeFailSafe();
    LinearActuatorFailSafe();
    is_connected = false;
}

void DriveFailSafe() {
    drive_front_left.writeMicroseconds(MOTOR_OFF);
    drive_rear_left.writeMicroseconds(MOTOR_OFF);
    drive_front_right.writeMicroseconds(MOTOR_OFF);
    drive_rear_right.writeMicroseconds(MOTOR_OFF);
}

void LinearActuatorFailSafe() {
    digitalWrite(LIN_A_PIN, LOW);
    digitalWrite(LIN_B_PIN, LOW);
    digitalWrite(LIN_X_PIN, LOW);
    digitalWrite(LIN_Y_PIN, LOW);
}

void IntakeFailSafe() {
    intake.write(90);
}

void setup(){
    //declare the Serial1 port for comms
    //the paramater of the begin function is the BAUDRATE
    Serial.begin(BAUDRATE);
    Serial1.begin(BAUDRATE);
    // initialize the variables to 0
    memset(controller,0,sizeof(controller));
    memset(feedback,0,sizeof(feedback));
    is_connected = true;

    //subsystem initialization
    InitDrive(FRONT_LEFT_PWM, REAR_LEFT_PWM, FRONT_RIGHT_PWM, REAR_RIGHT_PWM);
    InitShooter(SHOOTER_PWM1, SHOOTER_PWM2);
    InitIntake(INTAKE_PWM);
    InitLinearActuators(LIN_A_PIN, LIN_B_PIN, is_lin_x_PIN, LIN_Y_PIN);
    FailSafe();
    read_time = millis();
    check_sum_rx = 0;
    x = 0;
    packet_index = 0;
}

void loop(){
    // this while block of code might not need the "packet_index == 0" condition
    // it causes the robot to be more tolerant of old data which can be bad
    // you might want to delete that condition
    is_connected = false;
    while (packet_index == 0 && Serial1.available() >= 22) {
        Serial1.read();
    }

    size1 = Serial1.available();
    while (size1 > 0) {
        if (packet_index == STARTING_PACKET_IDX) {
            if (Serial1.read() == 255) {
    //          Serial.println("Valid lead");
                packet_index++;
            }
      //    else Serial.println("Invalid lead");
        }
        else if (packet_index < 9) {
            data[packet_index-1] = Serial1.read();
            check_sum_rx += data[packet_index-1];
            packet_index++;
        }
        else if (packet_index == 9) {
            if (Serial1.read() == check_sum_rx) {
                packet_index++;
            } else {
                packet_index = 0;
            }
            check_sum_rx = 0;
        }
        else if (packet_index == 10) {
            if (Serial1.read() == 240) {
    //          Serial.println("Valid end packet");
                for (i = 0; i < 8; i++) {
                    controller[i] = data[i];
                }
                is_connected = true;
                read_time = millis();
                first_time = true;
                MainCode();
            }
      //    else Serial.println("Invalid end packet");
            packet_index = 0;
        }
        size1--;
    }
    if((first_time && millis() - read_time >= TIME_OUT)){
        FailSafe();
    }
}

// main code executed upon successful I/O
void MainCode() {
    UpdateDrive(controller[3], controller[5]);
    //UpdateShooter(B1 == ((controller[0] & B100000) >> 5)); // right bumper, set via janky bitwise stuff
    UpdateIntake(B1 == ((controller[0] & B10000) >> 4), B1 == ((controller[0] & B100000) >> 5)); // left bumper, through similar jank
    UpdateLinearActuators(B1 == ((controller[0] & B1)),
        B1 == ((controller[0] & B10) >> 1),
        B1 == ((controller[0] & B100) >> 2),
        B1 == ((controller[0] & B1000) >> 3));
}

// updates state of drive motors
void UpdateDrive(byte left_y, byte rightY) {
    // this might not work
    int left_prop = (int)left_y;
    int right_prop = (int)right_y;

    int right_prop_inverted = (int)right_y;
    int left_prop_inverted = (int)left_y;

    left_prop = map(left_prop, 0, 200, 1500 - 100*SPEED, 1500 + 100*SPEED);
    right_prop = map(right_prop, 0, 200, 1500 - 100*SPEED, 1500 + 100*SPEED);

    left_prop_inverted = map(left_prop_inverted, 0, 200, 1500 + 100*SPEED, 1500 - 100*SPEED);
    right_prop_inverted = map(right_prop_inverted, 0, 200, 1500 + 100*SPEED, 1500 - 100*SPEED);

    drive_front_left.writeMicroseconds(left_prop_inverted);
    drive_rear_left.writeMicroseconds(left_prop);
    drive_front_right.writeMicroseconds(right_prop_inverted);
    drive_rear_right.writeMicroseconds(right_prop);
}

// drive init function
void InitDrive(int left_PWM1, int left_PWM2, int right_PWM1, int right_PWM2) {
    drive_front_left.attach(left_PWM1);
    drive_rear_left.attach(left_PWM2);
    drive_front_right.attach(right_PWM1);
    drive_rear_right.attach(right_PWM2);

    drive_front_left.writeMicroseconds(MOTOR_OFF);
    drive_rear_left.writeMicroseconds(MOTOR_OFF);
    drive_front_right.writeMicroseconds(MOTOR_OFF);
    drive_rear_right.writeMicroseconds(MOTOR_OFF);
    
    delay(2000);
}

// updates state of shooter motors
void UpdateShooter(bool is_right_bumper) {
    intake.writeMicroseconds(is_right_bumper ? 2000 : MOTOR_OFF);
}

// shooter init function
void InitShooter(int left_PWM, int right_PWM) {
    shooter_left.attach(left_PWM);
    shooter_right.attach(right_PWM);
}

// updates state of intake motors
void UpdateIntake(bool is_left_bumper, bool is_right_bumper) {
  if (is_left_bumper ^ is_right_bumper) {
    if (is_left_bumper) {
      intake.writeMicroseconds(1000);
    }
    else if(is_right_bumper) {
      intake.writeMicroseconds(2000);
    }
  }
  else {
    intake.writeMicroseconds(MOTOR_OFF);
  }
}

// intake init function
void InitIntake(int PWM) {
    intake.attach(PWM);
}

// updates state of linear actuators
void UpdateLinearActuators(bool A, bool B, bool X, bool Y) {
    digitalWrite(LIN_A_PIN, A ? HIGH : LOW);
    digitalWrite(LIN_B_PIN, B ? HIGH : LOW);
    digitalWrite(is_lin_x_PIN, X ? HIGH : LOW);
    digitalWrite(LIN_Y_PIN, Y ? HIGH : LOW);
}

// linear actuator init function
void InitLinearActuators(int A, int B, int X, int Y) {
    pinMode(A, OUTPUT);
    pinMode(B, OUTPUT);
    pinMode(X, OUTPUT);
    pinMode(Y, OUTPUT);
}
