#include <DynamixelSerial1.h>
#include <math.h>

#define L 100 // length of the base in mm
#define l 120 // length of the moving platform in mm

// Servo IDs
#define SERVO_ID1 1
#define SERVO_ID2 2
#define SERVO_ID3 3

// Dynamixel baud rate
#define BAUDRATE 57600

// coordinates of the moving platform center
double x, y, z;

// coordinates of the actuator's ends
double a[3], b[3], c[3];

// lengths of the actuators
double d1, d2, d3;

// positions of the servos (in degrees)
int pos1, pos2, pos3;

void setup() {
  // Start the Dynamixel serial port
  Dynamixel.begin(BAUDRATE, Serial1);
  
  // Enable torque on the servos
  Dynamixel.torqueEnable(SERVO_ID1);
  Dynamixel.torqueEnable(SERVO_ID2);
  Dynamixel.torqueEnable(SERVO_ID3);
}

void loop() {
  // Let's assume we want to move the platform to position (50, 50, 50)
  x = 50;
  y = 50;
  z = 50;

  // Calculate the coordinates of the actuator's ends
  a[0] = L * cos(0);
  b[0] = L * sin(0);
  c[0] = 0;
  
  a[1] = L * cos(2 * M_PI / 3);
  b[1] = L * sin(2 * M_PI / 3);
  c[1] = 0;
  
  a[2] = L * cos(4 * M_PI / 3);
  b[2] = L * sin(4 * M_PI / 3);
  c[2] = 0;

  // Calculate the lengths of the actuators
  d1 = sqrt(pow(x - a[0], 2) + pow(y - b[0], 2) + pow(z - c[0], 2));
  d2 = sqrt(pow(x - a[1], 2) + pow(y - b[1], 2) + pow(z - c[1], 2));
  d3 = sqrt(pow(x - a[2], 2) + pow(y - b[2], 2) + pow(z - c[2], 2));

  // Convert the lengths to angles (in degrees)
  // Assuming that the home position of the actuator corresponds to 90 degrees and every mm of extension corresponds to 1 degree
  pos1 = 90 + (d1 - l);
  pos2 = 90 + (d2 - l);
  pos3 = 90 + (d3 - l);

  // Move the servos to the calculated positions
  Dynamixel.moveSpeed(SERVO_ID1, pos1, 100);
  Dynamixel.moveSpeed(SERVO_ID2, pos2, 100);
  Dynamixel.moveSpeed(SERVO_ID3, pos3, 100);

  delay(2000);
}
