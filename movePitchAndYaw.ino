#include <Stepper.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution for your motor

// initialize the Stepper library on pins 8 through 11:
Stepper myYawStepper(stepsPerRevolution, 2,5);
//corresponds to X on cnc shield
Stepper myPitchStepper(stepsPerRevolution, 3,6);
//3 corresponds to Y on CNC Shield

const float stepsPerRad = 31.8309;

int flag = 1;
bool yawDone = false;
bool pitchDone = false;
int motorSpeed = 50;
//int desiredYaw = 10;
//int desiredPitch = 0;

void setup() {
  // initialize the serial port:
  Serial.begin(9600);
  myYawStepper.setSpeed(50);
  myPitchStepper.setSpeed(900);

  Serial.println("Orientation Sensor Test"); Serial.println("");

  // Initialize sensor
  if (!bno.begin()) {
    Serial.println("No sensor detected");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}



void loop() {
  //get message over pythons serial
  while(Serial.available()==0) {
    
  }
  String msg = Serial.readStringUntil('\r'); // "23021517";
  int count = 0;
  int p_len = msg[count] - '0';
  count =+ 1;
  String pitch_string = "";
  while (count < p_len + 1) {
    pitch_string += msg[count];
    count += 1;
  }
  int y_len = msg[count] - '0';
  String yaw_string = "";
  count += 1;
  while (count < y_len + p_len + 2) {
    yaw_string += msg[count];
    count += 1;
  }
  int v_len = msg[count] - '0';
  String vel_string = "";
  count += 1;
  while (count < v_len +y_len + p_len + 3) {
    vel_string += msg[count];
    count += 1;
  }

  int desiredPitch = pitch_string.toInt();
  int desiredYaw = yaw_string.toInt();
  int launchVel = vel_string.toInt();


  //Rotate motors according to pitch string, yaw string


  
  sensors_event_t event;
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // initial config 
  Serial.print("Initial Configuration");
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print("Y: ");
  Serial.print(euler.y());
  Serial.print("Z: ");
  Serial.println(euler.z());
  desiredYaw = desiredYaw + euler.y();
  desiredPitch = desiredPitch + euler.x();
  // get serial desiredYaw and DesiredPitch
  

  // Move Yaw angle
  Serial.println("Moving Yaw");
  while (!yawDone) {
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    Serial.print(euler.x());
    Serial.print("Y: ");
    Serial.print(euler.y());
    Serial.print("Z: ");
    Serial.println(euler.z( ));
    myYawStepper.step(-1);
    if (euler.x() > desiredYaw) {
      break;   
    }
  }

  // Move Pitch angle
  Serial.println("Moving Pitch");
  int sign = 1; // down
  float init_pitch = euler.y();
  Serial.print("Init pitch "); Serial.print(init_pitch);Serial.println(" ");
  
  if (desiredPitch < init_pitch) {
    sign = -1;  // up
    Serial.print("In pitch");
    int steps = 0;
    while (true) {
      myPitchStepper.setSpeed(900);
      myPitchStepper.step(sign);
      if (steps % 200 == 0) {
        euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        if (euler.y() < desiredPitch) {
          break;   
        }
      }
      steps += 1;
     }
  } else{
    int steps = 0;
    while (true) {
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      myPitchStepper.setSpeed(900);
      myPitchStepper.step(sign);
      if (steps % 200 == 0) {
        euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        if (euler.y() > desiredPitch) {
          break;   
        }
      } 
      steps += 1;
    }
  }

  while(true) {}
   
}
    
   
