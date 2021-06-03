#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include "printf.h"
#include <SFE_BMP180.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#define DEBUG true;
int strobeLightPin = 10;

MPU6050 mpu;

int ch_throttle = 0;
int ch_elevator = 0;
int ch_width_3 = 0;
int ch_aileron = 0;

Servo throttle;
Servo elevator;
Servo rudder;
Servo aileron;

struct Signal {
  byte throttle;
  byte pitch;
  byte roll;
  byte yaw;
};

Signal data;
const uint64_t pipeIn = 0xE9E8F0F0E1LL;
RF24 radio(7, 8);

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);


SFE_BMP180 pressure;

double baseline;

double getPressure() {
  char status;
  double T, P, p0, a;
  status = pressure.startTemperature();
  if (status != 0) {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0) {
      status = pressure.startPressure(3);
      if (status != 0) {
        delay(status);
        status = pressure.getPressure(P, T);
        if (status != 0) {
          return (P);
        } else
          Serial.println("error retrieving pressure measurement\n");
      } else
        Serial.println("error starting pressure measurement\n");
    } else
      Serial.println("error retrieving temperature measurement\n");
  } else
    Serial.println("error starting temperature measurement\n");
}

void resetData() {
  data.throttle = 127;
  data.pitch = 127;
  data.roll = 127;
  data.yaw = 127;
}

void initializeRadio() {
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}
  } else {
    Serial.println(F("radio started!"));

  }
  resetData();
  //radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  radio.openReadingPipe(1, pipeIn);
  radio.startListening();
}

void initializeI2CDevices() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  
  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();
  Serial.println(F("Testing connection..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  mpu.setXAccelOffset(-1184);
  mpu.setYAccelOffset(-1025);
  mpu.setZAccelOffset(2140);
  mpu.setXGyroOffset(44);
  mpu.setYGyroOffset(-5);
  mpu.setZGyroOffset(50);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  if (!mag.begin()) {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1) ;
  }
}

unsigned long strobeIntervals[] = {25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 2000};
const byte STROBE_INTERVALS = sizeof(strobeIntervals) / sizeof(unsigned long);
byte strobe_currentInterval = 0;

unsigned long strobe_previousMillis = 0;

void initializeStrobeLight() {
  pinMode(strobeLightPin, OUTPUT);
  digitalWrite(strobeLightPin, HIGH);
}

void runStrobeLight() {
  unsigned long currentMillis = millis();
  if (currentMillis - strobe_previousMillis >= strobeIntervals[strobe_currentInterval]) {
    strobe_currentInterval = strobe_currentInterval + 1;
    if (strobe_currentInterval >= STROBE_INTERVALS)
    
      strobe_currentInterval = 0;
    digitalWrite(strobeLightPin, not digitalRead(strobeLightPin));
    strobe_previousMillis = currentMillis;
  }
}

void setup() {
  Serial.begin(115200);
  throttle.attach(2);
  elevator.attach(3);
  //rudder.attach();
  aileron.attach(4);
  
  while (!Serial);

  initializeRadio();
  initializeI2CDevices();  
  initializeStrobeLight();

}

unsigned long lastReceivedTime = 0;

void receiveData() {
  while ( radio.available() ) {
    radio.read(&data, sizeof(Signal));    
    lastReceivedTime = millis();
    Serial.print(data.throttle);
    Serial.print("\t");
    Serial.print(data.pitch);
    Serial.print("\t");
    Serial.print(data.roll);
    Serial.print("\t");
    Serial.println(data.yaw);
  }
}

void readMagnometer() {
    // read a packet from FIFO
    sensors_event_t event;
    mag.getEvent(&event);

    float heading = atan2(event.magnetic.y, event.magnetic.x);
    float declinationAngle = 0.22;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if (heading < 0)
      heading += 2 * PI;

    // Check for wrap due to addition of declination.
    if (heading > 2 * PI)
      heading -= 2 * PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180 / M_PI;

    Serial.print("\tX: ");
    Serial.print(event.magnetic.x);
    Serial.print("\tY: ");
    Serial.print(event.magnetic.y);
    Serial.print("\tZ: ");
    Serial.print(event.magnetic.z);
    Serial.print(" uT\t");
    Serial.print("\thH: ");
    Serial.print(headingDegrees);
    Serial.println();
}

void runStabilization(float ypr[3]) {
  // int y = map(int(ypr[1] * 180/M_PI)+90,    0, 255, 1000, 2000);
  // int x = map(int(ypr[2] * -180/M_PI)+90,    0, 255, 1000, 2000);
  int y = int(ypr[1] * 180/M_PI)+90;
  int x = int(ypr[2] * -180/M_PI)+90;
  aileron.write(y);   // Rotation around Y
  elevator.write(x); 
  Serial.print("\tSt Ele: ");
  Serial.print(x);
  Serial.print("\tSt Ailer: ");
  Serial.print(y);


}

void readMPU() {
  if (dmpReady) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[2] * 180 / M_PI);

        //runStabilization(ypr);
    } else {
      Serial.print("dmp package not ready...");
    }
  }
}
void readSensors() {
  readMPU();
  readMagnometer();
}

void loop() {
  runStrobeLight();
  receiveData();
  readSensors();
  unsigned long now = millis();
  
  if ( now - lastReceivedTime > 1000 ) {
    resetData();
  }
  
  ch_throttle = map(data.throttle, 0, 255, 1000, 2000); 
  ch_elevator = map(data.pitch,    0, 255, 1000, 2000);
  //ch_rudder = map(data.roll,     0, 255, 1000, 2000);
  ch_aileron = map(data.yaw,      0, 255, 1000, 2000);

  Serial.print("\tch_elevator: ");
  Serial.print(ch_elevator);
  Serial.print("\tch_aileron: ");
  Serial.print(ch_aileron);

  throttle.writeMicroseconds(ch_throttle);
  elevator.writeMicroseconds(ch_elevator);
  //rudder.writeMicroseconds(ch_rudder);
  aileron.writeMicroseconds(ch_aileron);
}