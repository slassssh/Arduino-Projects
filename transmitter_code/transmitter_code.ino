// Uncomment to enable Serial debug output
#define PRINT_DEBUG   // Remove this line to disable debug prints

// Needed for MPU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Needed for NRF24L01
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// NRF24L01 Setup
const uint64_t pipeOut = 0xF9E8F0F0E1LL; // Must match the receiver
RF24 radio(8, 9); // CE, CSN

struct PacketData {
  byte xAxisValue;
  byte yAxisValue;
} data;

void setupRadioTransmitter() {
  bool radioStatus = radio.begin();
  if (!radioStatus) {
    #ifdef PRINT_DEBUG
      Serial.println(F("NRF24L01 initialization failed!"));
    #endif
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate error
    while (1); // Halt here
  }

  radio.setDataRate(RF24_250KBPS);  // Lower speed = better range/stability
  radio.openWritingPipe(pipeOut);
  radio.stopListening();

  data.xAxisValue = 127;
  data.yAxisValue = 127;
}

void setupMPU() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  #ifdef PRINT_DEBUG
    Serial.println(F("Initializing MPU6050..."));
  #endif

  mpu.initialize();

  #ifdef PRINT_DEBUG
    Serial.println(mpu.testConnection() ? F("MPU6050 connected!") : F("MPU6050 connection failed!"));
  #endif

  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    
    #ifdef PRINT_DEBUG
      mpu.PrintActiveOffsets();
      Serial.println(F("Enabling DMP..."));
    #endif

    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    #ifdef PRINT_DEBUG
      Serial.println(F("MPU6050 DMP ready."));
    #endif

  } else {
    #ifdef PRINT_DEBUG
      Serial.print(F("DMP Init failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    #endif
    digitalWrite(LED_BUILTIN, HIGH); // Error indicator
    while (1); // Halt here
  }
}

void setup() {
  // Optional LED for error/debug
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  #ifdef PRINT_DEBUG
    Serial.begin(115200);
    while (!Serial); // Wait for Serial monitor
    Serial.println(F("Starting setup..."));
  #endif

  setupRadioTransmitter();
  setupMPU();
}

void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    int xAxisValue = constrain(ypr[2] * 180/M_PI, -90, 90);
    int yAxisValue = constrain(ypr[1] * 180/M_PI, -90, 90);

    data.xAxisValue = map(xAxisValue, -90, 90, 0, 254); 
    data.yAxisValue = map(yAxisValue, -90, 90, 254, 0); // Inverted

    radio.write(&data, sizeof(PacketData));

    #ifdef PRINT_DEBUG
      Serial.print("Pitch: ");
      Serial.print(yAxisValue);
      Serial.print("\tRoll: ");
      Serial.println(xAxisValue);
    #endif
  }
}
