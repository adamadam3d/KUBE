#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Kalman.h>
#include <SPI.h>
#include "driver/spi_master.h"
#include "hal/spi_types.h"
#include "soc/spi_struct.h"

// MPU setup
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
int16_t gx, gy, gz;

Quaternion q;
VectorFloat gravity;
float ypr[3];

Kalman kalmanX;
Kalman kalmanY;

// SPI2 (HSPI) for rollInt
#define HSPI_CS_PIN 15
SPIClass spiHSPI(HSPI);  // Avoid name conflict
#define SPI2_DEV ((spi_dev_t *)DR_REG_SPI2_BASE)

// SPI3 (VSPI) for gx
#define VSPI_CS_PIN 5
SPIClass spiVSPI(VSPI);  // Avoid name conflict
#define SPI3_DEV ((spi_dev_t *)DR_REG_SPI3_BASE)

bool isSPI2Busy() {
  return SPI2_DEV->cmd.usr == 1;
}

bool isSPI3Busy() {
  return SPI3_DEV->cmd.usr == 1;
}

void setup() {
  Wire.begin(21, 22);
  Serial.begin(115200);

  // SPI2 (HSPI) setup for rollInt
  pinMode(HSPI_CS_PIN, OUTPUT);
  digitalWrite(HSPI_CS_PIN, HIGH);
  spiHSPI.begin(14, 12, 13);  // SCK, MISO, MOSI

  // SPI3 (VSPI) setup for gx
  pinMode(VSPI_CS_PIN, OUTPUT);
  digitalWrite(VSPI_CS_PIN, HIGH);
  spiVSPI.begin(18, 19, 23);  // SCK, MISO, MOSI

  delay(1000);
  Serial.println("Initializing MPU...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP ready.");
  } else {
    Serial.print("DMP init failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
}

void loop() {
  if (!dmpReady) return;

  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
  } else if (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.getRotation(&gx, &gy, &gz);

    float rawRoll = ypr[2] * 180 / M_PI;
    float filteredRoll = kalmanX.getAngle(rawRoll, 0.0, 0.01);
    gx /= 131;

    int16_t rollInt = (int16_t)filteredRoll;
    int16_t gxInt = gx;

    // --- HSPI: Send rollInt ---
    while (isSPI2Busy());
    spiHSPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_CS_PIN, LOW);
    while (isSPI2Busy());
    spiHSPI.transfer(0xFF);
    while (isSPI2Busy());
    spiHSPI.transfer(rollInt & 0xFF);  // LSB only
    digitalWrite(HSPI_CS_PIN, HIGH);
    spiHSPI.endTransaction();

    // --- VSPI: Send gxInt ---
    while (isSPI3Busy());
    spiVSPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0));
    digitalWrite(VSPI_CS_PIN, LOW);
    while (isSPI3Busy());
    spiVSPI.transfer(0xFF);
    while (isSPI3Busy());
    spiVSPI.transfer(gxInt & 0xFF);  // LSB only
    digitalWrite(VSPI_CS_PIN, HIGH);
    spiVSPI.endTransaction();

    // Debug output
    Serial.print("filteredRoll: ");
    Serial.print(rollInt);
    Serial.print(" | gx: ");
    Serial.println(gxInt);
  }
}
