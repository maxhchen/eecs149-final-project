#include "BluetoothSerial.h"
#include "esp_bt_device.h"
#include "MPU9250.h"

uint8_t addrs[7] = {0};
uint8_t device_count = 0;

// Scans for connection to MPU 9250 over I2C.
// Helper function from https://github.com/hideakitai/MPU9250.
template <typename WireType = TwoWire>
void scan_mpu(WireType& wire = Wire) {
  Serial.println("Searching for i2c devices...");
  device_count = 0;

  for (uint8_t i = 0x68; i < 0x70; ++i) {
    wire.beginTransmission(i);
    if (wire.endTransmission() == 0) {
      addrs[device_count++] = i;
      delay(10);
    }
  }

  Serial.print("Found "); Serial.print(device_count, DEC); Serial.println(" I2C devices");

  Serial.print("I2C addresses are: ");
  for (uint8_t i = 0; i < device_count; ++i) {
    Serial.print("0x"); Serial.print(addrs[i], HEX); Serial.print(" ");
  }
  Serial.println();
}

// https://makeabilitylab.github.io/physcomp/esp32/esp32.html
// ADC#1 is the only ADC that works when using WiFi
// ADC#2 only works when WiFi has not started.
// Nov 13th: Checked ADC on all pins -> Worked as expected

// Note: Serial   --> ESP32
//       SerialBT --> Computer
BluetoothSerial SerialBT;
MPU9250 mpu;
// const int pin14 = 14;
// const int pin15 = 15;
// const int pin13 = 13;
// const int pin12 = 12;

void setup() {
  // Setup Bluetooth
  Serial.begin(/*Baud Rate = */ 115200);
  SerialBT.begin(/*Device Name = */ "ESP32 EECS149");
  Serial.println("The device has started -- ready to pair with bluetooth!");

  // Join I2C bus with IMU (I2Cdev library doesn't do this automatically)
  Wire.begin();
  delay(2000);

  // Calibrate MPU (https://github.com/hideakitai/MPU9250)
  scan_mpu();

  if (device_count == 0) {
    Serial.println("No device found on I2C bus. Please check your hardware connection.");
    while (1); // TODO: Change to return or throw error?
  }

  for (uint8_t i = 0; i < device_count; ++i) {
    Serial.print("I2C address 0x"); Serial.println(addrs[i], HEX);
  }

  Serial.print("Found "); Serial.print(device_count, DEC); Serial.println(" I2C devices.");

  if (!mpu.setup(0x68)) {
    Serial.println("Could not connect to MPU. Please check your hardware connection.");
    while (1); // TODO: Change to return or throw error?
  }

  // Start Calibration 
   Serial.println("Starting Accelerometer/Gyroscope calibration in 5s...");
   Serial.println("Please leave your device still on a flat plane.");
   delay(5000);
   mpu.calibrateAccelGyro();
   Serial.println("Accelerometer/Gyroscope calibration finished!");
  
   Serial.println("Starting Magnetometer calibration in 5s...");
   Serial.println("Please wave your device in a figure-eight until done.");
   delay(5000);
   mpu.calibrateMag();
   Serial.println("Magnetometer calibration finished!");
  
   print_calibration();
}


// Can only write a maximum of 8-bit integer values
void loop() {
  if (mpu.update()) {
    // Echo sensor data to ESP32 monitor
    Serial.print("Accel: ");
    Serial.print(mpu.getAccX()); Serial.print(", ");
    Serial.print(mpu.getAccY()); Serial.print(", ");
    Serial.println(mpu.getAccZ());

    // "SerialBT" will actually send sensor data to computer
    SerialBT.print(mpu.getAccX());

    // Delimiter to separate values when parsing data on computer
    SerialBT.print("|");

    // Serial.print("Gyro: ");
    // Serial.print(mpu.getGyroX()); Serial.print(", ");
    // Serial.print(mpu.getGyroY()); Serial.print(", ");
    // Serial.println(mpu.getGyroZ());

    // Serial.print("Mag: ");
    // Serial.print(mpu.getMagX()); Serial.print(", ");
    // Serial.print(mpu.getMagY()); Serial.print(", ");
    // Serial.println(mpu.getMagZ());
  }
  delay(10);
}
