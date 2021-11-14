#include "BluetoothSerial.h"
#include "esp_bt_device.h"
#include "MPU9250.h"

uint8_t addrs[7] = {0};
uint8_t device_count = 0;

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
  Serial.print("Found ");
  Serial.print(device_count, DEC);
  Serial.println(" I2C devices");

  Serial.print("I2C addresses are: ");
  for (uint8_t i = 0; i < device_count; ++i) {
    Serial.print("0x");
    Serial.print(addrs[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

template <typename WireType = TwoWire>
uint8_t readByte(uint8_t address, uint8_t subAddress, WireType& wire = Wire) {
  uint8_t data = 0;
  wire.beginTransmission(address);
  wire.write(subAddress);
  wire.endTransmission(false);
  wire.requestFrom(address, (size_t)1);
  if (wire.available()) data = wire.read();
  return data;
}

void printDeviceAddress() {

  const uint8_t* point = esp_bt_dev_get_address();

  for (int i = 0; i < 6; i++) {

    char str[3];

    sprintf(str, "%02X", (int)point[i]);
    Serial.print(str);

    if (i < 5) {
      Serial.print(":");
    }

  }
}



// https://makeabilitylab.github.io/physcomp/esp32/esp32.html
// ADC#1 is the only ADC that works when using WiFi
// ADC#2 only works when WiFi has not started.
// Nov 13th: Checked ADC on all pins -> Worked as expected
BluetoothSerial SerialBT;
MPU9250 mpu;
const int pin14 = 14;
const int pin15 = 15;
const int pin13 = 13;
const int pin12 = 12;
void setup() {
  // Setup Bluetooth
  Serial.begin(115200);
  SerialBT.begin("ESP32 EECS149");
  Serial.println("The device started, now you can pair it with bluetooth!");


  // Join I2C bus with IMU (I2Cdev library doesn't do this automatically)
  Wire.begin();
  delay(2000);

  // Calibrate MPU https://github.com/hideakitai/MPU9250
  scan_mpu();

  if (device_count == 0) {
    Serial.println("No device found on I2C bus. Please check your hardware connection");
    while (1);
  }

  for (uint8_t i = 0; i < device_count; ++i) {
    Serial.print("I2C address 0x");
    Serial.print(addrs[i], HEX);
    byte ca = readByte(addrs[i], WHO_AM_I_MPU9250);
    if (ca == MPU9250_WHOAMI_DEFAULT_VALUE) {
      Serial.println(" is MPU9250 and ready to use");
    } else if (ca == MPU9255_WHOAMI_DEFAULT_VALUE) {
      Serial.println(" is MPU9255 and ready to use");
    } else if (ca == MPU6500_WHOAMI_DEFAULT_VALUE) {
      Serial.println(" is MPU6500 and ready to use");
    } else {
      Serial.println(" is not MPU series");
      Serial.print("WHO_AM_I is ");
      Serial.println(ca, HEX);
      Serial.println("Please use correct device");
    }
  }

  Serial.print("Found ");
  Serial.print(device_count, DEC);
  Serial.println(" I2C devices");

  mpu.setup(0x68);

  // Start Calibration 
  //  Serial.println("Accel Gyro calibration will start in 5sec.");
  //  Serial.println("Please leave the device still on the flat plane.");
  //  mpu.verbose(true);
  //  delay(5000);
  //  mpu.calibrateAccelGyro();
  //
  //  Serial.println("Mag calibration will start in 5sec.");
  //  Serial.println("Please Wave device in a figure eight until done.");
  //  delay(5000);
  //  mpu.calibrateMag();
  //
  //  print_calibration();
  //  mpu.verbose(false);
}


// can only write a maximum of 8 bit integer
void loop() {
  //    if (mpu.update()) {
  //        Serial.print(mpu.getYaw()); Serial.print(", ");
  //        Serial.print(mpu.getPitch()); Serial.print(", ");
  //        Serial.println(mpu.getRoll());
  //    }


  if (mpu.update()) {
    Serial.print("Accel: "); Serial.print(mpu.getAccX()); Serial.print(", ");
    Serial.print(mpu.getAccY()); Serial.print(", ");
    Serial.println(mpu.getAccZ());
    SerialBT.print(mpu.getAccX());
    SerialBT.print("-");
//    Serial.print("Gyro: "); Serial.print(mpu.getGyroX()); Serial.print(", ");
//    Serial.print(mpu.getGyroY()); Serial.print(", ");
//    Serial.println(mpu.getGyroZ());

    //float getMagX() const;
    //float getMagY() const;
    //float getMagZ() const;

  }


  //  Serial.println(analogRead(pin12));
  // SerialBT.write(200);
  //  SerialBT.println(Serial.available());
  //  SerialBT.println(SerialBT.available());
  //  SerialBT.println(46);
  //  SerialBT.println("----");

  // Reads from the arduino serial and writes it to computer via bluetooth
  //  if (Serial.available()) {
  //    SerialBT.write(40);
  //    SerialBT.write(Serial.read());
  //  }
  //
  //  // Reads from the computer via bluetooth and writes it to the Arduino serial
  //  if (SerialBT.available()) {
  //    SerialBT.write(50);
  //    Serial.write(SerialBT.read());
  //  }
  delay(10);
}

void print_calibration() {
  Serial.println("< calibration parameters >");
  Serial.println("accel bias [g]: ");
  Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.println();
  Serial.println("gyro bias [deg/s]: ");
  Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.println();
  Serial.println("mag bias [mG]: ");
  Serial.print(mpu.getMagBiasX());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasY());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasZ());
  Serial.println();
  Serial.println("mag scale []: ");
  Serial.print(mpu.getMagScaleX());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleY());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleZ());
  Serial.println();
}
