#include "BluetoothSerial.h"
#include "esp_bt_device.h"
#include "MPU9250.h"
#include <SparkFunADXL313.h> 

uint8_t addrs[7] = {0};
uint8_t device_count = 0;
// /Click here to get the library: http://librarymanager/All#SparkFun_ADXL313
// DXL313 myAdxl;

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
const int pin27 = 27;
void setup() {
  // Setup Bluetooth
  Serial.begin(115200);
  Wire.begin();
  SerialBT.begin("ESP32 EECS149");
  Serial.println("The device started, now you can pair it with bluetooth!");
  printDeviceAddress();

  // join I2C bus (I2Cdev library doesn't do this automatically)
    
//  if (myAdxl.begin() == false) //Begin communication over I2C
//  {
//    Serial.println("The sensor did not respond. Please check wiring.");
//    while(1); //Freeze
//  }
//  Serial.print("Sensor is connected properly.");
//  
//  myAdxl.measureModeOn(); // wakes up the sensor from standby and puts it into measurement mode
//  delay(2000);

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
  
    //Calibrate the sensors
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();
  
    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();
    print_calibration();
    mpu.verbose(false);
}


// can only write a maximum of 8 bit integer
void loop() {
if (mpu.update()) {

    // Accelerometer Readings
    // Serial.print("Accel: "); Serial.print(mpu.getAccX()); Serial.print(", "); Serial.print(mpu.getAccY()); Serial.print(", "); Serial.println(mpu.getAccZ());

    // Sending the readings to bluetooth
    SerialBT.print(mpu.getAccX());
    SerialBT.print("_");
    SerialBT.print(mpu.getAccY());
    SerialBT.print("_");
    SerialBT.print(mpu.getAccZ());
    SerialBT.print("_");

    // Gyroscope Readings
    // Serial.print("Gyro: "); Serial.print(mpu.getGyroX()); Serial.print(", "); Serial.print(mpu.getGyroY());  Serial.print(", "); Serial.print(mpu.getGyroZ()); 

    // Sending the readings to bluetooth
    SerialBT.print(mpu.getGyroX()); 
    SerialBT.print("_");
    SerialBT.print(mpu.getGyroY()); 
    SerialBT.print("_");
    SerialBT.print(mpu.getGyroZ()); 
    SerialBT.print("_");


    int pin27_val = analogRead(pin27);
    int pin15_val = analogRead(pin15);
    int pin13_val = analogRead(pin13);
    int pin12_val = analogRead(pin12);
    
    // Analog Pin Readings
    Serial.println(pin27_val);
    Serial.println(pin15_val);
    Serial.println(pin13_val);
    Serial.println(pin12_val);
//    

    // Sending the readings to bluetooth
    SerialBT.print(pin27_val);
    SerialBT.print("_");
    SerialBT.print(pin15_val);
    SerialBT.print("_");
    SerialBT.print(pin13_val);
    SerialBT.print("_");
    SerialBT.print(pin12_val);
    SerialBT.print("|");

    // Delay for sending the values
    delay(50);
  }
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