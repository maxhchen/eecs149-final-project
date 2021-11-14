#include "BluetoothSerial.h"
#include "esp_bt_device.h"
 
void printDeviceAddress() {
 
  const uint8_t* point = esp_bt_dev_get_address();
 
  for (int i = 0; i < 6; i++) {
 
    char str[3];
 
    sprintf(str, "%02X", (int)point[i]);
    Serial.print(str);
 
    if (i < 5){
      Serial.print(":");
    }
 
  }
}

// https://makeabilitylab.github.io/physcomp/esp32/esp32.html
// ADC#1 is the only ADC that works when using WiFi 
// ADC#2 only works when WiFi has not started. 

BluetoothSerial SerialBT; 
const int pin14 = 14; 
const int pin39 = 39; 
const int pin34 = 34; 
const int pin35 = 35; 
void setup() {
  // Setup Bluetooth
  Serial.begin(115200);
  SerialBT.begin("ESP32 EECS149"); 
  Serial.println("The device started, now you can pair it with bluetooth!");
  printDeviceAddress();

 
}

// can only write a maximum of 8 bit integer 
void loop() {

  Serial.println(analogRead(pin14)); 
//  Serial.println(analogRead(pin39)); 
//  Serial.println(analogRead(pin34)); 
//  Serial.println(analogRead(pin35)); 
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
  delay(50); 
}
