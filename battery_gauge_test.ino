#include "Adafruit_LC709203F.h"

Adafruit_LC709203F lc;
#define HC12 Serial2
#define RXD2 16  //(RX2)
#define TXD2 17 //(TX2)
#define SDA_2 33
#define SCL_2 32
#define LED_BUILTIN 2
//Declare a global TwoWire object
TwoWire I2C = TwoWire(1); 

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  HC12.begin(9600, SERIAL_8N1, RXD2, TXD2);      // Serial port to HC12
  //Setup I2C at a slower speed where SDA=21 and SCL=22 for ESP32
  I2C.begin(SDA_2, SCL_2, 100000);
  // I2C.setClock(100000);

  Serial.println("\nAdafruit LC709203F demo");

  //pass lc.begin our slower TwoWire object
  if (!lc.begin(&I2C)) {
    Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
    digitalWrite(LED_BUILTIN, HIGH);
    while (1) delay(10);
  } else {
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(100);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(100);  
    }
  }
  Serial.println(F("Found LC709203F"));
  Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);


  lc.setPackSize(LC709203F_APA_3000MAH);

  lc.setAlarmVoltage(3.8);
}

void loop() {
  Serial.print("Batt Voltage: "); Serial.println(lc.cellVoltage(), 3);
  Serial.print("Batt Percent: "); Serial.println(lc.cellPercent(), 1);
  Serial.print("Batt Temp: "); Serial.println(lc.getCellTemperature(), 1);

  delay(2000);  // dont query too often!
}



/* ======================= */


// #include <Wire.h>
 
// void setup() {
//   Wire.begin();
//   Serial.begin(115200);
//   Serial.println("\nI2C Scanner");
//   Wire.setClock(100000);

// }
 
// void loop() {
//   byte error, address;
//   int nDevices;
//   Serial.println("Scanning...");
//   nDevices = 0;
//   for(address = 1; address < 127; address++ ) {
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();
//     if (error == 0) {
//       Serial.print("I2C device found at address 0x");
//       if (address<16) {
//         Serial.print("0");
//       }
//       Serial.println(address,HEX);
//       nDevices++;
//     }
//     else if (error==4) {
//       Serial.print("Unknow error at address 0x");
//       if (address<16) {
//         Serial.print("0");
//       }
//       Serial.println(address,HEX);
//     }    
//   }
//   if (nDevices == 0) {
//     Serial.println("No I2C devices found\n");
//   }
//   else {
//     Serial.println("done\n");
//   }
//   delay(5000);          
// }