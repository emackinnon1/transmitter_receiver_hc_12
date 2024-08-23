#include "Adafruit_LC709203F.h"

Adafruit_LC709203F lc;
#define HC12 Serial2
#define RXD2 16  //(RX2)
#define TXD2 17 //(TX2)
//Declare a global TwoWire object
TwoWire I2C = TwoWire(0); 

void setup() {
  Serial.begin(115200);
  delay(1000);
  HC12.begin(9600, SERIAL_8N1, RXD2, TXD2);      // Serial port to HC12
  //Setup I2C at a slower speed where SDA=21 and SCL=22 for ESP32
  I2C.begin(21,22, 10000);

  Serial.println("\nAdafruit LC709203F demo");

  //pass lc.begin our slower TwoWire object
  if (!lc.begin(&I2C)) {
    Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
  }
  Serial.println(F("Found LC709203F"));
  Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);


  lc.setPackSize(LC709203F_APA_500MAH);

  lc.setAlarmVoltage(3.8);
}

void loop() {
  Serial.print("Batt Voltage: "); Serial.println(lc.cellVoltage(), 3);
  Serial.print("Batt Percent: "); Serial.println(lc.cellPercent(), 1);
  Serial.print("Batt Temp: "); Serial.println(lc.getCellTemperature(), 1);

  delay(2000);  // dont query too often!
}