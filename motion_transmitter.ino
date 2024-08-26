#include "Adafruit_LC709203F.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


/* 
For reference, the message dict used to communicate with the receiver is here:
{
    "1": {
        "state/pico_central_receiver/motion_sensor/": {
            "A": {"movement": {"a": "Movement Detected", "b": "No Movement"}},
            "B": {
                "battery": {},
            },
            "C": {"status": {"a": "Online", "b": "Error"}},
            "D": {"movement_type": {"a": "Pitch", "b": "Acceleration"}},
        }
    },
}
*/

#define LED_BUILTIN 2
#define HC12 Serial2
#define RXD2 16  //(RX2)
#define TXD2 17 //(TX2)
#define SDA_2 33
#define SCL_2 32
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
// #define TIME_TO_SLEEP 15 // for testing
#define TIME_TO_SLEEP 10800

Adafruit_MPU6050 mpu;
Adafruit_LC709203F lc;

int ErrorState = 0;

void flash_led(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(100);  
  }
}

void parse_battery_data() {
  lc.setPowerMode(LC709203F_POWER_OPERATE);
  Serial.print("Batt Voltage: "); Serial.println(lc.cellVoltage(), 3);
  Serial.print("Batt Percent: "); Serial.println(lc.cellPercent(), 1);
  char buffer[7];
  char batt_reading[5] = "1,B,";
  dtostrf(lc.cellPercent(), 4, 1, buffer);
  HC12.write(strcat(batt_reading, buffer));
  flash_led(2);
  lc.setPowerMode(LC709203F_POWER_SLEEP);
}

void parse_movement_data() {

  if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    HC12.write("1,A,a");
    Serial.println("Wrote to HC12");

    /* Print out the values */
    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(a.acceleration.z);
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(g.gyro.z);
    Serial.println("");
    
    flash_led(4);
  }
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void send_serial_data(String s) {
  byte buf[10];
  int count = s.length(); 
  s.getBytes(buf, count + 1);
  Serial.write(buf, count + 1);
  Serial.println("Wrote to Serial");
}


/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void wakeup_routine(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); parse_movement_data(); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); parse_battery_data(); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  HC12.begin(9600, SERIAL_8N1, RXD2, TXD2);      // Serial port to HC12
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  
  /* LC709203F set up */
  Wire1.begin(SDA_2, SCL_2, 100000);
  Serial.println("\nAdafruit LC709203F demo");
  /* pass lc.begin our slower TwoWire object */
  if (!lc.begin(&Wire1)) {
    Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
    while (1) {
      if (!ErrorState) {
        HC12.write("1,C,b");
      }
      flash_led(10);

      esp_restart();
    };
  }
  Serial.println(F("Found LC709203F"));
  Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);
  lc.setPackSize(LC709203F_APA_3000MAH);
  lc.setAlarmVoltage(3.8);


  // MPU6050 setup
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      if (!ErrorState) {
        HC12.write("1,C,b");
      }
      flash_led(10);

      esp_restart();
    }
  }
  Serial.println("MPU6050 Found!");

  //setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(15);
  mpu.setMotionDetectionDuration(10);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");

  /* set up sleep */
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35,0); //1 = High, 0 = Low
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  HC12.write("1,C,a");
  delay(800);

}

void loop() {
  wakeup_routine();

  delay(700);
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
}