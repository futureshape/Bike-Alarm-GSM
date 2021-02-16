#include <Arduino.h>
#include <SPI.h>
#include <Ticker.h>
#include <driver/rtc_io.h>
#include <EasyBuzzer.h>
#include <Wire.h>

#include "driver/adc.h"
#include <esp_bt.h>

#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>

//Create software serial object to communicate with SIM800L
HardwareSerial gsmSerial(2);

TinyGsm modem(gsmSerial);
TinyGsmClient client(modem);
#define MODEM_RST 5       // SIM800 RESET but also IP5306 IRQ: use IRQ Analyzing signals IP5306 It is in working condition or in standby mode: IRQ = 1 Work, IRQ = 0 When in standby
#define MODEM_PWKEY 4     // PWRKEY SIM800
#define MODEM_POWER_ON 23 // EN SY8089 4v4 regulator for SIM800
#define MODEM_TX 27
#define MODEM_RX 26

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "8d53799a-e1c1-41ae-9905-9bf097479a12"
#define CHARACTERISTIC_UUID "6b2f494a-2861-4802-b199-c333e3658080"
BLEAdvertisementData advData;

#include <ADXL362.h>

#define ADXL362_SCK     18
#define ADXL362_MISO    19
#define ADXL362_MOSI    14
#define ADXL362_CS       2
#define ADXL362_INT     15

#define BUZZER_GPIO     12
#define LED_GPIO        13

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00
#define I2C_SDA              21
#define I2C_SCL              22

bool setPowerBoostKeepOn(int en)
{
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  if (en) {
    Wire.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    Wire.write(0x35); // 0x37 is default reg value
  }
  return Wire.endTransmission() == 0;
}

int8_t getBatteryLevel() {
    Wire.beginTransmission(IP5306_ADDR);
    Wire.write(0x78);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(IP5306_ADDR, 1)) {
        switch (Wire.read() & 0xF0) {
        case 0xE0: return 25;
        case 0xC0: return 50;
        case 0x80: return 75;
        case 0x00: return 100;
        default: return 0;
        }
    }
    return -1;
}

enum alarmStates {
  UNDEFINED,  // startup value
  ARMED,      // bike is parked, checking for motion
  DISARMED,   // legitimate use detected, movement allowed
  PREARM,     // woken up from disarmed mode, waiting to see if phone is still there
  PREALARM,   // movement detected, checking if legitimate or if it stops
  ALARM       // illegitimate movement, send notification
};

#define PRE_ALARM_TIMEOUT   10 // How many seconds after first movement to stay awake and check for repeated movement
#define FULL_ALARM_WAIT     20 // How many seconds of continuous movement to wait until sounding full alarm
#define REARM_CHECK_PERIOD  30 // How often (in seconds) to wake up and check that the phone is still nearby
#define REARM_TIMEOUT       15 // How many seconds to wait for phone to connect and provide unlock code before rearming

// Persists during sleep so we can distinguish between the different reasons for going to sleep
RTC_DATA_ATTR volatile enum alarmStates alarmState = UNDEFINED;

class CharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      log_i("Received unlock code: %s", pCharacteristic->getValue().c_str());

      // TODO: check for correct password
      alarmState = DISARMED;
    }
};

class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    log_d("BLEServer::onConnect");
  }

  void onDisconnect(BLEServer *pServer) {
    log_d("BLEServer::onDisconnect");    
  }
};

void start_ble() {
  BLEDevice::init("Alarm");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pServer->setCallbacks(new ServerCallbacks());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new CharacteristicCallbacks());
  pService->start();

  BLEAdvertising bleAdvertising;

  // https://medium.com/@cbartel/ios-scan-and-connect-to-a-ble-peripheral-in-the-background-731f960d520d
  advData.setAppearance(0);
  advData.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC);
  advData.setCompleteServices(BLEUUID(SERVICE_UUID));
  bleAdvertising.setAdvertisementData(advData);
  bleAdvertising.start();
}

void arm_and_sleep() {
  log_i("Arming and going to sleep");

  btStop();
  adc_power_off();
  esp_bt_controller_disable();

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, HIGH);
  esp_deep_sleep_start();  
}    

void disarm_and_sleep() {
  log_i("Alarm has been disarmed, will go into sleep state and check again later");

  esp_sleep_enable_timer_wakeup(REARM_CHECK_PERIOD * 1000000L);
  esp_deep_sleep_start();  
}

IRAM_ATTR void preAlarmReset() {
  alarmState = ARMED;
  log_i("No more movement detected, ending pre-alarm");
}

IRAM_ATTR void fullAlarm() {
  if(alarmState == PREALARM) {
    log_i("Full alarm");
    alarmState = ALARM;
  }
}

IRAM_ATTR void reArm() {
  log_i("Timed out waiting for unlock code, re-arming");
  alarmState = ARMED;
}

Ticker preAlarmResetTimer = Ticker();
Ticker fullAlarmTimer = Ticker();
Ticker reArmTimer = Ticker();

IRAM_ATTR void detectsMovement() {
  if(alarmState == PREALARM || alarmState == ARMED) {
    log_i("Motion detected on time %ld", millis());

    if(alarmState == ARMED) {
      log_i("Full alarm will sound in %d secs unless motion stops", FULL_ALARM_WAIT);
      fullAlarmTimer.once(FULL_ALARM_WAIT, fullAlarm);
    }

    alarmState = PREALARM;
    preAlarmResetTimer.detach(); // cancel existing timer, if running 
    preAlarmResetTimer.once(PRE_ALARM_TIMEOUT, preAlarmReset);

  }
}

void GSM_ON(uint32_t time_delay)
{
  // Set-up modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);

  Serial.println("MODEM_RST & IP5306 IRQ: HIGH"); // IP5306 HIGH
  digitalWrite(MODEM_RST, HIGH);
  delay(time_delay);

  Serial.println("MODEM_PWKEY: HIGH");
  digitalWrite(MODEM_PWKEY, HIGH); // turning modem OFF
  delay(time_delay);

  Serial.println("MODEM_POWER_ON: HIGH");
  digitalWrite(MODEM_POWER_ON, HIGH); //Enabling SY8089 4V4 for SIM800 (crashing when in battery)
  delay(time_delay);

  Serial.println("MODEM_PWKEY: LOW");
  digitalWrite(MODEM_PWKEY, LOW); // turning modem ON
  delay(time_delay);
}

void GSM_OFF()
{
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);

  digitalWrite(MODEM_PWKEY, HIGH);   // turn of modem in case its ON from previous state
  digitalWrite(MODEM_POWER_ON, LOW); // turn of modem psu in case its from previous state
  digitalWrite(MODEM_RST, HIGH);     // Keep IRQ high ? (or not to save power?)
}

void alarmToGSM()
{  
  //Begin serial communication with Arduino and SIM800L
  gsmSerial.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  GSM_ON(1000);

  log_i("Initializing GSM modem");

  modem.init();
  log_i("Modem Info: %s",  modem.getModemInfo().c_str());
 
  log_i("Waiting for network...");
  if (!modem.waitForNetwork()) {
    log_e("Failed to connect to network");
    return;
  }

  if (modem.isNetworkConnected()) {
    log_i("Network connected");
  }

  log_i("Opening GPRS connection");
  if (!modem.gprsConnect("hologram")) {
    log_e("Failed to start GPRS connection");
    return;
  }

  if (modem.isGprsConnected()) {
    log_i("GPRS connected");
  }

  client.connect("cloudsocket.hologram.io", 9999);
  client.write("{\"k\":\"G_G#XDW0\",\"d\":\"Alarm\",\"t\":\"ALARM\"}");
  client.stop();
  
  log_i("Cloud message sent, closing GPRS connection");
  modem.gprsDisconnect();

  log_i("GSM Radio OFF");
  modem.radioOff();

  log_i("GSM Power OFF");
  GSM_OFF();
}

void setup(void) {
  
  Serial.begin(115200);
  log_i("--- BIKE ALARM ---");

  Wire.begin(I2C_SDA, I2C_SCL);
  bool isOk = setPowerBoostKeepOn(1);
  log_i("IP5306 KeepOn %s", isOk ? "OK" : "FAIL");

  log_i("Battery level: %d", getBatteryLevel());

  EasyBuzzer.setPin(BUZZER_GPIO);
  pinMode(LED_GPIO, OUTPUT);

  delay(100);
  SPI.begin(ADXL362_SCK, ADXL362_MISO, ADXL362_MOSI, ADXL362_CS);
  adiAccelerometer.begin(ADXL362_CS, &SPI);
  adiAccelerometer.softwareReset();
  adiAccelerometer.setupActivityDetection(0x3F, 300, 0);
  adiAccelerometer.setupInactivityDetection(0x3F, 150, 3);
  adiAccelerometer.setIntMap1(ADXL362_INTMAP1_AWAKE);
  adiAccelerometer.setWakeupMode(true);                    
  adiAccelerometer.setMeasurementMode();
  log_i("Accelerometer set up");

  esp_reset_reason_t reset_reason = esp_reset_reason();

  if(reset_reason == ESP_RST_DEEPSLEEP) {
    if(alarmState == ARMED) {
      log_i("Waking up from ARMED state because of movement interrupt");

      detectsMovement(); // trigger once

      // set up interrupt for subsequent motion triggering while woken up
      attachInterrupt(ADXL362_INT, detectsMovement, RISING);
    } else if(alarmState == DISARMED) {
      log_i("Waking up from DISARMED state to check if phone still here");
      alarmState = PREARM;
      start_ble();
      reArmTimer.once(15, reArm);
    }
  } else {
    // normal restart
    log_i("Normal startup, going to sleep after setup");

    // test buzzer
    EasyBuzzer.singleBeep(2000, 200);
    delay(400);
    EasyBuzzer.singleBeep(4000, 200);

    alarmState = ARMED;
  }
}

void loop(void) {
  
  EasyBuzzer.update();

  if(alarmState == ARMED) {
      arm_and_sleep();
  } else if(alarmState == PREALARM) {
      if(!BLEDevice::getInitialized()) { // TODO - fix using BLE status as a flag for first pre-alarm run

        EasyBuzzer.beep(
          2000,		// Frequency in hertz(HZ).
          300, 		// On Duration in milliseconds(ms).
          700, 		// Off Duration in milliseconds(ms).
          20, 		// The number of beeps per cycle.
          0, 	// Pause duration.
          1, 		// The number of cycle.
          NULL		// [Optional] Function to call when done.
        );	

        log_i("Starting BLE to wait for unlock code");
        start_ble();
      }
      
      digitalWrite(LED_GPIO, HIGH);
  } else if(alarmState == DISARMED) {
    disarm_and_sleep();
  } else if(alarmState == ALARM) {
    EasyBuzzer.beep(4000);
    alarmToGSM();
    // TODO: Re-arm here?
  } 
}