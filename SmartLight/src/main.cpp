/**********************************************************************************
 *  TITLE: ESP RainMaker + IR + Manual control PWM Dimmer using ESP32 with EEPROM & Real time feedback
 *  Click on the following links to learn more. 
 *  YouTube Video: https://youtu.be/qyI0ChBk2hQ
 *  Related Blog : https://iotcircuithub.com/
 *  
 *  This code is provided free for project purpose and fair use only.
 *  Please do mail us to techstudycell@gmail.com if you want to use it commercially.
 *  Copyrighted © by Tech StudyCell
 *  
 *  Preferences--> Aditional boards Manager URLs : 
 *  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_dev_index.json, http://arduino.esp8266.com/stable/package_esp8266com_index.json
 *  
 *  Download Board ESP32 (2.0.14) : https://github.com/espressif/arduino-esp32
 *  
 *  Download the libraries: 
 *  IRremote Library by shirriff Armin Joachimsmeyer (4.5.0): https://github.com/Arduino-IRremote/Arduino-IRremote
 *  
 *  Please Install all the dependency related to these libraries. 

 **********************************************************************************/

#include <Arduino.h>
 #include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <IRremote.hpp>   // v4.5.0
#include <Preferences.h>

// ======= Hardware Pins =======
#define PWM_PIN         13
#define WIFI_LED        2
#define BUTTON_UP       18
#define BUTTON_DOWN     22
#define BUTTON_POWER    23
#define BUTTON_BOOT     0
#define IR_RECV_PIN     33

// ======= Configuration =======
#define DEFAULT_POWER_MODE true
#define DEFAULT_DIMMER_LEVEL 50
#define BRIGHTNESS_STEP 10
#define MAX_BRIGHTNESS 100
#define MIN_BRIGHTNESS 0

// ======= RainMaker Provisioning =======
const char *service_name = "PROV_Dimmer1";
const char *pop = "dimmer1234";

// ======= Global Variables =======
bool dimmer_state = DEFAULT_POWER_MODE;
int dimmer_level = DEFAULT_DIMMER_LEVEL;
bool wifi_connected = false;

static int gpio_pwm = PWM_PIN;
static Device dimmer_device("Window LED", ESP_RMAKER_DEVICE_LIGHT, &gpio_pwm);
Param brightness_param("Brightness", ESP_RMAKER_PARAM_BRIGHTNESS,
                       value(DEFAULT_DIMMER_LEVEL),
                       PROP_FLAG_READ | PROP_FLAG_WRITE);

Preferences prefs;

// ======= IR Remote Custom Codes (update as needed) =======
#define IR_POWER_CODE   0x6996F300
#define IR_DOWN_CODE    0x6897F300
#define IR_UP_CODE      0x718EF300

// ======= Function Prototypes =======
void applyPWM();
void updateRainMaker();
void saveState();

// ======= Provisioning Event =======
void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32S2
      printQR(service_name, pop, "softap");
#else
      printQR(service_name, pop, "ble");
#endif
      Serial.println("\nScan the QR code or copy the link above to provision the device.");
      break;
  }
}

// ======= Wi-Fi Events =======
void WiFiEvent(arduino_event_t *event) {
  switch (event->event_id) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("Wi-Fi connected ✅");
      wifi_connected = true;
      digitalWrite(WIFI_LED, HIGH);
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("Wi-Fi lost ❌, retrying...");
      wifi_connected = false;
      digitalWrite(WIFI_LED, LOW);
      WiFi.reconnect();
      break;
  }gpio_pwm;
}

// ======= RainMaker Write Callback =======
void write_callback(Device *device, Param *param, const param_val_t val,
                    void *priv_data, write_ctx_t *ctx) {
  const char *param_name = param->getParamName();

  if (strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0) {
    dimmer_state = val.val.b;
    Serial.printf("Power set to: %s\n", dimmer_state ? "ON" : "OFF");
  }
  else if (strcmp(param_name, "Brightness") == 0) {
    dimmer_level = val.val.i;
    Serial.printf("Brightness set via RainMaker: %d%%\n", dimmer_level);
  }

  applyPWM();
  saveState();
  param->updateAndReport(val);
}

// ======= Apply PWM =======
void applyPWM() {
  int pwmValue = (dimmer_state) ? map(dimmer_level, 0, 100, 0, 255) : 0;
  ledcWrite(0, pwmValue);
}

// ======= Save State =======
void saveState() {
  prefs.begin("dimmer", false);
  prefs.putBool("power", dimmer_state);
  prefs.putInt("level", dimmer_level);
  prefs.end();
  Serial.println("State saved to flash ✅");
}

// ======= Update RainMaker =======
void updateRainMaker() {
  if (wifi_connected) {
    dimmer_device.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, dimmer_state);
    dimmer_device.updateAndReportParam("Brightness", dimmer_level);
  }
}

// ======= Setup =======
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Pin setup
  pinMode(WIFI_LED, OUTPUT);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_POWER, INPUT_PULLUP);
  pinMode(BUTTON_BOOT, INPUT);
  digitalWrite(WIFI_LED, LOW);

  // PWM setup
  ledcSetup(0, 5000, 8);
  ledcAttachPin(PWM_PIN, 0);

  // IR setup
  IrReceiver.begin(IR_RECV_PIN, ENABLE_LED_FEEDBACK);
  Serial.println("IR Receiver ready...");

  // Load last state
  prefs.begin("dimmer", true);
  dimmer_state = prefs.getBool("power", DEFAULT_POWER_MODE);
  dimmer_level = prefs.getInt("level", DEFAULT_DIMMER_LEVEL);
  prefs.end();
  Serial.printf("Restored -> Power: %s, Brightness: %d%%\n",
                dimmer_state ? "ON" : "OFF", dimmer_level);

  applyPWM();

  // RainMaker setup
  Node my_node = RMaker.initNode("ESP32 PWM LED Dimmer");

  dimmer_device.addNameParam();
  dimmer_device.addPowerParam(dimmer_state);
  brightness_param.addBounds(value(0), value(100), value(1));
  brightness_param.addUIType(ESP_RMAKER_UI_SLIDER);
  dimmer_device.addParam(brightness_param);
  dimmer_device.assignPrimaryParam(dimmer_device.getParamByName(ESP_RMAKER_DEF_POWER_NAME));
  dimmer_device.addCb(write_callback);

  my_node.addDevice(dimmer_device);

  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();
  RMaker.start();

  WiFi.onEvent(sysProvEvent);
  WiFi.onEvent(WiFiEvent);

#if CONFIG_IDF_TARGET_ESP32S2
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE,
                          WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_NONE,
                          WIFI_PROV_SECURITY_1, pop, service_name);
#endif

  Serial.println("Setup complete. Waiting for provisioning...");
}

// ======= Loop =======
void loop() {
  static unsigned long lastPress = 0;
  unsigned long now = millis();

  // --- Manual Buttons ---
  if (now - lastPress > 250) {
    if (digitalRead(BUTTON_UP) == LOW) {
      dimmer_level = min(dimmer_level + BRIGHTNESS_STEP, MAX_BRIGHTNESS);
      Serial.printf("Brightness Up: %d%%\n", dimmer_level);
      applyPWM(); saveState(); updateRainMaker();
      lastPress = now;
    }
    else if (digitalRead(BUTTON_DOWN) == LOW) {
      dimmer_level = max(dimmer_level - BRIGHTNESS_STEP, MIN_BRIGHTNESS);
      Serial.printf("Brightness Down: %d%%\n", dimmer_level);
      applyPWM(); saveState(); updateRainMaker();
      lastPress = now;
    }
    else if (digitalRead(BUTTON_POWER) == LOW) {
      dimmer_state = !dimmer_state;
      Serial.printf("Power toggled: %s\n", dimmer_state ? "ON" : "OFF");
      applyPWM(); saveState(); updateRainMaker();
      lastPress = now;
    }
  }

  // --- IR Remote ---
  if (IrReceiver.decode()) {
    uint32_t code = IrReceiver.decodedIRData.decodedRawData;
    Serial.printf("IR Code: 0x%lX\n", code);

    if (code == IR_POWER_CODE) {
      dimmer_state = !dimmer_state;
    } else if (code == IR_UP_CODE) {
      dimmer_level = min(dimmer_level + BRIGHTNESS_STEP, MAX_BRIGHTNESS);
    } else if (code == IR_DOWN_CODE) {
      dimmer_level = max(dimmer_level - BRIGHTNESS_STEP, MIN_BRIGHTNESS);
    }

    applyPWM(); saveState(); updateRainMaker();
    IrReceiver.resume();
  }

  // --- Boot Button (Reset Handling) ---
  if (digitalRead(BUTTON_BOOT) == LOW) {
    delay(100);
    int startTime = millis();
    while (digitalRead(BUTTON_BOOT) == LOW) delay(50);
    int elapsed = millis() - startTime;

    if (elapsed > 10000) {
      Serial.println("Factory Reset Triggered!");
      RMakerFactoryReset(2);
    } else if (elapsed > 3000) {
      Serial.println("Wi-Fi Reset Triggered!");
      RMakerWiFiReset(2);
    }
  }

  delay(100);
}
