/* SmartStove
 *  powered with Blynk (http://www.blynk.cc)
 * 
 * Electric heater upgrade with WiFi connectivity,
 * temperature/humidity sensing, remote control and
 * thermostate functions.
 * 
 * GregNau © 2016-2020
 */

// Software configuration
// #define HOSTNAME    "eetkamer-kachel"
#define HOSTNAME    "woonkamer-kachel"
#define MYWIFI      "Bennekom58"
#define MYPASS      "kIfudood4Frr"
// #define BLYNK_AUTH  "cd6baccbedca4894b7a765c1704f10de" // EETKAMER
#define BLYNK_AUTH  "tl_Yu0HMzdyPShmhkeqq8Iip8dnds1zl" // WOONKAMER
#define DEBUG       true
#define DEBUG_BPS   115200

// Pins configuration (do not change, this the only usable config)
#define THERMO_PIN  A0
#define LOW_PIN      5  // GPIO  5 = D1
#define HIGH_PIN     4  // GPIO  4 = D2
#define DHT_PIN     14  // GPIO 14 = D5
#define BTN_PIN     12  // GPIO 12 = D6
#define FLAME_PIN   13  // GPIO 13 = D7

// Select Blynk debug output
#if DEBUG
  #define BLYNK_PRINT Serial
#endif

// Include libraries
#include <avr/pgmspace.h>  // Needed to store vars in program memory
#include <Arduino.h>  // Just in case
// #include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>  // Enables wireless firmware upload
#include <BlynkSimpleEsp8266.h>

// Initialize library instances
BlynkTimer timer;
// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 7200, 60000);

// Heater queue action identifiers
static const uint8_t QUEUE_LOW  PROGMEM = 0x02  // B00000010
static const uint8_t QUEUE_HIGH PROGMEM = 0x05  // B00000101

// Blynk default colors
static const int8_t BLYNK_WHITE[]  PROGMEM = "#FFFFFF";
static const int8_t BLYNK_GREY[]   PROGMEM = "#444444";
static const int8_t BLYNK_GREEN[]  PROGMEM = "#23C48E";
static const int8_t BLYNK_BLUE[]   PROGMEM = "#04C0F8";
static const int8_t BLYNK_YELLOW[] PROGMEM = "#ED9D00";
static const int8_t BLYNK_RED[]    PROGMEM = "#D3435C";
static const int8_t BLYNK_PURPLE[] PROGMEM = "#5F7CD8";
static const int8_t BLYNK_BLACK[]  PROGMEM = "#000000";

// Global variables
int8_t heater_queue;
bool fan,
     flame,
     heating_low,
     heating_high,
     is_first_connect,
     thermostate_active;
int16_t humidity,
        temperature,
        thermo_knob_last,
        thermostate_temp;
uint32_t thermostate_last_temp

void switchFanOn() { switchFan(true); }
void switchFanOff() { switchFan(false); }
void switschFlameOn() { switchFlame(true); }
void switchFlameOff() { switchFlame(false); }
void switchLowOn() { switchLow(true); }
void switchLowOff() { switchLow(false); }
void switchHighOn() { switchHigh(true); }
void switchHighOff() { switchHigh(false); }

void lockHeaterQueue() {
  // Lock execution of queue, by changing MSB to 1 (negative)
  heater_queue = heater_queue | 0x80;  // 0x80 = 1B0000000
}

void unlockHeaterQueue() {
  // Unlock execution of queue, by changing MSB to 0 (positive)
  heater_queue = heater_queue & 0x7F;  // 0x7F = 0B1111111

  // Shift current action bits out from queue
  heater_queue = heater_queue >> 4;
}

void processHeaterQueue() {
  // Execute current queue action...
  if (heater_queue > 0) {  // ...only when queue is not locked
    current_action = heater_queue & 0x0F;  // 0x0F = B00001111

    switch (current_action) {
      case QUEUE_LOW:
        lockHeaterQueue();
        switchLow(!heating_low);
        break;
      case QUEUE_HIGH:
        lockHeaterQueue();
        switchHigh(!heating_high);
        break;
      default:
        BLYNK_LOG(PSTR("Heater queue corrupted, clearing it"));
        heater_queue = 0x00;
        break;
    }
  }
}

void addHeaterQueue(uint8_theater_state) {
  // When queue busy, prepare to add the action to next queue position
  if (heater_queue < 0) heater_state = heater_state << 4;

  // Store the new action in queue, but don't touch anything else
  heater_queue = (heater_queue & 0x8F) | heater_state;  // 0x8F = 10001111
}

void switchFlame(bool fl) {
  digitalWrite(FLAME_PIN, fl);
  flame = fl;

  BLYNK_LOG(PSTR("Flame is switched") + flame ? "on" : "off");
}

void switchFan(bool fs) {
  digitalWrite(FAN_PIN, fs);
  fan = fs;

  BLYNK_LOG(PSTR("Fan is switched") + fs ? "on" : "off");
}

void switchLow(bool low_state) {
  if (low_state) {
    if (heating_high) {
      switchHigh(false);
    }
    else {
      digitalWrite(LOW_PIN, HIGH);
      heating_low = true;
      BLYNK_LOG(PSTR("Heating low switched on"));
    }
  }
  else {
    if (heating_high) {
      switchHigh(false);
      timer.setTimeout(15000L, switchHighOff);
    }
    else {
      digitalWrite(LOW_PIN, LOW);
      heating_low = false;
      BLYNK_LOG(PSTR("Heating low switched off"));
    }
  }
}

void switchHigh(bool high_state) {
  if (high_state) {
    if (heating_low) {
      digitalWrite(HIGH_PIN, HIGH);
      heating_high = true;
      BLYNK_LOG(PSTR("Heating high switched on"));
    }
    else {
      switchLow(true);
      timer.setTimeout(15000L, switchHighOn);
    }
  }
  else {
    digitalWrite(HIGH_PIN, LOW);
    heating_high = false;
    BLYNK_LOG(PSTR("Heating high switched off"));
  }
}

void switchThermostate(bool thermo_state) {
  thermostate_active = thermo_state;
  uiSwitchColors(thermo_state ? true : false);
  
  if (thermo_state) {
    BLYNK_LOG(PSTR("Thermostate enabled"));
    thermostate();
  }
  else {
    if (heating_low || heating_high) {
      switchLow(false);
    }
    BLYNK_LOG(PSTR("Thermostate disabled"));
  }
}

void thermostate() {
  int8_t temp_delta = thermostate_temp - temperature;

  if (temp_delta >= 13) {
    Blynk.virtualWrite(V8, HIGH);
    switchHigh(true);
    BLYNK_LOG(PSTR("Thermostate started heating high"));
  }
  else if (temp_delta >= -7) {
    Blynk.virtualWrite(V7, HIGH);
    switchLow(true);
    BLYNK_LOG(PSTR("Thermostate started heating low"));
  }
  else {
    Blynk.virtualWrite(V7, LOW);
    switchLow(false);
    BLYNK_LOG(PSTR("Thermostate stopped heating"));
  }
}

void buttonTimer() {
  bool btn1_state = digitalRead(BTN1_PIN);
  if (!btn1_state) switchFlame(!flame_state);

  bool btn2_state = digitalRead(BTN2_PIN);
  if (!btn2_state) {
    if (heating_high) switchHighOff();
    else if (heating_low) switchHighOn();
    else switchLowOn();
  }

  int16_t thermo_knob = analogRead(BTN_PIN);
  if (thermo_knob != thermo_knob_last) {

    timer.setTimeout(15000L, switchHighOff);
  }
  thermostate_temp = param.asInt() * 10;
  BLYNK_LOG("Temperature is set to %d°C", thermostate_temp / 10);
  
  if (thermostate_active) thermostate();
}

void checkConn() {
  if (!Blynk.connected()){
    yield();
    if (WiFi.status() != WL_CONNECTED) {
      BLYNK_LOG(PSTR("Not connected to WiFi! Re-connecting..."));
      Blynk.connectWiFi(MYWIFI, MYPASS);

      delay(400); // Give it some time to connect

      if (WiFi.status() != WL_CONNECTED) {
        BLYNK_LOG(PSTR("Cannot connect to WiFi!"));
      }
      else {
        BLYNK_LOG(PSTR("Connected to WiFi!"));
      }
    }

    if (WiFi.status() == WL_CONNECTED && !Blynk.connected()) {
      BLYNK_LOG("Not connected to Blynk Server! Connecting...");
      Blynk.connect();  // It has 3 attempts of the defined BLYNK_TIMEOUT_MS to connect to the server, otherwise it goes to the enxt line 
      if (!Blynk.connected()) {
        BLYNK_LOG(PSTR("Connection to Blynk Server failed!"));
      }
      else {
        BLYNK_LOG(PSTR("Connected to Blynk Server!"));
        ArduinoOTA.begin();
        BLYNK_LOG(PSTR("Wireless OTA update initialized"));
      }
    }
  }
}

void uiSwitchColors(bool enabled) {
  if (enabled) {
    Blynk.setProperty(V4, "color", FPSTR(BLYNK_YELLOW));
    Blynk.setProperty(V5, "color", FPSTR(BLYNK_YELLOW));
    Blynk.setProperty(V6, "color", FPSTR(BLYNK_GREY));
    Blynk.setProperty(V7, "color", FPSTR(BLYNK_GREY));
    Blynk.setProperty(V8, "color", FPSTR(BLYNK_GREY));
  }
  else {
    Blynk.setProperty(V4, "color", FPSTR(BLYNK_GREY));
    Blynk.setProperty(V5, "color", FPSTR(BLYNK_GREY));
    Blynk.setProperty(V6, "color", FPSTR(BLYNK_YELLOW));
    Blynk.setProperty(V7, "color", FPSTR(BLYNK_YELLOW));
    Blynk.setProperty(V8, "color", FPSTR(BLYNK_YELLOW));
  }
}

void readDHT() {
  // Init buffer variables to receive data
  uint8_t bits[5];
  uint8_t idx = 0;
  uint8_t mask = 128;
  uint16_t loop_cnt = F_CPU / 40000;

  // Request a sample
  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, LOW);
  delay(1);
  digitalWrite(DHT_PIN, HIGH);
  delayMicroseconds(40);
  pinMode(DHT_PIN, INPUT);

  // Get Acknowledge or time-out
  while(!digitalRead(DHT_PIN)) {
    if (--loop_cnt == 0) {
      BLYNK_LOG(PSTR("Error! DHT data request is timeouted"));
      return;
    }
  }

  loop_cnt = F_CPU / 40000;
  while(digitalRead(DHT_PIN)) {
    if (--loop_cnt == 0) {
      BLYNK_LOG(PSTR("Error! DHT data request is timeouted"));
      return;
    }
  }

  // Read the output - 40 bits => 5 bytes
  for (uint8_t i=40; i!=0; i--) {
    loop_cnt = F_CPU / 40000;
    while(!digitalRead(DHT_PIN)) {
      if (--loop_cnt == 0) {
      BLYNK_LOG(PSTR("Error! DHT data request is timeouted"));
      return;
      }
    }

    unsigned long t = micros();

    loop_cnt = F_CPU / 40000;
    while(digitalRead(DHT_PIN)) {
      if (--loop_cnt == 0) {
      BLYNK_LOG(PSTR("Error! DHT data request is timeouted"));
      return;
      }
    }

    if ((micros() - t) > 40) {
      bits[idx] |= mask;
    }
    mask >>= 1;
    if (mask == 0) {  // next byte?
      mask = 128;
      idx++;
    }
  }

  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, HIGH);

  // Test checksum
  uint8_t sum = bits[0] + bits[1] + bits[2] + bits[3];
  if (bits[4] == sum) {
    humidity = word(bits[0], bits[1]);
    temperature = (bits[2] & 0x80 ? (word(bits[2] & 0x7F, bits[3]) * -1) : word(bits[2] & 0x7F, bits[3]));

    Blynk.virtualWrite(2, temperature * 0.1);
    Blynk.virtualWrite(3, humidity * 0.1);
    BLYNK_LOG("Sent DHT data (temp: %.1f°C, hum: %d%%", temperature * 0.1, humidity);
  }
  else {
    BLYNK_LOG(PSTR("Error! DHT data checksum invalid"));
  }
}

void mainTimer() {
  readDHT();

  if (thermostate_active) thermostate();

  checkConn();

  // timeClient.update();
}


void setup() {
  #if DEBUG
    #ifndef DEBUG_BPS
      #define DEBUG_BPS 115200
    #endif
    Serial.begin(DEBUG_BPS);
    Serial.println();
    BLYNK_LOG(PSTR("Serial debug activated"));
  #endif
  
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(THERMO_PIN, INPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(FLAME_PIN, OUTPUT);
  pinMode(LOW_PIN, OUTPUT);
  pinMode(HIGH_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  BLYNK_LOG(PSTR("Init IO pins succesful"));

  WiFi.hostname(HOSTNAME);
  Blynk.begin(BLYNK_AUTH, MYWIFI, MYPASS);
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.begin();
  BLYNK_LOG(PSTR("Wireless OTA update initialized"));

  mainTimer();
  timer.setInterval(60000L, mainTimer);
  BLYNK_LOG(PSTR("Main timer launched"));

  // buttonTimer();
  // timer.setInterval(100, buttonTimer);
  // BLYNK_LOG(PSTR("Button timer launched"));

  // thermoTimer();
  // timer.setInterval(3000, thermoTimer);
  // BLYNK_LOG(PSTR("Thermostate-knob timer launched"));

  // timeClient.begin();
}


BLYNK_CONNECTED() {
  BLYNK_LOG(PSTR("Connection alive"));

  if (is_first_connect) {
    Blynk.syncAll();  // this sets everything regard the server (i.p. remote)
    is_first_connect = false;  // reset the flag, to avoid flood
    BLYNK_LOG(PSTR("Synced with Blynk-server"));
   }
}

// FLAME switch
BLYNK_WRITE(V6) {
  if (!thermostate_active)
    switchFlame(param.asInt());
}

// LOW switch
BLYNK_WRITE(V7) {
  if (!thermostate_active)
    // switchLow(param.asInt());
    addHeaterQueue(QUEUE_LOW);
}

// HIGH switch
BLYNK_WRITE(V8) {
  if (!thermostate_active)
    // switchHigh(param.asInt());
    addHeaterQueue(QUEUE_HIGH);
}

// THERMOSTATE switch
BLYNK_WRITE(V9) {
  switchThermostate(param.asInt());
}

void setThermostateTemp(int16_t tt) {
  if (thermostate_temp_timer) {
    timer.restartTimer(thermostate_temp_timer);
  }
  else {
    thermostate_temp_timer = timer.setTimeout(3000, storeThermostateTemp);
  }

void storeThermostateTemp() {
  thermostate_temp = thermo_knob_last
}

  thermostate_temp = tt;
  thermostate_last_temp = millis();
}

void processThermostateTemp() {
  if ((uint16_t)(millis() - thermostate_last_temp) >= 3000) {
    thermostate_temp = 

  }
}

// THERMOSTATE TEMPERATURE input
BLYNK_WRITE(V5) {
  check if 
  timer.setInterval(100, buttonTimer);
  thermostate_temp = param.asInt() * 10;


  BLYNK_LOG("Temperature is set to %d°C", thermostate_temp / 10);
  
  if (thermostate_active) thermostate();
}

// TIMER widget
BLYNK_WRITE(V4) {}

void loop() {
  if (Blynk.connected) Blynk.run();
  else // try to reconnect every nth second
  
  processHeaterQueue();
  timer.run();
  
  if (wifi is connected) ArduinoOTA.handle();
}
