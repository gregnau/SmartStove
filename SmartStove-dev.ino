/* SmartStove
 *  powered with Blynk (http://www.blynk.cc)
 * 
 * Electric heater upgrade with WiFi connectivity,
 * temperature/humidity sensing, remote control and
 * thermostate functions.
 * 
 * GregNau © 2016-2020
 */

// Configuration
// #define   HOSTNAME      "eetkamer-kachel"
#define   HOSTNAME      "woonkamer-kachel"
#define   MYWIFI        "Bennekom58"
#define   MYPASS        "kIfudood4Frr"
// #define   BLYNK_AUTH    "cd6baccbedca4894b7a765c1704f10de" // EETKAMER
#define   BLYNK_AUTH    "tl_Yu0HMzdyPShmhkeqq8Iip8dnds1zl" // WOONKAMER
#define   THERMO_PIN    A0
#define   LOW_PIN       5  // GPIO 5 = D1
#define   HIGH_PIN      4  // GPIO 4 = D2
#define   DHT_PIN       14  // GPIO 14 = D5
#define   BTN_PIN       12  // GPIO 12 = D6
#define   FLAME_PIN     13  // GPIO 13 = D7
#define   DEBUG         true
#define   DEBUG_BPS     115200

// Select Blynk debug output
#if DEBUG
  #define BLYNK_PRINT Serial
#endif

// Include libraries
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <BlynkSimpleEsp8266.h>

// Initialize library instances
BlynkTimer timer;
IRrecv ir(IR_PIN);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 7200, 60000);

// Heater queue action identifiers
static const byte QUEUE_LOW  PROGMEM = 0x02  // B00000010
static const byte QUEUE_HIGH PROGMEM = 0x05  // B00000101

// Raw IR button codes
static const unsigned int IR_BTN_PWR[]   PROGMEM = { 9050, 4500, 600, 500, 600, 550, 600, 550, 600, 500, 600, 550, 600, 550, 550, 550, 600, 550, 600, 1650, 600, 1700, 550, 1700, 550, 1700, 600, 1700, 550, 1700, 550, 1700, 600, 1650, 600, 550, 600, 500, 600, 1700, 600, 500, 600, 550, 600, 550, 600, 1650, 600, 1650, 600, 1650, 600, 1700, 550, 550, 600, 1700, 550, 1700, 600, 1650, 600, 550, 550, 600, 550};  // Protocol=NEC Data=0xFF23DC
static const unsigned int IR_BTN_FLAME[] PROGMEM = { 9050, 4500, 600, 550, 600, 550, 550, 550, 600, 550, 550, 600, 550, 550, 600, 550, 550, 600, 550, 1700, 550, 1700, 600, 1650, 600, 1650, 600, 1700, 600, 1650, 600, 1650, 600, 1650, 600, 1700, 600, 500, 600, 550, 600, 550, 550, 550, 600, 550, 600, 1650, 600, 1700, 550, 550, 600, 1700, 550, 1700, 550, 1700, 600, 1650, 600, 1650, 600, 550, 600, 550, 600};  // Protocol=NEC Data=0xFF837C
static const unsigned int IR_BTN_LOW[]   PROGMEM = { 9100, 4500, 600, 550, 550, 550, 600, 550, 600, 550, 550, 550, 600, 550, 550, 600, 550, 550, 600, 1700, 550, 1700, 550, 1700, 600, 1650, 600, 1650, 600, 1700, 600, 1650, 600, 1650, 600, 1700, 600, 500, 600, 550, 600, 1650, 600, 550, 600, 500, 600, 1700, 550, 1700, 600, 550, 550, 1700, 600, 1650, 600, 550, 550, 1700, 600, 1700, 550, 550, 600, 550, 550};  // Protocol=NEC Data=0xFF936C
static const unsigned int IR_BTN_HIGH[]  PROGMEM = { 9050, 4550, 550, 550, 600, 550, 550, 600, 550, 550, 600, 550, 550, 550, 600, 550, 600, 550, 550, 1700, 600, 1650, 600, 1650, 600, 1700, 600, 1650, 600, 1650, 600, 1650, 600, 1700, 600, 500, 600, 550, 600, 550, 550, 1700, 600, 550, 550, 550, 600, 1700, 550, 1700, 600, 1650, 600, 1700, 550, 1700, 550, 600, 550, 1700, 550, 1700, 600, 550, 550, 550, 600};  // Protocol=NEC Data=0xFF13EC

// Blynk default colors
static const char BLYNK_WHITE[]  PROGMEM = "#FFFFFF";
static const char BLYNK_GREY[]   PROGMEM = "#444444";
static const char BLYNK_GREEN[]  PROGMEM = "#23C48E";
static const char BLYNK_BLUE[]   PROGMEM = "#04C0F8";
static const char BLYNK_YELLOW[] PROGMEM = "#ED9D00";
static const char BLYNK_RED[]    PROGMEM = "#D3435C";
static const char BLYNK_PURPLE[] PROGMEM = "#5F7CD8";
static const char BLYNK_BLACK[]  PROGMEM = "#000000";

// Global variables
char heater_queue;
boolean is_first_connect = true;
int humidity, temperature, thermo_knob_last, thermostate_temp;
boolean fan, flame, heating_low, heating_high, thermostate_active;


void switchFanOn() { switchFan(true); }
void switchFanOff() { switchFan(false); }
void switchFlameOn() { switchFlame(true); }
void switchFlameOff() { switchFlame(false); }
void switchLowOn() { switchLow(true); }
void switchLowOff() { switchLow(false); }
void switchHighOn() { switchHigh(true); }
void switchHighOff() { switchHigh(false); }

void lockHeaterQueue() {
  // Lock execution of queue, by changing MSB to 1 (negative)
  heater_queue = heater_queue | 0x80;  // 0x80 = B10000000
}

void unlockHeaterQueue() {
  // Unlock execution of queue, by changing MSB to 0 (positive)
  heater_queue = heater_queue & 0x7F;  // 0x7F = B01111111

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

void addHeaterQueue(byte heater_state) {
  // When queue busy, prepare to add the action to next queue position
  if (heater_queue < 0) heater_state = heater_state << 4;

  // Store the new action in queue, but don't touch anything else
  heater_queue = (heater_queue & 0x8F) | heater_state;  // 0x8F = 10001111
}

void switchFlame(boolean flame_state) {
  if (flame_state) {
    digitalWrite(FLAME_PIN, HIGH);
    flame = true;
  }
  else {
    digitalWrite(FLAME_PIN, LOW);
    flame = false;
  }
  BLYNK_LOG(PSTR("Flame is switched") + flame ? "on" : "off");
}

void switchFan(boolean fan_state) {
  if (fan_state) {
    digitalWrite(FAN_PIN, HIGH);
    fan = true;
  }
  else {
    digitalWrite(FAN_PIN, LOW);
    fan = false;
  }
  BLYNK_LOG(PSTR("Fan is switched") + flame ? "on" : "off");
}

void switchLow(boolean low_state) {
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

void switchHigh(boolean high_state) {
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

void switchThermostate(boolean thermo_state) {
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
  boolean btn1_state = digitalRead(BTN1_PIN);
  if (!btn1_state) switchFlame(!flame_state);

  boolean btn2_state = digitalRead(BTN2_PIN);
  if (!btn2_state) {
    if (heating_high) switchHighOff();
    else if (heating_low) switchHighOn();
    else switchLowOn();
  }

  int thermo_knob = analogRead(BTN_PIN);
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

void uiSwitchColors(boolean enabled) {
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
  byte bits[5];
  byte idx = 0;
  byte mask = 128;
  unsigned int loop_cnt = F_CPU / 40000;

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
  for (byte i=40; i!=0; i--) {
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
  byte sum = bits[0] + bits[1] + bits[2] + bits[3];
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

  timeClient.update();
}


void setup() {
  #if DEBUG
    Serial.begin(DEBUG_BPS);
    Serial.println();
    BLYNK_LOG(PSTR("Serial debug activated"));
  #endif
  
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
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

  buttonTimer();
  timer.setInterval(100, buttonTimer);
  BLYNK_LOG(PSTR("Button timer launched"));

  thermoTimer();
  timer.setInterval(1500, thermoTimer);
  BLYNK_LOG(PSTR("Thermostate-knob timer launched"));

  timeClient.begin();
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

// THERMOSTATE TEMPERATURE input
BLYNK_WRITE(V5) {
  thermostate_temp = param.asInt() * 10;
  BLYNK_LOG("Temperature is set to %d°C", thermostate_temp / 10);
  
  if (thermostate_active) thermostate();
}

// TIMER widget
BLYNK_WRITE(V4) {}

// ██       ██████   ██████  ██████
// ██      ██    ██ ██    ██ ██   ██
// ██      ██    ██ ██    ██ ██████
// ██      ██    ██ ██    ██ ██
// ███████  ██████   ██████  ██

void loop() {
  Blynk.run();
  processHeaterQueue();
  timer.run();
  ArduinoOTA.handle();
}
