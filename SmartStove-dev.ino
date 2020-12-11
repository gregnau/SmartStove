/* SmartStove
 *  powered with Blynk (http://www.blynk.cc)
 * 
 * Electric heater upgrade with WiFi connectivity,
 * temperature/humidity sensing, remote control and
 * thermostate functions.
 * 
 * GregNau © 2016-2020
 */

/* Software configuration */
#define HOSTNAME "eetkamer-kachel"
//#define HOSTNAME "woonkamer-kachel"
#define MYWIFI "Bennekom58"
#define MYPASS "kIfudood4Frr"
#define BLYNK_AUTH "cd6baccbedca4894b7a765c1704f10de" // EETKAMER
//#define BLYNK_AUTH "I3rdBE1yYzbivTm_8Sxmwpu8MKeMPIaS" // WOONKAMER
#define DEBUG true
#define DEBUG_BPS 115200


/* Pins configuration */
#define LOW_PIN 5 // GPIO 5 = D1
#define HIGH_PIN 4 // GPIO 4 = D2
#define DHT_PIN 14 // GPIO 14 = D5
#define BTN_PIN 12 // GPIO 12 = D6
#define FLAME_PIN 13 // GPIO 13 = D7
#define THERMO_PIN A0

/* Some shorthand aliases */
#define QUEUE_LOW_ON 8
#define QUEUE_LOW_OFF 16
#define QUEUE_HIGH_ON 32
#define QUEUE_HIGH_OFF 64
#define HOURS 0
#define MINUTES 1

/* Set Blynk debug output */
#if DEBUG
  #define BLYNK_PRINT Serial
#endif

/* Libraries */
#include <avr/pgmspace.h> // Needed to store vars in program memory
#include <Arduino.h> // Just in case, idk why its here
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ArduinoOTA.h> // Enables wireless firmware update
#include <BlynkSimpleEsp8266.h> // Blynk Library

/* Library instances */
WiFiUDP ntpUdp;
NTPClient timeClient(ntpUdp, 3600);
BlynkTimer timer;

// Blynk default colors [can be referenced like: FPSTR(BLYNK_COLOR)]
static const int8_t BLYNK_WHITE[8] PROGMEM = "#FFFFFF";
static const int8_t BLYNK_GREY[8] PROGMEM = "#444444";
static const int8_t BLYNK_GREEN[8] PROGMEM = "#23C48E";
static const int8_t BLYNK_BLUE[8] PROGMEM = "#04C0F8";
static const int8_t BLYNK_YELLOW[8] PROGMEM = "#ED9D00";
static const int8_t BLYNK_RED[8] PROGMEM = "#D3435C";
static const int8_t BLYNK_PURPLE[8] PROGMEM = "#5F7CD8";
static const int8_t BLYNK_BLACK[8] PROGMEM = "#000000";

/* Global variables */
bool is_first_connect = true;

bool hq_lock = false;
uint8_t heater_queue[2] = {0};

bool heating_low = false;
bool heating_high = false;

int16_t humidity = 0;
int16_t temperature = 0;

bool thermostate_active = false;
int16_t thermo_knob_last = 0;
int16_t thermostate_temp = 0;
int16_t thermostate_temp_timer = 0;

bool thermotimer_has_start = false;
bool thermotimer_has_stop = false;
bool thermotimer_days[7] = {false};  // in order: SUN, MON, TUE, WED, THU, FRI, SAT
int8_t thermotimer_start[2] = {-1};  // in order: HOURS, MINUTES
int8_t thermotimer_stop[2] = {-1};  // in order: HOURS, MINUTES


/* Functions */
void switchLowOn() {
  digitalWrite(LOW_PIN, HIGH);
  heating_low = true;
  Blynk.virtualWrite(V7, HIGH);
  BLYNK_LOG(PSTR("Heating low switched on"));
}

void switchLowOff() {
  digitalWrite(LOW_PIN, LOW);
  heating_low = false;
  Blynk.virtualWrite(V7, LOW);
  BLYNK_LOG(PSTR("Heating low switched off"));
}

void switchHighOn() {
  digitalWrite(HIGH_PIN, HIGH);
  heating_high = true;
  Blynk.virtualWrite(V8, HIGH);
  BLYNK_LOG(PSTR("Heating high switched on"));
}

void switchHighOff() {
  digitalWrite(HIGH_PIN, LOW);
  heating_high = false;
  Blynk.virtualWrite(V8, LOW);
  BLYNK_LOG(PSTR("Heating high switched off"));
}

void lockHeaterQueue() {
  hq_lock = true;
}

void unlockHeaterQueue() {
  hq_lock = false;
  heater_queue[0] = heater_queue[1];
  heater_queue[1] = 0x00;
}

void processHeaterQueue() {
  // Execute current queue action...
  if (heater_queue[0]) {
    if (!hq_lock) {  // ...only when queue is not locked
      uint8_t current_action = heater_queue[0];
      
      switch (current_action) {
        case QUEUE_LOW_ON:
          switchLow(true);
          break;
        case QUEUE_LOW_OFF:
          switchLow(false);
          break;
        case QUEUE_HIGH_ON:
          switchHigh(true);
          break;
        case QUEUE_HIGH_OFF:
          switchHigh(false);
          break;
        default:
          BLYNK_LOG(PSTR("Heater queue corrupted, clearing it"));
          memset(heater_queue, 0, sizeof(heater_queue));
          break;
      }
    }
  }
}

void addHeaterQueue(uint8_t hs) {
  heater_queue[hq_lock] = hs;
}

void switchFlame(bool fl) {
  digitalWrite(FLAME_PIN, fl);
  BLYNK_LOG(fl ? PSTR("Flame is switched on") : PSTR("Flame is switched off"));
}

void switchLow(bool ls) {
  lockHeaterQueue();
  
  if (ls) {  // switch ON
    if (!heating_low) {
      switchLowOn();
      unlockHeaterQueue();
    }
    else {
      unlockHeaterQueue();
    }
  }
  else {  // switch OFF
    if (heating_low && !heating_high) {
      switchLowOff();
      unlockHeaterQueue();
    }
    else if (heating_high) {
      switchHighOff();
      addHeaterQueue(QUEUE_LOW_OFF);
      timer.setTimeout(5000, unlockHeaterQueue);
    }
    else {
      unlockHeaterQueue();
    }
  }
}

void switchHigh(bool hs) {
  lockHeaterQueue();
  
  if (hs) {  // switch ON
    if (heating_low && !heating_high) {
      switchHighOn();
      unlockHeaterQueue();
    }
    else if (!heating_low) {
      switchLowOn();
      addHeaterQueue(QUEUE_HIGH_ON);
      timer.setTimeout(5000, unlockHeaterQueue);
    }
  }
  else {  // switch OFF
    if (heating_high) {
      switchHighOff();
      timer.setTimeout(3000, unlockHeaterQueue);
    }
    else {
      unlockHeaterQueue();
    }
  }
}

void switchThermostate(bool thermo_state) {
  thermostate_active = thermo_state;
  uiSwitchColors(thermo_state);
  
  if (thermo_state) {
    BLYNK_LOG(PSTR("Thermostate enabled"));
    thermostate();
  }
  else {
    if (heating_low || heating_high) {
      addHeaterQueue(QUEUE_LOW_OFF);
      Blynk.virtualWrite(7, LOW);
    }
    BLYNK_LOG(PSTR("Thermostate disabled"));
  }
}

void thermostate() {
  int16_t temp_delta = temperature - thermostate_temp;
  
  if (temp_delta <= 2 && !heating_high) {
    addHeaterQueue(QUEUE_HIGH_ON);
    BLYNK_LOG(PSTR("Thermostate started heating high"));
  }
  else if (temp_delta >= 6 && heating_high) {
    addHeaterQueue(QUEUE_HIGH_OFF);
    BLYNK_LOG(PSTR("Thermostate started heating low"));
  }
  else if (temp_delta >= 8 && heating_low) {
    addHeaterQueue(QUEUE_LOW_OFF);
    BLYNK_LOG(PSTR("Thermostate reached target temperature"));
  }
}

void setThermostateTemp(int16_t tt) {
  thermo_knob_last = tt;
  if (thermostate_temp_timer > 0) {
    timer.restartTimer(thermostate_temp_timer);
  }
  else {
    thermostate_temp_timer = timer.setTimeout(3000, activateThermostateTemp);
  }
}

void activateThermostateTemp() {
  thermostate_temp = thermo_knob_last;
  BLYNK_LOG("Thermostate is set to %d°C", thermostate_temp / 10);
  
  thermostate_temp_timer = 0;
  
  if (thermostate_active) thermostate();
}

void thermoTimer() {
  if (thermotimer_has_start) {
    if (thermotimer_days[timeClient.getDay()]) {
      if (thermotimer_start[HOURS] == timeClient.getHours()) {
        if (thermotimer_start[MINUTES] == timeClient.getMinutes()) {
          if (!thermostate_active) {
            switchThermostate(true);
            Blynk.virtualWrite(V9, HIGH);
          }
        }
      }
    }
  }

  if (thermotimer_has_stop) {
    if (thermotimer_days[timeClient.getDay()]) {
      if (thermotimer_stop[HOURS] == timeClient.getHours()) {
        if (thermotimer_stop[MINUTES] == timeClient.getMinutes()) {
          if (thermostate_active) {
            switchThermostate(false);
            Blynk.virtualWrite(V9, LOW);
          }
        }
      }
    }
  }
}

void checkConn() {
  if (!Blynk.connected()){
    yield();
    if (WiFi.status() != WL_CONNECTED) {
      BLYNK_LOG(PSTR("Not connected to WiFi, Re-connecting..."));
      Blynk.connectWiFi(MYWIFI, MYPASS);
      
      delay(400); // Give it some time to connect
      
      if (WiFi.status() != WL_CONNECTED) {
        BLYNK_LOG(PSTR("Cannot connect to WiFi"));
      }
      else {
        BLYNK_LOG(PSTR("Connected to WiFi"));
      }
    }
    
    if (WiFi.status() == WL_CONNECTED && !Blynk.connected()) {
      BLYNK_LOG(PSTR("Not connected to Blynk Server! Connecting..."));
      Blynk.connect();  // It has 3 attempts of the defined BLYNK_TIMEOUT_MS to connect to the server, otherwise it goes to the next line 
      if (!Blynk.connected()) {
        BLYNK_LOG(PSTR("Connection to Blynk Server failed"));
      }
      else {
        BLYNK_LOG(PSTR("Connected to Blynk Server"));
        ArduinoOTA.begin();
        BLYNK_LOG(PSTR("Wireless OTA update initialized"));
      }
    }
  }
}

void uiSwitchColors(bool enabled) {
  if (enabled) {
    Blynk.setProperty(V4, "color", FPSTR(BLYNK_GREEN));
    Blynk.setProperty(V5, "color", FPSTR(BLYNK_GREEN));
    Blynk.setProperty(V7, "color", FPSTR(BLYNK_GREY));
    Blynk.setProperty(V8, "color", FPSTR(BLYNK_GREY));
  }
  else {
    Blynk.setProperty(V4, "color", FPSTR(BLYNK_GREY));
    Blynk.setProperty(V5, "color", FPSTR(BLYNK_GREY));
    Blynk.setProperty(V7, "color", FPSTR(BLYNK_GREEN));
    Blynk.setProperty(V8, "color", FPSTR(BLYNK_GREEN));
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

  thermoTimer();
  
  checkConn();
}

void safetyTimer() {
  if (heating_high && !heating_low) {
    digitalWrite(HIGH_PIN, LOW);
    heating_high = false;
  }
}


/* Setup */
void setup() {
  #if DEBUG
    #ifndef DEBUG_BPS
      #define DEBUG_BPS 115200
    #endif
    Serial.begin(DEBUG_BPS);
    Serial.println();
    BLYNK_LOG(PSTR("Serial debug activated"));
  #endif
  
  pinMode(FLAME_PIN, OUTPUT);
  pinMode(LOW_PIN, OUTPUT);
  pinMode(HIGH_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(THERMO_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  BLYNK_LOG(PSTR("Init IO pins succesful"));

  WiFi.hostname(HOSTNAME);
  Blynk.begin(BLYNK_AUTH, MYWIFI, MYPASS);
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.begin();
  BLYNK_LOG(PSTR("Wireless OTA update initialized"));

  timeClient.begin();

  mainTimer();
  timer.setInterval(60000L, mainTimer);
  BLYNK_LOG(PSTR("Main timer launched"));

  safetyTimer();
  timer.setInterval(1000, safetyTimer);

  // buttonTimer();
  // timer.setInterval(100, buttonTimer);
  // BLYNK_LOG(PSTR("Button timer launched"));

  // thermoTimer();
  // timer.setInterval(3000, thermoTimer);
  // BLYNK_LOG(PSTR("Thermostate-knob timer launched"));
}


/* Blynk service routines */
BLYNK_CONNECTED() {
  BLYNK_LOG(PSTR("Connected to Blynk Server"));

  if (is_first_connect) {
    Blynk.syncAll();  // this sets everything regard the server (i.p. remote)
    is_first_connect = false;  // reset the flag, to avoid flood
    BLYNK_LOG(PSTR("Synced with Blynk-server"));
   }
}

// FLAME switch
BLYNK_WRITE(V6) {
  switchFlame(param.asInt());
}

// LOW switch
BLYNK_WRITE(V7) {
  if (!thermostate_active) {
    addHeaterQueue(param.asInt() ? QUEUE_LOW_ON : QUEUE_LOW_OFF);
  }
  else {
    Blynk.virtualWrite(V7, !param.asInt());
  }
}

// HIGH switch
BLYNK_WRITE(V8) {
  if (!thermostate_active) {
    addHeaterQueue(param.asInt() ? QUEUE_HIGH_ON : QUEUE_HIGH_OFF);
  }
  else {
    Blynk.virtualWrite(V8, !param.asInt());
  }
}

// THERMOSTATE switch
BLYNK_WRITE(V9) {
  switchThermostate(param.asInt());
}

// THERMOSTATE TEMPERATURE input
BLYNK_WRITE(V5) {
  setThermostateTemp(param.asInt() * 10);
}

// THERMOSTATE TIMER widget
BLYNK_WRITE(V4) {
  TimeInputParam t(param);
  char dbg_buf[67];

  // Process weekdays (1. Mon, 2. Tue, 3. Wed, 4. Thu, 5. Fri, 6. Sat, 7. Sun)
  if (t.hasStartTime() || t.hasStopTime()) {
    uint8_t iwsc;
    
    sprintf(dbg_buf, "Thermostate timer is set (");
    
    for (uint8_t a = 1; a <= 7; a++) {
      thermotimer_days[(a<7) ? a : 0] = t.isWeekdaySelected(a);
      iwsc += t.isWeekdaySelected(a);
    }

    if (iwsc > 0) {
      sprintf(dbg_buf,
                 "%sdays: %d%d%d%d%d%d%d, ",
                 dbg_buf,
                 thermotimer_days[0],
                 thermotimer_days[1],
                 thermotimer_days[2],
                 thermotimer_days[3],
                 thermotimer_days[4],
                 thermotimer_days[5],
                 thermotimer_days[6]
      );
    }
  }
  else {
    BLYNK_LOG(PSTR("Thermostate timer is disabled"));
  }
  
  // Process start time
  if (t.hasStartTime()) {
    thermotimer_has_start = true;
    thermotimer_start[HOURS] = t.getStartHour();
    thermotimer_start[MINUTES] = t.getStartMinute();
    sprintf(dbg_buf, "%sstart: %d:%d, ", dbg_buf, thermotimer_start[HOURS], thermotimer_start[MINUTES]);
  }
  else {
    thermotimer_has_start = false;
    thermotimer_start[HOURS] = -1;
    thermotimer_start[MINUTES] = -1;
  }

  // Process stop time
  if (t.hasStopTime()) {
    thermotimer_has_stop = true;
    thermotimer_stop[HOURS] = t.getStopHour();
    thermotimer_stop[MINUTES] = t.getStopMinute();
    sprintf(dbg_buf, "%sstop: %d:%d", dbg_buf, thermotimer_stop[HOURS], thermotimer_stop[MINUTES]);
  }
  else {
    thermotimer_has_stop = false;
    thermotimer_stop[HOURS] = -1;
    thermotimer_stop[MINUTES] = -1;
  }

  if (t.hasStartTime() || t.hasStopTime()) {
    sprintf(dbg_buf, "%s)", dbg_buf);
    BLYNK_LOG(dbg_buf);
  }
}


/* Main Loop */
void loop() {
  if (Blynk.connected()) Blynk.run();
//  else // try to reconnect every nth second
  
  processHeaterQueue();
  
  timer.run();

  timeClient.update();
  
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }
}
