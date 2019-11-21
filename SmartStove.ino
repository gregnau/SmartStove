/* SmartStove v1.2 (formerly IoT-Ketel :)
 * powered with Blynk (http://www.blynk.cc)
 * 
 * Electric heater upgrade with WiFi connectivity,
 * temperature/humidity sensing, remote control and
 * timer functions. Hopefully not going to burn the house down!
 * 
 * GregNau      2016
 */
// Pin Config
#define DHT_PIN 5  // DHT sensor connected to GPIO5
#define FLAME_PIN 14  // FLAME switch relay pin
#define LOW_PIN 12  // LOW heating relay pin
#define HIGH_PIN 13  // HIGH heating relay pin
#define BTN_PIN 16 // LOGIC button pin

// Blynk libraries
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// Simple timer library
#include <SimpleTimer.h>
SimpleTimer timer;  // init. timer library

// DHT Sensor library
#include <dht.h>
dht DHT;  // initialize DHT sensor library

// define some of the widgets
WidgetLED pwrled(V1);
WidgetLCD lcd(V10);

//char auth[] = "YourAuthToken";
//char mywifi[] = "NameOfYourWifi";
//char mypass[] = "YourWifiPass";
char auth[] = "cd6baccbedca4894b7a765c1704f10de";
char mywifi[] = "Ziggo56369";
char mypass[] = "kIfudood4Frr";

bool isFirstConnect = true;  // needed for sync on connection
bool flameOn, heatingLow, heatingHigh, thermoState;
int sliderPos, dhtData;

// function to read and pass data from DHT sensor
void updateHumTemp() {
  dhtData = DHT.read22(5);
  
  Blynk.virtualWrite(2, DHT.temperature);
  Blynk.virtualWrite(3, DHT.humidity);
}

// function to check logic button state
void checkLogicSwitch() {
  int switchState = digitalRead(BTN_PIN);

  // Hopefully this is going to trigger the BLYNK_WRITE(V6)
  // function below and set the flameState also.
  if (switchState && !flameOn) {
    Blynk.virtualWrite(V6, HIGH);
  } else if (!switchState && flameOn) {
    Blynk.virtualWrite(V6, LOW);
  }
}

void setup() {
  pinMode(FLAME_PIN, OUTPUT);
  pinMode(LOW_PIN, OUTPUT);
  pinMode(HIGH_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT);
  
  Blynk.begin(auth, mywifi, mypass);  // try to connect wifi and blynk server
  while (!Blynk.connect());  // wait here until connection alive

  pwrled.on();  // turn on powerled when connected
  
  // Print some greetings on the LCD
  lcd.clear();
  lcd.print(2, 0, "SmartStove :)");
  lcd.print(4, 1, "CONNECTED");
  
  // send initial temp/hum measurements
  updateHumTemp();
  // set a timer for reading+send the DHT sensor every 60s
  timer.setInterval(60000L, updateHumTemp);
}

// sync widgets after disconnect or reset
BLYNK_CONNECTED() {
  if (isFirstConnect) {
    Blynk.syncAll();  // this sets everything regard the remote
    isFirstConnect = false;  // reset the flag, to avoid flood
  }
}

// POWER LED safety function
// (since this led should indicate whether the device have power or not)
BLYNK_WRITE(V1) {
  int ledState = param.asInt();

  // do not let the led turned off while running and connected
  if (!ledState && Blynk.connect()) {
    Blynk.virtualWrite(1, HIGH);
  }
}

// FLAME switch
BLYNK_WRITE(V6) {
  int switchState = param.asInt();  // 0 when the button LOW, 1 when its HIGH
  
  if (switchState) {
    flameOn = true;  // set the flag
    digitalWrite(FLAME_PIN, HIGH);  // set the pin
    
    lcd.clear();
    lcd.print(3, 0, "FLAME - ON");
  } else {
    flameOn = false;
    digitalWrite(FLAME_PIN, LOW);
    
    lcd.clear();
    lcd.print(2, 0, "FLAME - OFF");
  }
}

// LOW switch
BLYNK_WRITE(V7) {
  int switchState = param.asInt();
  
  if (switchState) {
    heatingLow = true;
    digitalWrite(LOW_PIN, HIGH);
    
    lcd.clear();
    lcd.print(3, 0, "HEATING - LOW");
  } else if (!switchState && heatingHigh) {
    lcd.clear();
    lcd.print(0, 0, "FIRST TURN OFF");
    lcd.print(6, 1, "HIGH");
  } else {
    heatingLow = false;
    digitalWrite(LOW_PIN, LOW);
    
    lcd.clear();
    lcd.print(2, 0, "HEATING - OFF");
  }
}

// HIGH switch
BLYNK_WRITE(V8) {
  int switchState = param.asInt();
  
  if (switchState && heatingLow) {
    heatingHigh = true;
    digitalWrite(HIGH_PIN, HIGH);
    
    lcd.clear();
    lcd.print(3, 0, "HEATING - HIGH");
  } else if (switchState && !heatingLow) {
    lcd.clear();
    lcd.print(0, 0, "FIRST TURN ON");
    lcd.print(7, 1, "LOW");
  } else {
    heatingHigh = false;
    digitalWrite(HIGH_PIN, LOW);
    
    lcd.clear();
    lcd.print(2, 0, "HEATING - LOW");
  }
}

// THERMOSTAAT switch
BLYNK_WRITE(V9) {
  int switchState = param.asInt();

  if (switchState) {
    thermoState = true;

    if (sliderPos > DHT.temperature) {
      Blynk.virtualWrite(7, HIGH);
      Blynk.virtualWrite(8, HIGH);
    }
  } else {
    thermoState = false;
    
    Blynk.virtualWrite(8, LOW);
    Blynk.virtualWrite(7, LOW);
  }
}

// THERMOSTAAT slider
BLYNK_WRITE(V5) {
  sliderPos = param.asInt();
}

// TIMER widget
BLYNK_WRITE(V4) {
  int triggerState = param.asInt();

  if (triggerState && !heatingLow) {
    Blynk.virtualWrite(V7, HIGH);
    Blynk.virtualWrite(V8, HIGH);
  } else if (!triggerState && heatingLow) {
    Blynk.virtualWrite(V8, LOW);
    Blynk.virtualWrite(V7, LOW);
  }
}

void loop() {
  Blynk.run();
  timer.run();
  checkLogicSwitch();
}
