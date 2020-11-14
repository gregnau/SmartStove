#define FAN_PIN  13  // GPIO 13 = D7
#define LOW_PIN   5  // GPIO 5 = D1
#define HIGH_PIN  4  // GPIO 4 = D2
#define DHT_PIN  14  // GPIO 14 = D5

#include <dht.h>
#include <SimpleTimer.h>

dht DHT;
SimpleTimer timer;

int kamertemp;
bool thermostate_active = true;
bool heating_low = false;

void readDht() {
  int chk = DHT.read22(DHT_PIN);

  switch (chk) {
    case DHTLIB_OK:
      kamertemp = DHT.temperature * 10;
      Serial.println(kamertemp);
      break;
    case DHTLIB_ERROR_CHECKSUM:
      Serial.println("checksum-error");
      break;
    case DHTLIB_ERROR_TIMEOUT:
      Serial.println("timeout-error");
      break;
    default:
      Serial.println("general-error");
      break;
    }
}

void switchFan(bool fstate) {
  digitalWrite(FAN_PIN, fstate);
}

void switchFanOff() {
  digitalWrite(FAN_PIN, LOW);
}

void switchLowOn() {
  digitalWrite(LOW_PIN, HIGH);
  heating_low = true;
}

void switchLow(bool lstate) {
  if (lstate) {
    digitalWrite(FAN_PIN, HIGH);
    timer.setTimeout(20000, switchLowOn);
  }
  else {
    digitalWrite(LOW_PIN, LOW);
    heating_low = false;
    timer.setTimeout(40000L, switchFanOff);
  }
}

void thermostate() {
  if (kamertemp >= 212) {
    switchLow(false);
    Serial.println("heating off");
  }
  else if (kamertemp <= 204) {
    switchLow(true);
    Serial.println("heating on");
  }
}

void mainTimer() {
  Serial.println("maintimer");
  readDht();

  if (thermostate_active) {
    thermostate();
  }
}

void setup() {
  pinMode(FAN_PIN, OUTPUT);
  pinMode(LOW_PIN, OUTPUT);
  pinMode(HIGH_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(LOW_PIN, LOW);
  digitalWrite(HIGH_PIN, LOW);

  Serial.begin(115200);
  Serial.println("\nStarting...");

  mainTimer();
  timer.setInterval(10000, mainTimer);
}

void loop() {
  timer.run();
}
