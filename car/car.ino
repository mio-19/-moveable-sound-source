#include <WiFi.h>

typedef struct Trans {
  int64_t girl;
} Trans;

void printTrans(Trans pretty) {
  Serial.print("Trans:");
  Serial.print(pretty.girl);
}


Trans got = (Trans) {.girl = 0};

// Replace with your network credentials (STATION)
const char* ssid = "TRACKER-WIFI";
const char* password = NULL;


const IPAddress serverIP(192,168,4,1);
uint16_t serverPort = 233;
WiFiClient client;

 
unsigned long previousMillis = 0;
unsigned long interval = 1000;

// https://randomnerdtutorials.com/solved-reconnect-esp32-to-wifi/
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

#define ENGINE1_POSITIVE 34
#define ENGINE1_POSITIVE_CH 0
#define ENGINE1_NEGATIVE 35
#define ENGINE1_NEGATIVE_CH 1
#define ENGINE2_POSITIVE 32
#define ENGINE2_POSITIVE_CH 2
#define ENGINE2_NEGATIVE 33
#define ENGINE2_NEGATIVE_CH 3

#define BEEP_PIN 0
#define BEEP_VALID false

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

typedef enum {
    STATE_START,
    STATE_GOING,
    STATE_STOP
} State;

State state = STATE_START;

hw_timer_t * pid_timer = NULL;

void IRAM_ATTR pid_timer_callback() {
    // TODO
}

hw_timer_t * beep_timer = NULL;
void IRAM_ATTR beep_timer_callback() {
    static bool beeping = false;
    digitalWrite(BEEP_PIN, BEEP_VALID == beeping ? HIGH : LOW);
    beeping = !beeping;
}

void setup() {
  Serial.begin(115200);
  initWiFi();
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());

    ledcSetup(ENGINE1_POSITIVE_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ENGINE1_POSITIVE, ENGINE1_POSITIVE_CH);
    ledcWrite(ENGINE1_POSITIVE_CH, 0);
    ledcSetup(ENGINE1_NEGATIVE_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ENGINE1_NEGATIVE, ENGINE1_NEGATIVE_CH);
    ledcWrite(ENGINE1_NEGATIVE_CH, 0);
    ledcSetup(ENGINE2_POSITIVE_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ENGINE2_POSITIVE, ENGINE2_POSITIVE_CH);
    ledcWrite(ENGINE2_POSITIVE_CH, 0);
    ledcSetup(ENGINE2_NEGATIVE_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ENGINE2_NEGATIVE, ENGINE2_NEGATIVE_CH);
    ledcWrite(ENGINE2_NEGATIVE_CH, 0);

    pid_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(pid_timer, &pid_timer_callback, true);
    timerAlarmWrite(pid_timer, 1000, true);

    beep_timer = timerBegin(1, 80, true);
    timerAttachInterrupt(beep_timer, &beep_timer_callback, true);
    // 1000000 = 1s
    // 0.2s
    timerAlarmWrite(beep_timer, 1000000/5, true);

    pinMode(BEEP_PIN, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }

  if (client.connect(serverIP, serverPort)) {
    Trans next;

    while(client.connected() || client.available()) {
    if (client.read((uint8_t *) &next, sizeof(Trans))==sizeof(Trans)){
      got = next;
      Serial.print("读取到数据：");
      printTrans(got);
    }
    }

        client.stop();
  }
}
