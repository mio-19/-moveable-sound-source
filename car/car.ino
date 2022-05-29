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
unsigned long interval = 30000;

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

void setup() {
  Serial.begin(115200);
  initWiFi();
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
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
    Serial.println("访问成功");

    Trans next;

    if (client.read((uint8_t *) &next, sizeof(Trans))==sizeof(Trans)){
      got = next;
      Serial.print("读取到数据：");
      printTrans(got);
    } else {
      Serial.print("No Data");
    }
        client.stop();
  } else {
        Serial.println("访问失败");
        client.stop();
    }
}
