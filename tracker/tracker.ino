#include <WiFi.h>


WiFiServer server;


// sent to the car
typedef struct Trans {
  int64_t girl;
} Trans;

void printTrans(Trans pretty) {
  Serial.print("Trans:");
  Serial.print(pretty.girl);
}

#define SENSOR_A 4
#define SENSOR_B 0
#define SENSOR_C 2

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP("TRACKER-WIFI", NULL, false, 4);
  Serial.println(WiFi.softAPIP());
  server.begin(233);
}

Trans tosend = (Trans) {.girl = 0};

void loop() {
  WiFiClient client = server.available();
  if (client) //如果当前客户可用
    {
        Serial.print("[Client connected]:");
        Serial.print(client.remoteIP());
        Serial.print(":");
        Serial.println(client.remotePort());
        client.write((const uint8_t *) &tosend, sizeof(Trans));
        Serial.print("Sent=");
        printTrans(tosend);
        Serial.println("");
        client.flush();
        client.stop();
        Serial.println("[Client disconnected]");
    }
}
