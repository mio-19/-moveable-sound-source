#include <WiFi.h>

// sent to the car
typedef struct Trans {
  uint64_t girl;
} Trans;

#define SENSOR_A 4
#define SENSOR_B 0
#define SENSOR_C 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("TRACKER-WIFI", NULL, false, 4);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void loop() {
  // put your main code here, to run repeatedly:

}
