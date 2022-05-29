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

// in micros
// 0.1s
#define VALID_LENGTH 100000
// 0.1s
#define MAX_DIFF 100000

typedef unsigned long micros_t;

typedef struct {
    micros_t last_valid_start;
    micros_t last_valid_end;
    bool valid;
} Sensor;

Sensor sensor_a,sensor_b,sensor_c;

#define SENSOR_VALID 0

void init_Sensor(Sensor* x, int inPin) {
    micros_t t = micros();
    x->last_valid_start = t;
    x->last_valid_end = t;
    x->valid = false;
    pinMode(inPin, INPUT);
}

void IRAM_ATTR handle_Sensor(Sensor* x, int pin) {
    bool valid = digitalRead(pin) == SENSOR_VALID;
    micros_t t = micros();
    if (valid) {
        if (!x->valid) {
            x->last_valid_start = t;
        }
        x->last_valid_end = t;
    }
    x->valid = valid;
}

#define ABS(x) ((x) < 0 ? -(x) : (x))

bool IRAM_ATTR valid_state() {
    return ABS(sensor_a.last_valid_start-sensor_b.last_valid_start) < MAX_DIFF &&
           ABS(sensor_b.last_valid_start-sensor_c.last_valid_start) < MAX_DIFF &&
           ABS(sensor_c.last_valid_start-sensor_a.last_valid_start) < MAX_DIFF &&
            sensor_a.last_valid_end-sensor_a.last_valid_start > VALID_LENGTH &&
            sensor_b.last_valid_end-sensor_b.last_valid_start > VALID_LENGTH &&
            sensor_c.last_valid_end-sensor_c.last_valid_start > VALID_LENGTH;
}

Trans tosend = (Trans) {.girl = 0};

void IRAM_ATTR save_sensor() {
    tosend.girl = sensor_a.last_valid_start - sensor_b.last_valid_start;
}


void IRAM_ATTR sensor_isr() {
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL_ISR(&mux);

    handle_Sensor(&sensor_a, SENSOR_A);
    handle_Sensor(&sensor_b, SENSOR_B);
    handle_Sensor(&sensor_c, SENSOR_C);
    if(valid_state()) {
        save_sensor();
    }

    portEXIT_CRITICAL_ISR(&mux);
}

hw_timer_t * sensor_check_timer = NULL;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP("TRACKER-WIFI", NULL, false, 4);
  Serial.println(WiFi.softAPIP());
  server.begin(233);

  init_Sensor(&sensor_a, SENSOR_A);
    init_Sensor(&sensor_b, SENSOR_B);
    init_Sensor(&sensor_c, SENSOR_C);

    attachInterrupt(SENSOR_A, sensor_isr, CHANGE);
    attachInterrupt(SENSOR_B, sensor_isr, CHANGE);
    attachInterrupt(SENSOR_C, sensor_isr, CHANGE);

    sensor_check_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(sensor_check_timer, &sensor_isr, true);
    timerAlarmWrite(sensor_check_timer, 1000000, true);
    timerAlarmEnable(sensor_check_timer);
}

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
