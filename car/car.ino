#include <WiFi.h>

typedef struct
{
    double Target; // 期望
    double P; // 比例系数
    double I; // 积分系数
    double D; // 微分系数
    double T_Limit; // 积分限幅
    double Limit; // 计算结果限幅
    double Error; // 偏差
    double LastError; // 上次的值
    double PrevError; // 上上次偏差
    double SumError; // 累计误差
    double Realize; // 计算出的差值
} PidNode;

double _limit(double x, double min, double max) {
    return x > max ? max : x < min ? min : x;
}

/*
 * @func PID参数初始化
 */
void PID_ParameterInit(PidNode *pid, double P, double I, double D,
                       double Target, double T_Limit, double Limit) {
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->Target = Target;
    pid->T_Limit = T_Limit;
    pid->Limit = Limit;

    pid->LastError = 0;
    pid->PrevError = 0;
    pid->SumError = 0;
}
// 位置式PID控制
double PID_PlaceDouble(PidNode *pid, double target, double now) {
    pid->Error = target - now;   // 计算当前误差
    pid->SumError += pid->Error; // 误差积分
    pid->SumError = _limit(pid->SumError, -pid->T_Limit, pid->T_Limit);

    pid->Realize = (pid->P * pid->Error + pid->I * pid->SumError +
                    pid->D * (pid->Error - pid->LastError));
    pid->LastError = pid->Error; // 更新上次误差

    pid->Realize = _limit(pid->Realize, -pid->Limit, pid->Limit);
    return pid->Realize; // 返回输出实际值
}



typedef struct Trans {
  int64_t girl;
} Trans;

void printTrans(struct Trans pretty) {
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
// 0~256
#define PWM_RESOLUTION 8

typedef enum {
    STATE_START,
    STATE_GOING,
    STATE_STOP
} State;

// todo
#define ACCEPTABLE_DIFF 100

State state = STATE_START;

hw_timer_t * pid_timer = NULL;

PidNode pid;

void IRAM_ATTR writeEngine(int chPostive, int chNegative, int engine) {
    if (engine > 0) {
        ledcWrite(chPostive, engine);
        ledcWrite(chNegative, 0);
    } else {
        ledcWrite(chPostive, 0);
        ledcWrite(chNegative, -engine);
    }
}

// unsafe
#define ABS(x) ((x) < 0 ? -(x) : (x))

void IRAM_ATTR pid_timer_callback() {
    if (state == STATE_GOING) {
        int engine = (int) PID_PlaceDouble(&pid, 0, got.girl);
        writeEngine(ENGINE1_POSITIVE_CH, ENGINE1_NEGATIVE_CH, engine);
        writeEngine(ENGINE2_POSITIVE_CH, ENGINE2_NEGATIVE_CH, engine);
        if(ABS(got.girl)<ACCEPTABLE_DIFF){
            state = STATE_STOP;
            // todo
        }
    }
}

hw_timer_t * beep_timer = NULL;
void IRAM_ATTR beep_timer_callback() {
    static bool beeping = false;
    if (state == STATE_GOING) {
        digitalWrite(BEEP_PIN, BEEP_VALID == beeping ? HIGH : LOW);
        beeping = !beeping;
    } else if (state == STATE_STOP) {
        digitalWrite(BEEP_PIN, BEEP_VALID == true ? HIGH : LOW);
    }
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

    PID_ParameterInit(&pid, 128, 0, 0, 0, 256, 256);
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
