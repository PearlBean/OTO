#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ArduinoJson.h>
#include <WebSocketMCP.h>
#include <ESP32Servo.h>  // SERVO DOOR

/* ================= WiFi ================= */
const char* ssid = "P4";
const char* password = "p45024ltm";

/* ================= VL53L0X ================= */
Adafruit_VL53L0X sensorBack;
Adafruit_VL53L0X sensorLeft;
Adafruit_VL53L0X sensorRight;

/* XSHUT pins */
#define XSHUT_BACK   19
#define XSHUT_LEFT   18
#define XSHUT_RIGHT  4

/* ================= TB6612 (kênh A) ================= */
#define PWMA   32
#define AIN1   25
#define AIN2   33
#define STBY   27

/* ================= LED trạng thái ================= */
#define LED_PIN       5    // đèn chế độ / auto
#define WARN_LED_PIN  15   // 🔴 đèn cảnh báo vật cản

/* ================= SERVO CỬA XE ================= */
Servo doorServo;
#define SERVO_PIN 26          // Servo cửa gắn chân 26
int doorClosedAngle = 0;      // góc ĐÓNG cửa
int doorOpenAngle   = 90;     // góc MỞ cửa
int doorCurrentAngle = 0;
bool doorIsOpen = false;

void setDoor(bool open) {
  if (open) {
    doorCurrentAngle = doorOpenAngle;
  } else {
    doorCurrentAngle = doorClosedAngle;
  }
  doorServo.write(doorCurrentAngle);
  doorIsOpen = open;
}

/* ================= Web/State ================= */
WebServer server(80);

volatile bool autoReverse = false;  // chế độ lùi tự động

int dBack = -1, dLeft = -1, dRight = -1;      // mm
int maxSpeed  = 180;   // duty 0..255
int speedNow  = 0;
int accelStep = 10;
String stateText = "Dừng";

/* Ngưỡng có thể chỉnh qua MCP */
int THRESH_BACK_MM = 70;   // dừng (AUTO) khi sau ≤ ngưỡng
int THRESH_SIDE_MM = 50;   // cảnh báo khi bên ≤ ngưỡng

/* ======= Headlight mode (ưu tiên đèn) ======= */
enum HeadlightMode { HL_AUTO, HL_ON, HL_OFF, HL_BLINK };
HeadlightMode headlightMode = HL_AUTO;

bool sideAlert = false;
bool ledBlinkState = false;
unsigned long lastBlinkMs = 0;
const uint32_t BLINK_INTERVAL_MS = 300;

/* ======= Manual mode: chạy mãi đến khi có lệnh khác ======= */
enum ManualMode { MAN_NONE, MAN_FORWARD, MAN_BACKWARD };
ManualMode manualMode = MAN_NONE;
int manualDuty = 160;

/* ================= PWM (LEDC) =================
   GIỮ NGUYÊN THEO CODE CŨ (ledcAttachChannel/ledcWriteChannel) */
void setupPWM() {
  // pin, freq, resolution, channel
  ledcAttachChannel(PWMA, 10000, 8, 0);
  ledcWriteChannel(0, 0);
}

/* ================= Motor helpers ================= */
void motorBackward(int duty) {
  duty = constrain(duty, 0, 255);
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW);   // chiều lùi
  digitalWrite(AIN2, HIGH);
  ledcWriteChannel(0, duty);
}

void motorForward(int duty) {
  duty = constrain(duty, 0, 255);
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWriteChannel(0, duty);
}

void motorStop() {
  ledcWriteChannel(0, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(STBY, LOW);
}

/* ================= LED update (tôn trọng ưu tiên) ================= */
void updateLed() {
  switch (headlightMode) {
    case HL_ON:
      digitalWrite(LED_PIN, HIGH);
      ledBlinkState = false;
      return;
    case HL_OFF:
      digitalWrite(LED_PIN, LOW);
      ledBlinkState = false;
      return;
    case HL_BLINK: {
      unsigned long now = millis();
      if (now - lastBlinkMs >= BLINK_INTERVAL_MS) {
        lastBlinkMs = now;
        ledBlinkState = !ledBlinkState;
        digitalWrite(LED_PIN, ledBlinkState ? HIGH : LOW);
      }
      return;
    }
    case HL_AUTO:
    default: {
      // AUTO: ưu tiên cảnh báo bên -> nhấp nháy; nếu không thì theo autoReverse
      if (sideAlert) {
        unsigned long now = millis();
        if (now - lastBlinkMs >= BLINK_INTERVAL_MS) {
          lastBlinkMs = now;
          ledBlinkState = !ledBlinkState;
          digitalWrite(LED_PIN, ledBlinkState ? HIGH : LOW);
        }
      } else {
        // LED bật khi autoReverse đang ON (manual không ảnh hưởng đèn AUTO)
        digitalWrite(LED_PIN, autoReverse ? HIGH : LOW);
        ledBlinkState = false;
      }
      return;
    }
  }
}

/* ================= HTML ================= */
const char index_html[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Auto Reverse</title>
<style>
body{font-family:Arial;text-align:center;background:#111;color:white;margin:0;padding:0;}
h1{background:#007bff;color:white;padding:10px;margin:0;}
.box{border:1px solid #555;padding:10px;margin:10px;border-radius:8px;background:#222;display:inline-block;}
button{padding:10px 20px;font-size:16px;border:none;border-radius:5px;margin:5px;}
.on{background:#ff4444;color:white;}
.off{background:#44ff44;color:black;}
</style>
</head>
<body>
<h1>🚗 ESP32 Auto Lùi Xe</h1>
<div class="box">
<h3>📏 Cảm biến (mm)</h3>
Sau: <span id="dBack">-</span><br>
Trái: <span id="dLeft">-</span><br>
Phải: <span id="dRight">-</span><br><br>
Trạng thái: <b id="state">Dừng</b><br><br>
<button id="btn" class="off" onclick="toggle()">Bật Auto Lùi</button>
<hr>
Cửa: <b id="doorState">Đóng</b> (<span id="doorAngle">0</span>°)<br><br>
<button onclick="door('open')">Mở cửa</button>
<button onclick="door('close')">Đóng cửa</button>
</div>

<script>
function poll(){
  fetch('/sensor').then(r=>r.json()).then(j=>{
    document.getElementById("dBack").innerText=j.dBack;
    document.getElementById("dLeft").innerText=j.dLeft;
    document.getElementById("dRight").innerText=j.dRight;
    document.getElementById("state").innerText=j.state;
    const btn=document.getElementById("btn");
    if(j.auto){
      btn.innerText="Tắt Auto Lùi";
      btn.className="on";
    }else{
      btn.innerText="Bật Auto Lùi";
      btn.className="off";
    }
    // cập nhật trạng thái cửa
    document.getElementById("doorState").innerText = j.doorOpen ? "Mở" : "Đóng";
    document.getElementById("doorAngle").innerText = j.doorAngle;
  });
}

function toggle(){ fetch('/toggle'); }
function door(cmd){ fetch('/door?cmd='+cmd); }

setInterval(poll,300);
</script>
</body>
</html>
)HTML";

/* ================= Web handlers ================= */
void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

void handleToggle() {
  autoReverse = !autoReverse;
  if (autoReverse) {
    // Bật AUTO sẽ hủy manual mode để tránh xung đột
    manualMode = MAN_NONE;
    motorStop(); speedNow = 0;
  } else {
    motorStop(); speedNow = 0;
  }
  server.send(200, "text/plain", "OK");
}

/* /door: MỞ / ĐÓNG cửa bằng servo */
void handleDoor() {
  String cmd = server.hasArg("cmd") ? server.arg("cmd") : "";
  if (cmd == "open") {
    setDoor(true);
    Serial.println("[DOOR] Open");
    server.send(200, "text/plain", "door_open");
  } else if (cmd == "close") {
    setDoor(false);
    Serial.println("[DOOR] Close");
    server.send(200, "text/plain", "door_close");
  } else {
    server.send(400, "text/plain", "invalid_cmd");
  }
}

/* /sensor chỉ trả trạng thái (không điều khiển động cơ) */
void handleSensor() {
  StaticJsonDocument<400> doc;
  doc["dBack"] = dBack;
  doc["dLeft"] = dLeft;
  doc["dRight"] = dRight;
  doc["state"] = stateText;
  doc["auto"] = autoReverse;
  doc["sideAlert"] = sideAlert;
  doc["th_back"] = THRESH_BACK_MM;
  doc["th_side"] = THRESH_SIDE_MM;
  doc["hl_mode"] = (headlightMode==HL_AUTO?"auto":headlightMode==HL_ON?"on":headlightMode==HL_OFF?"off":"blink");
  doc["manual"]  = (manualMode==MAN_FORWARD?"forward":manualMode==MAN_BACKWARD?"backward":"none");
  doc["manualDuty"] = manualDuty;
  doc["doorOpen"] = doorIsOpen;
  doc["doorAngle"] = doorCurrentAngle;

  // gửi thêm flag vật cản để debug
  bool obstacleDetected =
    (dBack  >= 0 && dBack  <= THRESH_BACK_MM) ||
    (dLeft  >= 0 && dLeft  <= THRESH_SIDE_MM) ||
    (dRight >= 0 && dRight <= THRESH_SIDE_MM);
  doc["obstacle"] = obstacleDetected;

  String payload;
  serializeJson(doc, payload);
  server.send(200, "application/json", payload);
}

/* ================= Sensor init helpers ================= */
void sensorsPowerDownAll() {
  digitalWrite(XSHUT_BACK, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);
}

/* ================= Xiaozhi MCP ================= */
WebSocketMCP mcpClient;

// !!! Thay endpoint này bằng endpoint của bạn nếu cần (token từ Xiaozhi)
const char* mcpEndpoint =
  "wss://api.xiaozhi.me/mcp/?token=eyJhbGciOiJFUzI1NiIsInR5cCI6IkpXVCJ9.eyJ1c2VySWQiOjQ5NjI3NCwiYWdlbnRJZCI6OTg4MjAxLCJlbmRwb2ludElkIjoiYWdlbnRfOTg4MjAxIiwicHVycG9zZSI6Im1jcC1lbmRwb2ludCIsImlhdCI6MTc2MjUzNjkwNywiZXhwIjoxNzk0MDk0NTA3fQ.KnwYDmjvAe3IJW6u64VlbDg6R2Exs-Wuoi0cUNvTF4ELITAR95WMmfaaeCVLUVRxvIVgnmBR3LfwmMZtAnjlUQ";

void registerMcpTools() {
  // 1) Bật/tắt/toggle auto reverse
  mcpClient.registerTool(
    "auto_reverse",
    "Bật/tắt/toggle chế độ lùi tự động",
    "{\"type\":\"object\",\"properties\":{\"state\":{\"type\":\"string\",\"enum\":[\"on\",\"off\",\"toggle\"]}},\"required\":[\"state\"]}",
    [](const String& args){
      DynamicJsonDocument doc(128); deserializeJson(doc, args);
      String st = doc["state"].as<String>();
      if (st == "on")  { autoReverse = true;  manualMode = MAN_NONE; motorStop(); speedNow = 0; }
      if (st == "off") { autoReverse = false; /* không đụng manual */ }
      if (st == "toggle") { autoReverse = !autoReverse; if (autoReverse) { manualMode = MAN_NONE; motorStop(); speedNow = 0; } }
      return WebSocketMCP::ToolResponse(String("{\"auto\":") + (autoReverse?"true":"false") + "}");
    }
  );

  // 2) Dừng khẩn cấp
  mcpClient.registerTool(
    "stop_now",
    "Dừng motor ngay lập tức & hủy manual; AUTO vẫn giữ nguyên cờ",
    "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
    [](const String&){
      motorStop(); speedNow = 0;
      manualMode = MAN_NONE;
      return WebSocketMCP::ToolResponse("{\"stopped\":true}");
    }
  );

  // 3) Đặt tốc độ tối đa (cho AUTO)
  mcpClient.registerTool(
    "set_speed",
    "Đặt tốc độ PWM tối đa (0..255) cho AUTO",
    "{\"type\":\"object\",\"properties\":{\"maxSpeed\":{\"type\":\"integer\",\"minimum\":0,\"maximum\":255}},\"required\":[\"maxSpeed\"]}",
    [](const String& args){
      DynamicJsonDocument doc(128); deserializeJson(doc, args);
      int ms = doc["maxSpeed"].as<int>();
      maxSpeed = constrain(ms, 0, 255);
      if (speedNow > maxSpeed) speedNow = maxSpeed;
      return WebSocketMCP::ToolResponse(String("{\"maxSpeed\":") + maxSpeed + "}");
    }
  );

  // 4) Đặt ngưỡng dừng sau / cảnh báo bên
  mcpClient.registerTool(
    "set_thresholds",
    "Đặt ngưỡng sau (back_mm) và bên (side_mm) đơn vị mm",
    "{\"type\":\"object\",\"properties\":{\"back_mm\":{\"type\":\"integer\",\"minimum\":20,\"maximum\":2000},\"side_mm\":{\"type\":\"integer\",\"minimum\":20,\"maximum\":2000}},\"required\":[]}",
    [](const String& args){
      DynamicJsonDocument doc(128); deserializeJson(doc, args);
      if (doc.containsKey("back_mm")) THRESH_BACK_MM = constrain(doc["back_mm"].as<int>(), 20, 2000);
      if (doc.containsKey("side_mm")) THRESH_SIDE_MM = constrain(doc["side_mm"].as<int>(), 20, 2000);
      return WebSocketMCP::ToolResponse(
        String("{\"back_mm\":") + THRESH_BACK_MM + ",\"side_mm\":" + THRESH_SIDE_MM + "}"
      );
    }
  );

  // 5) Headlight: ON
  mcpClient.registerTool(
    "turn_on_headlight",
    "Bật đèn xe (sáng liên tục, override AUTO)",
    "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
    [](const String&){
      headlightMode = HL_ON;
      return WebSocketMCP::ToolResponse("{\"headlight\":\"on\"}");
    }
  );

  // 6) Headlight: OFF
  mcpClient.registerTool(
    "turn_off_headlight",
    "Tắt đèn xe (override AUTO)",
    "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
    [](const String&){
      headlightMode = HL_OFF;
      return WebSocketMCP::ToolResponse("{\"headlight\":\"off\"}");
    }
  );

  // 7) Headlight: BLINK
  mcpClient.registerTool(
    "led_blink",
    "Đèn nhấp nháy (override AUTO)",
    "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
    [](const String&){
      headlightMode = HL_BLINK;
      return WebSocketMCP::ToolResponse("{\"headlight\":\"blink\"}");
    }
  );

  // 8) Headlight: AUTO
  mcpClient.registerTool(
    "headlight_auto",
    "Đèn ở chế độ AUTO (theo sideAlert/autoReverse)",
    "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
    [](const String&){
      headlightMode = HL_AUTO;
      return WebSocketMCP::ToolResponse("{\"headlight\":\"auto\"}");
    }
  );

  // 9) Cho xe đi thẳng (manual)
  mcpClient.registerTool(
    "manual_forward",
    "Cho xe chạy tiến thủ công (chạy mãi đến khi lệnh khác)",
    "{\"type\":\"object\",\"properties\":{\"speed\":{\"type\":\"integer\",\"minimum\":0,\"maximum\":255}},\"required\":[]}",
    [](const String& args){
      DynamicJsonDocument doc(128);
      deserializeJson(doc, args);
      manualDuty = constrain((int)(doc["speed"] | 180), 0, 255);
      autoReverse = false;
      manualMode = MAN_FORWARD;
      stateText = "Đang tiến (manual)";
      return WebSocketMCP::ToolResponse(String("{\"manual\":\"forward\",\"speed\":") + manualDuty + "}");
    }
  );

  // 10) Cho xe chạy lùi (manual)
  mcpClient.registerTool(
    "manual_backward",
    "Cho xe chạy lùi thủ công (chạy mãi đến khi lệnh khác)",
    "{\"type\":\"object\",\"properties\":{\"speed\":{\"type\":\"integer\",\"minimum\":0,\"maximum\":255}},\"required\":[]}",
    [](const String& args){
      DynamicJsonDocument doc(128);
      deserializeJson(doc, args);
      manualDuty = constrain((int)(doc["speed"] | 180), 0, 255);
      autoReverse = false;
      manualMode = MAN_BACKWARD;
      stateText = "Đang lùi (manual)";
      return WebSocketMCP::ToolResponse(String("{\"manual\":\"backward\",\"speed\":") + manualDuty + "}");
    }
  );

  // 11) Lấy trạng thái hiện tại
  mcpClient.registerTool(
    "get_status",
    "Trả về trạng thái cảm biến/motor/json",
    "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
    [](const String&){
      DynamicJsonDocument doc(360);
      doc["dBack"]=dBack; doc["dLeft"]=dLeft; doc["dRight"]=dRight;
      doc["auto"]=autoReverse; doc["sideAlert"]=sideAlert;
      doc["speedNow"]=speedNow; doc["maxSpeed"]=maxSpeed;
      doc["th_back"]=THRESH_BACK_MM; doc["th_side"]=THRESH_SIDE_MM;
      doc["hl_mode"]=(headlightMode==HL_AUTO?"auto":headlightMode==HL_ON?"on":headlightMode==HL_OFF?"off":"blink");
      doc["manual"]=(manualMode==MAN_FORWARD?"forward":manualMode==MAN_BACKWARD?"backward":"none");
      doc["manualDuty"]=manualDuty;
      doc["doorOpen"]=doorIsOpen;
      doc["doorAngle"]=doorCurrentAngle;

      bool obstacleDetected =
        (dBack  >= 0 && dBack  <= THRESH_BACK_MM) ||
        (dLeft  >= 0 && dLeft  <= THRESH_SIDE_MM) ||
        (dRight >= 0 && dRight <= THRESH_SIDE_MM);
      doc["obstacle"] = obstacleDetected;

      String out; serializeJson(doc, out);
      return WebSocketMCP::ToolResponse(out);
    }
  );

  // 12) Mở cửa
  mcpClient.registerTool(
    "open_car_door",
    "Mở cửa xe bằng servo (góc mặc định 90°, có thể truyền angle)",
    "{\"type\":\"object\",\"properties\":{\"angle\":{\"type\":\"integer\",\"minimum\":0,\"maximum\":180}},\"required\":[]}",
    [](const String& args){
      DynamicJsonDocument doc(128);
      deserializeJson(doc, args);
      if (doc.containsKey("angle")) {
        doorOpenAngle = constrain((int)doc["angle"], 0, 180);
      }
      setDoor(true);
      return WebSocketMCP::ToolResponse(
        String("{\"doorOpen\":true, \"angle\":") + doorCurrentAngle + "}"
      );
    }
  );

  // 13) Đóng cửa
  mcpClient.registerTool(
    "close_car_door",
    "Đóng cửa xe bằng servo (góc mặc định 0°, có thể truyền angle)",
    "{\"type\":\"object\",\"properties\":{\"angle\":{\"type\":\"integer\",\"minimum\":0,\"maximum\":180}},\"required\":[]}",
    [](const String& args){
      DynamicJsonDocument doc(128);
      deserializeJson(doc, args);
      if (doc.containsKey("angle")) {
        doorClosedAngle = constrain((int)doc["angle"], 0, 180);
      }
      setDoor(false);
      return WebSocketMCP::ToolResponse(
        String("{\"doorOpen\":false, \"angle\":") + doorCurrentAngle + "}"
      );
    }
  );

  Serial.println("[MCP] 🛠️ Đã đăng ký tool cho Xiaozhi (kèm servo cửa)");
}

void onConnectionStatus(bool connected) {
  if (connected) {
    Serial.println("[MCP] ✅ Đã kết nối tới Xiaozhi");
    registerMcpTools();
  } else {
    Serial.println("[MCP] ⚠️ Mất kết nối Xiaozhi");
  }
}

/* ================= Setup ================= */
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(XSHUT_BACK, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(WARN_LED_PIN, OUTPUT);   // 🔴 đèn cảnh báo
  digitalWrite(LED_PIN, LOW);
  digitalWrite(WARN_LED_PIN, LOW);

  setupPWM();

  // SERVO cửa
  doorServo.attach(SERVO_PIN, 500, 2400);
  setDoor(false); // cửa đóng lúc khởi động

  // Khởi tạo cảm biến với địa chỉ khác nhau
  sensorsPowerDownAll();
  digitalWrite(XSHUT_BACK, HIGH);  delay(20); if (!sensorBack.begin(0x30)) Serial.println("❌ Sensor Back lỗi!");
  digitalWrite(XSHUT_LEFT, HIGH);  delay(20); if (!sensorLeft.begin(0x31)) Serial.println("❌ Sensor Left lỗi!");
  digitalWrite(XSHUT_RIGHT, HIGH); delay(20); if (!sensorRight.begin(0x32)) Serial.println("❌ Sensor Right lỗi!");

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("🔗 WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println();
  Serial.print("✅ IP: "); Serial.println(WiFi.localIP());

  // Web server
  server.on("/", handleRoot);
  server.on("/toggle", handleToggle);
  server.on("/sensor", handleSensor);
  server.on("/door", handleDoor);   // <-- THÊM ROUTE ĐIỀU KHIỂN CỬA
  server.begin();
  Serial.println("🌍 Web server ready");

  // MCP Xiaozhi
  mcpClient.begin(mcpEndpoint, onConnectionStatus);
}

/* ================= Loop ================= */
unsigned long lastSensorMillis = 0;
const long sensorInterval = 100;

void loop() {
  server.handleClient();
  mcpClient.loop(); // duy trì kết nối Xiaozhi

  unsigned long now = millis();
  if (now - lastSensorMillis >= sensorInterval) {
    lastSensorMillis = now;

    VL53L0X_RangingMeasurementData_t m;
    sensorBack.rangingTest(&m, false);  dBack  = (m.RangeStatus != 4) ? m.RangeMilliMeter : -1;
    sensorLeft.rangingTest(&m, false);  dLeft  = (m.RangeStatus != 4) ? m.RangeMilliMeter : -1;
    sensorRight.rangingTest(&m, false); dRight = (m.RangeStatus != 4) ? m.RangeMilliMeter : -1;

    // Cập nhật cảnh báo bên (chỉ ảnh hưởng đèn HL_AUTO)
    sideAlert = ((dLeft >= 0 && dLeft <= THRESH_SIDE_MM) || (dRight >= 0 && dRight <= THRESH_SIDE_MM));

    // ===== BẬT ĐÈN CẢNH BÁO VẬT CẢN (D15) =====
    bool obstacleDetected =
      (dBack  >= 0 && dBack  <= THRESH_BACK_MM) ||
      (dLeft  >= 0 && dLeft  <= THRESH_SIDE_MM) ||
      (dRight >= 0 && dRight <= THRESH_SIDE_MM);

    digitalWrite(WARN_LED_PIN, obstacleDetected ? HIGH : LOW);

    // ===== Ưu tiên điều khiển motor =====
    if (manualMode == MAN_FORWARD) {
      motorForward(manualDuty);
      stateText = "Đang tiến (manual)";
    }
    else if (manualMode == MAN_BACKWARD) {
      motorBackward(manualDuty);
      stateText = "Đang lùi (manual)";
      // (manual không auto dừng khi gặp vật cản – đèn cảnh báo chỉ báo hiệu)
    }
    else if (autoReverse) {
      // AUTO: dừng khi gần/sideAlert
      if (sideAlert) {
        motorStop(); speedNow = 0;
        stateText = "Dừng (AUTO: cảnh báo bên)";
      } else if (dBack < 0 || dBack <= THRESH_BACK_MM) {
        motorStop(); speedNow = 0;
        stateText = (dBack < 0) ? "Dừng (AUTO: mất đo sau)" : "Dừng (AUTO: vật cản sau gần)";
      } else {
        if (speedNow < maxSpeed) {
          speedNow += accelStep;
          if (speedNow > maxSpeed) speedNow = maxSpeed;
        }
        motorBackward(speedNow);
        stateText = "Lùi Auto";
      }
    }
    else {
      motorStop();
      stateText = sideAlert ? "Cảnh báo (LED nháy)" : "Dừng";
    }

    // Cập nhật đèn chế độ
    updateLed();

    Serial.printf("Back:%4d | Left:%4d | Right:%4d | Obstacle:%d | Auto:%d | Manual:%d | Duty:%3d | Alert:%d | HL:%d | TH(back:%d, side:%d) | Door:%s(%d°)\n",
                  dBack, dLeft, dRight, obstacleDetected,
                  autoReverse, (int)manualMode, manualDuty,
                  sideAlert, (int)headlightMode, THRESH_BACK_MM, THRESH_SIDE_MM,
                  doorIsOpen ? "OPEN" : "CLOSE", doorCurrentAngle);
  }
}
