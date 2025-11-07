#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ArduinoJson.h>
#include <WebSocketMCP.h>

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
#define XSHUT_RIGHT   4

/* ================= TB6612 (kênh A) ================= */
#define PWMA   32
#define AIN1   25
#define AIN2   33
#define STBY   27

/* ================= LED trạng thái ================= */
#define LED_PIN 5   // D5

/* ================= Web/State ================= */
WebServer server(80);
volatile bool autoReverse = false;

int dBack = -1, dLeft = -1, dRight = -1;      // mm
int maxSpeed  = 200;   // duty 0..255
int speedNow  = 0;
int accelStep = 10;
String stateText = "Dừng";

/* Ngưỡng có thể chỉnh qua MCP */
int THRESH_BACK_MM = 70;   // dừng khi sau ≤ ngưỡng
int THRESH_SIDE_MM = 50;   // cảnh báo khi bên ≤ ngưỡng

/* Side alert & nhấp nháy LED (không blocking) */
bool sideAlert = false;
bool ledBlinkState = false;
unsigned long lastBlinkMs = 0;
const uint32_t BLINK_INTERVAL_MS = 300;
bool forceBlink = false;    // cho phép ra lệnh blink từ MCP khi không có cảnh báo

/* ================= PWM (LEDC) =================
   GIỮ NGUYÊN THEO CODE CŨ CỦA BẠN (không đổi sang ledcSetup/ledcAttachPin) */
void setupPWM() {
  // pin, freq, resolution, channel
  ledcAttachChannel(PWMA, 10000, 8, 0);
  ledcWriteChannel(0, 0);
}

/* ================= Motor helpers ================= */
void motorBackward(int duty) {
  if (duty < 0) duty = 0;
  if (duty > 255) duty = 255;
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW);   // chiều lùi
  digitalWrite(AIN2, HIGH);
  ledcWriteChannel(0, duty);
}

void motorStop() {
  ledcWriteChannel(0, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(STBY, LOW);
}

/* ================= LED update (ưu tiên) ================= */
void updateLed() {
  bool shouldBlink = sideAlert || forceBlink;
  if (shouldBlink) {
    unsigned long now = millis();
    if (now - lastBlinkMs >= BLINK_INTERVAL_MS) {
      lastBlinkMs = now;
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_PIN, ledBlinkState ? HIGH : LOW);
    }
  } else {
    digitalWrite(LED_PIN, autoReverse ? HIGH : LOW);
    ledBlinkState = false;
  }
}

/* ================= HTML ================= */
const char index_html[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="vi"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Auto Lùi + Voice (Xiaozhi)</title>
<style>
  body{font-family:Arial,Helvetica,sans-serif;background:#0b0b0b;color:#eee;margin:0}
  h1{margin:0;padding:12px 16px;background:#0d6efd}
  .box{background:#2d2d2d;border:1px solid #4b4b4b;border-radius:10px;display:inline-block;margin:18px;padding:16px;min-width:260px}
  button{border:0;border-radius:8px;padding:10px 16px;font-size:16px;cursor:pointer}
  .on{background:#e74c3c;color:#fff}
  .off{background:#2ecc71;color:#000}
  .badge{display:inline-block;padding:4px 8px;border-radius:8px;margin-top:6px;background:#444}
  .alert{background:#ff9800;color:#000}
  .note{opacity:.7;font-size:12px}
</style>
</head>
<body>
  <h1>🚗 ESP32 Auto Lùi + 🎤 Xiaozhi</h1>
  <div class="box">
    <h3>📏 Cảm biến (mm)</h3>
    Sau: <span id="dBack">-</span><br>
    Trái: <span id="dLeft">-</span><br>
    Phải: <span id="dRight">-</span><br><br>
    Trạng thái: <b id="state">Dừng</b><br>
    <span id="alert" class="badge">Side alert: off</span><br>
    <div class="note">Ngưỡng sau ≤ <span id="thb">70</span> mm dừng • Bên ≤ <span id="ths">50</span> mm nháy</div><br>
    <button id="btn" class="off" onclick="toggle()">Bật Auto Lùi</button>
  </div>

<script>
async function poll(){
  try{
    const r = await fetch('/sensor',{cache:'no-store'});
    if(!r.ok) throw new Error(r.status);
    const j = await r.json();
    dBack.textContent  = j.dBack;
    dLeft.textContent  = j.dLeft;
    dRight.textContent = j.dRight;
    state.textContent  = j.state;
    thb.textContent    = j.th_back;
    ths.textContent    = j.th_side;
    const btn=document.getElementById('btn');
    if(j.auto){ btn.textContent='Tắt Auto Lùi'; btn.className='on'; }
    else      { btn.textContent='Bật Auto Lùi'; btn.className='off'; }
    const badge = document.getElementById('alert');
    if(j.sideAlert){ badge.textContent='Side alert: ON'; badge.className='badge alert'; }
    else           { badge.textContent='Side alert: off'; badge.className='badge'; }
  }catch(e){}
}
function toggle(){ fetch('/toggle').catch(()=>{}); }
setInterval(poll,500);
poll();
</script>
</body></html>
)HTML";

/* ================= Web handlers ================= */
void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

void handleToggle() {
  autoReverse = !autoReverse;
  if (!autoReverse) { motorStop(); speedNow = 0; }
  server.send(200, "text/plain", "OK");
}

/* ====== LOGIC tại /sensor:
 * - Auto ON & sideAlert  => DỪNG + nhấp nháy LED
 * - Auto ON & back ≤ THRESH_BACK_MM => DỪNG
 * - Auto ON & an toàn => lùi + tăng tốc dần
 * - Auto OFF: chỉ nháy nếu sideAlert/forceBlink, không đụng motor
 */
void handleSensor() {
  String state = "Dừng";

  if (autoReverse) {
    if (sideAlert) {
      motorStop(); speedNow = 0;
      state = "Dừng (cảnh báo bên)";
    } else if (dBack < 0 || dBack <= THRESH_BACK_MM) {
      motorStop(); speedNow = 0;
      state = (dBack < 0) ? "Dừng (mất đo sau)" : "Dừng (vật cản sau gần)";
    } else {
      if (speedNow < maxSpeed) {
        speedNow += accelStep;
        if (speedNow > maxSpeed) speedNow = maxSpeed;
      }
      motorBackward(speedNow);
      state = "Lùi Auto";
    }
  } else {
    state = (sideAlert || forceBlink) ? "Cảnh báo (LED nháy)" : "Dừng (Tắt Auto)";
  }

  StaticJsonDocument<256> doc;
  doc["dBack"] = dBack;
  doc["dLeft"] = dLeft;
  doc["dRight"] = dRight;
  doc["state"] = state;
  doc["auto"] = autoReverse;
  doc["sideAlert"] = sideAlert;
  doc["th_back"] = THRESH_BACK_MM;
  doc["th_side"] = THRESH_SIDE_MM;

String payload;
serializeJson(doc, payload);           // tạo String
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
      if (st == "on")  autoReverse = true;
      if (st == "off") { autoReverse = false; motorStop(); speedNow = 0; }
      if (st == "toggle") autoReverse = !autoReverse;
      return WebSocketMCP::ToolResponse(String("{\"auto\":") + (autoReverse?"true":"false") + "}");
    }
  );

  // 2) Dừng khẩn cấp
  mcpClient.registerTool(
    "stop_now",
    "Dừng motor ngay lập tức",
    "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
    [](const String&){
      motorStop(); speedNow = 0;
      return WebSocketMCP::ToolResponse("{\"stopped\":true}");
    }
  );

  // 3) Đặt tốc độ tối đa
  mcpClient.registerTool(
    "set_speed",
    "Đặt tốc độ PWM tối đa (0..255)",
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
      if (doc.containsKey("back_mm")) THRESH_BACK_MM = doc["back_mm"].as<int>();
      if (doc.containsKey("side_mm")) THRESH_SIDE_MM = doc["side_mm"].as<int>();
      THRESH_BACK_MM = constrain(THRESH_BACK_MM, 20, 2000);
      THRESH_SIDE_MM = constrain(THRESH_SIDE_MM, 20, 2000);
      return WebSocketMCP::ToolResponse(
        String("{\"back_mm\":") + THRESH_BACK_MM + ",\"side_mm\":" + THRESH_SIDE_MM + "}"
      );
    }
  );

  // 5) Điều khiển LED: on/off/blink (không blocking)
  mcpClient.registerTool(
    "led_blink",
    "Điều khiển LED báo: on/off/blink (không dùng delay)",
    "{\"type\":\"object\",\"properties\":{\"state\":{\"type\":\"string\",\"enum\":[\"on\",\"off\",\"blink\"]}},\"required\":[\"state\"]}",
    [](const String& args){
      DynamicJsonDocument doc(128); deserializeJson(doc, args);
      String st = doc["state"].as<String>();
      if (st == "on")  { forceBlink = false; digitalWrite(LED_PIN, HIGH); }
      if (st == "off") { forceBlink = false; digitalWrite(LED_PIN, LOW);  }
      if (st == "blink"){ forceBlink = true; } // updateLed() sẽ thực thi nháy
      return WebSocketMCP::ToolResponse(String("{\"led\":\"") + st + "\"}");
    }
  );

  // 6) Lấy trạng thái hiện tại (JSON)
  mcpClient.registerTool(
    "get_status",
    "Trả về trạng thái cảm biến/motor/json",
    "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
    [](const String&){
      DynamicJsonDocument doc(256);
      doc["dBack"]=dBack; doc["dLeft"]=dLeft; doc["dRight"]=dRight;
      doc["auto"]=autoReverse; doc["sideAlert"]=sideAlert;
      doc["speedNow"]=speedNow; doc["maxSpeed"]=maxSpeed;
      doc["th_back"]=THRESH_BACK_MM; doc["th_side"]=THRESH_SIDE_MM;
      String out; serializeJson(doc, out);
      return WebSocketMCP::ToolResponse(out);
    }
  );
    // 7) Bật đèn xe
  mcpClient.registerTool(
    "turn_on_headlight",
    "Bật đèn xe (LED_PIN sáng liên tục)",
    "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
    [](const String&){
      forceBlink = false;
      digitalWrite(LED_PIN, HIGH);
      return WebSocketMCP::ToolResponse("{\"headlight\":\"on\"}");
    }
  );

  // 8) Tắt đèn xe
  mcpClient.registerTool(
    "turn_off_headlight",
    "Tắt đèn xe (LED_PIN tắt)",
    "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
    [](const String&){
      forceBlink = false;
      digitalWrite(LED_PIN, LOW);
      return WebSocketMCP::ToolResponse("{\"headlight\":\"off\"}");
    }
  );

  // 9) Cho xe đi thẳng (manual forward)
  mcpClient.registerTool(
    "manual_forward",
    "Cho xe chạy tiến thủ công (AIN1=HIGH, AIN2=LOW)",
    "{\"type\":\"object\",\"properties\":{\"speed\":{\"type\":\"integer\",\"minimum\":0,\"maximum\":255}},\"required\":[]}",
    [](const String& args){
      DynamicJsonDocument doc(128);
      deserializeJson(doc, args);
      int duty = doc["speed"] | 180;
      duty = constrain(duty, 0, 255);
      digitalWrite(STBY, HIGH);
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      ledcWriteChannel(0, duty);
      return WebSocketMCP::ToolResponse(String("{\"forward_speed\":") + duty + "}");
    }
  );

  // 10) Cho xe chạy lùi (manual backward)
  mcpClient.registerTool(
    "manual_backward",
    "Cho xe chạy lùi thủ công (AIN1=LOW, AIN2=HIGH)",
    "{\"type\":\"object\",\"properties\":{\"speed\":{\"type\":\"integer\",\"minimum\":0,\"maximum\":255}},\"required\":[]}",
    [](const String& args){
      DynamicJsonDocument doc(128);
      deserializeJson(doc, args);
      int duty = doc["speed"] | 180;
      duty = constrain(duty, 0, 255);
      digitalWrite(STBY, HIGH);
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      ledcWriteChannel(0, duty);
      return WebSocketMCP::ToolResponse(String("{\"backward_speed\":") + duty + "}");
    }
  );


  Serial.println("[MCP] 🛠️ Đã đăng ký tool cho Xiaozhi");
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
  digitalWrite(LED_PIN, LOW);

  setupPWM();

  // Khởi tạo cảm biến với địa chỉ khác nhau
  sensorsPowerDownAll();
  digitalWrite(XSHUT_BACK, HIGH); delay(20); if (!sensorBack.begin(0x30)) Serial.println("❌ Sensor Back lỗi!");
  digitalWrite(XSHUT_LEFT, HIGH); delay(20); if (!sensorLeft.begin(0x31)) Serial.println("❌ Sensor Left lỗi!");
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

    // Cập nhật cảnh báo bên
    sideAlert = ((dLeft >= 0 && dLeft <= THRESH_SIDE_MM) || (dRight >= 0 && dRight <= THRESH_SIDE_MM));

    // Cập nhật LED theo ưu tiên (sideAlert hoặc forceBlink)
    updateLed();

    Serial.printf("Back:%4d | Left:%4d | Right:%4d | Auto:%d | Speed:%3d | Alert:%d | TH(back:%d, side:%d)\n",
                  dBack, dLeft, dRight, autoReverse, speedNow, sideAlert, THRESH_BACK_MM, THRESH_SIDE_MM);
  }
}
