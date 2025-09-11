#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

#define I2C_SLAVE_ADDR 0x08
#define SDA_PIN 21
#define SCL_PIN 22

String received = "";
int temperature = 0;
int humidity = 0;

const char* ssid = "VIETTEL_GPON_D434A0";
const char* password = "1234567890a";

WebServer server(80);

void receiveEvent(int len) {
  received = "";
  while (Wire.available()) {
    char c = Wire.read();
    received += c;
  }
  Serial.println("Received: " + received); // Debug dữ liệu nhận được

  int t, h;
  if (sscanf(received.c_str(), "T%dH%d", &t, &h) == 2) {
    temperature = t;
    humidity = h;
  } else {
    Serial.println("Lỗi: Dữ liệu I2C không đúng định dạng!");
  }
}

void handleRoot() {
  String html = "<html><head>";
  html += "<meta http-equiv='refresh' content='5'>";
  html += "<meta charset='UTF-8'>"; // Khai báo mã hóa UTF-8
  html += "</head><body>";
  html += "<h1 style='text-align:center;'>DHT11 (STM32)</h1>";
  html += "<p style='font-size:20px; text-align:center;'>TEM: " + String(temperature) + " °C</p>";
  html += "<p style='font-size:20px; text-align:center;'>HUM: " + String(humidity) + " %</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  delay(100); // Đợi ESP32 ổn định

  // Kiểm tra trạng thái chân I2C
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  Serial.printf("SDA pin state: %d\n", digitalRead(SDA_PIN));
  Serial.printf("SCL pin state: %d\n", digitalRead(SCL_PIN));

  // Khởi tạo I2C
  Wire.setPins(SDA_PIN, SCL_PIN);
  if (!Wire.begin(I2C_SLAVE_ADDR)) {
    Serial.println("I2C Slave init failed!");
    while (1); // Dừng để debug
  }
  Wire.setClock(100000);
  Wire.onReceive(receiveEvent);
  Serial.println("I2C Slave initialized successfully");

  // Kết nối WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nLỗi: Không thể kết nối WiFi!");
    while (1);
  }
  Serial.println("\nWiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.begin();
}

void loop() {
  server.handleClient();

  if (temperature != 0 && humidity != 0) {
    Serial.printf("Nhiệt độ: %d °C, Độ ẩm: %d %%\n", temperature, humidity);
    delay(2000);
  }
}