#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ===== WiFi Config =====
const char* ssid = "UET-Wifi-Free 2.4Ghz";
const char* password = "";

// ===== UART Config =====
#define RX_PIN 16
#define TX_PIN 17


// ===== Global Variables =====
AsyncWebServer server(80);
volatile int temperature = 0;
volatile int humidity = 0;

void handleRoot(AsyncWebServerRequest *request) {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
    html += "<meta http-equiv='refresh' content='3'>";
    html += "<title>DHT11 Monitor</title></head><body>";
    html += "<h1>DHT11 Data from STM32</h1>";
    html += "<p><b>Temp:</b> " + String(temperature) + " °C</p>";
    html += "<p><b>Hum:</b> " + String(humidity) + " %</p>";
    html += "</body></html>";
    request->send(200, "text/html", html);
}

void setup() {
    Serial.begin(115200); // UART0 để debug
    Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); // UART2 để kết nối STM32

    // WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Web Server
    server.on("/", HTTP_GET, handleRoot);
    server.begin();
    Serial.println("ESP32 WebServer Ready");
}
void loop() {
    if (Serial2.available()) {
        String data = Serial2.readStringUntil('\n');
        data.trim(); // Loại bỏ khoảng trắng thừa

        Serial.print("Raw data: [");
        Serial.print(data);
        Serial.println("]");

        int temp, hum;
        if (sscanf(data.c_str(), "T%dH%d", &temp, &hum) == 2) {
            temperature = temp;
            humidity = hum;
            Serial.printf("Received Data: T=%dC, H=%d%%\r\n", temperature, humidity);
        } else {
            Serial.println("Error parsing data!");
        } 
    }
}