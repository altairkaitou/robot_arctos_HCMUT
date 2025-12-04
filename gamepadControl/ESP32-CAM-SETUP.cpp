#include <WebServer.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <esp32cam.h>

// -------------------------------------
// WebServer
// -------------------------------------
WebServer server(80);

// -------------------------------------
// Camera Resolution
// -------------------------------------
static auto hiRes = esp32cam::Resolution::find(640, 480);   // SVGA

// -------------------------------------
// Capture Lock
// -------------------------------------
volatile bool capturing = false;

// -------------------------------------
// Serve JPEG
// -------------------------------------
void serveJpg() {
  if (capturing) {
    server.send(429, "text/plain", "BUSY");
    return;
  }
  capturing = true;

  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    capturing = false;
    server.send(503, "text/plain", "CAPTURE FAIL");
    return;
  }

  server.sendHeader("X-Timestamp", String(millis()));
  server.setContentLength(frame->size());
  server.send(200, "image/jpeg");

  WiFiClient client = server.client();
  frame->writeTo(client);

  capturing = false;
}

// -------------------------------------
// Detect Hotspot
// -------------------------------------
bool isMobileHotspot(IPAddress gw) {
  if (gw[0] == 10) return true;                                     // Carrier NAT
  if (gw[0] == 172) return true;                                    // iPhone
  if (gw[0] == 192 && gw[1] == 168 && gw[2] == 43) return true;     // Android
  return false;
}

// -------------------------------------
// WiFiManager Connection
// -------------------------------------
void setupWifi() {
  WiFiManager wm;

  Serial.println("Starting WiFiManager...");
  // wm.resetSettings();   // <- Use this ONCE if needed

  bool ok = wm.autoConnect("ESP32-CAM-Setup", "12345678");
  if (!ok) {
    Serial.println("WiFiManager FAILED. Rebooting...");
    ESP.restart();
  }

  Serial.println("WiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());

  // Hotspot? Keep DHCP.
  if (isMobileHotspot(WiFi.gatewayIP())) {
    Serial.println("Hotspot detected → Keeping DHCP");
    return;
  }

  // Normal router → Set static IP
  Serial.println("Normal router detected → Applying static IP");
  IPAddress local_IP(192,168,1,50);
  IPAddress gateway(192,168,1,1);
  IPAddress subnet(255,255,255,0);
  IPAddress dns(8,8,8,8);

  WiFi.config(local_IP, gateway, subnet, dns);

  Serial.print("Static IP Now: ");
  Serial.println(WiFi.localIP());
}

// -------------------------------------
// Setup
// -------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();

  // Camera Config
  {
    using namespace esp32cam;
    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(hiRes);
    cfg.setBufferCount(2);
    cfg.setJpeg(80);

    bool ok = Camera.begin(cfg);
    Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
  }

  // WiFiManager handles Wi-Fi
  setupWifi();

  Serial.print("Camera Ready: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/cam.jpg");

  server.on("/cam.jpg", [](){ serveJpg(); });
  server.begin();
}

// -------------------------------------
// Loop
// -------------------------------------
void loop() {
  server.handleClient();
}
