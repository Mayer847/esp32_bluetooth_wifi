#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <WiFi.h>

// Create a BluetoothSerial object
BluetoothSerial serial;

// Create a Preferences object
Preferences preferences;

// Declare wifiSSID and wifiPassword in the global scope
String wifiSSID;
String wifiPassword;

void setup() {
  // Initialize the serial port
  Serial.begin(115200);

  // Initialize the BluetoothSerial object
  serial.begin("ESP32");

  // Open Preferences with my-app-namespace. Each application module, library, etc.
  // has to use a namespace name to prevent key name collisions. We will open storage in RW-mode: read-write mode
  preferences.begin("my-app-namespace", false);

  // Receive WiFi credentials over Bluetooth
  receiveCredentials();
  
  // Close Bluetooth connection
  serial.end();

  // Connect to WiFi using received credentials
  connectToWiFi(wifiSSID, wifiPassword);
}

void loop() {
  // If WiFi is not connected, attempt to reconnect
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Lost connection to WiFi. Attempting to reconnect...");
    WiFi.disconnect();
    delay(1000); // Wait a bit before reconnecting
    connectToWiFi(wifiSSID, wifiPassword);
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Failed to reconnect to WiFi. Restarting ESP32...");
      ESP.restart();
    }
  }
}

void receiveCredentials() {
  bool ssidReceived = false;
  bool passwordReceived = false;

  while (!ssidReceived || !passwordReceived) {
    if (serial.available()) {
      String data = serial.readStringUntil('\n');

      if (data.startsWith("wifi_ssid")) {
        wifiSSID = data.substring(10);
        preferences.putString("wifiSSID", wifiSSID);
        Serial.println("WiFi SSID changed to: " + wifiSSID);
        ssidReceived = true;
      } else if (data.startsWith("wifi_password")) {
        wifiPassword = data.substring(14);
        preferences.putString("wifiPassword", wifiPassword);
        Serial.println("WiFi password changed to: " + wifiPassword);
        passwordReceived = true;
      }
    }
  }
}

void connectToWiFi(String ssid, String password) {
  WiFi.begin(ssid.c_str(), password.c_str());
  
  delay(5000);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
  } else {
    Serial.println("Failed to connect to WiFi.");
    WiFi.reconnect();
    delay(5000); // Wait a bit before checking connection status again

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Reconnected to WiFi");
    } else {
      Serial.println("Failed to reconnect to WiFi.");
    }
  }
}
