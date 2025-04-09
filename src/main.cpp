#include "Arduino.h"
#include "WiFi.h"
#include "esp_now.h"
#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_ADXL343.h"

#define SDA_PIN 5 // SDA pin for ESP32
#define SCL_PIN 6 // SCL pin for ESP32

Adafruit_ADXL343 accel1 = Adafruit_ADXL343(12345);

struct SensorData {
  float x;
  float y;
  float z;
};
SensorData sensorData;

// MAC address of the other ESP32 device
uint8_t otherDeviceMacAddress[] = {0x7C,0x9E,0xBD,0x28,0x82,0x70}; 
//7c:9e:bd:28:82:70 oled {0x7C,0x9E,0xBD,0x28,0x82,0x70}; 
//84:fc:e6:6b:85:58 accel {0x84,0xFC,0xE6,0x6B,0x85,0x58};
// Replace with the actual MAC address

// Callback function to handle received data
// This function is called when data is received
// It prints the received message to the Serial Monitor
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
}

// Callback function to handle sent data status
// This function is called when the data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.print("Data sent successfully  ");
  } else {
    Serial.print("Data send failed  ");
  }
}

void setup() {
  Serial.begin(115200); // Initialize Serial Monitor

  Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C with custom SDA and SCL pins

  // I2C Scanner
  Serial.println("Scanning for I2C devices...");
  byte count = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      count++;
    }
  }
  if (count == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Found ");
    Serial.print(count);
    Serial.println(" I2C devices.");
  }
  Serial.println("");

  /* Initialise the first sensors, this uses the default address */
  if(!accel1.begin(0x53))
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("Ooops, no ADXL343 nr1 detected ... Check your wiring!");
    while(1);
  }
  
  /* Set the range and data rate to whatever is appropriate for your project */
  /* See the sensortest example for more details */
  accel1.setRange(ADXL343_RANGE_2_G);

  accel1.setDataRate(ADXL343_DATARATE_1600_HZ);

  /* Display some basic information on these sensors */
  accel1.printSensorDetails();
  Serial.println("");

  WiFi.mode(WIFI_STA);  // Set WiFi mode to Station (client)

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, otherDeviceMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Register for send and receive callbacks
  esp_now_register_send_cb(OnDataSent); 
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  /* Get new sensor events */
  sensors_event_t event1;
  accel1.getEvent(&event1);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: ");Serial.print(event1.acceleration.x); Serial.print(", ");
  Serial.print("Y: ");Serial.print(event1.acceleration.y); Serial.print(", ");
  Serial.print("Z: ");Serial.print(event1.acceleration.z); Serial.println(".");
  sensorData.x = event1.acceleration.x;
  sensorData.y = event1.acceleration.y;
  sensorData.z = event1.acceleration.z;

  esp_now_send(otherDeviceMacAddress, (uint8_t *)&sensorData, sizeof(sensorData)); // Send message
  delay(500);
}