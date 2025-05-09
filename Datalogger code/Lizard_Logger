/*
  This code is developed for Lizard, an open source RC car. This code runs on the data logger (optional additional receiver to collect telemetry data).
  The code was designed in Arduino IDE for an ESP32 microcontroller (ESP32-WROOM-32U; 38 pin version of NodeMCU developer board)
*/

/*
  IMPORTANT: This code is written for esp32 boards manager version 2 (tested successfully with esp32 2.0.17).
  It does not work with 3.x.x versions of Espressif Systems' esp32 boards managers (see also https://docs.espressif.com/projects/arduino-esp32/en/latest/migration_guides/2.x_to_3.0.html).
*/

#include <WiFi.h> //library for wireless communication
#include <esp_now.h> //library for bidirectional wireless communication between two ESP32
#include <esp_wifi.h> //needed to adjust the MAC address

//adjust protocol and channel of the WiFi connection manually here - must match the car's settings!
int protocol = 3; //protocols: 0 -> 802.11B; 1 -> 802.11BG; 2 -> 802.11BGN; 3 -> 802.11LR
int channel  = 8; //possible channels: 1-13

//global variables
bool newdata = false;
bool newconn = false;
bool resend = true;
int timer = 0;

//ESP-NOW setup
/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

uint8_t FirstMAC[] = {0xBE, 0xA0, 0xBE, 0xEF, 0xFE, 0xED}; //we will set this MAC address for the car
uint8_t SecondMAC[] = {0xB0, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; //we will set this MAC address for the remote control
uint8_t ThirdMAC[] = {0xBE, 0xAE, 0xBE, 0xEF, 0xFE, 0xED}; //we will set this MAC address for the logging ESP32
uint8_t broadMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //using this MAC address as a receiver means broadcasting (no confirmation from receiver)

//Structure to send data; must match the receiver structure of the car!
typedef struct send_message {
  bool logging;
} send_message;

// Create a structured object
send_message outgoing;

//Structure to receive data; must match the sender structure of the car!
typedef struct receive_message {
  int rpm;
  int voltage;
  int current;
  int input;
  int motorpwm;
  int elapsedtime;
  int motor;
} receive_message;

receive_message incoming;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS){
      Serial.println("Logging command successfully sent!");
      resend = false;
    }else{
      Serial.println("No connection found!");      
    }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len==28){
  bool sendercheck = true;
  for (int i = 0; i< 6; i++){
    if (mac[i] != FirstMAC[i]){
      sendercheck = false;
      break;
    }
  }    
    memcpy(&incoming, incomingData, sizeof(incoming));
    newdata = true;      
  }
}
esp_now_peer_info_t peerInfo; //make sure that the ESP-NOW lines are in the exact same order as in this sketch. Otherwise, there may be malfunctions.

void setup() {

  Serial.begin(115200);

  // Activate ESP-NOW
  WiFi.mode(WIFI_STA); //Set device as a Wi-Fi Station
  esp_wifi_set_mac(WIFI_IF_STA, ThirdMAC); //Overwrite hardware MAC with this new MAC address
  if (protocol==0){
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B); //the long range WiFI protocol should theoretically improve the max. distance of the wireless connection (not verified)
  }else if(protocol==1){
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G);
  }else if(protocol==2){
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N);
  }else if(protocol==3){
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  }
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  esp_now_init(); // Initialize ESP-NOW
  
  esp_wifi_set_promiscuous(true); //set the desired WiFi channel
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);     
  peerInfo.channel = channel;  
  peerInfo.encrypt = false;

  // Register peer
  memcpy(peerInfo.peer_addr, FirstMAC, 6); //enter the MAC address of the car here
  esp_now_add_peer(&peerInfo); // Add peer 

  esp_now_register_send_cb(OnDataSent);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  timer = millis();

  outgoing.logging=true;
}

void loop() {

  if (resend&&millis()>timer+1000){ //tell the car that it should send its data to the logger (sent repeatedly until connection is found)
    esp_now_send(FirstMAC, (uint8_t *) &outgoing, sizeof(outgoing));
    timer = millis();
  }

  if(newdata){ //as soon as a new data packet arrives, it is printed to the serial connection
    serialOutput();
    newdata = false;
  }

}

void serialOutput(){ //this function writes telemetry data of the car to the serial connection; each package is sent as one line; variables are separated by colons
  Serial.print(incoming.rpm); //speed given in motor rpm
  Serial.print(":");
  Serial.print(incoming.voltage); //battery voltage analog value - conversion formula provided in the RC's sketch
  Serial.print(":");
  Serial.print(incoming.current); //current draw analog value - conversion formula provided in the RC's sketch
  Serial.print(":");
  Serial.print(incoming.input); //throttle input value - between -64 (max speed reverse) and 255 (max speed forward)
  Serial.print(":");
  Serial.print(incoming.motorpwm); //motor PWM duty cycle - between 0 and 255
  Serial.print(":");
  Serial.print(incoming.elapsedtime); //runtime in ms
  Serial.print(":");
  Serial.println(incoming.motor); //motor controller: 0 - open loop PWM; 1 - closed loop speed; 2 - closed loop torque
}
