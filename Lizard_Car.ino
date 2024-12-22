/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
//Required libraries
#include <Wire.h> //library for I2C devices
#include <Adafruit_ADS1X15.h> //library for ADS1115 ADC
#include <Adafruit_PWMServoDriver.h> //library for PCA9685 servo driver
#include <Preferences.h> //library for storing variables in flash memory
#include <WiFi.h> //library for wireless communication
#include <esp_now.h> //library for bidirectional wireless communication between two ESP32
#include <esp_wifi.h> //needed to adjust the MAC address

//Pin definitions
#define dir 18
#define pwm 19
#define rear 25
#define front 27
#define fanpin 33
#define enc  26

//Setup of PWM channels
const int freq       = 20000; //PWM frequency: 20 kHz
const int resolution = 8; //PWM with 8 bit (0 to 255)
const int enginePWM  = 0; //PWM channel #0 of the ESP32 (you can have different channels with different frequencies at the same time)
const int frontLED   = 1;
const int rearLED    = 2; 
const int fanPWM     = 3;

//Add peripherals
Adafruit_ADS1115 adc; //define first ADC in sketch
#define u_sense  0 //channel 0 of ADS1115 
#define i_sense  1 //channel 1 of ADS1115
#define t0_sense 2 //channel 2 of ADS1115
#define t1_sense 3 //channel 3 of ADS1115

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(); //define servo driver
//the following lines are copy-pasted from the Adafruit PWMServoDriver library:
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600

//Global variables
int intensity = 50;
bool direction = 0;
float current = 0;
long volatile counter = 0;
int rpm = 0;

//Global variables needed for the code
int mainspeed       = 0;
int speed           = 0;
int thedir          = 0;
int steering        = 0;
int blink           = 0;
int rctimer         = 0;
int analogcounter   = 0;
int motortimer      = 0;
int engineinversion = 0;
int i_val           = 0;
int u_val           = 0;
int t0_val          = 0;
int t1_val          = 0;
int blinkcounter    = 0;

void IRAM_ATTR INT0_ISR(){ //interrupt used for tacho sensor
  counter++;
}

//ESP-NOW
// REPLACE WITH THE MAC Address of your receiver 
//uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t FirstMAC[] = {0xBE, 0xA0, 0xBE, 0xEF, 0xFE, 0xED}; //we will set this MAC address for the first µC
uint8_t SecondMAC[] = {0xB0, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; //MAC address of the 2nd µC
int signalquality = 10;

//Structure example to send data
//Must match the receiver structure
typedef struct send_message {
  int voltage;
  int current;
  int temp1;
  int temp2;
  int rpm;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
send_message outgoing;

typedef struct receive_message {
  int mainspeed;
  int steering;
  int frontlight;
  int rearlight;
  int fan;
  int control;
} receive_message;
// Create a structured object
receive_message incoming;
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS&&signalquality<19){ //signal quality is measured by changing the value of "signalquality" according to how many data packages were not received
    signalquality = signalquality +1;
  }else if(signalquality>0){
    signalquality = signalquality -1;
  }  
}
// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
}
esp_now_peer_info_t peerInfo; //make sure that the ESP-Now lines are in the exact same order as in this sketch. Otherwise, there may be malfunctions.

//The preferences library allows you to store variables within the flash so that their value can be retrieved on startup. Used to store user-adjustable settings.
Preferences preferences;
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  pinMode(dir, OUTPUT);
  digitalWrite(dir, LOW);
  pinMode(pwm, OUTPUT);
  digitalWrite(pwm, LOW);
  pinMode(front, OUTPUT);
  digitalWrite(front, LOW);
  pinMode(rear, OUTPUT);
  digitalWrite(rear, LOW);
  pinMode(fanpin, OUTPUT);
  digitalWrite(fanpin, LOW);  
  pinMode(enc, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc), INT0_ISR, FALLING);

  //Initialize I2C peripherals
  adc.begin(0x48); //default I2C address of ADS1115 (ADDR pin unconnected/at GND)
  servo.begin(); //default I2C address of PCA9685 is 0x40
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, FirstMAC);
  //esp_wifi_set_mac(WIFI_IF_STA, SecondMAC);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, SecondMAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  //this is the setup for the PWM channels
  ledcSetup(enginePWM, freq, resolution); //setup of PWM control: PWM channel, frequency, resolution
  ledcAttachPin(pwm,enginePWM); //Pin attachment: Pin, PWM channel
  ledcWrite(enginePWM, 0); //Pin control: PWM channel, duty cycle

  ledcSetup(frontLED, freq, resolution); //setup of PWM control: PWM channel, frequency, resolution
  ledcAttachPin(front, frontLED); //Pin attachment: Pin, PWM channel
  ledcWrite(frontLED, 0); //Pin control: PWM channel, duty cycle

  ledcSetup(rearLED, freq, resolution); //setup of PWM control: PWM channel, frequency, resolution
  ledcAttachPin(rear, rearLED); //Pin attachment: Pin, PWM channel
  ledcWrite(rearLED, 0); //Pin control: PWM channel, duty cycle

  ledcSetup(fanPWM, freq, resolution); //setup of PWM control: PWM channel, frequency, resolution
  ledcAttachPin(fanpin, fanPWM); //Pin attachment: Pin, PWM channel
  ledcWrite(fanPWM, 0); //Pin control: PWM channel, duty cycle 

  //retrieve values from the flash memory
  preferences.begin("Defaults", false);
  incoming.frontlight = preferences.getInt("headlights", 0);
  incoming.rearlight = preferences.getInt("brakelights",0);
  engineinversion = preferences.getInt("EngInv",0);

}
 
void loop() {
  if (millis()>rctimer+25){
    detachInterrupt(digitalPinToInterrupt(enc));
    rpm = counter*120; //that's roughly it (25 ms interval -> multiply by 40*60 for revs/min, divide by number of slots of encoder wheel (20))
    outgoing.rpm = rpm;
    counter = 0;

  switch (analogcounter){ //only one analog channel is read per loop iteration to reduce the time required per loop run (reading the ADS1115 is comparably slow)
    case 0:
      u_val  = readAnalog(u_sense);
      outgoing.voltage = u_val; //voltage is transmitted as the analog readout of the voltage divider (conversion into V is done in the RC)
      analogcounter = 1;
      break;
    case 1:
      i_val  = readAnalog(i_sense);
      outgoing.current = i_val; //current is transmitted as the analog readout of the ACS712 sensor (conversion into A is done in the RC)
      analogcounter = 2;
      break;
    case 2:  
      t0_val  = readAnalog(t0_sense);
      outgoing.temp1 = t0_val; 
      analogcounter = 3;
      break;
    case 3:  
      t1_val = readAnalog(t1_sense);
      outgoing.temp2 = t1_val;
      analogcounter = 0;
      break;
  }
    //change the direction of the main engine according to the input from the RC and the current speed (direction is only changed when the current speed is close to 0)
  if (incoming.mainspeed<0&&speed<10){
    digitalWrite(dir, HIGH); //switch the values of this line and line 338 if the car drives reverse when trying to drive forward
    thedir = 0;
  }else if(incoming.mainspeed>0&&speed<10){
    digitalWrite(dir, LOW);
    thedir = 1;
  }
 
  if (incoming.steering>149&&incoming.steering<601){//the input for the steering from the RC should always stay in this interval; thus, the servo is only addressed if the value is in this interval
    servo.setPWM(0,0,incoming.steering);
  }

  mainspeed = incoming.mainspeed; //speed input from the RC is stored in this variable to enable interpolated changes

  if (mainspeed<-64&&engineinversion==0){ //to adjust the control of front/rear LED lights, the system needs to detect if the transmitted speed value has been inverted
    engineinversion=1; //that is done by measuring in which direction a speed value of >25 % duty cycle occurs. That has to be the forward direction.
    preferences.putInt("EngInv",engineinversion);
  }else if (mainspeed>64&&engineinversion==1){
    engineinversion=0;
    preferences.putInt("EngInv",engineinversion);
  }

  if (signalquality<6){ //set the speed to 0 if the connection is lost (less than 30% success rate for sending packages to the RC is considered as "connection is lost")
    mainspeed = 0;
    incoming.mainspeed = 0;
  }

  if (incoming.control==0){
    motortimer = motortimer+1; //this variable is used to adjust the PWM interpolation for the main engine 
    if (motortimer>1){ //increasing this value makes the accelerations slower (=smoother) but the car less responsive; reducing this value makes the care more responsive but results in tire skipping when accelerating fast
      updateMotor(); //the speed interpolation for the motor is computed in a separate function
    }
  }else if (incoming.control==1){
    updateMotor();
  }else if (incoming.control==2){
    speed=0;
    ledcWrite(enginePWM,abs(incoming.mainspeed));
  }

  if (signalquality>6){
    if (incoming.frontlight>0){ //the headlight LEDs are PWM-controlled based on the input from the RC
      ledcWrite(frontLED,incoming.frontlight);
    }else{
      ledcWrite(frontLED,0);
    }
    if (incoming.rearlight>0){ //the headlight LEDs are PWM-controlled based on the input from the RC
      ledcWrite(rearLED,incoming.rearlight);
    }else{
      ledcWrite(rearLED,0);
    }  
  }else{
    ledcWrite(frontLED,0);
    blinkcounter=blinkcounter+1;
    if (blinkcounter<20){
      ledcWrite(rearLED,75);
    }else{
      ledcWrite(rearLED,0);
    }
    if(blinkcounter>39){
      blinkcounter=0;
    }
  }
    esp_err_t result = esp_now_send(SecondMAC, (uint8_t *) &outgoing, sizeof(outgoing));
    rctimer = millis();
    attachInterrupt(digitalPinToInterrupt(enc), INT0_ISR, FALLING);
  } 
}

void updateMotor(){ //function that controls the main engine; runs an interpolation of the desired speed vs. current speed
  if (thedir==0&&mainspeed<=0||thedir==1&&mainspeed>=0){ //safety precaution: speed changes relative to input speed are only done when they have the same direction
    if (speed<abs(mainspeed)-20){ //simple interpolation logic: The size of the PWM duty cycle change increment depends on the gap between input speed and current speed
      speed = speed+10;  
    }else if (speed<abs(mainspeed)-10){
      speed = speed+5;
    }else if (speed<abs(mainspeed)){
      speed = speed+1;
    }else if(speed>abs(mainspeed)+20){
      speed = speed-10;
    }else if(speed>abs(mainspeed)+10){
      speed = speed-5;
    }else if(speed>abs(mainspeed)&&speed>0){
      speed = speed-1;
    }
  }else{
    speed = speed-15; //if speed input direction and system direction are not identical (=pushing the joystick in reverse while driving forward), the system reduces the speed quickly   
  }

  ledcWrite(enginePWM,speed);
  motortimer=0;
  preferences.putInt("headlights",incoming.frontlight); //the inputs from the RC are stored in the flash memory to be available if the car suffers from a power outage
  preferences.putInt("brakelights",incoming.rearlight); //the inputs from the RC are stored in the flash memory to be available if the car suffers from a power outage  
}

int readAnalog(int InputChannel){
  int val = 0;
  val = adc.readADC_SingleEnded(InputChannel);
  if (val>17670){ //the ADS1115 in default mode measures 0.0001875 V per count -> 17670 equals 3.31 V; there shouldn't be any value higher than that -> this threshold is used to cap the input
    val=17670;
  }else if(val<0){ //cap at 0 to prevent issues caused by negative reads (noise)
    val=0;
  }
  return val;
}