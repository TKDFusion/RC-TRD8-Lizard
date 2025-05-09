/*
  This code is developed for Lizard, an open source RC car. This code runs on the car's mircrontroller.
  The code was designed in Arduino IDE for an ESP32 microcontroller (ESP32-WROOM-32U; 38 pin version of NodeMCU developer board)
*/

/*
  IMPORTANT: This code is written for esp32 boards manager version 2 (tested successfully with esp32 2.0.17).
  It does not work with 3.x.x versions of Espressif Systems' esp32 boards managers (see also https://docs.espressif.com/projects/arduino-esp32/en/latest/migration_guides/2.x_to_3.0.html).
*/

//Required libraries
#include <Wire.h> //library for I2C devices
#include <Adafruit_ADS1X15.h> //library for ADS1115 ADC
#include <Adafruit_PWMServoDriver.h> //library for PCA9685 servo driver
#include <WiFi.h> //library for wireless communication
#include <esp_now.h> //library for bidirectional wireless communication between two ESP32
#include <esp_wifi.h> //needed to adjust the MAC address
#include <Preferences.h> //library for storing variables in flash memory

//Pin definitions
#define dir 18 //main motor direction
#define pwm 19 //main motor pwm input
#define rear 25 //tail light
#define front 27 //front light
#define fanpin 33 //fan connector
#define enc  26 //encoder pin for rpm measurement

//Setup of PWM channels
const int freq       = 20000; //PWM frequency: 20 kHz
const int resolution = 8; //PWM with 8 bit (0 to 255)
const int enginePWM  = 0; //PWM channel #0 of the ESP32 (you can have different channels with different frequencies at the same time)
const int frontLED   = 1;
const int rearLED    = 2; 
const int fanPWM     = 3;

//Add peripherals
Adafruit_ADS1115 adc; //define ADC in sketch
#define u_sense  0 //channel 0 of ADS1115; voltage 
#define i_sense  1 //channel 1 of ADS1115; current
#define t0_sense 2 //channel 2 of ADS1115; thermistor 1
#define t1_sense 3 //channel 3 of ADS1115; thermistor 2

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(); //define servo driver; based on the example of the Adafruit PWMServoDriver library
#define SERVOMIN 150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600

//Global variables
long volatile counter = 0;
int rpm               = 0;
int mainspeed         = 0;
int speed             = 0;
int mspeed            = 0;
int thedir            = 0;
int rctimer           = 0;
int analogcounter     = 0;
int i_val             = 0;
int u_val             = 0;
int t0_val            = 0;
int t1_val            = 0;
int blinkcounter      = 0;
int signaltimer       = 0;
int threshold         = 1000;
int protocol          = 0;
int channel           = 0;
int counts            = 0;
int loggingtimer      = 0;
bool sendercheck      = false;
int signalquality     = 0;
bool loggingon        = false;
double setpoint       = 0;
double input          = 0;
int error             = 0;
double acs712         = 2.505; //this value sets the zero-current-draw output of the ACS712. It is 2.5 according to the datasheet but may be off a little bit.

void IRAM_ATTR INT0_ISR(){ //interrupt used for tacho sensor
  counter++; //the interrupt is kept as simple as possible to allow a fast execution
}

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

//Structure to send data; must match the receiver structure of the RC!
typedef struct send_message {
  int voltage;
  int current;
  int temp1;
  int temp2;
  int rpm;
  int signal;
} send_message;

// Create a structured object
send_message outgoing;

//Structure to send data; must match the receiver structure of the data logger!
typedef struct logging_message {
  int rpm;
  int voltage;
  int current;
  int input;
  int motorpwm;
  int elapsedtime;
  int motor;
} logging_message;

logging_message outgoing2;

//Structure to receive data; must match the sender structure of the RC!
typedef struct receive_message {
  int mainspeed;
  int steering;
  int frontlight;
  int rearlight;
  int fan;
  int control;
  int motor;
  int clim;
  int protocol;
  int channel;
} receive_message;

receive_message incoming;

//Structure to send data; must match the receiver structure of the data logger!
typedef struct receive_message2 {
  bool logging;
} receive_message2;

receive_message2 incoming2;

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  bool sendercheck = true;
  for (int i = 0; i< 6; i++){ //When using ESP-NOW in broadcast mode, this code snippet ensures that only paired devices communicate with each other (verifying the sender's MAC address)
    if (mac[i] != SecondMAC[i]){
      sendercheck = false;
      break;
    }
  }
  if (sendercheck&&len==40){ //only process the data if it is from the RC and if the packet has the correct length (data from the car to logger has a different length!)
    memcpy(&incoming, incomingData, sizeof(incoming));
    signaltimer = millis();
    if (signalquality<100){
      signalquality = signalquality+1; 
    }        
  } 
   //same procedure when checking if the data logger is present
  sendercheck = true;
  for (int i = 0; i< 6; i++){
    if (mac[i] != ThirdMAC[i]){
      sendercheck = false;
      break;
    }
  }  
  if (sendercheck&&len==1){
    memcpy(&incoming2, incomingData, sizeof(incoming2));
    loggingon = incoming2.logging;      
  } 
}
esp_now_peer_info_t peerInfo; //make sure that the ESP-NOW lines are in the exact same order as in this sketch. Otherwise, there may be malfunctions.

//the preferences library allows you to store variable values within the flash so that their value can be retrieved on startup. Used to store user-adjustable settings.
Preferences preferences; 

void setup() {

  Serial.begin(115200); //Optional - for debugging

  //specify GPIO pin usage
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

  preferences.begin("Defaults", false); //Retrieve stored settings from flash memory
  protocol = preferences.getInt("Protocol",0);
  channel  = preferences.getInt("Channel",0);

  //Initialize I2C peripherals
  adc.begin(0x48); //default I2C address of ADS1115 (ADDR pin unconnected/at GND)
  adc.setDataRate(RATE_ADS1115_860SPS); //fastest possible sampling rate for the ADS1115 chip
  servo.begin(); //default I2C address of PCA9685 is 0x40
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  Wire.setClock(400000); //increase the I2C clock rate from 100 kHz (default) to 400 kHz to increase the data rate

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

  u_val  = readAnalog(u_sense);
  outgoing.voltage = u_val; //voltage is transmitted as the analog readout of the voltage divider (conversion into V is done in the RC)
  i_val  = readAnalog(i_sense);
  outgoing.current = i_val; //current is transmitted as the analog readout of the ACS712 sensor (conversion into A is done in the RC)
  t0_val  = readAnalog(t0_sense);
  outgoing.temp1 = t0_val; //temperature is transmitted as the analog readout of the voltage divider of the NTC thermister (conversion into °C is done in the RC)
  t1_val = readAnalog(t1_sense);
  outgoing.temp2 = t1_val;

  //startup procedure: activate lights and cooling fan for a short time
  ledcWrite(fanPWM, 255);
  for (int i=20; i<=255; i++){
    ledcWrite(frontLED,i);
    ledcWrite(rearLED,i);
    delay(5);
  }
  delay(500);
  ledcWrite(fanPWM, 0);
  for (int i=255; i>=20; i--){
    ledcWrite(frontLED,i);
    ledcWrite(rearLED,i);
    delay(5);
  }
  ledcWrite(frontLED, 0);
  ledcWrite(rearLED, 0);

  // Activate ESP-NOW
  WiFi.mode(WIFI_STA); //Set device as a Wi-Fi Station
  esp_wifi_set_mac(WIFI_IF_STA, FirstMAC); //Overwrite hardware MAC with this new MAC address
  if (protocol==0){
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B); //the long range WiFI protocol should theoretically improve the max. distance of the wireless connection (not verified)
  }else if(protocol==1){
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G);
  }else if(protocol==2){
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N);
  }else if(protocol==3){
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  }

  esp_now_init(); // Initialize ESP-NOW

  // Register peer
  memcpy(peerInfo.peer_addr, broadMAC, 6);
  esp_now_add_peer(&peerInfo); // Add peer 

  esp_wifi_set_promiscuous(true); //set WiFi channel
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);     
  peerInfo.channel = channel;  
  peerInfo.encrypt = false;

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  //initialize timers
  rctimer = millis();
  loggingtimer = millis();

}
 
void loop() {

  if (loggingon&&millis()>loggingtimer+10){ //data packages are sent to the data logger every 10 ms - if it has been found as a connected device
    outgoing2.voltage     = u_val;
    outgoing2.current     = i_val;
    outgoing2.input       = mainspeed;
    outgoing2.rpm         = rpm;
    outgoing2.elapsedtime = millis();
    outgoing2.motorpwm    = speed;
    outgoing2.motor       = incoming.motor;
    esp_now_send(broadMAC, (uint8_t *) &outgoing2, sizeof(outgoing2));
    loggingtimer = millis();
  } 

  if (incoming.motor<2&&incoming.clim>0){ //current limit option is available for open-loop PWM and closed-loop speed control
    if (thedir==1&&mainspeed>0){ //it is only active when driving forward
      i_val = readAnalog(i_sense);
      if(i_val>incoming.clim){
        input    = (double(i_val)*0.0001875*(22+33)/33-acs712)/0.066; //conversion of analog values into current in amps
        setpoint = (double(incoming.clim)*0.0001875*(22+33)/33-acs712)/0.066;
        speed    = speed + currentController(setpoint,input); //call the P controller, convert its output with the last value as a sort-of PI controller
        if (speed>255){ //upper and lower boundaries of the controller
          speed = 255;
        }else if (speed<0){
          speed = 0;
        }
        ledcWrite(enginePWM,speed);
      }
    }
  }

  if (incoming.motor>1){ //closed-loop torque controller
    i_val = readAnalog(i_sense); //reading the ADC takes around 2-3 ms; that's the baseline of how often the controller is refreshed
    input = (double(i_val)*0.0001875*(22+33)/33-acs712)/0.066;
    if (input<0){ //the controller is not supposed to deal with negative currents
      input = 0;
    }
    if (mainspeed>=0){ //case: forward driving is requested
      if(thedir==1){
        if (mainspeed>0){
          setpoint = double(mainspeed)*20/255+0.4; //convert input into setpoint: maximum of 20 A; offset by 0.4 A to account for non-motor related power consumption
        }else{
          setpoint = 0.0; //make sure the car doesn't move with the throttle joystick in neutral position due to a current offset in case the actual current draw is lower (e.g., lights off instead of on)
        }
        speed = speed + currentController(setpoint,input);
        if (speed>255){ 
          speed = 255;
        }else if (speed<0){
          speed = 0;
        }
        ledcWrite(enginePWM,speed);
      }else{ //let the car stop if the operator requests forward driving while driving reverse (before calling the torque controller)
        if(speed>0){
          speed = speed-1;
          ledcWrite(enginePWM,speed);
        }else{
          thedir=1;
          digitalWrite(dir, HIGH);         
        }
      }
    }else{ //case: reverse driving is requested (interpolated throttle response, similar to open-loop PWM control)
      if (thedir==0){
        if (abs(mainspeed)>speed){
          speed = speed+1;
          delay(1);
        }else if (abs(mainspeed)<speed){
          speed = speed-1;
          delay(1);
        }
        ledcWrite(enginePWM,speed);
      }else{
        if(speed>0){
          speed = speed-1;
          delay(1);
          ledcWrite(enginePWM,speed);
        }else{
          thedir=0;
          digitalWrite(dir, LOW);
        }        
      }
    }
  }

  if (incoming.steering>149&&incoming.steering<601){//the input for the steering from the RC should always stay in this interval; thus, the servo is only addressed if the value is in this interval
    servo.setPWM(0,0,incoming.steering); //the steering input is processed every loop iteration to prevent any lag
  }

  //the main code within the loop is only executed every 25 ms to let the interrupt measure speed for some ms to reduce noise
  if (millis()>=rctimer+25){ 
    detachInterrupt(digitalPinToInterrupt(enc)); //the interrupt is disabled while running the main loop code
    rpm = counter*120; //conversion of pulse count into rpm: counter*(1000/25)*60/20 [counts times 40 (to get seconds (1000/25), times 60 for minutes, divide by number of slots of encoder wheel (20) for revs/min]
    outgoing.rpm = rpm;

  if (signalquality>0){
    signalquality = signalquality-1; //the RC is supposed to send data packages every 20 ms while the car sends telemetry data every 25-30 ms; thus, the signalquality variable is not reduced by more than 1 when the connection is reliable
  }

  switch (analogcounter){ //only one analog channel is read per loop iteration to reduce the time required per loop run (reading the ADS1115 is comparably slow)
    case 0:
      u_val  = readAnalog(u_sense);
      outgoing.voltage = u_val; 
      analogcounter = 1;
      break;
    case 1:
      i_val  = readAnalog(i_sense);
      outgoing.current = i_val; 
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

  mainspeed = incoming.mainspeed; //speed input from the RC is stored in this variable to prevent sudden changes in this value when a new input is received during the loop 

  //change the direction of the main engine according to the input from the RC and the current speed (direction is only changed when the current speed is close to 0)
  if (incoming.motor<2){
    if (mainspeed<0&&speed<10){
      digitalWrite(dir, LOW); //switch high/low in this if-statement if the car drives into the wrong direction or flip the motor wires
      thedir = 0;
    }else if(mainspeed>0&&speed<10){
      digitalWrite(dir, HIGH);
      thedir = 1;
    }
  } 

  if (millis()>signaltimer+threshold){ //set the speed to 0 if the connection is lost (defined as no new data package received for 1 s)
    mainspeed = 0;
    incoming.mainspeed = 0;
  }

  if (incoming.motor==1){ //speed controller
    if(mainspeed>25&&counter>0&&counter<100){ //only apply the speed controller if more than 10% PWM duty cycle is requested (equals approx. 4km/h if no significant rpm drop under load occurs) and when the rpm is within reasonable limits
      updateMotor(speedController(mainspeed,counter));
    }else{
      mspeed = speed; //reset the speed controller whenever it not in use
      error = 0;
      updateMotor(mainspeed);
    }
  }

  if (incoming.motor==0){ //open-loop PWM controller
    updateMotor(mainspeed); 
  }

  if (millis()<signaltimer+threshold){ //fan and light control depend on whether the RC is connected
    if (incoming.frontlight>0){ //the head light, tail light, and fan are PWM-controlled based on the input from the RC
      ledcWrite(frontLED,incoming.frontlight);
    }else{
      ledcWrite(frontLED,0);
    }
    if (incoming.rearlight>0){ 
      ledcWrite(rearLED,incoming.rearlight);
    }else{
      ledcWrite(rearLED,0);
    }  
    if (incoming.fan>0){
      ledcWrite(fanPWM,incoming.fan);
    }else{
      ledcWrite(fanPWM,0);
    }    
  }else{ //the lights blink if the RC is not connected; the fan is off
    ledcWrite(fanPWM,0);
    blinkcounter=blinkcounter+1;
    if (blinkcounter<10){
      ledcWrite(rearLED,100);
      ledcWrite(frontLED,50);
    }else{
      ledcWrite(rearLED,0);
      ledcWrite(frontLED,0);
    }
    if(blinkcounter>19){
      blinkcounter=0;
    }
  }
 
    outgoing.signal = signalquality;
    esp_now_send(broadMAC, (uint8_t *) &outgoing, sizeof(outgoing));  
    preferences.putInt("Protocol",incoming.protocol);
    preferences.putInt("Channel",incoming.channel);

    rctimer = millis();
    counter = 0;
    attachInterrupt(digitalPinToInterrupt(enc), INT0_ISR, FALLING); //the interrupt is enabled again once this code is completed
  } 

}

int currentController(double setpoint, double input){ //P controller for torque
  double value = 0.04*(setpoint - input)*255/20; //maximum change per iteration: 10 out of 255 (PWM duty cycle) -> at least 50 ms to reach max PWM
  if (value>0){
    value = ceil(value);
  }else{
    value = floor(value);
  }
  return value;
}

int speedController(int setpoint, int input){ //PD controller for speed
  input = input*3.27; //get 255 counts for approx. 40 km/h, which is the designated maximum of the speed controller
  double value = 0.06*(setpoint-input)+0.02*((setpoint-input)-error);
  error = setpoint-input; //this is the "last error", from when the function was called last time, for the D term
  mspeed = mspeed+round(value);
  if (mspeed<25){ //set upper and lower limits for the controller
    mspeed=25;
  }else if (mspeed>255){
    mspeed=255;
  }
  return mspeed;
}

void updateMotor(int speedinput){ //function that controls the main engine; runs an interpolation of the desired speed vs. current speed
  if (incoming.control==0){ //this variable specifies how fast/slow speed changes are sent to the motor driver (0=fastest; 2=slowest)
    if (thedir==0&&speedinput<=0||thedir==1&&speedinput>=0){  //safety precaution: speed changes relative to input speed are only done when they have the same direction
      if (speed<abs(speedinput)-20){ //simple interpolation logic: The size of the PWM duty cycle change increment depends on the gap between throttle input and current PWM duty cycle
        speed = speed+12;  
      }else if (speed<abs(speedinput)-10){
        speed = speed+6;
      }else if (speed<abs(speedinput)-5){
        speed = speed+3;
      }else if (speed<abs(speedinput)){
        speed = speed+1;
      }else if(speed>abs(speedinput)+20){
        speed = speed-12;
      }else if(speed>abs(speedinput)+10){
        speed = speed-6;
      }else if (speed>abs(speedinput)+5){
        speed = speed-3;
      }else if(speed>abs(speedinput)&&speed>0){
        speed = speed-1;
      }
    }else{
      speed = speed-12;   //if speed input direction and system direction are not identical (=pushing the joystick in reverse while driving forward), the system reduces the speed quickly   
    }    
  }else if (incoming.control==1){
    if (thedir==0&&speedinput<=0||thedir==1&&speedinput>=0){ 
      if (speed<abs(speedinput)-10){
        speed = speed+8;
      }else if (speed<abs(speedinput)-5){
        speed = speed+3;
      }else if (speed<abs(speedinput)){
        speed = speed+1;
      }else if(speed>abs(speedinput)+10){
        speed = speed-8;
      }else if (speed>abs(speedinput)+5){
        speed = speed-3;
      }else if(speed>abs(speedinput)&&speed>0){
        speed = speed-1;
      }
    }else{
      speed = speed-8;  
    }      
  }else{
    if (thedir==0&&speedinput<=0||thedir==1&&speedinput>=0){ 
      if (speed<abs(speedinput)-10){
        speed = speed+5;
      }else if (speed<abs(speedinput)-5){
        speed = speed+2;
      }else if (speed<abs(speedinput)){
        speed = speed+1;
      }else if(speed>abs(speedinput)+10){
        speed = speed-5;
      }else if (speed>abs(speedinput)+5){
        speed = speed-2;
      }else if(speed>abs(speedinput)&&speed>0){
        speed = speed-1;
      }
    }else{
      speed = speed-5; 
    }       
  }

  ledcWrite(enginePWM,speed);
}

int readAnalog(int InputChannel){ //function to read analog channels from the ADS1115
  int val = 0;
  val = adc.readADC_SingleEnded(InputChannel);
  if (val>17670){ //the ADS1115 in default mode measures 0.0001875 V per count -> 17670 equals 3.31 V; there shouldn't be any value higher than that -> this threshold is used to cap the input
    val=17670;
  }else if(val<0){ //cap at 0 to prevent issues caused by negative reads (noise)
    val=0;
  }
  return val;
}
