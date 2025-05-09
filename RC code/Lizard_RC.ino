/*
  This code is developed for Lizard, an open source RC car. This code runs on the remote control's microcontroller.
  The code was designed in Arduino IDE for an ESP32 microcontroller (ESP32-WROOM-32U; 38 pin version of NodeMCU developer board)
*/

/*
  IMPORTANT: This code is written for esp32 boards manager version 2 (tested successfully with esp32 2.0.17).
  It does not work with 3.x.x versions of Espressif Systems' esp32 boards managers (see also https://docs.espressif.com/projects/arduino-esp32/en/latest/migration_guides/2.x_to_3.0.html).
*/

//Required libraries
#include <Wire.h> //library for I2C devices
#include <Adafruit_GFX.h> //library for graphics functions of various displays
#include <Adafruit_SSD1306.h> //library for OLED with SSD1306 driver chip
#include <Adafruit_ADS1X15.h> //library for ADS1115 ADC
#include <WiFi.h> //library for wireless communication
#include <esp_now.h> //library for bidirectional wireless communication between two ESP32
#include <esp_wifi.h> //needed to adjust the MAC address
#include <Preferences.h> //library for storing variables in flash memory

//Pin definitions
#define LED_LR 16
#define LED_UR 17
#define LED_LL 18
#define LED_UL 19
#define BTN_LR 26
#define BTN_LL 25
#define BTN_UR 27
#define BTN_UL 33

//Global variables
double batvolt      = 0;
double voltage      = 0;
double current      = 0;
int potival         = 0;
int mainspeed       = 0;
int mainspeed1      = 0;
int mainspeed2      = 0;
int steering        = 0;
int btnlrdebounce0  = 0;
int btnlrdebounce1  = 0;
int btnlrv          = 1;
int btnurdebounce0  = 0;
int btnurdebounce1  = 0;
int btnurv          = 1;
int btnlldebounce0  = 0;
int btnlldebounce1  = 0;
int btnllv          = 1;
int btnuldebounce0  = 0;
int btnuldebounce1  = 0;
int btnulv          = 1;
int oledtimer        = 0;
int rctimer         = 0;
int lighton         = 0;
int adj             = 0;
int counter         = 0;
int menu            = 0;
int menuval         = 0;
int calval          = 0;
int joylcen         = 0;
int joyll           = 0;
int joylr           = 0;
int joyltol         = 0;
int joyrcen         = 0;
int joyru           = 0;
int joyrd           = 0;
int joyrtol         = 0;
int flightint       = 0;
int rlightint       = 0;
int leftlimit       = 0;
int rightlimit      = 0;
int batcounter      = 0;
int batlow          = 0;
int lcdcounter      = 0;
int control         = 0;
int dispstat        = 0;
int btncounter      = 0;
double v0           = 0;
double v1           = 0;
double v2           = 0;
double v3           = 0;
double v4           = 0;
double vavg         = 0;
double vmax         = 0;
double p            = 0;
double pmax         = 0;
double pavg         = 0;
double energy       = 0;
int energycounter   = 0;
double t1           = 0;
double t2           = 0;
double t            = 0;
int fanmode         = 0;
int fanspeed        = 0;
int overtemp        = 0;
int tempcounter     = 0;
int stalling        = 0;
int stallcounter    = 0;
double totaltimeout = 0;
int timeout         = 0;
int timeout0        = 0;
int timeout1        = 1;
double track        = 0;
double vavgtotal    = 0;
int speedcounter    = 0;
int currentlimit    = 0;
int steeradj        = 0;
int revs            = 0;
int calctimer       = 0;
int voltage_analog  = 0;
int current_analog  = 0;
int temp1_analog    = 0;
int temp2_analog    = 0;
int signaltimer     = 0;
int threshold       = 999;
int protocol        = 0;
int channel         = 0;
int motorcontrol    = 0;
bool sendercheck    = false;
int signalquality   = 0;
int txbatcounter    = 0;
int txbatlow        = 0;
double acs712       = 2.505; //this value sets the zero-current-draw output of the ACS712. It is 2.5 according to the datasheet but may be off a little bit.

//Add I2C peripherals
Adafruit_ADS1115 adc; //ADS1115
int joyr = 0; //these variables indicate the input channel population of the ADC (right and left joystick, poti, battery voltage)
int joyl = 1;
int poti = 2;
int batt = 3;

//2x SSD1306 OLED display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D //OLED display 0 (left display)
#define SCREEN_ADDRESS1 0x3C //OLED display 1 (right display)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000UL, 400000UL); //The last two values set the I2C clock rate. Decrease them in case you encounter stability issues
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000UL, 400000UL); //The last two values set the I2C clock rate. Decrease them in case you encounter stability issues
int diamond = 0x04; //diamond symbol for the screen

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
uint8_t broadMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //using this MAC address as a receiver for sending packages means broadcasting (ignoring if the data has been received)

// Structure to send data; must match the receiver structure of the car!
typedef struct send_message {
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
} send_message;

// Create a structured object
send_message outgoing;

//Structure to receive data; must match the sender structure of the car!
typedef struct receive_message {
  int voltage;
  int current;
  int temp1;
  int temp2;
  int rpm;
  int signal;
} receive_message;

receive_message incoming;

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  bool sendercheck = true;
  for (int i = 0; i< 6; i++){ //When using ESP-NOW in broadcast mode, this code snippet ensures that only paired devices communicate with each other (verifying the sender's MAC address)
    if (mac[i] != FirstMAC[i]){
      sendercheck = false;
      break;
    }
  }
  if (sendercheck&&len==24){ //2nd approval step: only process the data if it is from the car and if the packet has the correct length (e.g., data from car to logger has a different length)
    memcpy(&incoming, incomingData, sizeof(incoming)); //write received data into the incoming-structure to make use of them
    signaltimer = millis();  
    if (signalquality<100){ //this snippet helps to check if the car is online: the "signalquality" variable is only counted up if data is received regularly
      signalquality = signalquality+1; 
    }
  }
}
esp_now_peer_info_t peerInfo; //make sure that the ESP-NOW lines are in the exact same order as in this sketch. Otherwise, there may be malfunctions.

//the preferences library allows you to store variable values within the flash so that their value can be retrieved on startup. Used to store user-adjustable settings.
Preferences preferences;

//Interrupt functions for reading buttons
void IRAM_ATTR btnlrint(){
  btnlrdebounce1 = millis();
  if (btnlrdebounce1>btnlrdebounce0+200){ //debouncing: the button only reacts to new inputs if the last input was at least 0.2 s ago
    btnlrv = 0;    
  }
  btnlrdebounce0 = btnlrdebounce1;
}

void IRAM_ATTR btnurint(){
  btnurdebounce1 = millis();
  if (btnurdebounce1>btnurdebounce0+200){ //debouncing: the button only reacts to new inputs if the last input was at least 0.2 s ago
    btnurv = 0;    
  }
  btnurdebounce0 = btnurdebounce1;
}

void IRAM_ATTR btnllint(){
  btnlldebounce1 = millis();
  if (btnlldebounce1>btnlldebounce0+200){ //debouncing: the button only reacts to new inputs if the last input was at least 0.2 s ago
    btnllv = 0;    
  }
  btnlldebounce0 = btnlldebounce1;
}

void IRAM_ATTR btnulint(){
  btnuldebounce1 = millis();
  if (btnuldebounce1>btnuldebounce0+200){ //debouncing: the button only reacts to new inputs if the last input was at least 0.2 s ago
    btnulv = 0;    
  }
  btnuldebounce0 = btnuldebounce1;
}

void setup() {

  Serial.begin(115200); //Optional - for debugging

  preferences.begin("Defaults", false); //Retrieve stored settings from flash memory
  joyltol      = preferences.getInt("joyLtol",20);
  joyll        = preferences.getInt("joyLleft",0);
  joylr        = preferences.getInt("joyLright",0);
  joyrtol      = preferences.getInt("joyRtol",20);
  joyru        = preferences.getInt("joyRup",0);
  joyrd        = preferences.getInt("joyRdown",0);
  flightint    = preferences.getInt("headlights",150);
  rlightint    = preferences.getInt("brakelights",150);
  lighton      = preferences.getInt("lighton",0);
  leftlimit    = preferences.getInt("LeftLimit",380);
  rightlimit   = preferences.getInt("RightLimit",370);
  control      = preferences.getInt("Control",0);
  dispstat     = preferences.getInt("DisplayStatus",0);
  currentlimit = preferences.getInt("CurrentLimit",0);
  steeradj     = preferences.getInt("SteeringAdjust",0);
  fanmode      = preferences.getInt("FanMode",0);
  protocol     = preferences.getInt("Protocol", 0);
  channel      = preferences.getInt("Channel", 0);
  motorcontrol = preferences.getInt("MotorControl", false);

  //set values for the car according to what was retrieved from the flash memory
  outgoing.protocol = protocol; //tells the car which WiFi protocol to use
  outgoing.channel = channel; //tells the car which WiFi channel to use
  outgoing.motor = motorcontrol; //tells the car which motor controller option to use

  if (currentlimit==0){ //the current limit specifies the overcurrent threshold during accelerations
    outgoing.clim=0; 
  }else{
    outgoing.clim=round((double(currentlimit)*0.066+acs712)*33/(0.0001875*(22+33))); //back-calculation from current to ADC value
  }

  outgoing.control=control; //control specifies the responsiveness of the throttle <-> engine relation

  if (lighton>0){ //headlight and taillight
    outgoing.frontlight = flightint;
    outgoing.rearlight  = rlightint;
  }

  //specify GPIO pin usage
  pinMode(LED_LR, OUTPUT);
  pinMode(LED_UR, OUTPUT);
  pinMode(LED_UL, OUTPUT);
  pinMode(LED_LL, OUTPUT);
  digitalWrite(LED_LR, LOW);
  digitalWrite(LED_UR, LOW);
  digitalWrite(LED_UL, LOW);
  digitalWrite(LED_LL, LOW);
  pinMode(BTN_LR, INPUT_PULLUP);
  pinMode(BTN_UR, INPUT_PULLUP);
  pinMode(BTN_UL, INPUT_PULLUP);
  pinMode(BTN_LL, INPUT_PULLUP);
  attachInterrupt(BTN_LR, btnlrint, FALLING);
  attachInterrupt(BTN_LL, btnllint, FALLING);
  attachInterrupt(BTN_UR, btnurint, FALLING);
  attachInterrupt(BTN_UL, btnulint, FALLING);

  //Start I2C peripherals
  adc.begin(0x48); //default I2C address of ADS1115 (ADDR pin unconnected/at GND)
  adc.setDataRate(RATE_ADS1115_860SPS); //fastest possible sampling rate for the ADS1115 chip
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS); //adjust addresses or switch cables according to your preferences of left/right display
  display1.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS1);
  display.display();
  display1.display();
  display.clearDisplay();
  display1.clearDisplay();

  // Activate ESP-NOW
  WiFi.mode(WIFI_STA); //Set device as a Wi-Fi Station
  esp_wifi_set_mac(WIFI_IF_STA, SecondMAC); //Overwrite hardware MAC with this new MAC address
  if (protocol==0){
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B); 
  }else if(protocol==1){
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G);
  }else if(protocol==2){
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N);
  }else if(protocol==3){
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR); //the long range WiFI protocol should theoretically improve the max. distance of the wireless connection (not verified)
  }

  esp_now_init(); // Initialize ESP-NOW

  esp_wifi_set_promiscuous(true); //set WiFi channel
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);     
  peerInfo.channel = channel;  
  peerInfo.encrypt = false;  

  memcpy(peerInfo.peer_addr, broadMAC, 6); //data is sent in broadcast mode: sent to all listening devices
  esp_now_add_peer(&peerInfo); // Add peer   

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  //initialize timers
  oledtimer = millis();
  rctimer  = oledtimer;
  energycounter = oledtimer;
  calctimer = oledtimer;

}
 
void loop() {

  if (menu==0){ //the remote control is using multiple menu pages for its function. 0 -> default, allows driving the car
    analogInputs(); //read the joysticks
    digitalInputs(); //read all digital inputs
    if (preferences.getInt("JoyCal",0)>0&&preferences.getInt("SteerCal",0)>0){ //only if the joysticks and the steering servo are calibrated, the RC transmits their values to the car
      outgoing.mainspeed = mainspeed;
      outgoing.steering  = steering;
    }else{
      outgoing.mainspeed = 0;
      outgoing.steering = 0;     
    }
  }else if (menu==1){ //menu 1 -> selection menu to enter the different submenus
    menuInputs();
  }else if (menu==2){ //menu 2 -> calibration of the joysticks
    joyCal();
  }else if (menu==3){ //menu 3 -> calibration of the steering servo
    steerCal();
    outgoing.mainspeed = 0;
    outgoing.steering  = steering;  //steering calibration requires checking the steering angle on the car
  }else if (menu==4){ //menu 4 -> modify steering linearity
    setSteering();   
  }else if (menu==5){ //menu 5 -> adjustment of current limit
    setclim();
  }else if (menu==6){ //menu 6 -> set motor controller
    setMotor();
  }else if (menu==7){ //menu 7 -> adjustment of LED light brightness
    setLight();  
  }else if (menu==8){ //menu 8 -> adjust fan control logic
    setFan();
  }else if (menu==9){ //menu 9 -> adjust WiFi settings
    setConnection();
  }else if (menu==10){ //menu 10 -> reset Vmax and Pmax counters
    resetCounters();
  }

  if (menu>0&&menu!=3){ //make sure that speed and steering are zero when being in one of the submenus
    outgoing.mainspeed = 0;
    outgoing.steering  = 0;
  }

  if (millis()<signaltimer+threshold){ //check for data connection with the car to prevent calculating wrong speed/power values in case of connection issues
    revs           = incoming.rpm;
    voltage_analog = incoming.voltage;
    current_analog = incoming.current;
    temp1_analog   = incoming.temp1;
    temp2_analog   = incoming.temp2;
    if (timeout==1){
      timeout  = 0;
      timeout1 = millis();
      totaltimeout = totaltimeout + (timeout1-timeout0); //calculation of time without connection to the car     
    }
  }else{
    revs           = 0;
    voltage_analog = 0;
    current_analog = round(acs712*33/(0.0001875*(22+33))); //this analog value equals 0 A  
    temp1_analog   = 4548; //this analog value equals 0 °C
    temp2_analog   = 4548; 
    if (timeout==0){
      timeout  = 1;
      timeout0 = millis();      
    }
  }

  if (millis()>=calctimer+50){ //calculations and error checking don't have to be performed in realtime (every 50 ms is sufficient)

    if (signalquality>0){
      signalquality = signalquality-1; //since a new package should arrive every ~30 ms, data quality should be at 99 or higher with good connectivity
    }

    //speed & total distance
    if (abs(outgoing.mainspeed)<20&&revs>1000&&outgoing.motor<2){ //the speed sensor may record false readings if the motor is standing still at an edge of the sensor wheel
      revs=1000; //at less than 8% PWM duty cycle but more than 4.1 km/h, the tachometer does not accept higher speed readings than 1000 rpm
    }
    v0 = v1; //get a floating average of speed values reported from the car to reduce the influence of noise onto the data
    v1 = v2; 
    v2 = v3;
    v3 = v4;    
    if (millis()>signaltimer+threshold){
      v4 = 0; //the speed is set to zero if the connection to the car is lost 
    }else{
      v4 = ((double(revs)*60/4.857143)*11/100000)*3.141592654; //conversion of rpm in km/h: rpm divided by transmission ratio (4.8...), multiplied by tire diameter and divided by pi (+unit conversion)
    }
    vavg = (v0+v1+v2+v3+v4)/5; //current speed
    if (vavg>vmax){
      vmax = vavg; //max speed
    }
    track = track + (vavg/3.6)*((double(millis())-double(speedcounter))/1000); //total distance: speed multiplied by time passed since last calculation, summarized over total runtime of the car; output in m(!)
    speedcounter = millis();
    vavgtotal = (track/(double(millis())-totaltimeout))*3600; //average speed: total distance divided by runtime; corrected for connection losses

    //power & energy
    voltage = double(voltage_analog)*0.0001875*(100+33)/33.1; //voltage value of the car's battery: analog value is computed based on the 100 and 33 kOhm voltage divider
    current = double(current_analog)*0.0001875*(22+33)/33; //current draw of the car: Analog value of the ACS712 sensor, corrected for its voltage divider with 22 and 33 kOhm
    current = (current-acs712)/.066; //conversion of analog input voltage into current: ACS712 outputs 2.5 V @ 0 A and the signal scales with 66 mV/A (30 A version) [(current-2.5)/.066 for negative flow]
    p = voltage*current; //current power draw
    if (p>pmax){
      pmax = p; //max power draw
    }
    energy = energy + p*(millis()-energycounter)/(1000*3600); //total energy consumption: power multiplied by time passed since last calculation; summarized over total runtime of the car
    pavg = energy/((double(millis())-totaltimeout)/(1000*3600)); //average power: energy consumption divided by runtime; corrected for connection losses
    energycounter = millis();

    //calculate the temperature & control cooling fans
    t1 = tempcalc(temp1_analog);
    t2 = tempcalc(temp2_analog);
    fancontrol();           

    if (millis()<signaltimer+threshold){
      if (vavg<0.5&&current>=5&&stallcounter<100){ //check if the motor is stalling: more than 3 A at less than 0.5 km/h for more than 1 s
        stallcounter = stallcounter+5;
      }else if(stallcounter>0){
        stallcounter = stallcounter-1;
      }
      if (stallcounter>99){
        stalling=1;
      }else if (stallcounter<1){
        stalling=0;
      }

      if ((t1>60||t2>60)&&tempcounter<100){ //check for overheat of motor or driver: more than 65 °C for more than 1 s
        tempcounter = tempcounter+5;
      }else if (t1<55&&t2<55&&tempcounter>0){
        tempcounter = tempcounter-1;
      }
      if (tempcounter>99){
        overtemp = 1;
        fanmode = 0; //if the temps are critical, the system automatically switches to automatic fan control (prevent user-override to keep the fan off)
      }else if (tempcounter<1){
        overtemp = 0;
      }

      if (voltage<11.1&&current<2||voltage<10.2){ //thresholds for low battery: less than 11.1V@idle (below 2A; 3.7V/cell) or less than 10.2V (3.4V/cell) under load
      //these limits include a safety margin that helps to extend the life of the battery while not making using of its full capacity
      //reducing these values is done at your own risk! Never discharge a 3S LiPo battery below 9.9 V (3.3 V/cell, also not under load!) or charge it to more than 12.6 V (4.2 V/cell)!
        if (batcounter<30){ 
          batcounter=batcounter+1; 
        }
      }else if (voltage>=11.4&&batcounter>0){ //account for lower readings during startup/shutdown: >=11.4V (3.8V/cell) resets the variable
        batcounter=batcounter-1;
      }
      if (batcounter>29){ //the low battery threshold is irreversible (restart required to reset!)
        batlow=1;
      }
    }

    if (batvolt<6.8){ //similar low battery calculation for the remote control; the NiMH cells are considered empty at less than 1.13 V
      if (txbatcounter<30){
        txbatcounter=txbatcounter+1;
      }
    }else if (batvolt>6.9&&txbatcounter>0){
      txbatcounter=txbatcounter-1;
    }
    if (txbatcounter>29){ //the low battery threshold is irreversible (restart required to reset!)
      txbatlow=1;
    }

    calctimer = millis();
  }

  if (millis()>oledtimer+33){ //the screens are updated up to 8 times per second
    if (menu==0){ //the menu variable determines the "program" for the OLED display. 
      switch (counter){ //update the displays in the main menu not at the same time and only every 0.125 s per screen (because that costs most time in this loop!)
        case 0:
          updateDisplay(); //a display update costs around 13 ms!
          counter = 1;     
          break;
        case 1: //read RC battery voltage 
          batvolt = readAnalog(batt); //battery voltage of the remote control
          batvolt = double(batvolt)*0.0001875*(47+22)/22; //voltage calculation: 1 count equals 0.1875 mV with default settings of ADS1115; voltage divider with 47 and 22 kOhm      
          counter = 2;
          break;  
        case 2: 
          updateDisplay1(); //a display update costs around 13 ms!  
          counter = 3;
          break; 
        case 3: //read the potentiometer
          potival = readAnalog(poti); //the potentiometer that defines the maximum speed of the car
          if (potival<20){
            potival=20;
          }else if(potival>17580){ //17580 counts *0.0001875 V/count = 3.296 V -> low enough to ensure max speed can be reached by the poti
            potival=17580;
          }
          potival = map(potival,20,17580,255,51); //speed selection: between 20% and 100% of full PWM range; invert the output of the map function to switch min-max between left and right  
          counter = 0;
          break;
      }
    }else if(menu==1){
      menuOLED(); //selection menu: Show the different menu entries and the cursor
    }else if(menu==2){
      joyCalOLED(); //screen page for joystick calibration
    }else if (menu==3){
      steerOLED(); //screen page for steering calibration
    }else if (menu==4){
      steeringOLED(); //screen page for adjusting steering linearity
    }else if (menu==5){
      climOLED(); //screen page for adjusting current limit
    }else if (menu==6){
      motorOLED(); //screen page for adjusting motor controller
    }else if (menu==7){
      lightOLED(); //screen pagefor adjusting headlight LED brightness
    }else if(menu==8){
      fanOLED();   //screen page for adjusting fan control settings
    }else if(menu==9){
      connectionOLED(); //screen page for adjusting WiFi settings
    }else if(menu==10){
      resetOLED(); //screen page for resetting internal counters
    }

    if (menu>1){ //clear the right display whenever it is not needed (in all the different adjustment pages)
      display1.clearDisplay();
      display1.display();
    }

    oledtimer = millis();
  }

  if (millis()>=rctimer+20){ //the RC sends its inputs to the car every 20 ms (50x per second - sufficient for a responsive control)
    //the maximum loop time (refreshing a display) is 18 ms; otherwise, the loop time is in the range of 4 to 6 ms
    esp_now_send(broadMAC, (uint8_t *) &outgoing, sizeof(outgoing));
    rctimer = millis();  
  }
}

void setConnection(){ //adjustment of WiFi protocol and channel
  if (btnulv==0){ //button 4 (upper left button) increases the value of the parameter that is currently adjusted
    if (calval==1){
      digitalWrite(LED_UL, HIGH);
      if (protocol<3){
        protocol=protocol+1;
      }else{
        protocol=0;
      }
    }else if (calval==2){
      digitalWrite(LED_UL, HIGH);
      if (channel<13){
        channel=channel+1;
      }else{
        channel=1;
      }
    }
    btnulv=1;
  }
  if (btnllv==0){ //button 1 (lower left button) decreases the value of the parameter that is currently adjusted
    if (calval==1){
      digitalWrite(LED_LL, HIGH);
      if (protocol>0){
        protocol=protocol-1;
      }else{
        protocol=3;
      }
    }else if (calval==2){
      digitalWrite(LED_LL, HIGH);
      if (channel>1){
        channel=channel-1;
      }else{
        channel=13;
      }   
    }
    btnllv=1;
  }  

  if (btnurv==0){ //button 3 (upper right button): proceed to next step (save current value as the parameter that is adjusted in this step)
    digitalWrite(LED_UR, HIGH);
    btnurv=1;
    calval=calval+1;
    if (calval==2){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Protocol confirmed: ");
      display.setCursor(0,20);
      if (protocol==0){
        display.print("802.11B");
      }else if(protocol==1){
        display.print("802.11BG");
      }else if(protocol==2){
        display.print("802.11BGN");
      }else if(protocol==3){
        display.print("802.11LR");
      } 
      preferences.putInt("Protocol",protocol);
      outgoing.protocol = protocol;
      display.display();
      delay(1000);
    }else if (calval==3){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Channel confirmed: ");
      display.setCursor(0,20);
      display.print(channel);
      preferences.putInt("Channel",channel);
      outgoing.channel = channel;
      display.display();
      delay(1000);
    }else if (calval==4){
      display.clearDisplay();
      display.setCursor(0,0);  
      display.print("Settings saved.");
      display.setCursor(0,20);
      display.print("Changes active");
      display.setCursor(0,30);
      display.print("after restart!");
      display.display();
      if (millis()>signaltimer+threshold){ //tell the user that the car needs to be connected for this adjustment
        display1.clearDisplay();
        display1.setCursor(0,0);
        display1.print("Changes require"); 
        display1.setCursor(0,10);
        display1.print("connection to car:"); 
        display1.setCursor(0,20);
        display1.print("Do not restart prior"); 
        display1.setCursor(0,30);
        display1.print("to syncing with car!"); 
        display1.display();
        delay(4000);              
      }      
      delay(4000);                
      menu=1;
      calval=0;
      digitalWrite(LED_LR, HIGH);
      
    }
    display.clearDisplay();
  }

  if (btnlrv==0){ //button 2 (lower right button): go one step back in the adjustment menu
    digitalWrite(LED_LR, HIGH);
    btnlrv=1;
    calval=calval-1;
    if (calval<0){
      menu=1;
      calval=0;
    }
    display.clearDisplay();
  }    
}

void connectionOLED(){ //screen page for WiFi protocol and channel adjustment
  digitalWrite(LED_LL, LOW);
  if(menu!=1){
    digitalWrite(LED_LR, LOW);
  }else{
    digitalWrite(LED_LR, HIGH);
  }
  digitalWrite(LED_UR, LOW);
  digitalWrite(LED_UL, LOW);  
  if (calval==0){
    display.setCursor(0,0);
    display.print("Connection settings");
    display.setCursor(0,10);
    display.print("Protocol: ");
    uint8_t protocol_bitmap;
    esp_wifi_get_protocol(WIFI_IF_STA, &protocol_bitmap); 
    if (protocol_bitmap & WIFI_PROTOCOL_LR) {
      display.print("802.11LR");     
    }else if (protocol_bitmap & WIFI_PROTOCOL_11N) {
      display.print("802.11BGN");   
    }else if (protocol_bitmap & WIFI_PROTOCOL_11G) {
      display.print("802.11BG");  
    }else if (protocol_bitmap & WIFI_PROTOCOL_11B) {
      display.print("802.11B");
    }
    display.setCursor(0,20);
    display.print("Channel: ");
    display.print(WiFi.channel());    
    display.setCursor(0,30);
    display.print("Start: UR button");  
    display.setCursor(0,40);
    display.print("Return: LR button");  
  }else if (calval==1){
    display.setCursor(0,0); 
    display.print("Adj with UL/LL");
    display.setCursor(0,10);
    display.print("Protocol: ");
    if (protocol==0){
      display.print("802.11B");
    }else if(protocol==1){
      display.print("802.11BG"); 
    }else if(protocol==2){
      display.print("802.11BGN");
    }else if(protocol==3){
      display.print("802.11LR");
    }
    uint8_t protocol_bitmap;
    esp_wifi_get_protocol(WIFI_IF_STA, &protocol_bitmap); 
    if (protocol_bitmap & WIFI_PROTOCOL_LR) {
      if(protocol!=3){
        display.print("*");       
      }    
    }else if (protocol_bitmap & WIFI_PROTOCOL_11N) {
      if(protocol!=2){
        display.print("*"); 
      }      
    }else if (protocol_bitmap & WIFI_PROTOCOL_11G) {
      if(protocol!=1){
        display.print("*");
      }            
    }else if (protocol_bitmap & WIFI_PROTOCOL_11B) {
      if(protocol!=0){
        display.print("*");
      }
    }
    display.print("   ");  
    display.setCursor(0,20);
    display.print("*: Change of setting");  
    display.setCursor(0,40);
    display.print("Confirm: UR button");   
    display.setCursor(0,50);
    display.print("Return : LR button");       
  }else if (calval==2){
    display.setCursor(0,0); 
    display.print("Adj with UL/LL");
    display.setCursor(0,10);
    display.print("Channel: ");
    display.print(channel);
    if(WiFi.channel()!=channel){
      display.print("*");
    }    
    display.print("  ");  
    display.setCursor(0,20);
    display.print("*: Change of setting");        
    display.setCursor(0,40);
    display.print("Confirm: UR button");   
    display.setCursor(0,50);
    display.print("Return : LR button");       
  }else if (calval==3){
    display.setCursor(0,0); 
    display.print("Adjustment done!");    
    display.setCursor(0,20);
    display.print("Confirm: UR button");  
    display.setCursor(0,30);
    display.print("Return: LR button");      
  }
  display.display();
}

void setMotor(){ //adjust motor controller
  if (btnulv==0){ 
    btnulv = 1;
    if(motorcontrol<2){
      motorcontrol = motorcontrol+1;
    }else{
      motorcontrol = 0;
    }
  }
  if (btnllv==0){ 
    btnllv = 1;
    if(motorcontrol>0){
      motorcontrol = motorcontrol-1;
    }else{
      motorcontrol = 2;
    }
  }  
  if (btnlrv==0){
    digitalWrite(LED_LR, HIGH);
    menu=1;
    btnlrv=1;
    outgoing.motor = motorcontrol;
    preferences.putInt("MotorControl",motorcontrol);
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Controller set to:"); 
    display.setCursor(0,10);
    if (motorcontrol==0){
      display.print("Open loop (PWM)");
    }else if(motorcontrol==1){
      display.print("Closed loop (speed)");
    }else{
      display.print("Closed loop (torque)");
    }
    display.display();   
    delay(1000);
    digitalWrite(LED_LR, LOW);
    display.clearDisplay();
    digitalWrite(LED_LR, HIGH);  
  }
}

void motorOLED(){ //screen page for adjusting the motor controller
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Motor controller:");
  display.setCursor(0,10);
  if (motorcontrol==0){
    display.print("Open loop (PWM)");
  }else if(motorcontrol==1){
    display.print("Closed loop (speed)");
  }else{
    display.print("Closed loop (torque)");
  }
  display.setCursor(0,40);
  display.print("Adjust: UL/LL buttons");
  display.setCursor(0,50);
  display.print("Confirm: LR button");
  display.display();
}

void setSteering(){ //adjusting the steering response curve
  if (btnulv==0){ 
    btnulv = 1;
    if (steeradj<4){
      steeradj = steeradj+1;
    }else{
      steeradj = 0;
    }
  }
  if (btnllv==0){ 
    btnllv = 1;
    if (steeradj>0){
      steeradj = steeradj-1;
    }else{
      steeradj = 4;
    }
  }

  if (btnlrv==0){
    btnlrv = 1;
    menu=1;
    preferences.putInt("SteeringAdjust", steeradj);
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Steering type set to");
    display.setCursor(0,10);
    if (steeradj==0){
      display.print("fully linear");
    }else if (steeradj==1){
      display.print("slightly exp");
    }else if (steeradj==2){
      display.print("medium exp");
    }else if (steeradj==3){
      display.print("strongly exp");
    }else if (steeradj==4){
      display.print("fully exp");
    }    
    display.display();
    delay(1000);
    display.clearDisplay();
    digitalWrite(LED_LR, HIGH);     
  }  
}

void steeringOLED(){ //screen page for adjusting the steering response curve
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Steering linearity:");
  display.setCursor(0,10);
  if (steeradj==0){
    display.print("fully linear");
  }else if (steeradj==1){
    display.print("slightly exp");
  }else if (steeradj==2){
    display.print("medium exp");
  }else if (steeradj==3){
    display.print("strongly exp");
  }else if (steeradj==4){
    display.print("fully exp");
  }
  display.setCursor(0,40);
  display.print("Adjust: UL/LL buttons");
  display.setCursor(0,50);
  display.print("Confirm: LR button");
  display.display();
}

void setclim(){ //adjusting the motor current limit
  if (btnulv==0){ 
    btnulv = 1;
    if (currentlimit<30){
      currentlimit = currentlimit+5; //adjustment is done in 5 A steps
    }else{
      currentlimit = 0;
    }
  }
  if (btnllv==0){ 
    btnllv = 1;
    if (currentlimit>0){
      currentlimit = currentlimit-5;
    }else{
      currentlimit = 30;
    }
  }

  if (btnlrv==0){
    btnlrv = 1;
    menu=1;
    preferences.putInt("CurrentLimit", currentlimit);
    if (currentlimit==0){
      outgoing.clim=0;
    }else{
      outgoing.clim=round((double(currentlimit)*0.066+acs712)*33/(0.0001875*(22+33)));
    }
    display.clearDisplay();
    display.setCursor(0,0);
    if (currentlimit>0){
      display.print("Current limit set to");
      display.setCursor(0,10);
      display.print(currentlimit);
      display.print(" A");
    }else{
      display.print("Current limit");
      display.setCursor(0,10);
      display.print("disabled!");
    }
    display.display();
    delay(1000);
    display.clearDisplay();
    digitalWrite(LED_LR, HIGH);      
  }  
}

void climOLED(){ //screen page for adjusting current limit
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Current limit:");
  display.setCursor(0,10);
  if (currentlimit>0){
    display.print("max. ");
    display.print(currentlimit);
    display.print(" A");
  }else{
    display.print("off");
  }
  display.setCursor(0,40);
  display.print("Adjust: UL/LL buttons");
  display.setCursor(0,50);
  display.print("Confirm: LR button");
  display.display();
}

void fanOLED(){ //screen page for adjusting fan control settings
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Fan operation mode:");
  display.setCursor(0,10);
  if (fanmode==0){
    display.print("automatic");
  }else if (fanmode==1){
    display.print("maximum");
  }else if (fanmode==2){
    display.print("medium");
  }else if (fanmode==3){
    display.print("silent");
  }else if (fanmode==4){
    display.print("off");
  }
  display.setCursor(0,40);
  display.print("Adjust: UL/LL buttons");
  display.setCursor(0,50);
  display.print("Confirm: LR button");
  display.display();
}

void setFan(){ //fan mode adjustment
  if (btnulv==0){ 
    btnulv = 1;
    if (fanmode<4){
      fanmode = fanmode + 1;
    }else{
      fanmode = 0;
    }
  }
  if (btnllv==0){ 
    btnllv = 1;
    if (fanmode>0){
      fanmode = fanmode - 1;
    }else{
      fanmode = 4;
    }
  }

  if (btnlrv==0){
    btnlrv = 1;
    menu = 1;
    preferences.putInt("FanMode",fanmode);
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Fan control set to");
    display.setCursor(0,10);
    if (fanmode==0){
      display.print("automatic");
    }else if (fanmode==1){
      display.print("maximum");
    }else if (fanmode==2){
      display.print("medium");
    }else if (fanmode==3){
      display.print("silent");
    }else if (fanmode==4){
      display.print("off");
    }
    display.display();
    delay(1000);
    display.clearDisplay();
    digitalWrite(LED_LR, HIGH);   
  }  
}

void fancontrol(){ //fan control - calculations are done in the RC, not in the car
  if (fanmode==0){ //mode 0: automatic (temperature-controlled)
    if (t2>t1){ //auto-mode is based on the higher of the two temperatures
      t=t2;
    }else{
      t=t1;
    }
    if (t>=35&&t<50){
      outgoing.fan=60+(floor(t)-35)*13; //linear fan speed curve between PWM duty cycle of 60/255 (smallest safe starting value for the fans) and max fan speed
    }else if(t>=50){
      outgoing.fan=255;
    }else if(t<30){ //prevent fast on/off switching of the fans by setting the "switch off" threshold temperature 5° lower than the "switch on" threshold temperature
      outgoing.fan=0;
    }
  }else if(fanmode==1){ //mode 1: full speed
    outgoing.fan=255;
  }else if (fanmode==2){ //mode 2: medium speed
    outgoing.fan=125;
  }else if (fanmode==3){ //mode 3: minimum speed
    outgoing.fan=60;
  }else if (fanmode==4){ //mode 4: off
    outgoing.fan=0;
  }
}

double tempcalc(int inputvalue){ //temperature calculation from NTC thermistor according to: https://learn.adafruit.com/thermistor/using-a-thermistor
  double val = 0;
  val = 10000*17600/double(inputvalue)-10000; //calculate resistance from voltage divider
  val = val/10000; // (R/Ro)
  val = log(val); // ln(R/Ro)
  val = val/3435; // 1/B * ln(R/Ro)
  val = val+1.0/(25+273.15); // + (1/To)
  val = 1.0/val; // Invert
  val = val-273.15; // convert absolute temp to °C  
  return val;
  //B-value in this calculation (coefficient with a value of 3435) according to the used thermistor's datasheet!
}

void digitalInputs(){ //check digital inputs (switches & buttons) as control inputs for the car
  if (btnurv==0){ //switching the headlights on and off with the upper right button
    btnurv = 1;
    if (lighton==0){
      lighton = 1;
      preferences.putInt("lighton",1);
    }else{
      lighton = 0;
      preferences.putInt("lighton",0);
    }      
  }
  if (lighton==1){
    outgoing.frontlight = flightint; //simple transmission of an 8bit duty cycle (0-255) for the headlight intensity
    outgoing.rearlight = rlightint;
    digitalWrite(LED_UR, HIGH);
  }else{
    outgoing.frontlight = 0;
    outgoing.rearlight = 0;
    digitalWrite(LED_UR, LOW);
  }

  if (btnulv==0){ //adjusting the throttle input interpolation with the upper left button
    btnulv = 1;
    if (control<2){
      control=control+1;
    }else{
      control=0;
    }
    outgoing.control=control;
    preferences.putInt("Control",control);
    digitalWrite(LED_UL, HIGH);
    btncounter = millis();
  }

  if (btnllv==0){ //adjusting the page of the 2nd screen with the lower left button
    btnllv = 1;
    if (dispstat<4){
      dispstat=dispstat+1;
    }else{
      dispstat=0;
    }
    preferences.putInt("DisplayStatus",dispstat);
    digitalWrite(LED_LL, HIGH);
    btncounter = millis();  
  }

  if ((millis()>btncounter+1000)&&(btncounter>0)){ //show the user that pressing the left buttons caused a reaction by flashing the LED rings on for 1 s
    btncounter = 0;
    digitalWrite(LED_UL, LOW);
    digitalWrite(LED_LL, LOW);
    if (dispstat==4){
      digitalWrite(LED_UR, LOW);
    }
  }

  if (btnlrv==0){ //call the selection menu using the lower right button
    btnlrv = 1;
    menu=1;
    menuval = 0;
    digitalWrite(LED_LL, LOW);
    digitalWrite(LED_LR, HIGH);
    digitalWrite(LED_UL, LOW);
    digitalWrite(LED_UR, LOW);
    display.clearDisplay();
    display.setTextSize(1);
  }
}

void analogInputs(){ //check the joysticks as control inputs for the car
  steering = readAnalog(joyl);
  if (preferences.getInt("JoyCal")>0){
    if (joyll>joylr){ //the value of the steering joystick is capped at the limits specified during calibration (also accounts for inverted wiring of the joystick's poti)
      if (steering>joyll){
        steering = joyll;
      }else if (steering<joylr){
        steering = joylr;
      }
    }else{
      if (steering<joyll){
        steering = joyll;
      }else if (steering>joylr){
        steering = joylr;
      }
    }
    steering = map(steering,joyll,joylr,-255,255); 
  }else{
    steering = 0;
  }
  if (steering<joyltol&&steering>-joyltol){  //the tolerance range is specified during calibration; values within +-tolerance are set to 0 to avoid shivering around the center point of the joystick
    steering = 0;
  }else if (steering>=joyltol){
    steering=map(steering,joyltol,255,1,255); //values are re-mapped to make use of the full available range
  }else if (steering<=-joyltol){
    steering=map(steering,-joyltol,-255,-1,-255);  
  }

  if(steeradj==1){ //the steering can also be tuned to react more exponential than linear onto the joystick input
    if(steering>0){
      steering = pow(steering/1.65,1.1); //these functions render the response curve in the range between 0 and 255 more exponential
    }else{
      steering = -pow(abs(steering)/1.65,1.1);
    }
  }else if(steeradj==2){
    if(steering>0){
      steering = pow(steering/3.02,1.25);
    }else{
      steering = -pow(abs(steering)/3.02,1.25);
    }
  }else if(steeradj==3){
    if(steering>0){
      steering = pow(steering/6.34,1.5);
    }else{
      steering = -pow(abs(steering)/6.34,1.5);
    }    
  }else if(steeradj==4){
    if(steering>0){
      steering = pow(steering/15.96,2);
    }else{
      steering = -pow(abs(steering)/15.96,2);
    }  
  }
  steering = map(steering,-255,255,leftlimit,rightlimit); //map the steering values to the limits specified during calibration of the steering servo

  mainspeed = readAnalog(joyr);
  if (joyru>joyrd){ //the value of the speed joystick is capped at the limits specified during calibration (also accounts for inverted wiring of the joystick's poti)
    if (mainspeed>joyru){
      mainspeed = joyru;
    }else if (mainspeed<joyrd){
      mainspeed = joyrd;
    }
  }else{
    if (mainspeed<joyru){
      mainspeed = joyru;
    }else if (mainspeed>joyrd){
      mainspeed = joyrd;
    }    
  }

  mainspeed = map(mainspeed,joyrd,joyru,-potival,potival); //re-map the values at the limits specified by the speed joystick to limit maximum speed
  if (mainspeed<-64){ //limit max speed in reverse: 25% PWM duty cycle
    mainspeed = -64;
  }    

  if (mainspeed<(double(joyrtol)/255)*potival&&mainspeed>-(double(joyrtol)/255)*potival){ //keep the center of the joystick numb
    mainspeed = 0;
  }
}

void joyCal(){ //function for joystick calibration
  if (btnulv==0){ //adjustment of joystick tolerance value around center (region to be set to zero to account for tolerances) using upper left (4) and lower left (1) buttons
    if (calval==3){  //only done in the respective step of the calibration process
      digitalWrite(LED_UL, HIGH);
      if (joyltol<64){ //maximum tolerance: 25%
        joyltol=joyltol+1;
      }
    }else if (calval==6){
      digitalWrite(LED_UL, HIGH);
      if (joyrtol<64){
        joyrtol=joyrtol+1;
      }
    }
    btnulv=1;
  }
  if (btnllv==0){
    if (calval==3){
      digitalWrite(LED_LL, HIGH);
      if (joyltol>1){ //minimum tolerance: 1
        joyltol=joyltol-1;
      }
    }else if (calval==6){
      digitalWrite(LED_LL, HIGH);
      if (joyrtol>1){
        joyrtol=joyrtol-1;
      }
    }
    btnllv=1;
  }  
  if (btnurv==0){ //upper right button (3): proceed to next step of the calibration (confirm current input)
    digitalWrite(LED_UR, HIGH);
    btnurv=1;
    calval=calval+1;
    preferences.putInt("JoyCal",0);
    if (calval==1){
      joylcen = readAnalog(joyl); //the center values are recorded at the beginning to later on account for the two wiring options of the joystick poti
      joyrcen = readAnalog(joyr);
    }else if (calval==2){ //step one: left limit of left joystick
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("Value confirmed: ");
      display.setCursor(0,10);
      display.print(map(readAnalog(joyl),0,17670,0,255));
      if (readAnalog(joyl)>joylcen){ //the left and right limits are recorded and slightly reduced to ensure that the maximum can be reached even if there is noise in the readings
        preferences.putInt("joyLleft",readAnalog(joyl)-50);
      }else{
        preferences.putInt("joyLleft",readAnalog(joyl)+50);      
      }
      display.display();
      delay(1000);
    }else if (calval==3){ //step two: right limit of left joystick
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("Value confirmed: ");
      display.setCursor(0,10);
      display.print(map(readAnalog(joyl),0,17670,0,255));
      if (readAnalog(joyl)>joylcen){
        preferences.putInt("joyLright",readAnalog(joyl)-50);
      }else{
        preferences.putInt("joyLright",readAnalog(joyl)+50);      
      }
      display.display();
      delay(1000);
    }else if (calval==4){ //step three: center tolerance of left joystick
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("Value confirmed: ");
      display.setCursor(0,10);
      display.print((double(joyltol)/double(255))*double(100),1);
      display.print(" %   ");
      preferences.putInt("joyLtol",joyltol);
      display.display();
      delay(1000);
    }else if (calval==5){ //step four: upper limit of right joystick
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("Value confirmed: ");
      display.setCursor(0,10);
      display.print(map(readAnalog(joyr),0,17670,0,255));
      if (readAnalog(joyr)>joyrcen){
        preferences.putInt("joyRup",readAnalog(joyr)-50);
      }else{
        preferences.putInt("joyRup",readAnalog(joyr)+50);      
      }
      display.display();
      delay(1000);
    }else if (calval==6){ //step five: lower limit of right joystick
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("Value confirmed: ");
      display.setCursor(0,10);
      display.print(map(readAnalog(joyr),0,17670,0,255));
      if (readAnalog(joyr)>joyrcen){
        preferences.putInt("joyRdown",readAnalog(joyr)-50);
      }else{
        preferences.putInt("joyRdown",readAnalog(joyr)+50);      
      }
      display.display();
      delay(1000);
    }else if (calval==7){ //step six: center tolerance of right joystick
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("Value confirmed: ");
      display.setCursor(0,10);
      display.print((double(joyrtol)/double(255))*double(100),1);
      display.print(" %   ");
      preferences.putInt("joyRtol",joyrtol);
      display.display();
      delay(1000);
    }else if (calval==8){ //step eight: confirmation of calibration
      menu=1;
      calval=0;
      preferences.putInt("JoyCal",1);
      joyltol = preferences.getInt("joyLtol",20);
      joyll = preferences.getInt("joyLleft",0);
      joylr = preferences.getInt("joyLright",0);
      joyrtol = preferences.getInt("joyRtol",20);
      joyru = preferences.getInt("joyRup",0);
      joyrd = preferences.getInt("joyRdown",0);
      digitalWrite(LED_LR, HIGH);
    }
    display.clearDisplay();
    display.setTextSize(1);
  }
  if (btnlrv==0){ //lower right button: go back one step in the calibration
    digitalWrite(LED_LR, HIGH);
    btnlrv=1;
    calval=calval-1;
    if (calval<0){
      menu=1;
      calval=0;
    }
    display.clearDisplay();
    display.setTextSize(1);
  }  
}

void joyCalOLED(){ //LCD screen for joystick calibration
  digitalWrite(LED_LL, LOW);
  if(menu!=1){
    digitalWrite(LED_LR, LOW);
  }else{
    digitalWrite(LED_LR, HIGH);
  }
  digitalWrite(LED_UR, LOW);
  digitalWrite(LED_UL, LOW);  
  if (calval==0){
    if (preferences.getInt("JoyCal",0)>0){
      display.setCursor(0,0);
      display.print("Joystick calibration");
      display.setCursor(0,10);
      display.print("Calibration found   ");
      display.setCursor(0,20);
      display.print("Override: UR button");  
      display.setCursor(0,30);
      display.print("Return: LR button");  
    }else{
      display.setCursor(0,0);
      display.print("Joystick calibration");
      display.setCursor(0,10);      
      display.print("No calibration found");
      display.setCursor(0,20);
      display.print("Start cal: UR button");  
      display.setCursor(0,30);
      display.print("Return: LR button");        
    }
  }else if (calval==1){
    display.setCursor(0,0); 
    display.print("Move JoyL left ");
    display.setCursor(0,10);
    display.print("JoyL value: ");
    display.print(map(readAnalog(joyl),0,17670,0,255));
    display.print("  ");    
    display.setCursor(0,20);
    display.print("Confirm: UR button");   
    display.setCursor(0,30);
    display.print("Return : LR button");       
  }else if (calval==2){
    display.setCursor(0,0); 
    display.print("Move JoyL right");
    display.setCursor(0,10);
    display.print("JoyL value: ");
    display.print(map(readAnalog(joyl),0,17670,0,255));
    display.print("  ");    
    display.setCursor(0,20);
    display.print("Confirm: UR button");   
    display.setCursor(0,30);
    display.print("Return : LR button");       
  }else if (calval==3){
    display.setCursor(0,0); 
    display.print("Joy L: tolerance");    
    display.setCursor(0,10);  
    if (joyltol>1&&joyltol<64){ //tolerance values are given in per cent
      display.print("Value: ");   
      display.print((double(joyltol)/double(255))*double(100),1);
      display.print(" %   ");
    }else{
      display.print("Limit reached! ");
    }
    display.setCursor(0,20);
    display.print("Confirm: UR button");  
    display.setCursor(0,30);
    display.print("Return: LR button");     
  }else if (calval==4){
    display.setCursor(0,0); 
    display.print("Move JoyR up ");
    display.setCursor(0,10);
    display.print("JoyR value: ");
    display.print(map(readAnalog(joyr),0,17670,0,255));
    display.print("  ");
    display.setCursor(0,20);
    display.print("Confirm: UR button");   
    display.setCursor(0,30);
    display.print("Return : LR button");       
  }else if (calval==5){
    display.setCursor(0,0); 
    display.print("Move JoyR down");
    display.setCursor(0,10);
    display.print("JoyR value: ");
    display.print(map(readAnalog(joyr),0,17670,0,255));
    display.print("  ");
    display.setCursor(0,20);
    display.print("Confirm: UR button");   
    display.setCursor(0,30);
    display.print("Return : LR button");       
  }else if (calval==6){
    display.setCursor(0,0); 
    display.print("Joy R: tolerance");    
    display.setCursor(0,10); 
    if (joyrtol>1&&joyrtol<64){
      display.print("Value: ");   
      display.print((double(joyrtol)/double(255))*double(100),1);
      display.print(" %   ");
    }else{
      display.print("Limit reached! ");
    }
    display.setCursor(0,20);
    display.print("Confirm: UR button");  
    display.setCursor(0,30);
    display.print("Return: LR button");         
  }else if (calval==7){
    display.setCursor(0,0); 
    display.print("Calibration done!");    
    display.setCursor(0,20);
    display.print("Confirm: UR button");  
    display.setCursor(0,30);
    display.print("Return: LR button");       
  }
  display.display();
}

void menuInputs(){ //function for navigating in the selection menu
  if (btnulv==0){ //upper left button: move cursor up by one entry
    digitalWrite(LED_UL, HIGH);
    if (menuval>0){
      menuval = menuval-1;
    }else{
      menuval = 9;
    }
    btnulv=1;
  }
  if (btnllv==0){ //lower left button: move cursor down by one entry
    digitalWrite(LED_LL, HIGH);
    if (menuval<9){
      menuval = menuval+1;
    }else{
      menuval = 0;
    }
    btnllv=1;
  }
  if (btnlrv==0){ //go from menu to the selected menu entry by pressing the lower right button
    btnllv = 1; //in case any of the buttons was pressed: return to default state
    btnlrv = 1;
    btnurv = 1;
    btnulv = 1;
    if(menuval==0){ //the cursor of the selection menu has a different number than the menu entries!
      menu=2;
    }else if(menuval==1){
      menu=3;
    }else if(menuval==2){
      menu=4;  
    }else if(menuval==3){
      menu=5;
    }else if(menuval==4){
      menu=6;
    }else if(menuval==5){
      menu=7;
    }else if(menuval==6){
      menu=8;
    }else if(menuval==7){
      menu=9;
    }else if(menuval==8){
      menu=10;
    }else if(menuval==9){
      menu=0;
    }
    digitalWrite(LED_LR, LOW);
    display.clearDisplay();
    display1.clearDisplay();
    display.setTextSize(1);
    display1.clearDisplay();
  }   
}

void menuOLED(){ //screen page for the selection menu
  display.clearDisplay();
  display.setCursor(8,0);
  display.print("Joystick calibration");
  display.setCursor(8,10);
  display.print("Steering calibration");
  display.setCursor(8,20);
  display.print("Steering mode");
  display.setCursor(8,30);
  display.print("Motor current limit");
  display.setCursor(8,40);
  display.print("Motor controller");

  display1.clearDisplay();
  display1.setCursor(8,0); 
  display1.print("Light intensity");    
  display1.setCursor(8,10);
  display1.print("Fan control"); 
  display1.setCursor(8,20);
  display1.print("WiFi settings");
  display1.setCursor(8,30);
  display1.print("Counter reset"); 
  display1.setCursor(8,40);
  display1.print("Return       "); 
  for (int i=0; i<=4; i++){ //show the current position of the cursor
    display.setCursor(0,i*10);
    if (menuval==i){
      display.write(diamond);
    }else{
      display.print(" ");
    }
  }
  for (int i=0; i<=4; i++){
    display1.setCursor(0,i*10);
    if (menuval-5==i){
      display1.write(diamond);
    }else{
      display1.print(" ");
    }
  }
  display.display();
  display1.display();
  digitalWrite(LED_UL, LOW);
  digitalWrite(LED_UR, LOW);
  digitalWrite(LED_LL, LOW);
}

void resetCounters(){ //reset Vmax and Pmax counters
  if (btnurv==0){
    digitalWrite(LED_UR, HIGH);
    vmax=0;
    pmax=0;
    menu=1;
    btnurv=1;
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Reset done!");
    display.display();
    delay(1000);
    digitalWrite(LED_UR, LOW);
    display.clearDisplay();
    digitalWrite(LED_LR, HIGH);   
  }
  if (btnlrv==0){
    digitalWrite(LED_LR, HIGH);
    menu=1;
    btnlrv=1;
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Reset aborted!"); 
    display.display();   
    delay(1000);
    digitalWrite(LED_LR, LOW);
    display.clearDisplay();
    digitalWrite(LED_LR, HIGH);  
  }

}

void resetOLED(){ //screen page for resetting internal counters
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Reset counters?");
  display.setCursor(0,10);
  display.print("Vmax: ");
  display.print(vmax,1);
  display.print(" km/h");
  display.setCursor(0,20);
  display.print("Pmax: ");
  display.print(pmax,0);
  display.print(" W");
  display.setCursor(0,40);
  display.print("Reset: UR button");
  display.setCursor(0,50);
  display.print("Abort: LR button");
  display.display();
}

void steerCal(){ //calibration of the steering servo
  if (calval==1){ //the remote control transmits fixed values for the steering servo depending on the step of the calibration
    steering=leftlimit; //step one: left limit of the steering servo
  }else if (calval==2){
    steering=rightlimit; //step two: right limit of the steering servo
  }else{
    steering=(leftlimit+rightlimit)/2; //step three: center position of the steering servo
  }
  if (btnulv==0){ //upper left button increases the value of the limit that is currently adjusted
    if (calval==1){
      digitalWrite(LED_UL, HIGH);
      if (leftlimit<596){
        leftlimit=leftlimit+5;
      }
    }else if (calval==2){
      digitalWrite(LED_UL, HIGH);
      if (rightlimit<596){
        rightlimit=rightlimit+5;
      }
    }else if (calval==3){ //step 3 does fine tuning of the position by changing the values by smaller increments
      digitalWrite(LED_UL, HIGH);
      if (leftlimit<600&&rightlimit<600){ //usable limit for servos: between 150 and 600 according to Adafruit's guidelines for the PWMServo library
        leftlimit=leftlimit+1;
        rightlimit=rightlimit+1;
      }
    }
    btnulv=1;
  }
  if (btnllv==0){ //lower left button decreases the value of the limit that is currently adjusted
    if (calval==1){
      digitalWrite(LED_LL, HIGH);
      if (leftlimit>154){
        leftlimit=leftlimit-5;
      }
    }else if (calval==2){
      digitalWrite(LED_LL, HIGH);
      if (rightlimit>154){
        rightlimit=rightlimit-5;
      }
    }else if (calval==3){
      digitalWrite(LED_LL, HIGH);
      if (leftlimit>150&&rightlimit>150){
        leftlimit=leftlimit-1;
        rightlimit=rightlimit-1;
      }      
    }
    btnllv=1;
  }  
  if (btnurv==0){ //upper right button: proceed to next step (save current value as the limit that is adjusted in this step)
    digitalWrite(LED_UR, HIGH);
    btnurv=1;
    calval=calval+1;
    preferences.putInt("SteerCal",0);
    if (calval==2){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Value confirmed: ");
      display.setCursor(0,20);
      display.print(leftlimit);
      preferences.putInt("LeftLimit",leftlimit);
      display.display();
      delay(1000);
    }else if (calval==3){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Value confirmed: ");
      display.setCursor(0,20);
      display.print(rightlimit);
      preferences.putInt("RightLimit",rightlimit);
      display.display();
      delay(1000);
    }else if (calval==4){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Value confirmed: ");
      display.setCursor(0,20);
      display.print((leftlimit+rightlimit)/2);
      preferences.putInt("LeftLimit",leftlimit);
      preferences.putInt("RightLimit",rightlimit);
      display.display();
      delay(1000);
    }else if (calval==5){
      menu=1;
      calval=0;
      preferences.putInt("SteerCal",1); //store in flash memory that the calibration of the steering servo has been performed
      digitalWrite(LED_LR, HIGH);
    }
    display.clearDisplay();
  }
  if (btnlrv==0){ //lower right button: go one step back in the calibration
    digitalWrite(LED_LR, HIGH);
    btnlrv=1;
    calval=calval-1;
    if (calval<0){
      menu=1;
      calval=0;
    }
    display.clearDisplay();
  }  
}

void steerOLED(){ //screen page of the steering servo calibration
  digitalWrite(LED_LL, LOW);
  if(menu!=1){
    digitalWrite(LED_LR, LOW);
  }else{
    digitalWrite(LED_LR, HIGH);
  }
  digitalWrite(LED_UR, LOW);
  digitalWrite(LED_UL, LOW);  
  if (calval==0){
    if (preferences.getInt("SteerCal",0)>0){
      display.setCursor(0,0);
      display.print("Steering calibration");
      display.setCursor(0,10);
      display.print("Calibration found   ");
      display.setCursor(0,20);
      display.print("Override: UR button");  
      display.setCursor(0,30);
      display.print("Return: LR button");  
    }else{
      display.setCursor(0,0);
      display.print("Steering calibration");
      display.setCursor(0,10);      
      display.print("No calibration found");
      display.setCursor(0,20);
      display.print("Start cal: UR button");  
      display.setCursor(0,30);
      display.print("Return: LR button");        
    }
  }else if (calval==1){
    display.setCursor(0,0); 
    display.print("Adj left with UL/LL");
    display.setCursor(0,10);
    display.print("Left limit: ");
    display.print(leftlimit);
    display.print("  ");    
    display.setCursor(0,20);
    display.print("Confirm: UR button");   
    display.setCursor(0,30);
    display.print("Return : LR button");       
  }else if (calval==2){
    display.setCursor(0,0); 
    display.print("Adj right with UL/LL");
    display.setCursor(0,10);
    display.print("Right limit: ");
    display.print(rightlimit);
    display.print("  ");    
    display.setCursor(0,20);
    display.print("Confirm: UR button");   
    display.setCursor(0,30);
    display.print("Return : LR button");       
  }else if (calval==3){
    display.setCursor(0,0); 
    display.print("Adj center with UL/LL");    
    display.setCursor(0,10); 
    display.print("Center: ");    
    display.print((leftlimit+rightlimit)/2);
    display.setCursor(0,20);
    display.print("Confirm: UR button");  
    display.setCursor(0,30);
    display.print("Return: LR button");     
  }else if (calval==4){
    display.setCursor(0,0); 
    display.print("Calibration done!");    
    display.setCursor(0,20);
    display.print("Confirm: UR button");  
    display.setCursor(0,30);
    display.print("Return: LR button");      
  }
  display.display();
}

void setLight(){ //adjustment of the headlight and taillight LED brightness
  if (btnulv==0){ //upper left button increases the value that is currently adjusted
    if (calval==1){
      digitalWrite(LED_UL, HIGH);
      if (flightint<251){
        flightint=flightint+5;
      }
    }else if (calval==2){
      digitalWrite(LED_UL, HIGH);
      if (rlightint<251){
        rlightint=rlightint+5;
      }
    }
    btnulv=1;
  }
  if (btnllv==0){ //lower left button decreases the value that is currently adjusted
    if (calval==1){
      digitalWrite(LED_LL, HIGH);
      if (flightint>24){
        flightint=flightint-5;
      }
    }else if (calval==2){
      digitalWrite(LED_LL, HIGH);
      if (rlightint>24){
        rlightint=rlightint-5;
      }     
    }
    btnllv=1;
  }  

  if (btnurv==0){ //upper right button: proceed to next step (save current value that is adjusted in this step)
    digitalWrite(LED_UR, HIGH);
    btnurv=1;
    calval=calval+1;
    if (calval==2){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Value confirmed: ");
      display.setCursor(0,20);
      display.print(flightint);
      preferences.putInt("headlights",flightint);
      display.display();
      delay(1000);
    }else if (calval==3){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Value confirmed: ");
      display.setCursor(0,20);
      display.print(rlightint);
      preferences.putInt("brakelights",rlightint);
      display.display();
      delay(1000);
    }else if (calval==4){
      menu=1;
      calval=0;
      digitalWrite(LED_LR, HIGH);
    }
    display.clearDisplay();
  }

  if (btnlrv==0){ //lower right button: go one step back
    digitalWrite(LED_LR, HIGH);
    btnlrv=1;
    calval=calval-1;
    if (calval<0){
      menu=1;
      calval=0;
    }
    display.clearDisplay();
  }    
}

void lightOLED(){ //screen page for headlight LED adjustment
  digitalWrite(LED_LL, LOW);
  if(menu!=1){
    digitalWrite(LED_LR, LOW);
  }else{
    digitalWrite(LED_LR, HIGH);
  }
  digitalWrite(LED_UR, LOW);
  digitalWrite(LED_UL, LOW);  
  if (calval==0){
    display.setCursor(0,0);
    display.print("Light adjustment");
    display.setCursor(0,10);
    display.print("Front light: ");
    display.print(flightint);
    display.setCursor(0,20);
    display.print("Rear light: ");
    display.print(rlightint);    
    display.setCursor(0,30);
    display.print("Start: UR button");  
    display.setCursor(0,40);
    display.print("Return: LR button");  
  }else if (calval==1){
    display.setCursor(0,0); 
    display.print("Adj front with UL/LL");
    display.setCursor(0,10);
    display.print("Intensity: ");
    display.print(flightint);
    display.print("  ");    
    display.setCursor(0,20);
    display.print("Confirm: UR button");   
    display.setCursor(0,30);
    display.print("Return : LR button");       
  }else if (calval==2){
    display.setCursor(0,0); 
    display.print("Adj rear with UL/LL");
    display.setCursor(0,10);
    display.print("Intensity: ");
    display.print(rlightint);
    display.print("  ");    
    display.setCursor(0,20);
    display.print("Confirm: UR button");   
    display.setCursor(0,30);
    display.print("Return : LR button");       
  }else if (calval==3){
    display.setCursor(0,0); 
    display.print("Adjustment done!");    
    display.setCursor(0,20);
    display.print("Confirm: UR button");  
    display.setCursor(0,30);
    display.print("Return: LR button");      
  }
  display.display();
}

void updateDisplay(){ //standard page of left OLED display
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE,BLACK);
  display.setCursor(0,0);
  if (txbatlow==1){
    display.setTextSize(1);
    display.print("TX ");
    display.print(batvolt);  
    display.print(" V");
    display.setCursor(0,10);
    display.print("TX battery low!");
    display.setTextSize(2);  
  }else{
    display.print("TX ");
    display.print(batvolt);  
    display.print(" V");
  }
  if (preferences.getInt("JoyCal",0)==0||preferences.getInt("SteerCal",0)==0){
    display.setTextSize(1);
    display.setCursor(0,28);
    if (preferences.getInt("JoyCal",0)==0&&preferences.getInt("SteerCal",0)==0){
      display.print("Joysticks & steering");
      display.setCursor(0,38);
      display.print("are not calibrated!");
    }else if(preferences.getInt("JoyCal",0)>0&&preferences.getInt("SteerCal",0)==0){
      display.print("Steering system");
      display.setCursor(0,38);
      display.print("is not calibrated!");      
    }else if(preferences.getInt("JoyCal",0)==0&&preferences.getInt("SteerCal",0)>0){
      display.print("Joysticks");
      display.setCursor(0,38);
      display.print("are not calibrated!");        
    }
    display.setCursor(0,53);
    if (millis()<(signaltimer+threshold)){    
      display.print("RX connected!");
    }else{
      display.print("RX not available!");     
    }
  }else if (millis()<(signaltimer+threshold)&&batlow<1&&stalling<1&&overtemp<1){
    display.setCursor(0,20);
    display.print("RX ");
    display.print(voltage);
    display.print(" V");
    display.setTextSize(1);
    display.setCursor(0,40);
    if (motorcontrol==0){
      display.print("PWM, ");
    }else if(motorcontrol==1){
      display.print("SPD, ");
    }else{
      display.print("TRQ, ");
    }
    if (motorcontrol<2){
      if (control==0){
        display.print("overdrive, ");
      }else if(control==1){
        display.print("responsive, ");
      }else if(control==2){
        display.print("stagnant, ");
      }      
      if (currentlimit>0){
        display.print(currentlimit);
        display.print(" A");
      }else{ //a "custom" infinity symbol
        display.setCursor(display.getCursorX(),display.getCursorY()-1);
        display.print("o");
        display.setCursor(display.getCursorX()-1,display.getCursorY());
        display.print("o");
      }
    }else{
      display.print(double(potival)*20/255,0);
      display.print(" A");        
    }
    display.setCursor(0,50);
    if (outgoing.frontlight==0&&outgoing.rearlight==0){
      display.print("Lights off");
    }else{
      display.print("Lights on");
    }
  }else if(millis()<(signaltimer+threshold)&&batlow>0){
      display.setTextSize(2);
      display.setCursor(0,20);
      display.print("RX battery");
      display.setCursor(0,40);  
      display.print("low!");  
  }else if(millis()<(signaltimer+threshold)&&stalling>0){
      display.setTextSize(2);
      display.setCursor(0,20);
      display.print("Motor");  
      display.setCursor(0,40);
      display.print("stalling!");
  }else if(millis()<(signaltimer+threshold)&&overtemp>1){
      display.setTextSize(2);
      display.setCursor(0,20);
      display.print("Temp.");
      display.setCursor(0,40);
      display.print("critical!");     
  }else{
    display.setTextSize(1);
    display.setCursor(0,30);
    display.print("RX not available!");
  }
  display.display();

}

void updateDisplay1(){ //standard page of right OLED display
  display1.clearDisplay();
  display1.setTextSize(3);
  display1.setTextColor(WHITE,BLACK);
  display1.setCursor(0, 0);
  if (dispstat==0){ //show power values
    if (p<100){
      display1.print(p,1);
    }else{
      display1.print(p,0);
    }  
    display1.print(" W");
    display1.setTextSize(1);
    display1.setTextColor(WHITE,BLACK);
    display1.setCursor(0,30);
    display1.print("Pavg: ");
    display1.print(pavg, 1);
    display1.print(" W");         
    display1.setCursor(0,40);
    display1.print("Pmax: ");
    display1.print(pmax, 1);
    display1.print(" W");   
    display1.setCursor(0,50);
    display1.print("Total: ");
    display1.print(energy,1);
    display1.print(" Wh (");
    display1.print(100*energy/55.5,0);
    display1.print("%)");
  }else if (dispstat==1){ //show speed values
    if (vavg<10){
      display1.print(vavg, 1);
    }else{
      display1.print(vavg, 0);
    }
    display1.setTextSize(2);
    display1.setCursor(display1.getCursorX(), 7);
    display1.print(" km/h");   
    display1.setTextSize(1);
    display1.setTextColor(WHITE,BLACK);
    display1.setCursor(0,30);
    display1.print("Dist: ");
    display1.print(track/1000, 1);
    display1.print(" km");   
    display1.setCursor(0,40);
    display1.print("Vavg: ");
    display1.print(vavgtotal, 1);
    display1.print(" km/h");         
    display1.setCursor(0,50);
    display1.print("Vmax: ");
    display1.print(vmax, 1);
    display1.print(" km/h");  
  }else if (dispstat==2){ //show sensor values
    display1.setTextSize(1);
    display1.setTextColor(WHITE,BLACK);
    display1.setCursor(0, 0);
    display1.print("Sensor status:");
    display1.setCursor(0, 10);
    display1.print("Battery: ");
    display1.print(voltage);
    display1.print(" V");
    display1.setCursor(0, 20);
    display1.print("Current: ");
    display1.print(current);
    display1.print(" A");
    display1.setCursor(0,30);
    display1.print("Motor temp: ");      
    display1.print(t1,0);
    display1.print(" ");
    display1.print(char(247));
    display1.print("C");
    display1.setCursor(0, 40);
    display1.print("Driver temp: ");   
    display1.print(t2,0);
    display1.print(" ");
    display1.print(char(247));
    display1.print("C");
    display1.setCursor(0,50);
    display1.print("Speed: ");
    display1.print(incoming.rpm);
    display1.print(" rpm ");      
  }else if (dispstat==3){ //show settings
    display1.setTextSize(1);
    display1.setTextColor(WHITE,BLACK);
    display1.setCursor(0, 0);
    display1.print("Settings:");
    display1.setCursor(0,10);
    display1.print("Motor: ");
    if (motorcontrol==0){
      display1.print("OL PWM, ");
    }else if(motorcontrol==1){
      display1.print("CL SPD, ");
    }else{
      display1.print("CL TRQ, ");
    }  
    if (motorcontrol<2){
      display1.setCursor(0,20);
      if (control==0){
        display1.print("overdrive, ");
      }else if(control==1){
        display1.print("responsive, ");
      }else if(control==2){
        display1.print("stagnant, ");
      }
      if (currentlimit>0){
        display1.print(currentlimit);
        display1.print(" A");  
      }else{
        display1.print("no limit");
      }  
    }else{
      display1.print(double(potival)*20/255,0);
      display1.print(" A");        
    }
    display1.setCursor(0,33);
    display1.print("Steer.: ");      
    if (steeradj==0){
      display1.print("fully linear");
    }else if (steeradj==1){
      display1.print("slightly exp");
    }else if (steeradj==2){
      display1.print("medium exp");
    }else if (steeradj==3){
      display1.print("strongly exp");
    }else if (steeradj==4){
      display1.print("fully exp");
    }          
    display1.setCursor(0, 43);
    display1.print("Lights: F");
    display1.print(flightint);
    display1.print(", R");
    display1.print(rlightint);
    display1.setCursor(0, 53);
    display1.print("Fan mode: ");   
    if (fanmode==0){
      display1.print("automatic");
    }else if (fanmode==1){
      display1.print("maximum");
    }else if (fanmode==2){
      display1.print("medium");
    }else if (fanmode==3){
      display1.print("silent");
    }else if (fanmode==4){
      display1.print("off");
    }     
  }else if(dispstat==4){ //show signal status
    display1.setTextSize(1);
    display1.setTextColor(WHITE,BLACK);
    display1.setCursor(0, 0);
    display1.print("Signal status:"); 
    display1.setCursor(0, 10);      
    display1.print("TX: ");
    if (signalquality>95){
      display1.print("excellent");
    }else if (signalquality>90){
      display1.print("very good");
    }else if (signalquality>85){
      display1.print("good");
    }else if (signalquality>80){
      display1.print("acceptable");
    }else{
      display1.print("poor");
    }
    display1.setCursor(0, 20);      
    display1.print("RX: ");
    if (incoming.signal>95){
      display1.print("excellent");
    }else if (incoming.signal>90){
      display1.print("very good");
    }else if (incoming.signal>85){
      display1.print("good");
    }else if (incoming.signal>80){
      display1.print("acceptable");
    }else{
      display1.print("poor");
    }
    display1.setCursor(0,30);
    display1.print("Protocol: ");
    uint8_t protocol_bitmap;
    esp_wifi_get_protocol(WIFI_IF_STA, &protocol_bitmap); 
    if (protocol_bitmap & WIFI_PROTOCOL_LR) {
      display1.print("802.11LR");
      if(protocol!=3||WiFi.channel()!=channel){
        if(protocol!=3){
          display1.print("*");
        }
        display1.setCursor(0,50);
        display1.print("Restart: ");
        if (protocol==0){
          display1.print("B, ");
          display1.print(channel);
        }else if(protocol==1){
          display1.print("BG, ");
          display1.print(channel);
        }else if(protocol==2){
          display1.print("BGN, ");
          display1.print(channel);
        }else if(protocol==3){
          display1.print("LR, ");
          display1.print(channel);
        }          
      }else{
        display1.setCursor(0,50);
        display1.print("Time: ");
        if(millis()<signaltimer+threshold){
          display1.print(double((millis()-totaltimeout)/60000),1);
        }else{
          display1.print(double((timeout0-totaltimeout)/60000),1);
        }
        display1.print(" min");
      }      
    }else if (protocol_bitmap & WIFI_PROTOCOL_11N) {
      display1.print("802.11BGN");
      if(protocol!=2||WiFi.channel()!=channel){
        if(protocol!=2){
          display1.print("*");
        }
        display1.setCursor(0,50);
        display1.print("Restart: ");
        if (protocol==0){
          display1.print("B, ");
          display1.print(channel);
        }else if(protocol==1){
          display1.print("BG, ");
          display1.print(channel);
        }else if(protocol==2){
          display1.print("BGN, ");
          display1.print(channel);
        }else if(protocol==3){
          display1.print("LR, ");
          display1.print(channel);
        }  
      }else{
        display1.setCursor(0,50);
        display1.print("Time: ");
        if(millis()<signaltimer+threshold){
          display1.print(double((millis()-totaltimeout)/60000),1);
        }else{
          display1.print(double((timeout0-totaltimeout)/60000),1);
        }
        display1.print(" min");
      }       
    }else if (protocol_bitmap & WIFI_PROTOCOL_11G) {
      display1.print("802.11BG");
      if(protocol!=1||WiFi.channel()!=channel){
        if(protocol!=1){
          display1.print("*");
        }
        display1.setCursor(0,50);
        display1.print("Restart: ");
        if (protocol==0){
          display1.print("B, ");
          display1.print(channel);
        }else if(protocol==1){
          display1.print("BG, ");
          display1.print(channel);
        }else if(protocol==2){
          display1.print("BGN, ");
          display1.print(channel);
        }else if(protocol==3){
          display1.print("LR, ");
          display1.print(channel);
        }         
      }else{
        display1.setCursor(0,50);
        display1.print("Time: ");
        if(millis()<signaltimer+threshold){
          display1.print(double((millis()-totaltimeout)/60000),1);
        }else{
          display1.print(double((timeout0-totaltimeout)/60000),1);
        }
        display1.print(" min");
      }       
    }else if (protocol_bitmap & WIFI_PROTOCOL_11B) {
      display1.print("802.11B");
      if(protocol!=0||WiFi.channel()!=channel){
        if(protocol!=0){
          display1.print("*");
        }
        display1.setCursor(0,50);
        display1.print("Restart: ");
        if (protocol==0){
          display1.print("B, ");
          display1.print(channel);
        }else if(protocol==1){
          display1.print("BG, ");
          display1.print(channel);
        }else if(protocol==2){
          display1.print("BGN, ");
          display1.print(channel);
        }else if(protocol==3){
          display1.print("LR, ");
          display1.print(channel);
        }        
      }else{
        display1.setCursor(0,50);
        display1.print("Time: ");
        if(millis()<signaltimer+threshold){
          display1.print(double((millis()-totaltimeout)/60000),1);
        }else{
          display1.print(double((timeout0-totaltimeout)/60000),1);
        }
        display1.print(" min");
      } 
    }
    display1.setCursor(0,40);
    display1.print("Channel: ");
    display1.print(WiFi.channel()); 
    if(WiFi.channel()!=channel){
      display1.print("*");
    }
  }

  display1.display();
}

int readAnalog(int InputChannel){ //function for reading analog input values and capping their upper and lower values
  int val = 0;
  val = adc.readADC_SingleEnded(InputChannel);
  if (val>17670){ //the ADS1115 in default mode measures 0.0001875 V per count -> 17670 equals 3.31 V; there shouldn't be any value higher than that -> this threshold is used to cap the input
    val=17670;
  }else if(val<0){ //cap at 0 to prevent issues caused by negative reads (noise)
    val=0;
  }
  return val;
}
