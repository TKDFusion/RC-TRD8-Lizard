//Required libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <esp_now.h>
#include <WiFi.h>
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
double batvolt = 0;
double voltage = 0;
double current = 0;
int potival = 0;
int mainspeed = 0;
int mainspeed1 = 0;
int mainspeed2 = 0;
int steering = 0;
int btnlrdebounce0 = 0;
int btnlrdebounce1 = 0;
int btnlrv = 1;
int btnurdebounce0 = 0;
int btnurdebounce1 = 0;
int btnurv = 1;
int btnlldebounce0 = 0;
int btnlldebounce1 = 0;
int btnllv = 1;
int btnuldebounce0 = 0;
int btnuldebounce1 = 0;
int btnulv = 1;
int lcdtimer = 0;
int rctimer = 0;
int analogtimer = 0;
int lighton = 0;
int adj = 0;
int counter = 0;
int menu = 0;
int menuval = 0;
int calval = 0;
int joylcen = 0;
int joyll = 0;
int joylr = 0;
int joyltol = 0;
int joyrcen = 0;
int joyru = 0;
int joyrd = 0;
int joyrtol = 0;
int joyrinv = 0;
int flightint = 0;
int rlightint = 0;
int leftlimit = 0;
int rightlimit = 0;
int batlow = 0;
int lcdcounter = 0;
int control = 0;

//Add I2C peripherals
//ADS1115
Adafruit_ADS1115 adc; //ADC
int joyr = 0; //these variables indicate the input channel population of the ADC (right and left joystick, poti)
int joyl = 1;
int poti = 2;
int batt = 3;

//2x SSD1306 OLED display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D //OLED display 0 (left display)
#define SCREEN_ADDRESS1 0x3C //OLED display 1 (right display)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int diamond = 0x04;

/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

// REPLACE WITH THE MAC Address of your receiver 
//uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t FirstMAC[] = {0xBE, 0xA0, 0xBE, 0xEF, 0xFE, 0xED}; //we will set this MAC address for the first µC
uint8_t SecondMAC[] = {0xB0, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};//MAC address of the 2nd µC
int signalquality = 0;
// Variable to store if sending data was successful
// Define a data structure for sending data to the remote control
typedef struct send_message {
  int mainspeed;
  int steering;
  int frontlight;
  int rearlight;
  int fan;
  int control;
} send_message;

// Create a struct_message called BME280Readings to hold sensor readings
send_message outgoing;

// Define a data structure for receiving information from the remote control
typedef struct receive_message {
  int voltage;
  int current;
  int temp1;
  int temp2;
  int rpm;
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

//preferences library allows you to store variable values within the flash so that their value can be retrieved on startup. Used to store user-adjustable settings.
Preferences preferences;

//Interrupt functions for reading buttons
void IRAM_ATTR btnlrint(){
  btnlrdebounce1 = millis();
  if (btnlrdebounce1>btnlrdebounce0+250){ //debouncing: the button only reacts to new inputs if the last input was at least 0.5 s ago
    btnlrv = 0;    
  }
  btnlrdebounce0 = btnlrdebounce1;
}

void IRAM_ATTR btnurint(){
  btnurdebounce1 = millis();
  if (btnurdebounce1>btnurdebounce0+250){ //debouncing: the button only reacts to new inputs if the last input was at least 0.5 s ago
    btnurv = 0;    
  }
  btnurdebounce0 = btnurdebounce1;
}

void IRAM_ATTR btnllint(){
  btnlldebounce1 = millis();
  if (btnlldebounce1>btnlldebounce0+250){ //debouncing: the button only reacts to new inputs if the last input was at least 0.5 s ago
    btnllv = 0;    
  }
  btnlldebounce0 = btnlldebounce1;
}

void IRAM_ATTR btnulint(){
  btnuldebounce1 = millis();
  if (btnuldebounce1>btnuldebounce0+250){ //debouncing: the button only reacts to new inputs if the last input was at least 0.5 s ago
    btnulv = 0;    
  }
  btnuldebounce0 = btnuldebounce1;
}

void setup() {
  Serial.begin(115200); //Not necessary, but kept in the code in case you want to use the serial monitor for debugging
  //preferences library allows you to store variable values within the flash so that their value can be retrieved on startup. Used to store user-adjustable settings.
  preferences.begin("Defaults", false);
  joyltol = preferences.getInt("joyLtol",20);
  joyll = preferences.getInt("joyLleft",0);
  joylr = preferences.getInt("joyLright",0);
  joyrtol = preferences.getInt("joyRtol",20);
  joyru = preferences.getInt("joyRup",0);
  joyrd = preferences.getInt("joyRdown",0);
  joyrinv = preferences.getInt("joyRInv",0);
  flightint = preferences.getInt("headlights",150);
  rlightint = preferences.getInt("brakelights",150);
  leftlimit = preferences.getInt("LeftLimit",400);
  rightlimit = preferences.getInt("RightLimit",350);
  control = preferences.getInt("Control",0);

  outgoing.control=control;

  lighton = preferences.getInt("lighton",0);
  if (lighton>0){
    outgoing.frontlight=flightint;
    outgoing.rearlight=rlightint;
  }

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
  adc.begin(0x48);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display1.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS1);
  display.display();
  display1.display();
  display.clearDisplay();
  display1.clearDisplay();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, SecondMAC);
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
  memcpy(peerInfo.peer_addr, FirstMAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // while(!adc.begin(0x48)){
  //   Serial.println("ADS1115 activation failed");
  // } //default I2C address of ADS1115 (ADDR pin unconnected/at GND)

  // // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  // if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
  //   Serial.println(F("SSD1306 allocation failed"));
  //   for(;;); // Don't proceed, loop forever
  // }

  // if(!display1.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS1)) {
  //   Serial.println(F("SSD1306 allocation failed"));
  //   for(;;); // Don't proceed, loop forever
  // }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  // display.display();
  // display1.display();
  // delay(500); // Pause for 2 seconds

  // // Clear the buffer
  // display.clearDisplay();
  // display1.clearDisplay();

  lcdtimer = millis();
  rctimer  = lcdtimer;

}
 
void loop() {
  if (menu==0){ //the remote control is using multiple menu pages for its function. 0 -> normal operation
    analogInputs(); //read all analog inputs
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
    outgoing.steering = steering;  
  }else if (menu==4){ //menu 4 -> adjustment of LED lights
    setLight();   
  }else if (menu==5){ //menu 5 -> adjustment of cooling fah
    //setFan();
  }else if (menu==6){ //menu 6 -> modify engine control logic
    //setControl();
  }else if (menu==7){ //menu 7 -> empty

  }else if (menu==8){ //menu 8 -> empty

  }

  if (menu>0&&menu!=3){ //make sure that speed and steering are zero when being in one of the submenus
    outgoing.mainspeed = 0;
    outgoing.steering = 0;
  }

  if (millis()>rctimer+25){ //the RC sends its inputs to the car every 25 ms (20x per second - sufficient for a responsive control)
    if (batlow>10){ //if the low battery voltage threshold of the car is reached, the LED light blinks to warn the user
      //SOS();
    }
    esp_err_t result = esp_now_send(FirstMAC, (uint8_t *) &outgoing, sizeof(outgoing));
    rctimer = millis();   
  }

  if (millis()>analogtimer+67){ //not all inputs need to be measured every time. This snippet is executed at a lower frequency
    counter = counter + 1;
    if (counter>2){
      potival = readAnalog(poti); //the potentiometer that defines the maximum speed of the car
      if (potival<20){
        potival=20;
      }else if(potival>17580){ //17580 counts *0.0001875 V/count = 3.296 V -> low enough to ensure max speed can be reached by the poti
        potival=17580;
      }
      potival = map(potival,20,17580,255,51); //speed selection: between 20% and 100% of full PWM range
      counter = 0;
    }else if (counter>1){
      batvolt = readAnalog(batt); //battery voltage of the remote control
      batvolt = double(batvolt)*0.0001875*(47+22)/22; //voltage calculation: 1 count equals 0.1875 mV with default settings of ADS1115; voltage divider with 47 and 22 kOhm
    }else{
      preferences.putInt("lighton",lighton);
    }
    voltage = double(incoming.voltage)*0.0001875*(100+33)/33; //voltage value of the car's battery: analog value is computed based on the 100 and 33 kOhm voltage divider
    current = double(incoming.current)*0.0001875*(22+33)/33; //current draw of the car: Analog value of the ACS712 sensor, corrected for its voltage divider with 22 and 33 kOhm
    current = (current-2.5)/.066; //conversion of analog input voltage into current: ACS712 outputs 2.5 V @ 0 A and the signal scales with 66 mV/A (30 A version) [(current-2.5)/.066 for negative flow]
    //current = (2.5-current)/.066; //conversion of analog input voltage into current: ACS712 outputs 2.5 V @ 0 A and the signal scales with 66 mV/A (30 A version) [(2.5-current)/.066 for positive flow]
    if (signalquality>10){
      if (voltage<11.1&&current<2||voltage<10.8){ //thresholds for low battery: less than 11.1V@idle (below 2A; 3.7V/cell) or less than 10.8V (3.6V/cell) under load
        if (batlow<20){ //these limits include a safety margin that helps to extend the life of the battery while not making using of its full capacity
          batlow=batlow+1; //reducing these values is done at your own risk! Never discharge the battery below 9.9 V (3.3 V/cell, also not under load!) or charge it to more than 12.6 V (4.2 V/cell)!
        }
      }else if (voltage>=11.25&&batlow>0){ //account for lower readings during startup/shutdown: >=11.25V (3.75V/cell) resets the variable
        batlow=batlow-1;
      }
    }
    analogtimer = millis();
  }

  if (millis()>lcdtimer+250){ //the lcd screen is updated 4 times per second (this low refresh rate looks better than refreshing the display faster because of the response time of the pixels)
    if (menu==0){ //the menu variable determines the "program" for the LCD display.
      updateDisplay(); //standard mode: Show status of the car
      updateDisplay1();
    }else if(menu==1){
      menuLCD(); //selection menu: Show the different menu entries and the cursor
    }else if(menu==2){
      joyCalLCD(); //LCD screen for joystick calibration
    }else if (menu==3){
      steerLCD(); //LCD screen for steering calibration
    }else if (menu==4){
      lightLCD(); //LCD screen for adjusting headlight LED brightness
    }else if (menu==5){
      //fanLCD(); //LCD screen for adjusting headlight LED brightness
    }else if (menu==6){
      //controlLCD(); //LCD screen for diff lock calibration
    }else if(menu==7){

    }else if(menu==8){

    }
    lcdtimer = millis();
    //serialOutput(); //this function can display information in the serial monitor for debugging. Shall be disabled for normal operation as it slows down the code to do so.
  }

  // if (!digitalRead(BTN_LR)){
  //   digitalWrite(LED_LR, HIGH);
  // }else{
  //   digitalWrite(LED_LR, LOW);
  // }

  // if (!digitalRead(BTN_LL)){
  //   digitalWrite(LED_LL, HIGH);
  // }else{
  //   digitalWrite(LED_LL, LOW);
  // }

  // if (!digitalRead(BTN_UR)){
  //   digitalWrite(LED_UR, HIGH);
  // }else{
  //   digitalWrite(LED_UR, LOW);
  // }  

  // if (!digitalRead(BTN_UL)){
  //   digitalWrite(LED_UL, HIGH);
  // }else{
  //   digitalWrite(LED_UL, LOW);
  // }  

  //updateDisplay();
  //updateDisplay1();

}

void digitalInputs(){ //check digital inputs (switches & buttons) as control inputs for the car
  if (btnurv==0){ //switching the headlights on and off
    btnurv = 1;
    if (lighton==0){
      lighton = 1;
      preferences.putInt("lighton",1);
    }else{
      lighton = 0;
      preferences.putInt("lighton",0);
    }
  }
  if (lighton==1&&batlow<11){
    outgoing.frontlight = flightint; //simple transmission of an 8bit duty cycle (0-255) for the headlight intensity
    outgoing.rearlight = rlightint;
    digitalWrite(LED_UR, HIGH);
  }else{
    outgoing.frontlight = 0;
    outgoing.rearlight = 0;
    digitalWrite(LED_UR, LOW);
  }  

  if (btnulv==0){
    btnulv = 1;
    if (control<2){
      control=control+1;
    }else{
      control=0;
    }
    outgoing.control=control;
    preferences.putInt("Control",control);
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
    //if (outgoing.gear==1){
    //  steering=map(steering,joyltol,255,1,153); //values are re-mapped to make use of the full available range. Fast gear: limited steering range to assist with stable high-speed driving
    //}else{
      steering=map(steering,joyltol,255,1,255); //values are re-mapped to make use of the full available range
    //}

  }else if (steering<=-joyltol){
    //if (outgoing.gear==1){
    //  steering=map(steering,-joyltol,-255,-1,-153);
    //}else{
      steering=map(steering,-joyltol,-255,-1,-255);  
    //}

  }

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

  if (joyrinv==0){ //adjust values in case of inverted joystick mode
    mainspeed = map(mainspeed,joyrd,joyru,-potival,potival); //re-map the values at the limits specified by the speed joystick to limit maximum speed
    if (mainspeed<-64){ //limit max speed in reverse: 25% PWM duty cycle
      mainspeed = -64;
    }    
  }else{
    mainspeed = map(mainspeed,joyrd,joyru,potival,-potival); //re-map the values at the limits specified by the speed joystick to limit maximum speed
    if (mainspeed>64){ //limit max speed in reverse: 25% PWM duty cycle
      mainspeed = 64;
    }
  }

  if (mainspeed<(double(joyrtol)/255)*potival&&mainspeed>-(double(joyrtol)/255)*potival){ //keep the center of the joystick numb
    mainspeed = 0;
  }

  if (batlow<11){
    steering = map(steering,-255,255,leftlimit,rightlimit); //map the steering values to the limits specified during calibration of the steering servo
  }else{
    steering = map(0,-255,255,leftlimit,rightlimit);
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
    }else if (calval==7){ //change between normal and inverted joystick mode
      digitalWrite(LED_UL, HIGH);
      if (joyrinv==0){
        joyrinv=1;
      }else{
        joyrinv=0;
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
    }else if (calval==7){
      digitalWrite(LED_LL, HIGH);
      if (joyrinv==0){
        joyrinv=1;
      }else{
        joyrinv=0;
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
    }else if (calval==8){ //step seven: specify if joyR input shall be inverted
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("Input confirmed: ");
      display.setCursor(0,10);
      display.print("Joystick ");
      if (joyrinv==0){
        display.print("default");
      }else{
        display.print("inverted");
      }
      preferences.putInt("joyRInv",joyrinv);
      display.display();
      delay(1000);
    }else if (calval==9){ //step eight: confirmation of calibration
      menu=1;
      calval=0;
      preferences.putInt("JoyCal",1);
      joyltol = preferences.getInt("joyLtol",20);
      joyll = preferences.getInt("joyLleft",0);
      joylr = preferences.getInt("joyLright",0);
      joyrtol = preferences.getInt("joyRtol",20);
      joyru = preferences.getInt("joyRup",0);
      joyrd = preferences.getInt("joyRdown",0);
      joyrinv = preferences.getInt("joyRInv",0);
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

void joyCalLCD(){ //LCD screen for joystick calibration
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
    display.print("JoyR mode: ");
    if (joyrinv==0){
      display.print("default ");
    }else{
      display.print("inverted");
    }
    display.setCursor(0,10);
    display.print("Adj: UL/LL button");
    display.setCursor(0,20);
    display.print("Confirm: UR button");   
    display.setCursor(0,30);
    display.print("Return : LR button");     
  }else if (calval==8){
    display.setCursor(0,0); 
    display.print("Calibration done!");    
    display.setCursor(0,20);
    display.print("Confirm: UR button");  
    display.setCursor(0,30);
    display.print("Return: LR button");       
  }
  display.display();
}

void menuInputs(){ //function for selection menu
  if (btnulv==0){ //upper left button (4): move cursor up by one entry
    digitalWrite(LED_UL, HIGH);
    if (menuval>0){
      menuval = menuval-1;
    }else{
      menuval = 7;
    }
    btnulv=1;
  }
  if (btnllv==0){ //lower left button (1): move cursor down by one entry
    digitalWrite(LED_LL, HIGH);
    if (menuval<7){
      menuval = menuval+1;
    }else{
      menuval = 0;
    }
    btnllv=1;
  }
  if (btnlrv==0){ //return from menu to the selected menu entry by pressing the lwoer right button (2)
    btnllv = 1; //in case the buttons were pressed: return to default state
    btnlrv = 1;
    btnurv = 1;
    btnulv = 1;
    if(menuval==0){ //the cursor of the selection menu has a different number than the menu entries
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
      menu=0;
    }
    digitalWrite(LED_LR, LOW);
    display.clearDisplay();
    display.setTextSize(1);
  }   
}

void menuLCD(){ //LCD screen for the selection menu
  display.setCursor(8,0);
  display.print("Joysticks");
  display.setCursor(8,10);
  display.print("Steering ");
  display.setCursor(8,20);
  display.print("Lights   ");
  display.setCursor(8,30);
  display.print("n.a.     ");
  display.setCursor(75,0);
  display.print("n.a.     ");  
  display.setCursor(75,10);
  display.print("n.a."); 
  display.setCursor(75,20);
  display.print("n.a."); 
  display.setCursor(75,30);
  display.print("Back     "); 
  for (int i=0; i<=3; i++){ //show the current position of the cursor
    display.setCursor(0,i*10);
    if (menuval==i){
      display.write(diamond);
    }else{
      display.print(" ");
    }
  }
  for (int i=0; i<=3; i++){
    display.setCursor(67,i*10);
    if (menuval-4==i){
      display.write(diamond);
    }else{
      display.print(" ");
    }
  }
  display.display();
  digitalWrite(LED_UL, LOW);
  digitalWrite(LED_UR, LOW);
  digitalWrite(LED_LL, LOW);
}

void steerCal(){ //calibration of the steering servo
  if (calval==1){ //the remote control transmits fixed values for the steering servo depending on the step of the calibration
    steering=leftlimit; //step one: left limit of the steering servo
  }else if (calval==2){
    steering=rightlimit; //step two: right limit of the steering servo
  }else{
    steering=(leftlimit+rightlimit)/2; //step three: center position of the steering servo
  }
  if (btnulv==0){ //button 4 (upper left button) increases the value of the limit that is currently adjusted
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
      if (leftlimit<600&&rightlimit<600){
        leftlimit=leftlimit+1;
        rightlimit=rightlimit+1;
      }
    }
    btnulv=1;
  }
  if (btnllv==0){ //button 1 (lower left button) decreases the value of the limit that is currently adjusted
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
  if (btnurv==0){ //button 3 (upper right button): proceed to next step (save current value as the limit that is adjusted in this step)
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
  if (btnlrv==0){ //button 2 (lower right button): go one step back in the calibration
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

void steerLCD(){ //the LCD screen of the steering servo calibration is equal to the calibration screens of the actuators
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

void setLight(){ //adjustment of the headlight LED brightness
  if (btnulv==0){ //button 4 (upper left button) increases the value of the limit that is currently adjusted
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
  if (btnllv==0){ //button 1 (lower left button) decreases the value of the limit that is currently adjusted
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

  if (btnurv==0){ //button 3 (upper right button): proceed to next step (save current value as the limit that is adjusted in this step)
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

  if (btnlrv==0){ //button 2 (lower right button): go one step back in the calibration
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

void lightLCD(){ //LCD screen for headlight LED adjustment
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

void updateDisplay(){
  // Display Readings on OLED Display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE,BLACK);
  display.setCursor(0, 0);
  display.print("Voltage: ");
  display.print(voltage);
  display.setCursor(0, 10);
  display.print("Current: ");
  if (current<0){
    current=0;
  }
  display.print(current,1);
  display.print(" A  ");
  display.setCursor(0, 20);
  display.print("Power: ");
  display.print(voltage*current,1);
  display.print(" W   ");
  display.setCursor(0, 30);
  display.print("Motor: ");
  display.print(incoming.rpm);
  display.print(" revs/min    ");
  display.setCursor(0,40);
  display.print("Speed: ");
  display.print(((double(incoming.rpm)*60/6)*10/100000)*3.14, 1);
  display.print(" km/h   ");
  display.display();
  
}

void updateDisplay1(){
  // Display Readings on OLED Display
  display1.clearDisplay();
  display1.setTextSize(1);
  display1.setTextColor(WHITE,BLACK);
  display1.setCursor(0, 0);
  display1.print("Battery voltage: ");
  display1.print(batvolt);
  display1.setCursor(0, 10);
  display1.print("JoyR: ");
  display1.print(mainspeed);
  display1.setCursor(0, 20);
  display1.print("JoyL: ");
  display1.print(steering);
  display1.setCursor(0,30);
  display1.print("Poti: ");
  display1.print(potival);
  display1.setCursor(0,40);
  display1.print("Engine: ");
  if (outgoing.control==0){
    display1.print("smooth");
  }else if(outgoing.control==1){
    display1.print("fast  ");
  }else{
    display1.print("direct");
  }
  display1.display();
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