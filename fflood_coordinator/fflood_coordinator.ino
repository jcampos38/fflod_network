/** SIM80L LIBRARIES **/
#include <SoftwareSerial.h>
#include "SIM800L.h"

/**NRF24L01 LIBRARIES**/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/** OLED LIBRARIES **/
#include <Wire.h>
#include <Adafruit_SSD1306.h>

/** ULTRASONIC SENSOR LIBRARIES **/
#include <Ultrasonic.h>

/** PIN DEFINITION **/
#define SIM800_TX_PIN 10
#define SIM800_RX_PIN 9
#define SIM800_RST_PIN 6
#define NRF24_CE 48
#define NRF24_CSN 49
#define HCSC04_TRIGGER 2
#define HCSC04_ECHO    3
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

/** HTTP Service Info **/
const char APN[] = "internet.itelcel.com";
const char URL[] = "http://10379717ccb9.ngrok.io/api/medicion";
const char CONTENT_TYPE[] = "application/json";
const int REQUEST_PERIOD = 60; // In seconds
const int RETRIES = 1;

/** SENSORS INFO **/
const int SENSORS_ID[] = {1, 2, 3};
const int RAIL_LENGTH = 37;

/** NRF24 NETWORK **/
const uint64_t ADDESSES[] = {0x7878787878LL, 0xB3B4B5B6F1LL, 0xB3B4B5B6CDLL, 0xB3B4B5B6A3LL, 0xB3B4B5B60FLL, 0xB3B4B5B605LL};

/** GLOBAL VARIABLES **/
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Ultrasonic ultrasonic(HCSC04_TRIGGER, HCSC04_TRIGGER);
RF24 radio(NRF24_CE, NRF24_CSN); 
SIM800L* sim800l;
int measures[] = {0, 0, 0};

void setup() {
  Serial.begin(115200);
  while(!Serial);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Initialize oled screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println(F("Setting up..."));
  display.display();

  // Initialize a SoftwareSerial
  SoftwareSerial* serial = new SoftwareSerial(SIM800_TX_PIN, SIM800_RX_PIN);
  serial->begin(9600);
  delay(1000);
   
  // Initialize SIM800L driver with an internal buffer of 200 bytes and a reception buffer of 512 bytes, debug disabled
  sim800l = new SIM800L((Stream *)serial, SIM800_RST_PIN, 200, 512);
  
  // Setup module for GPRS communication
  setupSIM();
  
  // Setup module for RF communication
  setupNRF();

  delay(1000);
}

void loop() {
  cleanScreen();
  
  unsigned long start_at = millis();
  int elapsed = 0;
  int last = 0;
  
  printTime(elapsed);
  radio.startListening();
  
  while( elapsed < REQUEST_PERIOD ) {
    measures[0] = RAIL_LENGTH - ultrasonic.read();

    if(last != elapsed) {
      printTime(elapsed);
    }
    last = elapsed;
    
    byte pipeNum = 0;
    int got_wlevel = 0;
    while(radio.available(&pipeNum)){
     radio.read( &got_wlevel, sizeof(int) );
     Serial.print("Received water level: "); 
     Serial.print(got_wlevel);
     Serial.print(" From: ");
     Serial.println(pipeNum + 1);
     measures[pipeNum+1] = got_wlevel;
    }
    delay(100);
    elapsed = round((millis()-start_at)/1000);
  }

  radio.stopListening();
  bool request_success = sendData();
  for(uint8_t i = 0; i < RETRIES && !request_success; i++) {
    delay(1000);
    request_success = sendData();
  }
}

void setupSIM() {
  while(!sim800l->isReady()) {
    Serial.println(F("Problem to initialize AT command, retry in 1 sec"));
    delay(1000);
  }

  checkLimit();
  display.println(F("SIM800L Setup Complete!"));
  display.display(); 
  
  // Wait for the GSM signal
  uint8_t signal = sim800l->getSignal();
  while(signal <= 0) {
    delay(1000);
    signal = sim800l->getSignal();
  }

  checkLimit();
  display.print(F("Signal OK (strenght: "));
  display.print(signal);
  display.println(")");
  display.display();
  delay(1000);

  NetworkRegistration network = sim800l->getRegistrationStatus();
  while(network != REGISTERED_HOME && network != REGISTERED_ROAMING) {
    delay(1000);
    network = sim800l->getRegistrationStatus();
  }

  checkLimit();
  display.println(F("Network registration OK"));
  display.display();

  // Setup APN for GPRS configuration
  bool success = sim800l->setupGPRS(APN);
  while(!success) {
    success = sim800l->setupGPRS(APN);
    delay(5000);
  }

  checkLimit();
  display.println(F("GPRS config OK"));
  display.display();
  delay(100);
}

void setupNRF() {
  radio.begin();  //Start the nRF24 module
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(108);

  // Open up to six pipes for PRX to receive data
  radio.openReadingPipe(0,ADDESSES[0]);
  radio.openReadingPipe(1,ADDESSES[1]);
  radio.openReadingPipe(2,ADDESSES[2]);
  radio.openReadingPipe(3,ADDESSES[3]);
  radio.openReadingPipe(4,ADDESSES[4]);
  radio.openReadingPipe(5,ADDESSES[5]);
  
  //radio.startListening();
  checkLimit();
  display.println(F("NRF24 Setup completed!"));
  display.display();
  delay(100);
}

bool sendData() {
  bool success = false;
  cleanScreen();
  display.println(F("Start sending data..."));
  display.display();

  // Establish GPRS connectivity (5 trials)
  bool connected = false;
  for(uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    connected = sim800l->connectGPRS();
  }

  // Check if connected, if not reset the module and setup the config again
  if(connected) {
    checkLimit();
    display.println(F("GPRS connected !"));
    display.display();
  } else {
    checkLimit();
    display.println(F("GPRS not connected !"));
    display.println(F("Reset the module."));
    display.display();
    sim800l->reset();
    setupSIM();
    return false;
  }

  checkLimit();
  display.println(F("Start HTTP POST..."));
  display.display();
  // Do HTTP POST communication with 10s for the timeout (read and write)
  String b = generateBody();
  char body[b.length()+1];
  b.toCharArray(body, b.length()+1); 
  Serial.println(body);
  uint16_t rc = sim800l->doPost(URL, CONTENT_TYPE, body, 10000, 10000);
  if(rc == 200) {
    success = true;
    // Success, output the data received on the serial
    Serial.print(F("HTTP POST successful ("));
    display.clearDisplay();
    display.print(F("HTTP POST successful ("));
    display.print(sim800l->getDataSizeReceived());
    display.println(F(" bytes)"));
    display.display();
    checkLimit();
  } else {
    // Failed...
    Serial.print(F("HTTP POST error "));
    Serial.println(rc);
    display.clearDisplay();
    display.print(F("HTTP POST error "));
    display.println(rc);
  }

  delay(2000);

  // Close GPRS connectivity (5 trials)
  bool disconnected = sim800l->disconnectGPRS();
  for(uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    disconnected = sim800l->disconnectGPRS();
  }
  
  delay(1000);
  if(disconnected) {
    checkLimit();
    display.println(F("GPRS disconnected !"));
  } else {
    checkLimit();
    display.println(F("GPRS still connected !"));
  }

  display.display();

  // Go into low power mode
  bool lowPowerMode = sim800l->setPowerMode(MINIMUM);
  //for(uint8_t i = 0; i < 5 && !connected; i++) {
  //  delay(500);
  //  lowPowerMode = sim800l->setPowerMode(MINIMUM);
  //}
  if(lowPowerMode) {
    display.println(F("Module in low power mode"));
  } else {
    display.println(F("Failed to switch module to low power mode"));
  }
  display.display();

  return success;
}

String generateBody() {
  String body = String("[");
  for(int i=0;i<3;i++) {
    if(i > 0)
      body += ",";
    body += "{\"id_sensor\":";
    body += SENSORS_ID[i];
    body += ",\"nivel_agua\":";
    body += measures[i];
    body += "}";
  }
  body += "]";
  return body;
}

void printTime(int elapsed) {
  cleanScreen();
  display.setCursor(0,0);
  display.print(F("Time: "));
  display.print(elapsed);
  display.println(F(" seconds"));
  printMeasure(0);
  printMeasure(1);
  printMeasure(2);
  display.display();
}

void printMeasure(int sensor) {
  display.setCursor(0,(8*sensor) + 8);
  display.print(F("Sensor "));
  display.print(sensor+1);
  display.print(F(": "));
  display.print(measures[sensor]);
  display.print(F("cm"));
}

void cleanScreen() {
  display.clearDisplay();
  display.setCursor(0,0);
}

void checkLimit() {
  if(display.getCursorY() >= 64){
    cleanScreen();
  }
}
