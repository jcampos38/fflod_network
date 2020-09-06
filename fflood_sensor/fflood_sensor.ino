/**NRF24L01 LIBRARIES**/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/**ULTRASONIC SENSOR LIBRARIES**/
#include <Ultrasonic.h>

/**PIN DEFINITION**/
#define NRF24_CE 7
#define NRF24_CSN 8
#define HCSC04_TRIGGER 2
#define HCSC04_ECHO 3
#define NODE_NUMBER 2     // 1-6
#define INDICATOR 4

RF24 radio(NRF24_CE, NRF24_CSN); 
Ultrasonic ultrasonic(HCSC04_TRIGGER, HCSC04_ECHO);

const int DATA_PERIOD = 1000; // In miliseconds
const uint64_t ADDESSES[] = {0x7878787878LL, 0xB3B4B5B6F1LL, 0xB3B4B5B6CDLL, 0xB3B4B5B6A3LL, 0xB3B4B5B60FLL, 0xB3B4B5B605LL};
const uint64_t TX_ADDRESS = ADDESSES[ NODE_NUMBER - 1 ];   // Select the address from the array
const int RAIL_LENGTH = 37;
int water_level = 0;

void setup() {
  Serial.begin(115200);
  pinMode(INDICATOR, OUTPUT);
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(108);
  radio.openReadingPipe(0,TX_ADDRESS);
  radio.openWritingPipe(TX_ADDRESS);
  radio.stopListening();
}

void loop() {
  digitalWrite(INDICATOR, LOW);
  water_level = RAIL_LENGTH - ultrasonic.read();
  Serial.println(water_level);
  
  if (!radio.write( &water_level, sizeof(int) )){
     Serial.println(F("Send try failed"));
     digitalWrite(INDICATOR, HIGH);
  }
        
  radio.startListening();
  
  unsigned long started_waiting_at = micros();
  boolean timeout = false;
  while ( !radio.available() ){
    if (micros() - started_waiting_at > 200000 ){ // 200ms timeout
        timeout = true;
        break;
    }      
  }
      
  if ( timeout ){
      Serial.println(F("Failed, response timed out."));
      //digitalWrite(INDICATOR, HIGH);
  }else{
      digitalWrite(INDICATOR, LOW);
      int got_wlevel;
      radio.read( &got_wlevel, sizeof(int) );
      Serial.print(F("Got response "));
      Serial.print(got_wlevel);
      Serial.print(F("cm"));
  }
  
  radio.stopListening();
  delay(DATA_PERIOD); 
}
