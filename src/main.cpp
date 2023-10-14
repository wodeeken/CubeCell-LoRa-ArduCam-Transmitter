#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "Constants.cpp"
// Lora definitions.
#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             14        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 128 // Max payload size.

int16_t rssi,rxSize;
int CameraPingStart;
bool RxTimedout = false;
bool lora_idle = true;
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
//char* rxpacket_trunc;
static RadioEvents_t RadioEvents;
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnTxDone(void);
void OnTxTimeout();
void OnRxTimeout();
Constants::TransmitterState CurrentTransmitterState = Constants::Wait;

void setup() {
    // Setup serial.
    Serial.begin(38400);
    Serial.flush();
    // Setup Radio.
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
}



void loop()
{
  delay(200);
  Serial.println("Looping.");
  switch(CurrentTransmitterState){
    
    case Constants::Wait:
      // If waiting, watch for input from radio.
      if(lora_idle){
        lora_idle = false;
        Serial.println("Entering RX Mode.");
      
        Radio.Rx(Constants::RecTimeout);
        CurrentTransmitterState = Constants::RxMode;
        delay(1000);
        //delay(5000);
      }
      break;
    case Constants::RxMode:
      if(lora_idle){
          Serial.print("Just got out of the RX mode. what did I read: ");
          Serial.println(rxpacket);
          // Convert to string for ease of comparison.
          String str = String(rxpacket);
          str.trim();
          //Serial.println("1:" + str + "|");
          //Serial.println("1:" + String(Constants::FromRec_Ping_Camera_Command) + "|");
          if(str.compareTo(String(Constants::FromRec_Ping_Camera_Command)) == 0){
            Serial.println("Just got a ping request.");
            CurrentTransmitterState = Constants::CameraPingRequest;
          }else{
            Serial.println("Unrecognized Request.");
            CurrentTransmitterState = Constants::Wait;
          }
          memset(rxpacket,0,BUFFER_SIZE);
        }else{Serial.println("Still busy in RxMode.");}
    break;
    case Constants::CameraPingRequest:
      Serial.println("We inside the ping request..");
      // Send a ping back, and go back to waiting.
      if(lora_idle){
      
        lora_idle = false;
        sprintf(txpacket, Constants::ToRec_Ping_Camera_Response);
        Serial.print("About to send a packet!");
        Serial.println(txpacket);
        Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
        CurrentTransmitterState = Constants::Wait;
      }else
        // Stay in state until lora isn't busy.
      
      break;
  }
  
}
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
  {
      
      rssi=rssi;
      rxSize=size;
      memcpy(rxpacket, payload, size );
      //memcpy(rxpacket_trunc,payload,size );
      rxpacket[size]='\0';
      turnOnRGB(COLOR_RECEIVED,0);
      Radio.Sleep( );
      Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n",rxpacket,rssi,rxSize);
      lora_idle = true;
  }
  void OnTxDone( void )
  {
    turnOffRGB();
    //Serial.println("TX done......");
    lora_idle = true;
  }

  void OnTxTimeout( void )
  {
    turnOffRGB();
    Radio.Sleep( );
    //Serial.println("TX Timeout......");
    lora_idle = true;
  }
  void OnRxTimeout( void )
  {
    RxTimedout = true;
    turnOffRGB();
    Radio.Sleep( );
    Serial.println("Timedout");
    lora_idle = true;
  }