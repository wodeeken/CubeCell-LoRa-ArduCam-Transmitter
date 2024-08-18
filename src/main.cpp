#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "Constants.cpp"
#include <Regexp.h>
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
int DataRequestPacket = 0;
//char* rxpacket_trunc;
static RadioEvents_t RadioEvents;
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnTxDone(void);
void OnTxTimeout();
void OnRxTimeout();
Constants::TransmitterState CurrentTransmitterState = Constants::Wait;

// MOCK Camera data.
char* MockCameraData = "1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200";
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
          
          
          // Use regex for data transfer request.
          MatchState ms;
          ms.Target(rxpacket);

          char drResult = ms.Match(Constants::FromRec_Data_Transfer_Command);
          Serial.println("result from match: ");
          Serial.println(drResult );
          //Serial.println("1:" + str + "|");
          //Serial.println("1:" + String(Constants::FromRec_Ping_Camera_Command) + "|");
          if(str == ""){
            CurrentTransmitterState = Constants::Wait;
          }else if(str.compareTo(String(Constants::FromRec_Ping_Camera_Command)) == 0){
            Serial.println("Just got a ping request.");
            CurrentTransmitterState = Constants::CameraPingRequest;
          }else if(str.compareTo(String(Constants::FromRec_Camera_Image_Request_Command)) == 0){
            Serial.println("Just got a camera request.");
            CurrentTransmitterState = Constants::CameraCaptureRequest;
          }else if(drResult == REGEXP_MATCHED){
            Serial.println("Just got a data request.");
            char packetNumResult = ms.Match("%d+");
            if(packetNumResult == REGEXP_MATCHED){
              DataRequestPacket = str.substring(ms.MatchStart, ms.MatchStart + ms.MatchLength).toInt();
              CurrentTransmitterState = Constants::DataTransferPacketSend;
            }else{
              Serial.println("Data request, could not find packet number.");
              CurrentTransmitterState = Constants::Wait;
            }
            
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
      }else;
        // Stay in state until lora isn't busy.
      
      break;
      case Constants::CameraCaptureRequest:
        // TO DO: implement camera logic. Use mock data for now.
        CurrentTransmitterState = Constants::CameraCaptureWait;
      break;
      case Constants::CameraCaptureWait:
      // Do nothing while camera works. 
        CurrentTransmitterState = Constants::CameraCaptureComplete;
      break;
      case Constants::CameraCaptureComplete:
        // Send back response with num of packets.
        Serial.println("We inside the camera capture complete case..");
        // Send data back, and go back to waiting.
        if(lora_idle){
        
          lora_idle = false;
          String response = String(Constants::ToRec_Camera_Image_Request_Response);
          response.replace("PACKETS", String((strlen(MockCameraData) / Constants::PacketSize) + 1));
          sprintf(txpacket, response.c_str());
          Serial.print("About to send a packet!");
          Serial.println(txpacket);
          Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
          CurrentTransmitterState = Constants::Wait;
        }else;
          // Stay in state until lora isn't busy.
      break;
      case Constants::DataTransferPacketSend:
        Serial.println("We inside the data transfer case..");
        // Send data back, and go back to waiting.
        if(lora_idle){
        
          lora_idle = false;
          String response = String(Constants::ToRec_Data_Transfer_Response_Header);
          response.replace("PACKET_NUM", String(DataRequestPacket));
          char currentPacketData[Constants::PacketSize];
          memcpy(currentPacketData, &MockCameraData[(DataRequestPacket * Constants::PacketSize)], Constants::PacketSize);
          response += String(currentPacketData) + String(Constants::ToRec_Data_Transfer_Response_Footer);
          
          sprintf(txpacket, response.c_str());
          Serial.print("About to send a packet!");
          Serial.println(txpacket);
          Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
          CurrentTransmitterState = Constants::Wait;
        }else;
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