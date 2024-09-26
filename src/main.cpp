#include <ArduCAM.h>
#include <SPI.h>
#include <Wire.h>
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

// ArduCAM definitions and declarations.
int CS = GPIO1;
ArduCAM myCAM ;//;
bool is_header = false;
int mode = 0;
char CameraData[7000];
int imageLength;
uint8_t read_fifo_burst(ArduCAM myCAM);

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

    // ArduCAM Setup code.
    uint8_t vid, pid;
    uint8_t temp;
    // put your setup code here, to run once:
    delay(3000);
    Wire.begin();
    pinMode(CS, OUTPUT);
    myCAM = ArduCAM(OV2640, CS);
    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);
    // initialize SPI:
    SPI.begin(SCK,MISO,MOSI,GPIO1);// sck, miso, mosi, nss

    //update frequency to 10000000;
    SPI.setFrequency(1000000);
    //Reset the CPLD
    myCAM.write_reg(0x07, 0x80);
    delay(100);
    myCAM.write_reg(0x07, 0x00);
    delay(100);
    while(1){
      //Check if the ArduCAM SPI bus is OK
      myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
      temp = myCAM.read_reg(ARDUCHIP_TEST1);

      if (temp != 0x55){
        Serial.println(temp);
        Serial.println(F("ACK CMD SPI interface Error! END"));
        delay(1000);
        continue;
      }else{
        Serial.println(F("ACK CMD SPI interface OK. END"));
        break;
      }
    }

    while(1){
      //Check if the camera module type is OV2640
      myCAM.wrSensorReg8_8(0xff, 0x01);
      myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
      myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
      delay(2000);
      if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))){
        Serial.println(F("ACK CMD Can't find OV2640 module! END"));
        delay(1000);
        continue;
      }
      else{
        Serial.println(F("ACK CMD OV2640 detected. END"));
        break;
      } 
    }
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    myCAM.OV2640_set_JPEG_size(OV2640_160x120);
    delay(1000);
    myCAM.clear_fifo_flag();
    Serial.println("Done setting up Cam.");
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
        delay(100);
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
        myCAM.flush_fifo();
        myCAM.clear_fifo_flag();
        //Start capture
        delay(250);
        myCAM.start_capture();
        delay(250);
        CurrentTransmitterState = Constants::CameraCaptureWait;
      break;
      case Constants::CameraCaptureWait:
      // Do nothing while camera works. 
      if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
        {
          delay(50);
          read_fifo_burst(myCAM);
          //Clear the capture done flag
          myCAM.clear_fifo_flag();
          CurrentTransmitterState = Constants::CameraCaptureComplete;
        }
        
      break;
      case Constants::CameraCaptureComplete:
        // Send back response with num of packets.
        Serial.println("We inside the camera capture complete case..");
        // Send data back, and go back to waiting.
        if(lora_idle){
        
          lora_idle = false;
          String response = String(Constants::ToRec_Camera_Image_Request_Response);
          response.replace("PACKETS", String((imageLength / Constants::PacketSize) + 1));
          sprintf(txpacket, response.c_str());
          Serial.print("About to send a packet!");
          Serial.println(txpacket);
          Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
          CurrentTransmitterState = Constants::Wait;
        }else
        Serial.println("Waiting");
          // Stay in state until lora isn't busy.
      break;
      case Constants::DataTransferPacketSend:
        Serial.println("We inside the data transfer case..");
        // Send data back, and go back to waiting.
        if(lora_idle){
        
          lora_idle = false;
          String response = String(Constants::ToRec_Data_Transfer_Response_Header);
          response.replace("PACKET_NUM", String(DataRequestPacket));
          response.trim();
          char currentPacketData[Constants::PacketSize];
          Serial.print("Sending " );
          Serial.print((DataRequestPacket * Constants::PacketSize));
          Serial.print(" through ");
          Serial.println((DataRequestPacket * Constants::PacketSize) + Constants::PacketSize);
          int curPacketIndex = 0;
          for(int i = DataRequestPacket * Constants::PacketSize; i < (DataRequestPacket * Constants::PacketSize) + Constants::PacketSize; i++){
            currentPacketData[curPacketIndex] = CameraData[i];
            curPacketIndex++;
          }
          Serial.print("Size of packet w/ header: ");
          Serial.println(sizeof(currentPacketData));
          
          
          memset(txpacket, 0, 128);
          
          response.toCharArray(txpacket, response.length() + 1);
          
          int y = strlen(txpacket);
          
          for(int i = 0; i < sizeof(currentPacketData); i++){
            txpacket[y] = currentPacketData[i];
            y++;
          }
          
          Radio.Send( (uint8_t *)txpacket, sizeof(txpacket) );
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
      rxpacket[size]='\0';
      turnOnRGB(COLOR_RECEIVED,0);
      Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n",rxpacket,rssi,rxSize);
      lora_idle = true;
  }
  void OnTxDone( void )
  {
    turnOffRGB();
    lora_idle = true;
  }

  void OnTxTimeout( void )
  {
    turnOffRGB();
    lora_idle = true;
  }
  void OnRxTimeout( void )
  {
    RxTimedout = true;
    turnOffRGB();
    Serial.println("Timedout");
    lora_idle = true;
  }

  // ArduCAM Functions.
  uint8_t read_fifo_burst(ArduCAM myCAM)
  {
    uint8_t temp = 0, temp_last = 0;
    uint32_t length = 0;
    
    length = myCAM.read_fifo_length();
    Serial.print("Length of image ");
    Serial.println(length);
    if (length >= MAX_FIFO_SIZE) //512 kb
    {
      Serial.println("Assigning camdata = {}");
      return 0;
    }
    if (length == 0 ) //0 kb
    {
      Serial.println("Assigning camdata = 0");
      return 0;
    }
    myCAM.CS_LOW();
    myCAM.set_fifo_burst();//Set fifo burst mode
    temp =  SPI.transfer(0x00);
    length --;
    int index = 0;
    while ( length-- )
    {
      temp_last = temp;
      temp =  SPI.transfer(0x00);
      if (is_header == true)
      {
        CameraData[index] = temp;
        index++;
      
      }
      else if ((temp == 0xD8) & (temp_last == 0xFF))
      {
        is_header = true;
        CameraData[index] = temp_last;
        index++;
        CameraData[index] = temp;
        index++;
      
    
      }
      if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
      break;
      delayMicroseconds(15);
    }
    myCAM.CS_HIGH();
    is_header = false;
    Serial.print("Index: ");
    Serial.println(index);
    imageLength = index;
    return 1;
  }
