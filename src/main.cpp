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

// MOCK Camera data.
//char* MockCameraData = "1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200";
// ArduCAM definitions and declarations.
int CS = GPIO1;
ArduCAM myCAM ;//;
bool is_header = false;
int mode = 0;
char CameraData[4000];
//char MockCameraData[3844] = "1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ11121314151617189101a1b1c1d1e1f1g1h1i1j1k1l1m1n1o1p1q1r1s1t1u1v1w1x1y1z1A1B1C1D1E1F1G1H1I1J1K1L1M1N1O1P1Q1R1X1Y1Z212223242526272829202a22b2c2d2e2f2g2h2i2j2k2l2m2n2o2p2q2r2s2t2u2v2w2x2y2z2A2B2C2D2E2F2G2H2I2J2K2L2M2N2O2P2Q2R2S2T2U2V2W2X2Y2Z313233343536373839303a3b3c3d3e3f3g3h3i3j3k3l3m3n3o3p3q3r3s3t3u3v3w3x3y3z3A3B3C3D3E3F3G3H3I3J3K3L3M3N3O3P3Q3R3S3T3U3V3W3X3Y3Z414243444546474849404a4b4c4d4e4f4g4h4i4j4k45l4m4n4o4p4q4r4s4t4u4v4w4x4y4z4A4B4C4D4E4F4G4H4I4J4K4L4M4N4O4P4Q4R4S4T4U4V4W4X4Y4Z1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ11121314151617189101a1b1c1d1e1f1g1h1i1j1k1l1m1n1o1p1q1r1s1t1u1v1w1x1y1z1A1B1C1D1E1F1G1H1I1J1K1L1M1N1O1P1Q1R1X1Y1Z212223242526272829202a22b2c2d2e2f2g2h2i2j2k2l2m2n2o2p2q2r2s2t2u2v2w2x2y2z2A2B2C2D2E2F2G2H2I2J2K2L2M2N2O2P2Q2R2S2T2U2V2W2X2Y2Z313233343536373839303a3b3c3d3e3f3g3h3i3j3k3l3m3n3o3p3q3r3s3t3u3v3w3x3y3z3A3B3C3D3E3F3G3H3I3J3K3L3M3N3O3P3Q3R3S3T3U3V3W3X3Y3Z414243444546474849404a4b4c4d4e4f4g4h4i4j4k45l4m4n4o4p4q4r4s4t4u4v4w4x4y4z4A4B4C4D4E4F4G4H4I4J4K4L4M4N4O4P4Q4R4S4T4U4V4W4X4Y4Z1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ11121314151617189101a1b1c1d1e1f1g1h1i1j1k1l1m1n1o1p1q1r1s1t1u1v1w1x1y1z1A1B1C1D1E1F1G1H1I1J1K1L1M1N1O1P1Q1R1X1Y1Z212223242526272829202a22b2c2d2e2f2g2h2i2j2k2l2m2n2o2p2q2r2s2t2u2v2w2x2y2z2A2B2C2D2E2F2G2H2I2J2K2L2M2N2O2P2Q2R2S2T2U2V2W2X2Y2Z313233343536373839303a3b3c3d3e3f3g3h3i3j3k3l3m3n3o3p3q3r3s3t3u3v3w3x3y3z3A3B3C3D3E3F3G3H3I3J3K3L3M3N3O3P3Q3R3S3T3U3V3W3X3Y3Z414243444546474849404a4b4c4d4e4f4g4h4i4j4k45l4m4n4o4p4q4r4s4t4u4v4w4x4y4z4A4B4C4D4E4F4G4H4I4J4K4L4M4N4O4P4Q4R4S4T4U4V4W4X4Y4Z1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ11121314151617189101a1b1c1d1e1f1g1h1i1j1k1l1m1n1o1p1q1r1s1t1u1v1w1x1y1z1A1B1C1D1E1F1G1H1I1J1K1L1M1N1O1P1Q1R1X1Y1Z212223242526272829202a22b2c2d2e2f2g2h2i2j2k2l2m2n2o2p2q2r2s2t2u2v2w2x2y2z2A2B2C2D2E2F2G2H2I2J2K2L2M2N2O2P2Q2R2S2T2U2V2W2X2Y2Z313233343536373839303a3b3c3d3e3f3g3h3i3j3k3l3m3n3o3p3q3r3s3t3u3v3w3x3y3z3A3B3C3D3E3F3G3H3I3J3K3L3M3N3O3P3Q3R3S3T3U3V3W3X3Y3Z414243444546474849404a4b4c4d4e4f4g4h4i4j4k45l4m4n4o4p4q4r4s4t4u4v4w4x4y4z4A4B4C4D4E4F4G4H4I4J4K4L4M4N4O4P4Q4R4S4T4U4V4W4X4Y4Z1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ11121314151617189101a1b1c1d1e1f1g1h1i1j1k1l1m1n1o1p1q1r1s1t1u1v1w1x1y1z1A1B1C1D1E1F1G1H1I1J1K1L1M1N1O1P1Q1R1X1Y1Z212223242526272829202a22b2c2d2e2f2g2h2i2j2k2l2m2n2o2p2q2r2s2t2u2v2w2x2y2z2A2B2C2D2E2F2G2H2I2J2K2L2M2N2O2P2Q2R2S2T2U2V2W2X2Y2Z313233343536373839303a3b3c3d3e3f3g3h3i3j3k3l3m3n3o3p3q3r3s3t3u3v3w3x3y3z3A3B3C3D3E3F3G3H3I3J3K3L3M3N3O3P3Q3R3S3T3U3V3W3X3Y3Z414243444546474849404a4b4c4d4e4f4g4h4i4j4k45l4m4n4o4p4q4r4s4t4u4v4w4x4y4z4A4B4C4D4E4F4G4H4I4J4K4L4M4N4O4P4Q4R4S4T4U4V4W4X4Y4Z1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ11121314151617189101a1b1c1d1e1f1g1h1i1j1k1l1m1n1o1p1q1r1s1t1u1v1w1x1y1z1A1B1C1D1E1F1G1H1I1J1K1L1M1N1O1P1Q1R1X1Y1Z212223242526272829202a22b2c2d2e2f2g2h2i2j2k2l2m2n2o2p2q2r2s2t2u2v2w2x2y2z2A2B2C2D2E2F2G2H2I2J2K2L2M2N2O2P2Q2R2S2T2U2V2W2X2Y2Z313233343536373839303a3b3c3d3e3f3g3h3i3j3k3l3m3n3o3p3q3r3s3t3u3v3w3x3y3z3A3B3C3D3E3F3G3H3I3J3K3L3M3N3O3P3Q3R3S3T3U3V3W3X3Y3Z414243444546474849404a4b4c4d4e4f4g4h4i4j4k45l4m4n4o4p4q4r4s4t4u4v4w4x4y4z4A4B4C4D4E4F4G4H4I4J4K4L4M4N4O4P4Q4R4S4T4U4V4W4X4Y4Z1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ11121314151617189101a1b1c1d1e1f1g1h1i1j1k1l1m1n1o1p1q1r1s1t1u1v1w1x1y1z1A1B1C1D1E1F1G1H1I1J1K1L1M1N1O1P1Q1R1X1Y1Z212223242526272829202a22b2c2d2e2f2g2h2i2j2k2l2m2n2o2p2q2r2s2t2u2v2w2x2y2z2A2B2C2D2E2F2G2H2I2J2K2L2M2N2O2P2Q2R2S2T2U2V2W2X2Y2Z313233343536373839303a3b3c3d3e3f3g3h3i3j3k3l3m3n3o3p3q3r3s3t3u3v3w3x3y3z3A3B3C3D3E3F3G3H3I3J3K3L3M3N3O3P3Q3R3S3T3U3V3W3X3Y3Z414243444546474849404a4b4c4d4e4f4g4h4i4j4k45l4m4n4o4p4q4r4s4t4u4v4w4x4y4z4A4B4C4D4E4F4G4H4I4J4K4L4M4N4O4P4Q4R4S4T4U4V4W4X4Y4Z";//[6000];
int imageLength; // = sizeof(CameraData);
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
    //myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);
    myCAM.OV2640_set_JPEG_size(OV2640_176x144);
    //myCAM.OV2640_set_JPEG_size(OV2640_352x288);
    //myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);
    //myCAM.OV2640_set_Special_effects(BW);
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
        //CurrentTransmitterState = Constants::CameraCaptureWait;
        //break;
        myCAM.flush_fifo();
        myCAM.clear_fifo_flag();
        //Start capture
        delay(250);
        myCAM.start_capture();
        delay(250);
        CurrentTransmitterState = Constants::CameraCaptureWait;
      break;
      case Constants::CameraCaptureWait:
      //CurrentTransmitterState = Constants::CameraCaptureComplete;
      //break;
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
          //Serial.println("Data1");
          //Serial.println(CameraData);
          lora_idle = false;
          String response = String(Constants::ToRec_Camera_Image_Request_Response);
          response.replace("PACKETS", String((imageLength / Constants::PacketSize) + 1));
          //Serial.println("data2");
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
          //memcpy(currentPacketData, &CameraData[(DataRequestPacket * Constants::PacketSize)], Constants::PacketSize);
          Serial.print("Size of packet w/ header: ");
          Serial.println(sizeof(currentPacketData));
          // Serial.println("<BEGIN>");
          // Serial.println(currentPacketData);
          // Serial.println("<END>");
          //response += String(currentPacketData) + String(Constants::ToRec_Data_Transfer_Response_Footer);
          
          memset(txpacket, 0, 128);
          //char headerChars[response.length()];
          //response.toCharArray(headerChars, response.length());
          response.toCharArray(txpacket, response.length() + 1);
          //strcat(txpacket, headerChars);
          // Serial.println("currentPack1:");
          // Serial.println(txpacket);
          // Serial.println("End");
          int y = strlen(txpacket);
          Serial.print("Begin index for txpacket: ");
          Serial.println(y);
          for(int i = 0; i < sizeof(currentPacketData); i++){
            txpacket[y] = currentPacketData[i];
            y++;
          }
          Serial.println("Current Data Packet DATA: ");
          for(int i = 0; i < sizeof(currentPacketData); i++){
            Serial.print("0x");
            Serial.print(currentPacketData[i], HEX);
            Serial.print(",");
          }
          
          //strcat(txpacket, currentPacketData);

          // Serial.println("currentPack2:");
          // Serial.println(txpacket);
          // Serial.println("End");
          //strcat(txpacket, Constants::ToRec_Data_Transfer_Response_Footer);
          // Serial.println("currentPack3:");
          // Serial.println(txpacket);
          // Serial.println("End");
          // Serial.print("About to send a packet!");
          // Serial.println(txpacket);
          // Print off Camera Data, 
          Serial.println("Camera Data Packet: " + String(DataRequestPacket));
          for(int i = 0; i < sizeof(txpacket); i++){
            Serial.print(txpacket[i], HEX);
            Serial.print(",");
          }
          Serial.println("<DONE> Size of packet: ");
          Serial.println(sizeof(txpacket));
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

  // ArduCAM Functions.
  uint8_t read_fifo_burst(ArduCAM myCAM)
  {
    uint8_t temp = 0, temp_last = 0;
    uint32_t length = 0;
    Serial.println("Assiging camdata = {asdf");
    
    length = myCAM.read_fifo_length();
    Serial.print("Length of image ");
    Serial.println(length);
    if (length >= MAX_FIFO_SIZE) //512 kb
    {
      //Serial.println(F("ACK CMD Over size. END"));
      Serial.println("Assigning camdata = {}");
      //tempCameraData = "";
      return 0;
    }
    if (length == 0 ) //0 kb
    {
      //Serial.println(F("ACK CMD Size is 0. END"));
      Serial.println("Assigning camdata = 0");
      //tempCameraData = "0";
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
        //Serial.println("Assigning cat 1");
        // Serial.print("0x");
        // Serial.print(temp, HEX);
        // Serial.print(",");
        CameraData[index] = temp;
        index++;
        //Serial.write(temp);
      
      }
      else if ((temp == 0xD8) & (temp_last == 0xFF))
      {
        is_header = true;
        //Serial.println("Assigning cat 2");
        // Serial.print("0x");
        // Serial.print(temp_last, HEX);
        // Serial.print(",");
        CameraData[index] = temp_last;
        index++;
        //Serial.write(temp_last);
        
        // Serial.print("0x");
        // Serial.println(temp, HEX);
        // Serial.println(",");
        CameraData[index] = temp;
        index++;
      
    
      }
      if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
      break;
      delayMicroseconds(15);
    }
    myCAM.CS_HIGH();
    is_header = false;
    //CameraData = tempCameraData;
    Serial.print("Index: ");
    Serial.println(index);
    imageLength = index;
    for(int i = 0; i < imageLength; i++){
      Serial.print("0x");
      Serial.print(CameraData[i], HEX);
      Serial.print(",");
    }
    //Serial.println("Print out camdata.");
    //Serial.println(tempCameraData);
    //Serial.print("Size of tempCameraData: ") ;
    //Serial.println(sizeof(tempCameraData));
    //Serial.print("Size of camdata: ");
    //Serial.println(sizeof(CameraData));
    return 1;
  }

  void TriggerCamera(){

  }