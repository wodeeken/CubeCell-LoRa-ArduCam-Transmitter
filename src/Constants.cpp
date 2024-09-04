#include "Arduino.h"
struct Constants{
    static constexpr char* FromRec_Ping_Camera_Command = "<PING_CAMERA>";
    static constexpr char* ToRec_Ping_Camera_Response = "<PING_CAMERA_RESPONSE>";
    static constexpr char* FromRec_Camera_Image_Request_Command = "<CAPTURE_CAMERA_REQUEST>";
    static constexpr char* ToRec_Camera_Image_Request_Response = "<CAPTURE_CAMERA_RESPONSE {PACKETS}>";
    static constexpr char* FromRec_Data_Transfer_Command = "^<DATA_TRANSFER_REQUEST {%d+}>$";
    static constexpr char* ToRec_Data_Transfer_Response_Header = "<DATA_TRANSFER_RESPONSE {PACKET_NUM}>";
    //static constexpr char* ToRec_Data_Transfer_Response_Footer = "</DATA_TRANSFER_RESPONSE>";
    static constexpr char* ToRec_Camera_Malfunction = "<CAMERA_ERROR>";
    static const int RecTimeout = 3000;
    static const int PacketSize = 95;
    // Time to wait for confirmation to begin data transfer, or the time between a packet send and t
    //the request to send the next one.
    static const int DataTransferWaitTimeout = 5000; 
    enum TransmitterState{
        Wait, // Doing nothing, waiting to enter into Rx mode.
        RxMode, // In Rx mode, waiting to either recieve something over the radio, or timeout.
        CameraPingRequest, // Camera ping request over radio, respond back.
        CameraCaptureRequest, // Camera capture request over radio, trigger camera.
        CameraCaptureWait, // Camera is currently working. Wait until image is finished. 
        CameraCaptureComplete, // Camera is finished working, or failed. Sending
        DataTransferRequestWait, // Wait until receiving OK to begin data transfer, or until timeout.
        DataTransferPacketSend, // Sending a packet.
        DataTransferPacketWait // Wait until confirmation from receiver to send another, or until timeout. 
    };
};