#include "Arduino.h"
struct Constants{
    static constexpr char* FromRec_Ping_Camera_Command = "<PING_CAMERA>";
    static constexpr char* ToRec_Ping_Camera_Response = "<PING_CAMERA_RESPONSE>";
    static const int RecTimeout = 3000;
    enum TransmitterState{
        Wait, // Doing nothing, waiting to enter into Rx mode.
        RxMode, // In Rx mode, waiting to either recieve something over the radio, or timeout.
        CameraPingRequest, // Camera ping request over radio, respond back.
    };
};