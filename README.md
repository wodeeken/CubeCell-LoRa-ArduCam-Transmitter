# CubeCell-LoRa-ArduCam-Transmitter

A VSCode/Platform IO project for the HelTec CubeCell HTCC-AB01 LoRa board wired to an ArduCAM OV2640 2MP Camera. 

# Notes

1. This program is intended to be used in conjunction with the CubeCell-LoRa-ArduCam-Receiver(https://github.com/wodeeken/CubeCell-LoRa-ArduCam-Receiver) and Lora-ArduCAM-Host-App(https://github.com/wodeeken/Lora-ArduCAM-Host-App) projects. This program should be uploaded to an HelTec CubeCell HTCC-AB01 board configured with an ArduCAM camera. The transmitter sends image data to the receiver, which then relays image data to the Lora-ArduCAM-Host-App application running on a computer that is connected to the receiver LoRa board via USB.
2. This program uses a forked version of the ArduCAM library located at (https://github.com/wodeeken/ArduCAM-CubeCell). This library is already included in the repository and should not require further action by the user.
# Usage 

1. This program was created using the PlatformIO extension for VSCode.
2. The ArduCAM is wired to the HTCC-AB01 board in the following table:

| ArduCAM | HTCC-AB01 Pins (Printed Labels) |
|---------|---------------------------------|
| CS      | 1                               |
| MOSI    | MO                              |
| MISO    | MI                              |
| SCK     | SCK                             |
| GND     | GND                             |
| VCC     | 3v3                             |
| SDA     | SDA                             |
| SCL     | SCL                             |


