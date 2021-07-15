# ESP32_PDM_MIC
Interface for ESP32 to record audio with TDK's InvenSense ICS-41352 PDM microphone  

The I2S interface is based on code provided by https://github.com/GrahamM/ESP32_I2S_Microphone  
UDP server is based on example code provided by https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_server  

Compile and flash with ESP IDF: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/  

Currently, -O2 is set in compiler options. Change in menuconfig for debug builds.   