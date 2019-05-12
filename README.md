# ESP32-Serial-Bridge

Transparent WiFi (TCP/MQTT) to UART Bridge, supports STATION WiFi modes. The .ino file is the code for the ESP32. Use Arduino IDE for ESP32 to compile and upload it to the ESP32.

set SSID and PW in config for wifi;
set MQTT_server, user and pw in config to connect MQTT broker;
set subTopic and pubTopic in config to send and receive buffer via the bridge.
(OPT 2)connect your_esp32_ip:8880 to send and receive buffer via the bridge.  (your_esp32_ip:8880  <-> COM0)


                               
===============================================================

Used Libraries: (must be installed in the arduino IDE):

https://github.com/espressif/arduino-esp32

===============================================================


# Hardware
here is the wiring diagram recomendation:
https://raw.githubusercontent.com/AlphaLima/ESP32-Serial-Bridge/master/ESP32-SerialBridge.jpg             
Pinning                                                                                     
COM0 Rx <-> GPIO21                                                                               
COM0 Tx <-> GPIO01                                                                                 
                                                                          

NOTE: The PIN assignment has changed and may not look straigt forward (other PINs are marke as Rx/Tx), but this assignment allows to flash via USB also with hooked MAX3232 serial drivers.

I recomend to start your project with a Node32s or compatible evaluation board. For a TTL to RS232 level conversion search google for "TTL RS3232 Converter"



https://tech.scargill.net/wp-content/uploads/2017/05/ESP326.jpg

A discussion incl. the similar ESP8266 projekt can be found here:

http://www.postfrontal.com/forum/topic.asp?TOPIC_ID=8467
