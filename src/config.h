// config: ////////////////////////////////////////////////////////////
#ifndef _ESP32_SERIAL_BRIDGE_CONFIG_H_
#define _ESP32_SERIAL_BRIDGE_CONFIG_H_

#define PROTOCOL_TCP
#ifndef MAX_NMEA_CLIENTS
#define MAX_NMEA_CLIENTS 1
#endif
//#define relay1 19
//#define relay2 18
const char *host = "espSer2net";
const char *ssid = "";
const char *password = "";
const char *MQTT_server = "";
const char *MQTT_user = "";
const char *MQTT_pass = "";
const char *ClientID = "Esp32";
const char *pubTopic = "Esp32/result";
const char *subTopic = "Esp32/cmd";
unsigned char qos = 1; //subscribe qos
bool retained = false;

#if (DEBUG)
bool debug = true;
#else
bool debug = false;
#endif

/*************************  COM Port 0 *******************************/
#define UART_BAUD0 115200        // Baudrate UART0
#ifdef ESP8266
#define SERIAL_PARAM0 SWSERIAL_8N1 // Data/Parity/Stop UART0
#define SERIAL0_RXPIN D5         // receive Pin UART0
#define SERIAL0_TXPIN D6         // transmit Pin UART0
#else
#define SERIAL_PARAM0 SERIAL_8N1 // Data/Parity/Stop UART0
#define SERIAL0_RXPIN 21         // receive Pin UART0
#define SERIAL0_TXPIN 1          // transmit Pin UART0
#endif
#define SERIAL0_TCP_PORT 8880    // Wifi Port UART0

#define BUFFERSIZE 1024

#endif
//////////////////////////////////////////////////////////////////////////
