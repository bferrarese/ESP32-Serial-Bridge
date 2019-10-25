// ESP32 WiFi（TCP/MQTT） <-> UART Bridge
// Forked from AlphaLima/ESP32-Serial-Bridge

#include "config.h"

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include "SoftwareSerial.h"
#else
#include <WiFi.h>
#include <ESPmDNS.h>
#endif

#include <ESPAsyncWebServer.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESPAsyncWiFiManager.h>

void callback(char *topic, byte *payload, unsigned int length);

// For debug log output/update FW
HardwareSerial *COM = &Serial;

uint8_t buf1[BUFFERSIZE];
uint16_t i1 = 0;

uint8_t buf2[BUFFERSIZE];
uint16_t i2 = 0;

#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiServer server_0(SERIAL0_TCP_PORT);
WiFiServer *server = &server_0;
WiFiClient *TCPClient[MAX_NMEA_CLIENTS];
#endif

WiFiClient wfclient;
PubSubClient client(MQTT_server, 1883, callback, wfclient);

AsyncWebServer wifiManagerServer(80);
DNSServer wifiManagerDNS;

void callback(char *topic, byte *payload, unsigned int length)
{

  byte *p = (byte *)malloc(length);
  memcpy(p, payload, length); // Copy the payload to the new buffer

  /*
  char msg[length+1];
  for(int i=0;i<length;i++){
    msg[i]=(char)p[i];
  }
  msg[length]='\0';
  String msgText(msg);

  if ( msgText == "open relay1") digitalWrite(relay1, HIGH);
  else if( msgText == "close relay1") digitalWrite(relay1, LOW);
  else if ( msgText == "open relay2") digitalWrite(relay2, HIGH);
  else if ( msgText == "close relay2") digitalWrite(relay2, LOW);
  else {
    client.publish(pubTopic, p, length, retained);
    COM->write(p, length); // UART write buffer received via MQTT sub_topic
  }
  */
  client.publish(pubTopic, p, length, retained);
  COM->write(p, length); // UART write buffer received via MQTT sub_topic
  free(p);
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_server, MQTT_user, MQTT_pass))
    {
      // Once connected, publish an announcement...
      client.publish(pubTopic, "UART<-->WiFi Bridge Reconnected", false);
      // ... and resubscribe
      client.subscribe(subTopic, qos);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  delay(500);
#ifdef ESP8266
  COM->begin(UART_BAUD0, SERIAL_PARAM0);
#else
  COM->begin(UART_BAUD0, SERIAL_PARAM0, SERIAL0_RXPIN, SERIAL0_TXPIN);
#endif
  if (debug)
    COM->println("\n\n WiFi Serial Bridge V2.00");

  //pinMode(relay1, OUTPUT);
  //pinMode(relay2, OUTPUT);
  if (strlen(ssid) == 0)
  {
    if (debug)
      COM->println("No SSID defined, use wifi manager.");
    AsyncWiFiManager wifiManager(&wifiManagerServer, &wifiManagerDNS);
    wifiManager.autoConnect(host);
  }
  else
  {
    if (debug)
    {
      COM->print("SSID defined, connect to ");
      COM->println(ssid);
    }
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
  }
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  if (debug)
    COM->println(WiFi.localIP());

  // Enable MDNS
  MDNS.begin(host);

  // Enable OTA

  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname(host);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else
    { // U_FS
      type = "filesystem";
    }

    if (debug)
      COM->println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    if (debug)
      COM->println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (debug)
      COM->printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    COM->printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
    {
      COM->println("Auth Failed");
    }
    else if (error == OTA_BEGIN_ERROR)
    {
      COM->println("Begin Failed");
    }
    else if (error == OTA_CONNECT_ERROR)
    {
      COM->println("Connect Failed");
    }
    else if (error == OTA_RECEIVE_ERROR)
    {
      COM->println("Receive Failed");
    }
    else if (error == OTA_END_ERROR)
    {
      COM->println("End Failed");
    }
  });
  ArduinoOTA.begin();

  if (strlen(MQTT_server) && client.connect(ClientID, MQTT_user, MQTT_pass))
  {
    client.publish(pubTopic, "UART<-->WiFi Bridge Connected", false);
    client.subscribe(subTopic, qos);
  }

#ifdef PROTOCOL_TCP
  if (debug)
    COM->println("Starting TCP Server");
  server->begin(); // start TCP server
  server->setNoDelay(true);
#endif

#ifndef ESP8266
  esp_err_t esp_wifi_set_max_tx_power(50); //lower WiFi Power
#endif
}

void loop()
{
  if (strlen(MQTT_server))
  {
    if (!client.connected())
    {
      reconnect();
    }
    client.loop();
  }

#ifdef PROTOCOL_TCP
  for (byte i = 0; i < MAX_NMEA_CLIENTS; i++)
  {
    //find disconnected spot
    if (TCPClient[i] && !TCPClient[i]->connected())
    {
      TCPClient[i]->stop();
      delete TCPClient[i];
      TCPClient[i] = NULL;
      if (debug)
      {
        COM->print(i);
        COM->println("Client disconnected");
      }
    }
  }
  if (server->hasClient())
  {
    for (byte i = 0; i < MAX_NMEA_CLIENTS; i++)
    {
      //find free/disconnected spot
      if (!TCPClient[i] || !TCPClient[i]->connected())
      {
        if (TCPClient[i])
        {
          TCPClient[i]->stop();
          delete TCPClient[i];
          TCPClient[i] = NULL;
          if (debug)
          {
            COM->print(i);
            COM->println("Client disconnected");
          }
        }
        TCPClient[i] = new WiFiClient;
        *TCPClient[i] = server->available();
        if (debug)
        {

          COM->print("New client for COM");
          COM->print(0);
          COM->println(i);
        }
        continue;
      }
    }
    //no free/disconnected spot so reject
    WiFiClient TmpserverClient = server->available();
    TmpserverClient.stop();
  }
#endif

  if (COM != NULL)
  {
    for (byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
    {
      if (TCPClient[cln])
      {
        while (TCPClient[cln]->available())
        {
          buf1[i1] = TCPClient[cln]->read(); // read char from TCP port:8880
          if (i1 < BUFFERSIZE - 1)
            i1++;
        }
        COM->write(buf1, i1); // now send to UART
        i1 = 0;
      }
    }

    if (COM->available())
    {
      while (COM->available())
      {
        buf2[i2] = COM->read(); // read char from UART
        if (i2 < BUFFERSIZE - 1)
          i2++;
      }
      // now send to WiFi:
      for (byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
      {
        if (TCPClient[cln])
          TCPClient[cln]->write(buf2, i2); //send the buffer to TCP port:8880
      }
      if (strlen(MQTT_server))
      {
        client.publish(pubTopic, buf2, i2, retained); //Publish the buffer received via serial
      }
      i2 = 0;
    }
  }

#ifdef ESP8266
  MDNS.update();
#endif
  ArduinoOTA.handle();
}
