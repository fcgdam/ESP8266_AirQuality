#ifndef _SECRETS_H
#define _SECRETS_H

#define NUMAPS 4                            // Define the number of Access Points that we try to connect to
static char const *APs[NUMAPS][2] = {
  {"AP1","pass1"},                          // SSID , Password.  Make sure that the number of entries matchs the NUMAPS
  {"AP2","pass2"},
  {"AP3","pass3"},
  {"AP4","pass4"}
};

// For connecting to the MQTT Broker
#define MQTT_Server    "192.168.1.17"       // MQTT Broker address
#define MQTT_Port      1883                 // MQTT Broker port
#define MQTT_ClientID  "ESP8266_AIRQ"       // Client ID. This is what will be used for the DEVICEID on iot/device/DEVICEID/telemetry;attributes topic
#define MQTT_UserID    "ESP8266_AIRQ"       // User Id if the MQTT broker requires authentication
#define MQTT_Password  "password1"          // User password for the MQTT user.

// For the UDP Log Server
#define UDPLOG_Server  "192.168.1.17"       // IP address where we might run the logServer.sh script to see the log output of the device
#define UDPLOG_Port    5014

#endif
