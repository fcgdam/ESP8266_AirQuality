#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <MQTT.h>               // For MQTT support
#include <ArduinoOTA.h>         // For OTA firmware update support
#include <Adafruit_BMP085.h>    // For supporting the BMP180 sensor
#include <ESP8266mDNS.h>
#include <SimpleTimer.h>
#include <WebServer.hpp>
#include <LogClient.hpp>
#include <TimeProvider.hpp>
#include <AppData.hpp>
#include "secrets.h"

#define  FW_Version "1.0.0"

int             MONITOR_LED   = 2;                // Monitoring led to send some visual indication to the user.
//unsigned long   tick;                             // Used for blinking the MONITOR_LED: ON -> OTA , Blink FAST: Connecting, Blink SLOW: Working
unsigned long   ledBlink = 200;                   // Led blink interval: Fast - Connecting to PZEM, slow - Connected
int             monitor_led_state = LOW;  

WiFiClient      WIFIClient;
IPAddress       thisDevice;
String          Wifi_ssid;
char            hostname[32];

SimpleTimer     timer;

MQTTClient      MQTT_client(512);
char            MQTT_AttributesTopic[256];
char            MQTT_TelemetryTopic[256];
char            SensorAttributes[512];
char            SensorTelemetry[512];

unsigned long   previousMillis = 0;
unsigned long   pingMqtt = 5 * 60 * 1000;  // Ping the MQTT broker every 5 minutes by sending the IOT Atributes message.
unsigned long   previousPing = 0;

// BMP180 Sensor support
int             BMP180_present = 0;
Adafruit_BMP085 bmp;

// DSM501a Sensor support
unsigned long   lpoPM10 = 0;
unsigned long   lpoPM25 = 0;

unsigned long starttime;
unsigned long sampletime_ms = 6000;  // Smaple time 3s

float ratio1 = 0;
float ratio2 = 0;
float concentrationPM10 = 0;
float concentrationPM25 = 0;

/* DSM501a sensor connection. Please note that someteimes the color wires change...
 * https://www.elektronik.ropla.eu/pdf/stock/smy/dsm501.pdf
 * 1 Black  - Not used
 * 2 Red    - Vout2 - 1 microns (PM1.0)
 * 3 White  - Vcc 5V
 * 4 Yellow - Vout1 - 2.5 microns (PM2.5)
 * 5 Orange - GND
*/
#define DUST_SENSOR_DIGITAL_PIN_PM10  12          // DSM501 Pin 2 of DSM501a
#define DUST_SENSOR_DIGITAL_PIN_PM25  14          // DSM501 Pin 4 of DSM501a

/*
 * Hostname:
 * 
 * Sets the device hostname for OTA and MDNS.
 * 
 * */
void setHostname() {
 
  // Set Hostname for OTA and network mDNS (add only 2 last bytes of last MAC Address)
  sprintf_P( hostname, PSTR("ESP-AIRQ-%04X"), ESP.getChipId() & 0xFFFF);
}

/*
 * MQTT Support
 * 
 * Static atributes like, IP, SSID, and so on are set to the MQTT atributes topic.
 * Telemetry data, data that changes through time, are sent to the MQTT telemetry topic.
 * 
 */

//* Supporting functions:
void calcAttributesTopic() {
    String s = "iot/device/" + String(MQTT_ClientID) + "/attributes";
    s.toCharArray(MQTT_AttributesTopic,256,0);
}

void calcTelemetryTopic() {
    String s = "iot/device/" + String(MQTT_ClientID) + "/telemetry";
    s.toCharArray(MQTT_TelemetryTopic,256,0);
}

/*
 * IOT Support:
 * 
 * Functions that using MQTT send data to the IOT server by publishing data on specific topics.
 * 
 */
void IOT_setAttributes() {
    String s = "[{\"type\":\"ESP8266\"}," \
                 "{\"ipaddr\":\"" + thisDevice.toString() + "\"}," \
                 "{\"ssid\":\""+ WiFi.SSID() + "\"}," \
                 "{\"rssi\":\""+ String(WiFi.RSSI()) + "\"}," \
                 "{\"web\":\"http://" + thisDevice.toString() + "\"}" \
                 "]";
    
    s.toCharArray( SensorAttributes, 512,0);
    Log.I("AIRQ Attributes:");
    Log.I(SensorAttributes);
    MQTT_client.publish( MQTT_AttributesTopic, SensorAttributes);
}

void IOT_sendTelemetry(String SensorTelemetry) {
    //s.toCharArray(SensorAttributes, 512,0);
    MQTT_client.publish( MQTT_TelemetryTopic, SensorTelemetry);
}

void  IOT_setTelemetry( String aq,  float cPM10, float cPM25, float pPM10, float pPM25, float temp , unsigned long press ) {
    String tlm;

    tlm = "{\"AQ\":\""+ aq + "\"" \
          ",\"cPM10\":" + String( cPM10 ) + \
          ",\"cPM25\":" + String( cPM25 ) + \
          ",\"pPM10\":" + String( pPM10 ) + \
          ",\"pPM25\":" + String( pPM25 );

    if ( BMP180_present == 1) {
        tlm = tlm + ",\"TEMP\":"  + String( temp ) + \
                    ",\"PRESS\":" + String( press );
   }

   tlm = tlm + "}";

  Log.I( tlm );
  IOT_sendTelemetry( tlm );
}


// MQTT Calback function for receiving subscribed messages.
void MQTT_callback(String &topic, String &payload) {
    /* Just a standard callback. */
    Log.I("Message arrived in topic: ");
    Log.I(topic);

    Log.I("Message:");
    Log.I( payload );
}

//* Connects to the MQTT Broker
void MQTT_Connect() {
    Log.I("Connecting to MQTT Broker...");
    MQTT_client.begin( MQTT_Server, MQTT_Port , WIFIClient );
    MQTT_client.onMessage( MQTT_callback );
    MQTT_client.setOptions( 120, true, 120 );

    while (! MQTT_client.connect( MQTT_ClientID, MQTT_UserID, MQTT_Password ) ) {
        Log.E("MQTT Connection failed.");
        delay(1000);
    }

    calcAttributesTopic();
    calcTelemetryTopic();

    Log.I("Connected to MQTT!");
}

/*
 * OTA support
 * 
 */
void OTA_Setup() {

    Log.I("Setting up OTA...");
    ArduinoOTA.setHostname( hostname );
    ArduinoOTA.setPort(8266);
    ArduinoOTA.begin();

    // OTA callbacks
    ArduinoOTA.onStart([]() {
      Log.I(F("\r\nOTA Starting"));
      digitalWrite( MONITOR_LED, HIGH);
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      uint8_t percent = progress/( total/100 );
      
      if (percent % 10 == 0) {
        Log.I( String(percent));
        digitalWrite( MONITOR_LED, !digitalRead( MONITOR_LED));    
      }
    });

    ArduinoOTA.onEnd([]() {

      Log.I(F("OTA Done\nRebooting..."));
      digitalWrite( MONITOR_LED, LOW);
    });

    ArduinoOTA.onError([](ota_error_t error) {
      Log.E("OTA Error: " + String(error));

      if (error == OTA_AUTH_ERROR) {
        Log.E("OTA Auth Failed");
      } else
      if (error == OTA_BEGIN_ERROR) {
        Log.E("OTA Begin Failed");
      } else
      if (error == OTA_CONNECT_ERROR) {
        Log.E("OTA Connect Failed");
      } else
      if (error == OTA_RECEIVE_ERROR) {
        Log.E("OTA Receive Failed");
      } else
      if (error == OTA_END_ERROR) {
        Log.E("OTA End Failed");
      }

      ESP.restart();
    });

    Log.I("OTA setup done!");
}

void display_WIFIInfo() {
    Log.I("Connected to WIFI: " + WiFi.SSID() );

    thisDevice = WiFi.localIP();
    Log.I(" IP: " + thisDevice.toString() );
}

/*
 * WIFI_Setup: Setup the WIFI connection.
 * 
 * It cycles over the configured access points until a sucessufull connection is done
 * 
 */
void WIFI_Setup() {
    bool   connected = false;
    char   *ssid;
    char   *pwd;
    int    cntAP = 0;
    int    tries = 0;
    String out;

    Log.I("Connecting to WIFI...");
    
    // Station mode.
    WiFi.disconnect();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);

    while ( !connected ) {
        ssid = (char *)APs[cntAP][0];
        pwd = (char *)APs[cntAP][1];
        Log.I("Connecting to: ");
        Log.I(ssid);

        WiFi.begin(ssid, pwd );

        if (WiFi.waitForConnectResult() != WL_CONNECTED) {
                Log.E("Connection Failed! Trying next AP...");
                Log.E("Number of tries: " + String(tries));
                
                cntAP++;
                tries++;
                // Circle the array one entry after another
                if (cntAP == NUMAPS )
                cntAP = 0;
                
                // Set Monitor led on:  Light up the led to show that something is working
                digitalWrite( MONITOR_LED, LOW); 
                delay(1000);
                digitalWrite( MONITOR_LED, HIGH); 

                yield();
        } else
                connected = true;
    }

/*    WiFi.macAddress(MAC_address);
    for (unsigned int i = 0; i < sizeof(MAC_address); ++i){
      sprintf(MAC_char,"%s%02x:",MAC_char,MAC_address[i]);
    }
*/  
    display_WIFIInfo();
}

/*
 * check_Connectivity:  Checks the connectivity. 
 * 
 * Checks the connectivity namely if we are connected to WIFI and to the MQTT broker.
 * If not, we try to reconnect.
 * 
 */

void check_Connectivity() {

        /* Check WIFI connection first. */        
        if ( WiFi.status() != WL_CONNECTED ) {
            WIFI_Setup();
            MQTT_Connect();
        } else {
            /* Check MQTT connectivity: */
            if ( !MQTT_client.connected() ) {
                MQTT_Connect();
                // Send the IOT device attributes at MQTT connection
                IOT_setAttributes();
            }
        }
}

/*
 * back_tasks:  Executes the background tasks
 * 
 * Calls the functions necessary to keep everything running smoothly while waiting or looping
 * Such tasks include mantaining the MQTT connection, checking OTA status and updating the timers.
 * 
 */

void back_tasks() {
    MQTT_client.loop();           // Handle MQTT
    timer.run();                  // Handle SimpleTimer
}

// Just blink the onboard led according to the defined period
void Blink_MonitorLed() {
    digitalWrite( MONITOR_LED , monitor_led_state );
    if ( monitor_led_state == LOW ) 
        monitor_led_state = HIGH;
    else 
        monitor_led_state = LOW;

    timer.setTimeout( ledBlink , Blink_MonitorLed ); // With this trick we can change the blink rate of the led
}

// Used to send the system atributes to the MQTT topic regarding the device attributes.
void IOT_SendAttributes() {
    IOT_setAttributes();
}

// Prints time.
void printTime() {
    timeProvider.logTime();
}

/*
 * setup_dustsensor:  Executes the background tasks
 * 
 * Calls the functions necessary to keep everything running smoothly while waiting or looping
 * Such tasks include mantaining the MQTT connection, checking OTA status and updating the timers.
 * 
 */
void setup_hardware() {

    pinMode(DUST_SENSOR_DIGITAL_PIN_PM10,INPUT);
    pinMode(DUST_SENSOR_DIGITAL_PIN_PM25,INPUT);

    // wait 60s for DSM501 to warm up
    /*  for (int i = 1; i <= 20; i++)
    {
      delay(1000); // 1s
      Serial.print(i);
      Serial.println(" s (wait 60s for DSM501 to warm up)");
    }
    */

    // Connect to the BMP180 sensor
    if ( !bmp.begin() ) {
      Log.E("Sensor BMP180 not found.");
    } else {
      BMP180_present = 1;
    }

}

float getParticlemgm3( float r ) {
    /*
      * with data sheet...regression function is
      *    y=0.1776*x^3-2.24*x^2+ 94.003*x
      */
    // https://github.com/R2D2-2019/R2D2-2019/wiki/Is-the-given-formula-for-calculating-the-mg-m3-for-the-dust-sensor-dsm501a-correct%3F
    
    float mgm3 = 0.001915 * pow(r , 2) + 0.09522 * r - 0.04884;
    return mgm3  < 0.0 ? 0.0 : mgm3;
}


String  setAirQuality( float pm10, float pm25 ) {

    if ( pm10 < 1000 ) {
      appData.setAirQuality("Clean");
      return "Clean";
    }

    if ( pm10 < 10000 ) {
      appData.setAirQuality("Good");
      return "Good";
    }

    if ( pm10 < 20000 ) {
      appData.setAirQuality("Acceptable");
      return "Acceptable";
    }

    if ( pm10 < 50000 ) {
      appData.setAirQuality("Bad");
      return "Bad";
    }

    appData.setAirQuality("Hazzard!");
    return "Hazzard!";
}


/*
 * MAIN CODE
 * 
 */

void setup() {
    IPAddress udpServerAddress;
    udpServerAddress.fromString(UDPLOG_Server);

    appData.setFWVersion(FW_Version);
    appData.setLogServerIPInfo(udpServerAddress.toString());

    Serial.begin(115200);
    delay (200);                           // Wait for the serial port to settle.
    setHostname();
    Log.setSerial( true );                 // Log to Serial
    Log.setServer( udpServerAddress, UDPLOG_Port );
    Log.setTagName("AIRQ");                // Define a tag for log lines output

    // Indicator onbord LED
    pinMode( MONITOR_LED, OUTPUT);
    digitalWrite( MONITOR_LED, LOW);

    // We set WIFI first...
    WIFI_Setup();

    // Setup Logging system.
    Log.I("Enabling UDP Log Server...");
    Log.setUdp( true );                  // Log to UDP server when connected to WIFI.

    // Print a boot mark
    Log.W("------------------------------------------------> System REBOOT");

    // Setup OTA
    OTA_Setup();

    // Set time provider to know current date and time
    timeProvider.setup();
    timeProvider.logTime();

    //Connect to the MQTT Broker:
    MQTT_Connect();

    // Send the IOT device attributes at MQTT connection
    IOT_setAttributes();

    // Setup WebServer so that we can have a web page while connecting to the PZEM004T
    Log.I("Setting up the embedded web server...");
    webServer.setup();
    Log.I("Web server available at port 80.");

    delay(100);
    display_WIFIInfo();                   // To display wifi info on the UDP socket.

    // Setup the monitor blinking led
    timer.setTimeout( ledBlink , Blink_MonitorLed ); 

    // Periodically send to the MQTT server the IOT device state
    timer.setInterval( pingMqtt , IOT_SendAttributes );

    // Periodically log the time
    timer.setInterval( 3 * 60 * 1000 , printTime );

    // Setup MDNS
    MDNS.begin( hostname );
    MDNS.addService("http", "tcp", 80);

    // Setup the hardware
    Log.I("Setting up the hardware");
    setup_hardware();
}

void loop() {
    // Check if we are still connected.
    check_Connectivity();

    // Retrieve DSM501a sensor data
    lpoPM10 += pulseIn(DUST_SENSOR_DIGITAL_PIN_PM10, LOW);
    lpoPM25 += pulseIn(DUST_SENSOR_DIGITAL_PIN_PM25, LOW);
    
    if ((millis()-starttime) > sampletime_ms) {       // Check if we've sampled at least the defined sample time 
        ratio1 = lpoPM10 / ( sampletime_ms * 10.0);   // Integer percentage 0=>100
        concentrationPM10 = 1.1 * pow( ratio1 ,3 ) - 3.8 * pow( ratio1,2 ) + 520 * ratio1 + 0.62; // using spec sheet curve

        ratio2 = lpoPM25 / ( sampletime_ms * 10.0);   // Integer percentage 0=>100
        concentrationPM25 = 1.1 * pow( ratio2, 3 ) - 3.8 * pow( ratio2, 2) + 520 * ratio2 + 0.62; // 

        float pPM10 = getParticlemgm3( ratio1 );
        float pPM25 = getParticlemgm3( ratio2 );

        //String msg = "DSM501a => cPM10: " + String( concentrationPM10) + " ppm, cPM2.5: " + String( concentrationPM25) + " ppm (Samples #: " + appData.getSamplesCount() + ")";
        //Log.I( msg );
        
        //msg = "DSM501a =>  PM10: " + String( pPM10 ) + " ug/m3, PM2.5: " + String( pPM25 ) + " ug/m3"; 
        //Log.I( msg );

        appData.incSamplesCount();
        appData.setcPM10( concentrationPM10 );
        appData.setcPM25( concentrationPM25 );

        appData.setpPM10( pPM10 );
        appData.setpPM25( pPM25 );

        String aq = setAirQuality(concentrationPM10, concentrationPM25 );

        if ( BMP180_present == 1 ) {
          float temp = bmp.readTemperature();
          float press= bmp.readPressure();

          appData.setTemperature( temp );         
          appData.setPressure( press );

          //msg = "BMP180  => Temp: " + appData.getTemperature() + "C , Press: " + appData.getPressureH() + " hPa";
          //Log.I( msg );

          IOT_setTelemetry( aq, concentrationPM10, concentrationPM25, pPM10, pPM25, temp , press );

        } else
        {
          IOT_setTelemetry( aq, concentrationPM10, concentrationPM25, pPM10, pPM25, -1 , -1 );
        }
        
        lpoPM10 = 0;
        lpoPM25 = 0;
        starttime = millis();

    }
    // Execute the background tasks.
    back_tasks();

    ArduinoOTA.handle();          // Handle OTA.
}
