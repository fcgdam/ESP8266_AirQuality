#ifndef __APP_DATA_H__
#define __APP_DATA_H__

#include <Arduino.h>

#define PZEM_DISCONNECTED   0
#define PZEM_CONNECTING     1
#define PZEM_CONNECTED      2
#define PZEM_CONNECTFAIL    3

class AppData
{
private:
    String  m_FWVersion;
    String  m_logServerIP;
    
    float         m_temperature;
    unsigned long m_pressure;
    float         m_cpm10;              // Concentration
    float         m_cpm25;
    float         m_ppm10;              // Particles
    float         m_ppm25;
    String        m_airquality;       // Air quality description
    unsigned long m_samplescount;

public:
    void setFWVersion( String);
    void setLogServerIPInfo( String );

    String getFWVersion();
    String getLogServerIPInfo();
    String getDevIP();
    String getGWIP();
    String getSSID();
    String getRSSI();
    String getHeap();

    void   setTemperature( float );
    void   setPressure( unsigned long );
    void   setcPM10( float );
    void   setcPM25( float );
    void   setpPM10( float );
    void   setpPM25( float );
    void   setAirQuality( String);
    void   setSamplesCount( unsigned long );
    void   incSamplesCount( );

    String   getTemperature( );
    String   getPressure( );        // Returns PA
    String   getPressureH( );       // Returns hPa
    String   getcPM10( );
    String   getcPM25( );
    String   getpPM10( );
    String   getpPM25( );    
    String   getAirQuality( );
    String   getSamplesCount( );
};

extern AppData appData;

#endif