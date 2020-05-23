#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <AppData.hpp>

AppData appData;

void AppData::setFWVersion( String fwversion ) {
    m_FWVersion = fwversion;
}

void AppData::setLogServerIPInfo( String lsIP ) {
    m_logServerIP = lsIP;
}

void AppData::setcPM10(float pm10) {
    m_cpm10 = pm10;
}

String AppData::getcPM10() {
    return String(m_cpm10);
}

void AppData::setcPM25( float pm25) {
    m_cpm25 = pm25;
}

String AppData::getcPM25() {
    return String(m_cpm25);
}

void AppData::setpPM10(float pm10) {
    m_ppm10 = pm10;
}

String AppData::getpPM10() {
    return String(m_ppm10);
}

void AppData::setpPM25( float pm25) {
    m_ppm25 = pm25;
}

String AppData::getpPM25() {
    return String(m_ppm25);
}

void AppData::setPressure( unsigned long press) {
    m_pressure = press;
}

String AppData::getPressure() {
    return String(m_pressure);
}

String AppData::getPressureH() {
    float hpa = ((float)m_pressure) / 100;
    char  buffer[16];
    dtostrf( hpa, 4, 2, buffer);
    return String( buffer );
}

void AppData::setTemperature(float temp) {
    m_temperature = temp;
}

String AppData::getTemperature() {
    char  buffer[16];
    dtostrf( m_temperature, 4, 2, buffer);
    return String( buffer );
}

void AppData::setAirQuality( String aq ) {
    m_airquality = aq;
}

String AppData::getAirQuality() {
    return m_airquality;
}

void AppData::setSamplesCount(unsigned long sc) {
    m_samplescount = sc;
}

String AppData::getSamplesCount() {
    return String(m_samplescount);
}

void AppData::incSamplesCount( ) {
    m_samplescount++;
}

String AppData::getFWVersion() {
    return m_FWVersion;
}

String AppData::getSSID() {
    return WiFi.SSID();
}

String AppData::getRSSI() {
    return String(WiFi.RSSI());
}

String AppData::getDevIP() {
    IPAddress ipdev = WiFi.localIP();

    return ipdev.toString();
}

String AppData::getGWIP() {
    IPAddress ipgw = WiFi.gatewayIP();

    return ipgw.toString();
}

String AppData::getLogServerIPInfo() {
    return m_logServerIP;
}

String AppData::getHeap() {
    return String(ESP.getFreeHeap());
}