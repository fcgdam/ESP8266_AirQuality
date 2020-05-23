#include <Arduino.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebServer.hpp>
#include "AppData.hpp"
#include "html_templates.h"
#include "trianglify.min.js-gz.h"

WebServer webServer;

static AsyncWebServer server(80);

/*
 * Web Server pages:
 * 
 */

/* Handle page not founds. */
void Web_PageNotFound( AsyncWebServerRequest *request )
{
  request->send(404);
}

/* Root Page: / */

void Web_RootPage( AsyncWebServerRequest *request )
{
  AsyncResponseStream *response = request->beginResponseStream("text/html");
  response->printf( TEMPLATE_HEADER,  appData.getFWVersion().c_str(), \
                                      appData.getDevIP().c_str(), \
                                      appData.getGWIP().c_str(), \
                                      appData.getSSID().c_str(), \
                                      appData.getRSSI().c_str(), \
                                      appData.getLogServerIPInfo().c_str(), \
                                      appData.getHeap().c_str(), \
                                      appData.getAirQuality().c_str(), \
                                      appData.getcPM10().c_str(), \
                                      appData.getcPM25().c_str(), \
                                      appData.getpPM10().c_str(), \
                                      appData.getpPM25().c_str(), \                                      
                                      appData.getSamplesCount().c_str(), \
                                      appData.getTemperature().c_str(), \
                                      appData.getPressureH().c_str() );
                                      
  response->print( TEMPLATE_FOOTER);

  request->send(response);
}


/*
 * WebServer Class functions.
 */

WebServer::WebServer()
{ 
  initialized = false;
}

void WebServer::setup()
{
    server.on( "/", HTTP_GET, Web_RootPage );

    server.onNotFound( Web_PageNotFound );

    server.on("/trianglify.min.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse_P(200, "text/javascript",
            trianglify_min_js_gz, sizeof(trianglify_min_js_gz));
        response->addHeader("Content-Encoding", "gzip");
        request->send(response);
    });

  server.begin();

  initialized = true;

}

void WebServer::handle()
{
  if( ! initialized )
  {
    setup();
  }
}

