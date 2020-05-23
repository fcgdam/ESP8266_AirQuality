# ESP8266 DSM501a and BMP180 Air quality monitor

Simple IoT device that collects data from the DSM501a dust sensor and temperature and pressure from BMP180 and feeds it to a MQTT broker.

It provide a web page where the collected data can be seen.

![](/images/web_page.png?raw=true)

# Configuration and installation

Configure the number of access points that the device might access to connect to the network and respective SSID and passwords on the **secrets.h** file.

On that file also change the MQTT broker address and credentials used to access the MQTT broker.

The log server address IP can be any Linux server that runs the **logServer.sh** script to monitor the virtual serial port across UDP.

After the changes just run: *pio run* and *pio run --target upload* to flash the board.


