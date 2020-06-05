[{"id":"809bdd80.251f7","type":"tab","label":"Air Quality","disabled":false,"info":""},{"id":"66358a9.32ea7f4","type":"mqtt in","z":"809bdd80.251f7","name":"","topic":"iot/device/ESP8266_AIRQ/telemetry","qos":"2","datatype":"auto","broker":"2a552b3c.de8d2c","x":180,"y":40,"wires":[["e9006b24.fd33"]]},{"id":"adf61f3a.c680f8","type":"debug","z":"809bdd80.251f7","name":"","active":true,"tosidebar":true,"console":false,"tostatus":false,"complete":"false","x":1010,"y":460,"wires":[]},{"id":"e9006b24.fd33","type":"json","z":"809bdd80.251f7","name":"","property":"payload","action":"","pretty":false,"x":110,"y":460,"wires":[["adf61f3a.c680f8","cd3c1b5f.a9884","d29a340.14e0b5","573fcf3.54fc4b","c9c25483.ce19b","d1c5ff23.d4c41","8147b745.889c5","d57fd37d.facba8","9f5c09af.db608"]]},{"id":"7c04c57c.2fa65c","type":"ui_text","z":"809bdd80.251f7","group":"6bc3973c.f4166","order":0,"width":0,"height":0,"name":"Air Quality","label":"","format":"{{msg.payload}}","layout":"row-spread","x":670,"y":160,"wires":[]},{"id":"cd3c1b5f.a9884","type":"function","z":"809bdd80.251f7","name":"AQ","func":"msg.payload = msg.payload.AQ;\n\n\nreturn msg;","outputs":1,"noerr":0,"x":370,"y":160,"wires":[["7c04c57c.2fa65c"]]},{"id":"f1f8e61.d449018","type":"ui_chart","z":"809bdd80.251f7","name":"","group":"17877e15.37bd42","order":0,"width":0,"height":0,"label":"PM 1.0 Concentration","chartType":"line","legend":"false","xformat":"HH:mm:ss","interpolate":"linear","nodata":"","dot":false,"ymin":"","ymax":"","removeOlder":1,"removeOlderPoints":"","removeOlderUnit":"3600","cutout":0,"useOneColor":false,"colors":["#1f77b4","#aec7e8","#ff7f0e","#2ca02c","#98df8a","#d62728","#ff9896","#9467bd","#c5b0d5"],"useOldStyle":false,"x":700,"y":300,"wires":[[],[]]},{"id":"d29a340.14e0b5","type":"function","z":"809bdd80.251f7","name":"C PM10","func":"msg.payload = msg.payload.cPM10;\n\n\nreturn msg;","outputs":1,"noerr":0,"x":360,"y":260,"wires":[["f1f8e61.d449018","ecdf93d2.303b8"]]},{"id":"ecdf93d2.303b8","type":"ui_text","z":"809bdd80.251f7","group":"17877e15.37bd42","order":0,"width":"0","height":"0","name":"cPM 1.0","label":"cPM 1.0","format":"{{msg.payload}}","layout":"row-spread","x":660,"y":240,"wires":[]},{"id":"573fcf3.54fc4b","type":"function","z":"809bdd80.251f7","name":"C PM2.5","func":"msg.payload = msg.payload.cPM25;\n\nreturn msg;","outputs":1,"noerr":0,"x":360,"y":380,"wires":[["44bd0ab4.5be9b4","8435f8f0.34d7a"]]},{"id":"44bd0ab4.5be9b4","type":"ui_text","z":"809bdd80.251f7","group":"17877e15.37bd42","order":0,"width":0,"height":0,"name":"cPM 2.5","label":"cPM 2.5","format":"{{msg.payload}}","layout":"row-spread","x":660,"y":360,"wires":[]},{"id":"8435f8f0.34d7a","type":"ui_chart","z":"809bdd80.251f7","name":"","group":"17877e15.37bd42","order":0,"width":0,"height":0,"label":"PM 2.5 Concentration","chartType":"line","legend":"false","xformat":"HH:mm:ss","interpolate":"linear","nodata":"","dot":false,"ymin":"","ymax":"","removeOlder":1,"removeOlderPoints":"","removeOlderUnit":"3600","cutout":0,"useOneColor":false,"colors":["#1f77b4","#aec7e8","#ff7f0e","#2ca02c","#98df8a","#d62728","#ff9896","#9467bd","#c5b0d5"],"useOldStyle":false,"x":700,"y":420,"wires":[[],[]]},{"id":"5348253f.ec1064","type":"ui_chart","z":"809bdd80.251f7","name":"","group":"b8285802.12f9a","order":0,"width":0,"height":0,"label":"PM 1.0 Particles","chartType":"line","legend":"false","xformat":"HH:mm:ss","interpolate":"linear","nodata":"","dot":false,"ymin":"","ymax":"","removeOlder":1,"removeOlderPoints":"","removeOlderUnit":"3600","cutout":0,"useOneColor":false,"colors":["#1f77b4","#aec7e8","#ff7f0e","#2ca02c","#98df8a","#d62728","#ff9896","#9467bd","#c5b0d5"],"useOldStyle":false,"x":680,"y":560,"wires":[[],[]]},{"id":"c9c25483.ce19b","type":"function","z":"809bdd80.251f7","name":"C PM10","func":"msg.payload = msg.payload.pPM10;\n\n\nreturn msg;","outputs":1,"noerr":0,"x":360,"y":560,"wires":[["5348253f.ec1064","593c07de.c77268"]]},{"id":"593c07de.c77268","type":"ui_text","z":"809bdd80.251f7","group":"b8285802.12f9a","order":0,"width":"0","height":"0","name":"cPM 1.0","label":"pPM 1.0","format":"{{msg.payload}}","layout":"row-spread","x":660,"y":500,"wires":[]},{"id":"d1c5ff23.d4c41","type":"function","z":"809bdd80.251f7","name":"C PM2.5","func":"msg.payload = msg.payload.pPM25;\n\nreturn msg;","outputs":1,"noerr":0,"x":360,"y":680,"wires":[["2ab5d550.009902","f77a1201.ffe568"]]},{"id":"2ab5d550.009902","type":"ui_text","z":"809bdd80.251f7","group":"b8285802.12f9a","order":0,"width":0,"height":0,"name":"cPM 2.5","label":"pPM 2.5","format":"{{msg.payload}}","layout":"row-spread","x":660,"y":640,"wires":[]},{"id":"f77a1201.ffe568","type":"ui_chart","z":"809bdd80.251f7","name":"","group":"b8285802.12f9a","order":0,"width":0,"height":0,"label":"PM 2.5 Particles","chartType":"line","legend":"false","xformat":"HH:mm:ss","interpolate":"linear","nodata":"","dot":false,"ymin":"","ymax":"","removeOlder":1,"removeOlderPoints":"","removeOlderUnit":"3600","cutout":0,"useOneColor":false,"colors":["#1f77b4","#aec7e8","#ff7f0e","#2ca02c","#98df8a","#d62728","#ff9896","#9467bd","#c5b0d5"],"useOldStyle":false,"x":680,"y":700,"wires":[[],[]]},{"id":"543b895.a860078","type":"ui_chart","z":"809bdd80.251f7","name":"","group":"56731861.fd122","order":0,"width":0,"height":0,"label":"Temperature","chartType":"line","legend":"false","xformat":"HH:mm:ss","interpolate":"linear","nodata":"","dot":false,"ymin":"","ymax":"","removeOlder":1,"removeOlderPoints":"","removeOlderUnit":"3600","cutout":0,"useOneColor":false,"colors":["#1f77b4","#aec7e8","#ff7f0e","#2ca02c","#98df8a","#d62728","#ff9896","#9467bd","#c5b0d5"],"useOldStyle":false,"x":670,"y":820,"wires":[[],[]]},{"id":"8147b745.889c5","type":"function","z":"809bdd80.251f7","name":"Temp","func":"msg.payload = msg.payload.TEMP;\n\n\nreturn msg;","outputs":1,"noerr":0,"x":350,"y":780,"wires":[["543b895.a860078","34818dd0.623712"]]},{"id":"34818dd0.623712","type":"ui_text","z":"809bdd80.251f7","group":"56731861.fd122","order":0,"width":"0","height":"0","name":"Temperature","label":"Temperature","format":"{{msg.payload}}","layout":"row-spread","x":670,"y":760,"wires":[]},{"id":"d57fd37d.facba8","type":"function","z":"809bdd80.251f7","name":"Pressure","func":"msg.payload = msg.payload.PRESS;\n\nreturn msg;","outputs":1,"noerr":0,"x":360,"y":920,"wires":[["c97f60fc.dd9c4","b5287e35.453ef"]]},{"id":"c97f60fc.dd9c4","type":"ui_text","z":"809bdd80.251f7","group":"56731861.fd122","order":0,"width":0,"height":0,"name":"Pressure","label":"Pressure","format":"{{msg.payload}}","layout":"row-spread","x":660,"y":900,"wires":[]},{"id":"b5287e35.453ef","type":"ui_chart","z":"809bdd80.251f7","name":"","group":"56731861.fd122","order":0,"width":0,"height":0,"label":"Pressure","chartType":"line","legend":"false","xformat":"HH:mm:ss","interpolate":"linear","nodata":"","dot":false,"ymin":"","ymax":"","removeOlder":1,"removeOlderPoints":"","removeOlderUnit":"3600","cutout":0,"useOneColor":false,"colors":["#1f77b4","#aec7e8","#ff7f0e","#2ca02c","#98df8a","#d62728","#ff9896","#9467bd","#c5b0d5"],"useOldStyle":false,"x":660,"y":960,"wires":[[],[]]},{"id":"9f5c09af.db608","type":"influxdb out","z":"809bdd80.251f7","influxdb":"705311c.942c77","name":"DustData","measurement":"DustData","precision":"","retentionPolicy":"","x":120,"y":580,"wires":[]},{"id":"2a552b3c.de8d2c","type":"mqtt-broker","z":"","broker":"192.168.1.17","port":"1883","clientid":"NodeRed","usetls":false,"verifyservercert":true,"compatmode":true,"keepalive":"15","cleansession":true,"birthTopic":"iot/devices/NodeRed/attributes","birthQos":"0","birthPayload":"[{\"active\":true}, {\"web\":\"http://192.168.1.17:1880\"}]","willTopic":"iot/devices/NodeRed/attributes","willQos":"0","willPayload":"[{\"active\":false}, {\"web\":\"http://192.168.1.17:1880\"}]"},{"id":"6bc3973c.f4166","type":"ui_group","z":"","name":"Air Quality","tab":"40e3d6f.56249a8","disp":true,"width":"6"},{"id":"17877e15.37bd42","type":"ui_group","z":"","name":"DSM501a Concentrations","tab":"40e3d6f.56249a8","disp":true,"width":"6"},{"id":"b8285802.12f9a","type":"ui_group","z":"","name":"DSM501a Particles ug/m3","tab":"40e3d6f.56249a8","disp":true,"width":"6"},{"id":"56731861.fd122","type":"ui_group","z":"","name":"Temperature/Pressure","tab":"40e3d6f.56249a8","disp":true,"width":"6"},{"id":"705311c.942c77","type":"influxdb","z":"","hostname":"127.0.0.1","port":"8086","protocol":"http","database":"DustData","name":"DustData","usetls":false,"tls":""},{"id":"40e3d6f.56249a8","type":"ui_tab","z":"","name":"Air Quality","icon":"dashboard"}]