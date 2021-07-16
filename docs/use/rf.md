
# RF gateways  (433mhz/315mhz)

## Changing Active Receiver Modules

With version 0.9.7 the ability to switch active signal receiver and decoder is supported between RF, RF2, RTL_433 and Pilight receiver modules.

### Switching Active Receiver Module

Switching of the active receiver module is available between the RF, RF2, RTL_433 and Pilight Gateway modules, allowing for changing of signal decoders without redploying the openMQTTGateway package.  Sending a JSON message to the command topic of the desired receiver will change the active receiver module.

To enable the RF Gateway module send a json message to the RF Gateway module command subject with the key being 'active', and any value.  The value at this time is ignored. 

Example:
`mosquitto_pub -t "home/OpenMQTTGateway/commands/MQTTto433" -m '{"active":true}'`

To enable the PiLight Gateway module send a json message to the PiLight Gateway module command subject with the key being 'active', and any value.  The value at this time is ignored. 

Example:
`mosquitto_pub -t "home/OpenMQTTGateway/commands/MQTTtoPilight" -m '{"active":true}'`

To enable the RF2 Gateway module send a json message to the RF2 Gateway module command subject with the key being 'active', and any value.  The value at this time is ignored. 

Example:
`mosquitto_pub -t "home/OpenMQTTGateway/commands/MQTTtoRF2" -m '{"active":true}'`

To enable the RTL_433 Gateway module send a json message to the RTL_433 Gateway module command subject with the key being 'active', and any value.  The value at this time is ignored. 

Example:
`mosquitto_pub -t "home/OpenMQTTGateway/commands/MQTTtoRTL_433" -m '{"active":true}'`

### Status Messages

The openMQTTGateway status message contains a key `actRec` which is the current active receiver module.

1 - PiLight
2 - RF
3 - RTL_433
4 - RF2

## RCSwitch based gateway

### Receiving data from RF signal

Subscribe to all the messages with mosquitto or open your MQTT client software:

`    sudo mosquitto_sub -t +/# -v`

Generate your RF signals by pressing a remote button or other and you should see :

`home/OpenMQTTGateway/433toMQTT {"value":1315156,"protocol":1,"length":24,"delay":317}`

### Disabling Transmit function to safe a PIN

To disable transmit functions to allow the use of another pin, add the following to the config_rf.h file :

`#define RF_DISABLE_TRANSMIT`

### Send data by MQTT to convert it on RF signal 

`mosquitto_pub -t "home/OpenMQTTGateway/commands/MQTTto433" -m '{"value":1315156}'`

This command will send by RF the code 1315156 and use the default parameters (protocol 1, delay 350)

Arduino IDE serial data received when publishing data by MQTT

![](../img/OpenMQTTGateway_serial1.jpg)

We see that the Arduino receive the value 1315156 on the MQTT subject "MQTTto433" and send the data by RF

Arduino IDE serial data received when receiving data by 433Mhz

![](../img/OpenMQTTGateway_serial2.jpg)

### Send data by MQTT with advanced RF parameters

RF sending support three advanced parameters; bits length, RF protocol and RF pulselength
if you want to use a different RCswitch protocol put inside your payload the protocol number 2, "protocol":2.

if you want to use a pulselength 315 put inside your topic "delay":315

if you want to use a bits number different than 24 put inside your topic "length":24 for example

Example:
`mosquitto_pub -t "home/OpenMQTTGateway/commands/MQTTto433" -m '{"value":1315156,"protocol":2,"length":24,"delay":315}'`
will make RCSwitch use the protocol 2 with a pulselength of 315ms and a bits number of 24

### Repeat the RF signal OpenMQTTGateway receive
So as to repeat the RF signal received by the gateway once set the following parameter to true in config_RF.h

`#define repeatRFwMQTT true`

### Repeat the RF signal several times
You can add a "repeat" key/value to the MQTTto433 JSON message to override the default number of repeats.

Example:
`home/OpenMQTTGateway/commands/MQTTto433 {"value":1315156,"protocol":1,"length":24,"delay":317, "repeat":10}`

### Set Transmit and Receive Frequency of CC1101 Transceiver Module

Default transmit frequency of the CC1101 module is 433.92 Mhz, and this can be can changed by including the frequency in the transmit message.  Parameter is `mhz` and valid values are 300-348 Mhz, 387-464Mhz and 779-928Mhz.  Actual frequency support will depend on your CC1101 board.

`home/OpenMQTTGateway/commands/MQTTto433 {"value":1150,"protocol":6,"length":12,"delay":450,"repeat":8,"mhz":303.732}`

Default receive frequency of the CC1101 module is 433.92 Mhz, and this can be can changed by sending a message with the frequency.  Parameter is `mhz` and valid values are 300-348 Mhz, 387-464Mhz and 779-928Mhz.  Actual frequency support will depend on your CC1101 board

`home/OpenMQTTGateway/commands/MQTTto433 {"mhz":315.026}`

Messages received will include the frequency, and when transmitting on a different frequency the module return to the receive frequency afterwards.  ie transmit messages on 303.732 Mhz then receive messages on 433.92 Mhz 

`{"value":4534142,"protocol":6,"length":26,"delay":356,"mhz":315.026}`

### Whitelisting/Blacklisting devices

This happens based on the unique numeric id that is sent by each RF device. To configure whitelisting one has to write a specific json message on the designated MQTT config topic (a list of numeric id's):

Whitelisting example:

`home/OpenMQTTGateway_ESP32_RF/commands/MQTTto433/config {"white-list": ["110355","110361","110365"]}`

Blacklisting example:

`home/OpenMQTTGateway_ESP32_RF/commands/MQTTto433/config {"black-list": ["931859","931865"]}`

If one needs to disable whitelisting and/or blacklisting just send an empty list like this for each in turn:

Empty the whitelist (also disables whitelisting):

`home/OpenMQTTGateway_ESP32_RF/commands/MQTTto433/config {"white-list": []}`

Empty the blacklist (also disables blacklisting):

`home/OpenMQTTGateway_ESP32_RF/commands/MQTTto433/config {"black-list": []}`

**Notes**:

* The json message size is limited! This means that for each list (black/white) there's a limit and the **JSON_MSG_BUFFER** size must be adjusted in `User_config.h` accordingly for the corresponding platform **if and only if** it has enough **RAM** resources. One can evaluate the buffer size using the [ArduinoJson assistant](https://arduinojson.org/v5/assistant).
* Only `ESP32/ESP8266/ATMEGA1280/ATMEGA2560` MCUs are supported by this feature
* Only `ZgatewayRF` is supported by this feature for now (`-DZgatewayRF="RF"` build flag)

## Pilight gateway

### Receiving data from RF signal

Subscribe to all the messages with mosquitto or open your MQTT client software:

`    sudo mosquitto_sub -t +/# -v`

Generate your RF signals by pressing a remote button or other and you will see :

![](../img/OpenMQTTGateway_Pilight_Digoo-DG-R8S.png)

### Send data by MQTT to transmit a RF signal

#### Using a known protocol
**ON**
`mosquitto_pub -t "home/OpenMQTTGateway/commands/MQTTtoPilight" -m '{"message":"{\"systemcode\":12,\"unitcode\":22,\"on\":1}","protocol":"elro_400_switch"}'`

**OFF**
`mosquitto_pub -t "home/OpenMQTTGateway/commands/MQTTtoPilight" -m '{"message":"{\"systemcode\":12,\"unitcode\":22,\"off\":1}","protocol":"elro_400_switch"}'`

These commands will transmit by RF the signals to actuate an elro_400 switch.

#### Using a raw signal
You can transmit raw signal data by using the "raw" protocol. This uses the Pilight pulse train string format. One such example string, representing a transmission for Nexus protocol weather stations, looks like this: `c:03020202010102020102010101010101010202020201020102020202020101010201010202;p:500,1000,2000,4000;r:12@`. This string represents pulses and gaps directly.

Each number in the list after `p:` that ends with `;` stands for **p**ulse and gap lengths in microseconds (µs). In this example, we have a list containing lengths of 500µs, 1000µs, 2000µs, and 4000µs.

Each number after `c:` and ended by `;` represents a **c**ode that references the `p:` list by index. In this example, the first 4 numbers after `c:` are 0, 3, 0, and 2, which reference `p:`[0] = 500, `p:`[3] = 4000, `p:`[0] = 500, and `p:`[2] = 2000, respectively. In the language of digital radio transceiving, the most basic unit is usually a pulse and gap pair; in other words, 0s and 1s are represented by a pulse followed by a gap (lack of pulse) and the time lengths of these pulses and gaps. Different protocols have different pulse lengths and gap lengths representing 0, and a different one representing 1. Because of this pulse-gap nature, the codes in `c:` must be taken as pairs; the first number in a pair represents the length of the pulse, and the second number the subsequent gap. In this example, the first pair, 03, represents a pulse of 500µs followed by a gap of 4000µs. The next pair, 02, represents a pulse of 500µs followed by a gap of 2000µs.

The number after `r:` represents how many times the message in the string is to be **r**epeated. The `r:` block is optional. The default number of repeats if `r:` is not specified is 10. Greater than about 100 repeats will cause a crash due to memory usage. If this example were written without specifying repeats, it would look like this: `{"raw":"c:03020202010102020102010101010101010202020201020102020202020101010201010202;p:500,1000,2000,4000@"}`

The entire string must end in a `@`. Each block must end in a `;`, but if it is the last block in the string, the `@` replaces the `;`. Since the `r:` block is optional, this last block could be either `p:` or `r:`.

The JSON for the MQTT message to `home/OpenMQTTGateway/commands/MQTTtoPilight` should specify the pulse train string as the value for the "raw" key: `{"raw":"c:03020202010102020102010101010101010202020201020102020202020101010201010202;p:500,1000,2000,4000;r:12@"}`.

e.g. `mosquitto_pub -t "home/OpenMQTTGateway/commands/MQTTtoPilight" -m '{"raw":"c:03020202010102020102010101010101010202020201020102020202020101010201010202;p:500,1000,2000,4000;r:12@"}'`

## RF with SONOFF RF BRIDGE
### Receiving data from RF signal

Subscribe to all the messages with mosquitto or open your MQTT client software:

`    sudo mosquitto_sub -t +/# -v`

Generate your RF signals by pressing a remote button or other and you will see:
```
home/OpenMQTTGateway/SRFBtoMQTT {"raw":"2B660186042E00E7E5","value":"59365","delay":"1111","val_Thigh":"390","val_Tlow":"1070"}
```

The first parameter is the raw value extracted from the RF module of the Sonoff bridge. The data are in hexadecimal and correspond to the details below:
https://www.itead.cc/wiki/images/5/5e/RF_Universal_Transeceive_Module_Serial_Protocol_v1.0.pdf
OpenMQTTGateway process the raw value to extract the other decimal values that can be reused to reproduce a signal (raw value can also be reused).

NOTE: currently the device doesn't receive correct values from Switches remote control

### Send data by MQTT to convert it on RF signal 
`mosquitto_pub -t "home/OpenMQTTGateway/commands/MQTTtoSRFB" -m '{"value":1315156}'`

This command will send by RF the code 1315156 and use the default parameters:
Repeat = 1
Low time= 320
High time= 900
SYNC = 9500

### Send data by MQTT with advanced RF parameters

RF bridge sending support four advanced parameters; Repeat, Low time, High time & Sync
if you want to repeat your signal sending put into your json payload "repeat":2, 2 means 2 repetitions of signal

if you want to use a low time of 315 put inside your json payload "Tlow":315

if you want to use a high time of 845 put inside your json payload "Thigh":845

if you want to use a sync time of 9123 put inside your json payload "Tsyn":9123 

Example:
`mosquitto_pub -t home/OpenMQTTGateway/commands/MQTTtoSRFB/Tlow_315/Thigh_845/Tsyn_9123 -m '{"value":"33151562","delay":"9123","val_Thigh":"845","val_Tlow":"315"}'`
will make RF Bridge send a signal with the use of listed parameters 315, 845, 9123...

`mosquitto_pub -t home/OpenMQTTGateway/commands/MQTTtoSRFB/Raw -m '{"raw":"267A013603B6140551"}'`
will make RF Bridge send a signal with the use of advanced parameters defined in the raw string

## RF2 gateway KAKU
RF2 gateway enables to send command to RF devices with the KAKU protocol. DIO chacon devices are an example.
It uses the same pinout as the RF gateway and both gateways can be used on the same setup.

Receiving RF codes with the KAKU protocol is not compatible with ZgatewayRF , so as to get the code of your remotes you should comment ZgatewayRF in User_config.h.
Transmitting can be done with both ZgatewayRF and ZgatewayRF2

### Receiving data from KAKU signal

Subscribe to all the messages with mosquitto or open your MQTT client software:

`    sudo mosquitto_sub -t +/# -v`

Generate your RF signals by pressing a remote button or other and you will see :

`home/OpenMQTTGateway/RF2toMQTT {"unit":0,"groupBit":0,"period":273,"address":8233228,"switchType":0}`

### Send data by MQTT to convert it on KAKU signal 

Once you get the infos publish the parameters with mqtt like that for off:

`mosquitto_pub -t home/OpenMQTTGateway/commands/MQTTtoRF2 -m "{"unit":0,"groupBit":0,"period":273,"address":8233228,"switchType":0}"`

for on:

`mosquitto_pub -t home/OpenMQTTGateway/commands/MQTTtoRF2 -m "{"unit":0,"groupBit":0,"period":273,"adress":8233228,"switchType":1}"`

## rtl_433 device decoders

This feature is only available on a ESP32 based device with a CC1101 transceiver connected due to the resource requirements of the rtl_433 device decoders.  At the present time only Pulse Position Modulation (OOK_PPM) and Pulse Width Modulation (OOK_PWM) based decoders are available.

```
Registering protocol [2] "Acurite 609TXC Temperature and Humidity Sensor"
Registering protocol [3] "Acurite 592TXR Temp/Humidity, 5n1 Weather Station, 6045 Lightning, 3N1, Atlas"
Registering protocol [4] "Acurite 986 Refrigerator / Freezer Thermometer"
Registering protocol [5] "Acurite 606TX Temperature Sensor"
Registering protocol [6] "Acurite 00275rm,00276rm Temp/Humidity with optional probe"
Registering protocol [7] "Acurite 590TX Temperature with optional Humidity"
Registering protocol [8] "Akhan 100F14 remote keyless entry"
Registering protocol [9] "AlectoV1 Weather Sensor (Alecto WS3500 WS4500 Ventus W155/W044 Oregon)"
Registering protocol [10] "Ambient Weather TX-8300 Temperature/Humidity Sensor"
Registering protocol [11] "Auriol AFW2A1 temperature/humidity sensor"
Registering protocol [12] "Auriol HG02832, HG05124A-DCF, Rubicson 48957 temperature/humidity sensor"
Registering protocol [13] "BlueLine Power Monitor"
Registering protocol [14] "Blyss DC5-UK-WH"
Registering protocol [16] "Bresser Thermo-/Hygro-Sensor 3CH"
Registering protocol [18] "Burnhard BBQ thermometer"
Registering protocol [19] "Calibeur RF-104 Sensor"
Registering protocol [20] "Cardin S466-TX2"
Registering protocol [21] "Chuango Security Technology"
Registering protocol [22] "Companion WTR001 Temperature Sensor"
Registering protocol [25] "Ecowitt Wireless Outdoor Thermometer WH53/WH0280/WH0281A"
Registering protocol [26] "Eurochron EFTH-800 temperature and humidity sensor"
Registering protocol [30] "Esperanza EWS"
Registering protocol [32] "Fine Offset Electronics, WH2, WH5, Telldus Temperature/Humidity/Rain Sensor"
Registering protocol [33] "Fine Offset Electronics, WH0530 Temperature/Rain Sensor"
Registering protocol [34] "Fine Offset WH1050 Weather Station"
Registering protocol [35] "Fine Offset Electronics WH1080/WH3080 Weather Station"
Registering protocol [37] "FT-004-B Temperature Sensor"
Registering protocol [38] "Generic wireless motion sensor"
Registering protocol [39] "Generic Remote SC226x EV1527"
Registering protocol [40] "Generic temperature sensor 1"
Registering protocol [41] "Globaltronics QUIGG GT-TMBBQ-05"
Registering protocol [42] "Globaltronics GT-WT-02 Sensor"
Registering protocol [43] "Globaltronics GT-WT-03 Sensor"
Registering protocol [44] "Microchip HCS200 KeeLoq Hopping Encoder based remotes"
Registering protocol [45] "Honeywell ActivLink, Wireless Doorbell"
Registering protocol [46] "HT680 Remote control"
Registering protocol [47] "inFactory, nor-tec, FreeTec NC-3982-913 temperature humidity sensor"
Registering protocol [49] "Interlogix GE UTC Security Devices"
Registering protocol [51] "Kedsum Temperature & Humidity Sensor, Pearl NC-7415"
Registering protocol [52] "Kerui PIR / Contact Sensor"
Registering protocol [53] "LaCrosse TX Temperature / Humidity Sensor"
Registering protocol [54] "LaCrosse TX141-Bv2, TX141TH-Bv2, TX141-Bv3, TX141W, TX145wsdth sensor"
Registering protocol [55] "LaCrosse/ELV/Conrad WS7000/WS2500 weather sensors"
Registering protocol [56] "LaCrosse WS-2310 / WS-3600 Weather Station"
Registering protocol [58] "Maverick et73"
Registering protocol [60] "Missil ML0757 weather station"
Registering protocol [64] "Nexus, FreeTec NC-7345, NX-3980, Solight TE82S, TFA 30.3209 temperature/humidity sensor"
Registering protocol [66] "Opus/Imagintronix XT300 Soil Moisture"
Registering protocol [67] "Oregon Scientific SL109H Remote Thermal Hygro Sensor"
Registering protocol [69] "Philips outdoor temperature sensor (type AJ3650)"
Registering protocol [70] "Philips outdoor temperature sensor (type AJ7010)"
Registering protocol [71] "Prologue, FreeTec NC-7104, NC-7159-675 temperature sensor"
Registering protocol [73] "Quhwa"
Registering protocol [75] "Rubicson Temperature Sensor"
Registering protocol [76] "Rubicson 48659 Thermometer"
Registering protocol [77] "Conrad S3318P, FreeTec NC-5849-913 temperature humidity sensor"
Registering protocol [78] "Silvercrest Remote Control"
Registering protocol [79] "Skylink HA-434TL motion sensor"
Registering protocol [80] "Wireless Smoke and Heat Detector GS 558"
Registering protocol [81] "Solight TE44/TE66, EMOS E0107T, NX-6876-917"
Registering protocol [82] "Springfield Temperature and Soil Moisture"
Registering protocol [83] "TFA Dostmann 30.3221.02 T/H Outdoor Sensor"
Registering protocol [84] "TFA Drop Rain Gauge 30.3233.01"
Registering protocol [85] "TFA pool temperature sensor"
Registering protocol [86] "TFA-Twin-Plus-30.3049, Conrad KW9010, Ea2 BL999"
Registering protocol [87] "Thermopro TP11 Thermometer"
Registering protocol [88] "Thermopro TP08/TP12/TP20 thermometer"
Registering protocol [90] "TS-FT002 Wireless Ultrasonic Tank Liquid Level Meter With Temperature Sensor"
Registering protocol [91] "Visonic powercode"
Registering protocol [92] "Waveman Switch Transmitter"
Registering protocol [93] "WG-PB12V1 Temperature Sensor"
Registering protocol [94] "WS2032 weather station"
Registering protocol [95] "Hyundai WS SENZOR Remote Temperature Sensor"
Registering protocol [96] "WT0124 Pool Thermometer"
Registering protocol [98] "X10 Security"
```

### Change receive frequency

Default receive frequency of the CC1101 module is 433.92 Mhz, and this can be can changed by sending a message with the frequency.  Parameter is `mhz` and valid values are 300-348 Mhz, 387-464Mhz and 779-928Mhz.  Actual frequency support will depend on your CC1101 board

`home/OpenMQTTGateway/commands/MQTTtoRTL_433 {"mhz":315.026}`

### Minimum Signal Strength

Default minimum signal strength to enable the receiver is -82, and this setting can be changed with the following command.

`home/OpenMQTTGateway/commands/MQTTtoRTL_433 {"rssi":-75}`

### Enable rtl_433 device decoder verbose debug

This function does not work when all available decoders are enabled and triggers an out of memory restart.

`home/OpenMQTTGateway/commands/MQTTtoRTL_433 {"debug":4}`

### Retrieve current status of receiver

`home/OpenMQTTGateway/commands/MQTTtoRTL_433 {"status":1}`

```
{"model":"status",
"protocol":"debug",
"debug":0,                  - rtl_433 verbose setting
"duration":11799327,        - duration of current signal
"Gap length":-943575,       - duration of gap between current signal
"signalRssi":-38,           - most recent received signal strength
"train":1,                  - signal processing train #
"messageCount":3,           - total number of signals received
"_enabledReceiver":1,       - which recevier is enabled
"receiveMode":0,            - is the receiver currently receiving a signal
"currentRssi":-89,          - current rssi level
"minimumRssi":-82,          - minimum rssi level to start signal processing
"pulses":0,                 - how many pulses have been recieved in the current signal
"StackHighWaterMark":5528,  - ESP32 Stack
"freeMem":112880}           - ESP32 memory available
```
