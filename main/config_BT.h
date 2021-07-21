/*  
  OpenMQTTGateway  - ESP8266 or Arduino program for home automation 

   Act as a wifi or ethernet gateway between your 433mhz/infrared IR signal  and a MQTT broker 
   Send and receiving command by MQTT
 
   This files enables to set your parameter for the bluetooth low energy gateway (beacons detection)
  
    Copyright: (c)Florian ROBERT
  
    This file is part of OpenMQTTGateway.
    
    OpenMQTTGateway is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    OpenMQTTGateway is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef config_BT_h
#define config_BT_h

extern void setupBT();
extern bool BTtoMQTT();
extern void MQTTtoBT(char* topicOri, JsonObject& RFdata);
extern void emptyBTQueue();
extern void launchBTDiscovery();

#ifdef ESP32
extern int btQueueBlocked;
extern int btQueueLengthSum;
extern int btQueueLengthCount;
#  ifndef AttemptBLECOnnect
#    define AttemptBLECOnnect true //do we by default attempt a BLE connection to sensors with ESP32
#  endif
bool bleConnect = AttemptBLECOnnect;

// Sets whether to filter publishing of scanned devices that require a connection.
// Setting this to 1 prevents overwriting the publication of the device connection data with the advertised data (Recommended for use with OpenHAB).
#  ifndef BLE_FILTER_CONNECTABLE
#    define BLE_FILTER_CONNECTABLE 0
#  endif
#  include "NimBLEDevice.h"
#endif

/*----------------------BT topics & parameters-------------------------*/
#define subjectBTtoMQTT    "/BTtoMQTT"
#define subjectMQTTtoBTset "/commands/MQTTtoBT/config"
#define MinimumRSSI        -100 //default minimum rssi value, all the devices below -90 will not be reported

#ifndef Scan_duration
#  define Scan_duration 10000 //define the time for a scan
#endif
#ifndef BLEScanInterval
#  define BLEScanInterval 52 // How often the scan occurs / switches channels; in milliseconds,
#endif
#ifndef BLEScanWindow
#  define BLEScanWindow 30 // How long to scan during the interval; in milliseconds.
#endif
#ifndef ActiveBLEScan
#  define ActiveBLEScan true // Set active scanning, this will get more data from the advertiser.
#endif
#ifndef ScanBeforeConnect
#  define ScanBeforeConnect 10 //define number of scans before connecting to BLE devices (ESP32 only, minimum 1)
#endif
#ifndef BLEScanDuplicateCacheSize
#  define BLEScanDuplicateCacheSize 200
#endif
#ifndef TimeBtwRead
#  define TimeBtwRead 55555 //define default time between 2 scans
#endif
#ifndef PublishOnlySensors
#  define PublishOnlySensors false //false if we publish all BLE devices discovered or true only the identified sensors (like temperature sensors)
#endif
#ifndef HassPresence
#  define HassPresence false //false if we publish into Home Assistant presence topic
#endif

#ifndef BTQueueSize
#  define BTQueueSize 4 // lockless queue size for multi core cases (ESP32 currently)
#endif

#define HMSerialSpeed 9600 // Communication speed with the HM module, softwareserial doesn't support 115200
//#define HM_BLUE_LED_STOP true //uncomment to stop the blue led light of HM1X

#define BLEdelimiter       "4f4b2b444953413a" // OK+DISA:
#define BLEEndOfDiscovery  "4f4b2b4449534345" // OK+DISCE
#define BLEdelimiterLength 16
#define CRLR               "0d0a"
#define CRLR_Length        4
#define BLE_CNCT_TIMEOUT   3000

#define ServicedataMinLength 27

unsigned int BLEinterval = TimeBtwRead; //time between 2 scans
unsigned int BLEscanBeforeConnect = ScanBeforeConnect; //Number of BLE scans between connection cycles
unsigned long scanCount = 0;
bool publishOnlySensors = PublishOnlySensors;
bool hassPresence = HassPresence;

#ifndef pubKnownBLEServiceData
#  define pubKnownBLEServiceData false // define true if you want to publish service data belonging to recognised sensors
#endif

#ifndef pubUnknownBLEServiceData
#  define pubUnknownBLEServiceData true // define false if you don't want to publish service data to unrecognised sensors (in case you are having too heavy service data) https://github.com/1technophile/OpenMQTTGateway/issues/318#issuecomment-446064707
#endif

#ifndef pubBLEManufacturerData
#  define pubBLEManufacturerData false // define true if you want to publish the manufacturer's data (sometimes contains characters that aren't valid with receiving client)
#endif

#ifndef pubBLEServiceUUID
#  define pubBLEServiceUUID false // define true if you want to publish the service UUID data
#endif

/*-------------------HOME ASSISTANT ROOM PRESENCE ----------------------*/
#define subjectHomePresence "home_presence/" // will send Home Assistant room presence message to this topic (first part is same for all rooms, second is room name)

enum ble_sensor_model {
  UNKNOWN_MODEL = -1,
  BEGINING = 0,
  HHCCJCY01HHCC,
  VEGTRUG,
  LYWSDCGQ,
  JQJCY01YM,
  LYWSD02, //5
  CGG1,
  CGP1W,
  MUE4094RT,
  CGD1,
  MIBAND, //10
  XMTZC04HM,
  XMTZC05HM,
  IBSTH1,
  LYWSD03MMC,
  MHO_C401, //15
  LYWSD03MMC_ATC,
  INODE_EM,
  CGDK2,
  LYWSD03MMC_PVVX,
  CGH1, //20
  CGPR1,
  WS02,
  IBSTH2,
  IBT4XS,
  DT24, //25
  EDDYSTONE_TLM,
  MOKOBEACON,
  MOKOBEACONXPRO,
  IBEACON,
  TPMS,
  GENERIC,
};

/*-------------------PIN DEFINITIONS----------------------*/
#if !defined(BT_RX) || !defined(BT_TX)
#  ifdef ESP8266
#    define BT_RX 13 //D7 ESP8266 RX connect HM-10 or 11 TX
#    define BT_TX 12 //D6 ESP8266 TX connect HM-10 or 11 RX
#  elif defined(ESP32)
#    define BT_RX 18 // not tested
#    define BT_TX 19 // not tested
#  else
#    define BT_RX 5 //arduino RX connect HM-10 or 11 TX
#    define BT_TX 6 //arduino TX connect HM-10 or 11 RX
#  endif
#endif

/*---------------INTERNAL USE: DO NOT MODIFY--------------*/
#ifdef ESP32
enum ble_val_type {
  BLE_VAL_STRING = 0,
  BLE_VAL_HEX,
  BLE_VAL_INT,
  BLE_VAL_FLOAT,
};

struct BLEAction {
  std::string value;
  char addr[18];
  NimBLEUUID service;
  NimBLEUUID characteristic;
  bool write;
  bool complete;
  uint8_t ttl;
  ble_val_type value_type;
};
#endif

struct BLEdevice {
  char macAdr[18];
  bool isDisc;
  bool isWhtL;
  bool isBlkL;
  bool connect;
  ble_sensor_model sensorModel;
};

JsonObject& getBTJsonObject(const char* json = NULL, bool haPresenceEnabled = true);

#endif
