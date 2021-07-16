/*  
  OpenMQTTGateway  - ESP8266 or Arduino program for home automation 

   Act as a wifi or ethernet gateway between your 433mhz/infrared IR signal  and a MQTT broker 
   Send and receiving command by MQTT
 
  This gateway enables to:
 - receive MQTT data from a topic and send RF 433Mhz signal corresponding to the received MQTT data
 - publish MQTT data to a different topic related to received 433Mhz signal

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
#include "User_config.h"

#ifdef ZgatewayRF

#  if defined(ESP32) || defined(ESP8266) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#    include <vector>
using namespace std;

static vector<SIGNAL_SIZE_UL_ULL> whitelisted_devices;
static vector<SIGNAL_SIZE_UL_ULL> blacklisted_devices;

#    define isWhitelistedDevice(id) checkIfDevicePresentInList(id, whitelisted_devices)
#    define isBlacklistedDevice(id) checkIfDevicePresentInList(id, blacklisted_devices)
#  endif

#  ifdef ZradioCC1101
#    include <ELECHOUSE_CC1101_SRC_DRV.h>
#  endif

#  include <RCSwitch.h> // library for controling Radio frequency switch

RCSwitch mySwitch = RCSwitch();

#  if defined(ESP32) || defined(ESP8266) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
bool checkIfDevicePresentInList(SIGNAL_SIZE_UL_ULL id, vector<SIGNAL_SIZE_UL_ULL>& list) {
  for (size_t i = 0; i < list.size(); i++) {
    if (list[i] == id) {
      return true;
    }
  }
  return false;
}

void refreshDevicesList(vector<SIGNAL_SIZE_UL_ULL>& dev_list, const char* key, JsonObject& data) {
  dev_list.clear();

  for (int i = 0; i < data[key].size(); i++) {
    const char* id = data[key][i];
    dev_list.push_back(STRTO_UL_ULL(id, NULL, 10));
  }
}
#  endif

#  if defined(ZmqttDiscovery) && !defined(RF_DISABLE_TRANSMIT)
void RFtoMQTTdiscovery(SIGNAL_SIZE_UL_ULL MQTTvalue) { //on the fly switch creation from received RF values
  char val[11];
  sprintf(val, "%lu", MQTTvalue);
  Log.trace(F("switchRFDiscovery" CR));
  char* switchRF[8] = {"switch", val, "", "", "", val, "", ""};
  //component type,name,availability topic,device class,value template,payload on, payload off, unit of measurement

  Log.trace(F("CreateDiscoverySwitch: %s" CR), switchRF[1]);
  createDiscovery(switchRF[0],
                  subjectRFtoMQTT, switchRF[1], (char*)getUniqueId(switchRF[1], switchRF[2]).c_str(),
                  will_Topic, switchRF[3], switchRF[4],
                  switchRF[5], switchRF[6], switchRF[7],
                  0, "", "", true, subjectMQTTtoRF,
                  "", "", "", "", false);
}
#  endif

void setupRF() {
  //RF init parameters
  Log.notice(F("RF_EMITTER_GPIO: %d " CR), RF_EMITTER_GPIO);
  Log.notice(F("RF_RECEIVER_GPIO: %d " CR), RF_RECEIVER_GPIO);
#  ifdef ZradioCC1101 //receiving with CC1101
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.SetRx(receiveMhz);
#  endif
#  ifdef RF_DISABLE_TRANSMIT
  mySwitch.disableTransmit();
#  else
  mySwitch.enableTransmit(RF_EMITTER_GPIO);
#  endif
  mySwitch.setRepeatTransmit(RF_EMITTER_REPEAT);
  mySwitch.enableReceive(RF_RECEIVER_GPIO);
  Log.trace(F("ZgatewayRF command topic: %s%s%s" CR), mqtt_topic, gateway_name, subjectMQTTtoRF);
  Log.trace(F("ZgatewayRF setup done" CR));
}

void RFtoMQTT() {
  if (mySwitch.available()) {
#  ifdef ZradioCC1101 //receiving with CC1101
    const int JSON_MSG_CALC_BUFFER = JSON_OBJECT_SIZE(5);
#  else
    const int JSON_MSG_CALC_BUFFER = JSON_OBJECT_SIZE(4);
#  endif
    StaticJsonBuffer<JSON_MSG_CALC_BUFFER> jsonBuffer;
    JsonObject& RFdata = jsonBuffer.createObject();
    Log.trace(F("Rcv. RF" CR));
#  ifdef ESP32
    Log.trace(F("RF Task running on core :%d" CR), xPortGetCoreID());
#  endif
    SIGNAL_SIZE_UL_ULL MQTTvalue = mySwitch.getReceivedValue();
    RFdata.set("value", (SIGNAL_SIZE_UL_ULL)MQTTvalue);
    RFdata.set("protocol", (int)mySwitch.getReceivedProtocol());
    RFdata.set("length", (int)mySwitch.getReceivedBitlength());
    RFdata.set("delay", (int)mySwitch.getReceivedDelay());
#  ifdef ZradioCC1101 // set Receive off and Transmitt on
    RFdata.set("mhz", receiveMhz);
#  endif
    mySwitch.resetAvailable();
#  if defined(ESP32) || defined(ESP8266) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    // if there are no whitelisted devices then proceed as usual
    if (!whitelisted_devices.empty() && !isWhitelistedDevice(MQTTvalue)) {
      return;
    }

    // if there are no blacklisted devices then proceed as usual
    if (!blacklisted_devices.empty() && isBlacklistedDevice(MQTTvalue)) {
      return;
    }
#  endif
    if (!isAduplicateSignal(MQTTvalue) && MQTTvalue != 0) { // conditions to avoid duplications of RF -->MQTT
#  if defined(ZmqttDiscovery) && !defined(RF_DISABLE_TRANSMIT) //component creation for HA
      if (disc)
        RFtoMQTTdiscovery(MQTTvalue);
#  endif
      pub(subjectRFtoMQTT, RFdata);
      // Casting "receivedSignal[o].value" to (unsigned long) because ArduinoLog doesn't support uint64_t for ESP's
      Log.trace(F("Store val: %u" CR), (unsigned long)MQTTvalue);
      storeSignalValue(MQTTvalue);
      if (repeatRFwMQTT) {
        Log.trace(F("Pub RF for rpt" CR));
        pub(subjectMQTTtoRF, RFdata);
      }
    }
  }
}

#  ifdef simpleReceiving
void MQTTtoRF(char* topicOri, char* datacallback) {
#    ifdef ZradioCC1101 // set Receive off and Transmitt on
  ELECHOUSE_cc1101.SetTx(receiveMhz);
  mySwitch.disableReceive();
  mySwitch.enableTransmit(RF_EMITTER_GPIO);
#    endif
  SIGNAL_SIZE_UL_ULL data = STRTO_UL_ULL(datacallback, NULL, 10); // we will not be able to pass values > 4294967295 on Arduino boards

  // RF DATA ANALYSIS
  //We look into the subject to see if a special RF protocol is defined
  String topic = topicOri;
  int valuePRT = 0;
  int valuePLSL = 0;
  int valueBITS = 0;
  int pos = topic.lastIndexOf(RFprotocolKey);
  if (pos != -1) {
    pos = pos + +strlen(RFprotocolKey);
    valuePRT = (topic.substring(pos, pos + 1)).toInt();
  }
  //We look into the subject to see if a special RF pulselength is defined
  int pos2 = topic.lastIndexOf(RFpulselengthKey);
  if (pos2 != -1) {
    pos2 = pos2 + strlen(RFpulselengthKey);
    valuePLSL = (topic.substring(pos2, pos2 + 3)).toInt();
  }
  int pos3 = topic.lastIndexOf(RFbitsKey);
  if (pos3 != -1) {
    pos3 = pos3 + strlen(RFbitsKey);
    valueBITS = (topic.substring(pos3, pos3 + 2)).toInt();
  }

  if ((cmpToMainTopic(topicOri, subjectMQTTtoRF)) && (valuePRT == 0) && (valuePLSL == 0) && (valueBITS == 0)) {
    Log.trace(F("MQTTtoRF dflt" CR));
    mySwitch.setProtocol(1, 350);
    mySwitch.send(data, 24);
    // Acknowledgement to the GTWRF topic
    pub(subjectGTWRFtoMQTT, datacallback);
  } else if ((valuePRT != 0) || (valuePLSL != 0) || (valueBITS != 0)) {
    Log.trace(F("MQTTtoRF usr par." CR));
    if (valuePRT == 0)
      valuePRT = 1;
    if (valuePLSL == 0)
      valuePLSL = 350;
    if (valueBITS == 0)
      valueBITS = 24;
    Log.notice(F("RF Protocol:%d" CR), valuePRT);
    Log.notice(F("RF Pulse Lgth: %d" CR), valuePLSL);
    Log.notice(F("Bits nb: %d" CR), valueBITS);
    mySwitch.setProtocol(valuePRT, valuePLSL);
    mySwitch.send(data, valueBITS);
    // Acknowledgement to the GTWRF topic
    pub(subjectGTWRFtoMQTT, datacallback); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
  }
#    ifdef ZradioCC1101 // set Receive on and Transmitt off
  ELECHOUSE_cc1101.SetRx(receiveMhz);
  mySwitch.disableTransmit();
  mySwitch.enableReceive(RF_RECEIVER_GPIO);
#    endif
}
#  endif

#  ifdef jsonReceiving
void MQTTtoRF(char* topicOri, JsonObject& RFdata) { // json object decoding
#    if defined(ESP32) || defined(ESP8266) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  if (cmpToMainTopic(topicOri, subjectMQTTtoRFset)) {
    const char* whiteListJsonKey = "white-list";
    const char* blackListJsonKey = "black-list";

    if (RFdata.containsKey(whiteListJsonKey)) {
      refreshDevicesList(whitelisted_devices, whiteListJsonKey, RFdata);
    } else if (RFdata.containsKey(blackListJsonKey)) {
      refreshDevicesList(blacklisted_devices, blackListJsonKey, RFdata);
    } else {
      pub(subjectGTWRFtoMQTT, "{\"Status\": \"config error\"}"); // Fail feedback
    }
  } else
#    endif
      if (cmpToMainTopic(topicOri, subjectMQTTtoRF)) {
    Log.trace(F("MQTTtoRF json" CR));
    SIGNAL_SIZE_UL_ULL data = RFdata["value"];
    if (data != 0) {
      int valuePRT = RFdata["protocol"] | 1;
      int valuePLSL = RFdata["delay"] | 350;
      int valueBITS = RFdata["length"] | 24;
      int valueRPT = RFdata["repeat"] | RF_EMITTER_REPEAT;
      Log.notice(F("RF Protocol:%d" CR), valuePRT);
      Log.notice(F("RF Pulse Lgth: %d" CR), valuePLSL);
      Log.notice(F("Bits nb: %d" CR), valueBITS);
#    ifdef ZradioCC1101 // set Receive off and Transmitt on
      float trMhz = RFdata["mhz"] | CC1101_FREQUENCY;
      if (validFrequency((int)trMhz)) {
        ELECHOUSE_cc1101.SetTx(trMhz);
        Log.notice(F("Transmit mhz: %F" CR), trMhz);
        mySwitch.disableReceive();
        mySwitch.enableTransmit(RF_EMITTER_GPIO);
      }
#    endif
      mySwitch.setRepeatTransmit(valueRPT);
      mySwitch.setProtocol(valuePRT, valuePLSL);
      mySwitch.send(data, valueBITS);
      Log.notice(F("MQTTtoRF OK" CR));
      pub(subjectGTWRFtoMQTT, RFdata); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
      mySwitch.setRepeatTransmit(RF_EMITTER_REPEAT); // Restore the default value
    } else {
      bool success = false;
      if (RFdata.containsKey("active")) {
        Log.trace(F("RF active:" CR));
        activeReceiver = ACTIVE_RF;
        success = true;
      }
#    ifdef ZradioCC1101 // set Receive on and Transmitt off
      float tempMhz = RFdata["mhz"];
      if (RFdata.containsKey("mhz") && validFrequency(tempMhz)) {
        receiveMhz = tempMhz;
        Log.notice(F("Receive mhz: %F" CR), receiveMhz);
        success = true;
      }
#    endif
      if (success) {
        pub(subjectGTWRFtoMQTT, RFdata); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
      } else {
#    ifndef ARDUINO_AVR_UNO // Space issues with the UNO
        pub(subjectGTWRFtoMQTT, "{\"Status\": \"Error\"}"); // Fail feedback
#    endif
        Log.error(F("MQTTtoRF Fail json" CR));
      }
    }
    enableActiveReceiver();
  }
}
#  endif

int receiveInterupt = -1;

void disableRFReceive() {
  Log.trace(F("disableRFReceive %d" CR), receiveInterupt);
  if (receiveInterupt != -1) {
    receiveInterupt = -1;
    mySwitch.disableReceive();
  }
}

void enableRFReceive() {
#  ifdef ZradioCC1101
  Log.notice(F("Switching to RF Receiver: %F" CR), receiveMhz);
#  else
  Log.notice(F("Switching to RF Receiver" CR));
#  endif
#  ifndef ARDUINO_AVR_UNO // Space issues with the UNO
#    ifdef ZgatewayPilight
  disablePilightReceive();
#    endif
#    ifdef ZgatewayRTL_433
  disableRTLreceive();
#    endif
#  endif
#  ifdef ZgatewayRF2
  disableRF2Receive();
#  endif

#  ifdef ZradioCC1101 // set Receive on and Transmitt off
  ELECHOUSE_cc1101.SetRx(receiveMhz);
#  endif
  mySwitch.disableTransmit();
  receiveInterupt = RF_RECEIVER_GPIO;
  mySwitch.enableReceive(RF_RECEIVER_GPIO);
}
#endif