/*  
  OpenMQTTGateway  - ESP8266 or Arduino program for home automation 

   Act as a wifi or ethernet gateway between your 433mhz/infrared IR signal  and a MQTT broker 
   Send and receiving command by MQTT
 
  This gateway enables to:
 - publish MQTT data to a different topic related to received 433Mhz signal DIO/new kaku protocol

    Copyright: (c)Florian ROBERT
    Copyright: (c)Randy Simons http://randysimons.nl/
  
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

/*Command example for switching off:
sudo mosquitto_pub -t home/commands/MQTTtoRF2/CODE_8233372/UNIT_0/PERIOD_272 -m 0
Command example for switching on:
sudo mosquitto_pub -t home/commands/MQTTtoRF2/CODE_8233372/UNIT_0/PERIOD_272 -m 1
Command example for dimming:
sudo mosquitto_pub -t home/commands/MQTTtoRF2/CODE_8233372/UNIT_0/PERIOD_272 -m/DIM 8
*/
#include "User_config.h"

#ifdef ZgatewayRF2

#  ifdef ZradioCC1101
#    include <ELECHOUSE_CC1101_SRC_DRV.h>
#  endif

#  include <NewRemoteReceiver.h>
#  include <NewRemoteTransmitter.h>

struct RF2rxd {
  unsigned int period;
  unsigned long address;
  unsigned long groupBit;
  unsigned long unit;
  unsigned long switchType;
  bool hasNewData;
};

RF2rxd rf2rd;

void setupRF2() {
#  ifdef ZradioCC1101 //receiving with CC1101
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setMHZ(receiveMhz);
  ELECHOUSE_cc1101.SetRx(receiveMhz);
#  endif
  NewRemoteReceiver::init(RF_RECEIVER_GPIO, 2, rf2Callback);
  Log.notice(F("RF_EMITTER_GPIO: %d " CR), RF_EMITTER_GPIO);
  Log.notice(F("RF_RECEIVER_GPIO: %d " CR), RF_RECEIVER_GPIO);
  Log.trace(F("ZgatewayRF2 command topic: %s%s%s" CR), mqtt_topic, gateway_name, subjectMQTTtoRF2);
  Log.trace(F("ZgatewayRF2 setup done " CR));
  pinMode(RF_EMITTER_GPIO, OUTPUT);
  digitalWrite(RF_EMITTER_GPIO, LOW);
}

#  ifdef ZmqttDiscovery
//Register for autodiscover in Home Assistant
void RF2toMQTTdiscovery(JsonObject& data) {
  Log.trace(F("switchRF2Discovery" CR));
  String payloadonstr;
  String payloadoffstr;

  int org_switchtype = data["switchType"]; // Store original switchvalue
  data.set("switchType", 1); // switchtype = 1 turns switch on.
  data.printTo(payloadonstr);
  data.set("switchType", 0); // switchtype = 0 turns switch off.
  data.printTo(payloadoffstr);
  data.set("switchType", org_switchtype); // Restore original switchvalue

  String switchname;
  switchname = "RF2_" + String((int)data["unit"]) + "_" +
               String((int)data["groupbit"]) + "_" +
               String((unsigned long)data["address"]);

  char* switchRF[8] = {"switch",
                       (char*)switchname.c_str(),
                       "",
                       "",
                       "",
                       (char*)payloadonstr.c_str(),
                       (char*)payloadoffstr.c_str(),
                       ""};
  // component type,name,availability topic,device class,value template,payload
  // on, payload off, unit of measurement

  Log.trace(F("CreateDiscoverySwitch: %s" CR), switchRF[1]);

  // As RF2 433Mhz switches do not render their state, no state topic should be
  // provided in the discovery. This will cause the switch to be in optimistic
  // mode in HA with separate on and off icons.
  // The two separate on/off icons allow for subsequent on commands to support
  // the dimming feature of KAKU switches like ACM-300.
  createDiscovery(switchRF[0], "", switchRF[1],
                  (char*)getUniqueId(switchRF[1], "").c_str(), will_Topic,
                  switchRF[3], switchRF[4], switchRF[5], switchRF[6],
                  switchRF[7], 0, "", "", true, subjectMQTTtoRF2,
                  "", "", "", "", false);
}
#  endif

void RF2toMQTT() {
  if (rf2rd.hasNewData) {
    Log.trace(F("Creating RF2 buffer" CR));
    const int JSON_MSG_CALC_BUFFER = JSON_OBJECT_SIZE(5);
    StaticJsonBuffer<JSON_MSG_CALC_BUFFER> jsonBuffer;
    JsonObject& RF2data = jsonBuffer.createObject();

    rf2rd.hasNewData = false;

    Log.trace(F("Rcv. RF2" CR));
    RF2data.set("unit", (int)rf2rd.unit);
    RF2data.set("groupBit", (int)rf2rd.groupBit);
    RF2data.set("period", (int)rf2rd.period);
    RF2data.set("address", (unsigned long)rf2rd.address);
    RF2data.set("switchType", (int)rf2rd.switchType);
#  ifdef ZmqttDiscovery //component creation for HA
    if (disc)
      RF2toMQTTdiscovery(RF2data);
#  endif

    pub(subjectRF2toMQTT, RF2data);
  }
}

void rf2Callback(unsigned int period, unsigned long address, unsigned long groupBit, unsigned long unit, unsigned long switchType) {
  rf2rd.period = period;
  rf2rd.address = address;
  rf2rd.groupBit = groupBit;
  rf2rd.unit = unit;
  rf2rd.switchType = switchType;
  rf2rd.hasNewData = true;
}

#  ifdef simpleReceiving
void MQTTtoRF2(char* topicOri, char* datacallback) {
#    ifdef ZradioCC1101
  NewRemoteReceiver::disable();
  ELECHOUSE_cc1101.SetTx(CC1101_FREQUENCY); // set Transmit on
#    endif

  // RF DATA ANALYSIS
  //We look into the subject to see if a special RF protocol is defined
  String topic = topicOri;
  bool boolSWITCHTYPE;
  boolSWITCHTYPE = to_bool(datacallback);
  bool isDimCommand = false;

  long valueCODE = 0;
  int valueUNIT = -1;
  int valuePERIOD = 0;
  int valueGROUP = 0;
  int valueDIM = -1;

  int pos = topic.lastIndexOf(RF2codeKey);
  if (pos != -1) {
    pos = pos + +strlen(RF2codeKey);
    valueCODE = (topic.substring(pos, pos + 8)).toInt();
    Log.notice(F("RF2 code: %l" CR), valueCODE);
  }
  int pos2 = topic.lastIndexOf(RF2periodKey);
  if (pos2 != -1) {
    pos2 = pos2 + strlen(RF2periodKey);
    valuePERIOD = (topic.substring(pos2, pos2 + 3)).toInt();
    Log.notice(F("RF2 Period: %d" CR), valuePERIOD);
  }
  int pos3 = topic.lastIndexOf(RF2unitKey);
  if (pos3 != -1) {
    pos3 = pos3 + strlen(RF2unitKey);
    valueUNIT = (topic.substring(pos3, topic.indexOf("/", pos3))).toInt();
    Log.notice(F("Unit: %d" CR), valueUNIT);
  }
  int pos4 = topic.lastIndexOf(RF2groupKey);
  if (pos4 != -1) {
    pos4 = pos4 + strlen(RF2groupKey);
    valueGROUP = (topic.substring(pos4, pos4 + 1)).toInt();
    Log.notice(F("RF2 Group: %d" CR), valueGROUP);
  }
  int pos5 = topic.lastIndexOf(RF2dimKey);
  if (pos5 != -1) {
    isDimCommand = true;
    valueDIM = atoi(datacallback);
    Log.notice(F("RF2 Dim: %d" CR), valueDIM);
  }

  if ((topic == subjectMQTTtoRF2) || (valueCODE != 0) || (valueUNIT != -1) || (valuePERIOD != 0)) {
    Log.trace(F("MQTTtoRF2" CR));
    if (valueCODE == 0)
      valueCODE = 8233378;
    if (valueUNIT == -1)
      valueUNIT = 0;
    if (valuePERIOD == 0)
      valuePERIOD = 272;
    NewRemoteReceiver::disable();
    Log.trace(F("Creating transmitter" CR));
    NewRemoteTransmitter transmitter(valueCODE, RF_EMITTER_GPIO, valuePERIOD, RF2_EMITTER_REPEAT);
    Log.trace(F("Sending data" CR));
    if (valueGROUP) {
      if (isDimCommand) {
        transmitter.sendGroupDim(valueDIM);
      } else {
        transmitter.sendGroup(boolSWITCHTYPE);
      }
    } else {
      if (isDimCommand) {
        transmitter.sendDim(valueUNIT, valueDIM);
      } else {
        transmitter.sendUnit(valueUNIT, boolSWITCHTYPE);
      }
    }
    Log.trace(F("Data sent" CR));
    NewRemoteReceiver::enable();

    // Publish state change back to MQTT
    String MQTTAddress;
    String MQTTperiod;
    String MQTTunit;
    String MQTTgroupBit;
    String MQTTswitchType;
    String MQTTdimLevel;

    MQTTAddress = String(valueCODE);
    MQTTperiod = String(valuePERIOD);
    MQTTunit = String(valueUNIT);
    MQTTgroupBit = String(rf2rd.groupBit);
    MQTTswitchType = String(boolSWITCHTYPE);
    MQTTdimLevel = String(valueDIM);
    String MQTTRF2string;
    Log.trace(F("Adv data MQTTtoRF2 push state via RF2toMQTT" CR));
    if (isDimCommand) {
      MQTTRF2string = subjectRF2toMQTT + String("/") + RF2codeKey + MQTTAddress + String("/") + RF2unitKey + MQTTunit + String("/") + RF2groupKey + MQTTgroupBit + String("/") + RF2dimKey + String("/") + RF2periodKey + MQTTperiod;
      pub((char*)MQTTRF2string.c_str(), (char*)MQTTdimLevel.c_str());
    } else {
      MQTTRF2string = subjectRF2toMQTT + String("/") + RF2codeKey + MQTTAddress + String("/") + RF2unitKey + MQTTunit + String("/") + RF2groupKey + MQTTgroupBit + String("/") + RF2periodKey + MQTTperiod;
      pub((char*)MQTTRF2string.c_str(), (char*)MQTTswitchType.c_str());
    }
  }
#    ifdef ZradioCC1101
  ELECHOUSE_cc1101.SetRx(receiveMhz); // set Receive on
  NewRemoteReceiver::enable();
#    endif
}
#  endif

#  ifdef jsonReceiving
void MQTTtoRF2(char* topicOri, JsonObject& RF2data) { // json object decoding

  if (cmpToMainTopic(topicOri, subjectMQTTtoRF2)) {
    Log.trace(F("MQTTtoRF2 json" CR));
    int boolSWITCHTYPE = RF2data["switchType"] | 99;
    bool success = false;
    if (boolSWITCHTYPE != 99) {
#    ifdef ZradioCC1101
      NewRemoteReceiver::disable();
      ELECHOUSE_cc1101.SetTx(CC1101_FREQUENCY); // set Transmit on
#    endif
      Log.trace(F("MQTTtoRF2 switch type ok" CR));
      bool isDimCommand = boolSWITCHTYPE == 2;
      unsigned long valueCODE = RF2data["address"];
      int valueUNIT = RF2data["unit"] | -1;
      int valuePERIOD = RF2data["period"];
      int valueGROUP = RF2data["group"];
      int valueDIM = RF2data["dim"] | -1;
      if ((valueCODE != 0) || (valueUNIT != -1) || (valuePERIOD != 0)) {
        Log.trace(F("MQTTtoRF2" CR));
        if (valueCODE == 0)
          valueCODE = 8233378;
        if (valueUNIT == -1)
          valueUNIT = 0;
        if (valuePERIOD == 0)
          valuePERIOD = 272;
        NewRemoteReceiver::disable();
        NewRemoteTransmitter transmitter(valueCODE, RF_EMITTER_GPIO, valuePERIOD, RF2_EMITTER_REPEAT);
        Log.trace(F("Sending" CR));
        if (valueGROUP) {
          if (isDimCommand) {
            transmitter.sendGroupDim(valueDIM);
          } else {
            transmitter.sendGroup(boolSWITCHTYPE);
          }
        } else {
          if (isDimCommand) {
            transmitter.sendDim(valueUNIT, valueDIM);
          } else {
            transmitter.sendUnit(valueUNIT, boolSWITCHTYPE);
          }
        }
        Log.notice(F("MQTTtoRF2 OK" CR));
        NewRemoteReceiver::enable();

        success = true;
      }
    }
    if (RF2data.containsKey("active")) {
      Log.trace(F("RF2 active:" CR));
      activeReceiver = ACTIVE_RF2;
      success = true;
    }
#    ifdef ZradioCC1101 // set Receive on and Transmitt off
    float tempMhz = RF2data["mhz"];
    if (RF2data.containsKey("mhz") && validFrequency(tempMhz)) {
      receiveMhz = tempMhz;
      Log.notice(F("Receive mhz: %F" CR), receiveMhz);
      success = true;
    }
#    endif
    if (success) {
      pub(subjectGTWRF2toMQTT, RF2data); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
    } else {
#    ifndef ARDUINO_AVR_UNO // Space issues with the UNO
      pub(subjectGTWRF2toMQTT, "{\"Status\": \"Error\"}"); // Fail feedback
#    endif
      Log.error(F("MQTTtoRF2 failed json read" CR));
    }
    enableActiveReceiver();
  }
}
#  endif

void disableRF2Receive() {
  Log.trace(F("disableRF2Receive" CR));
  NewRemoteReceiver::deinit();
  NewRemoteReceiver::init(-1, 2, rf2Callback); // mark _interupt with -1
  NewRemoteReceiver::deinit();
}

void enableRF2Receive() {
#  ifdef ZradioCC1101
  Log.notice(F("Switching to RF2 Receiver: %F" CR), receiveMhz);
#  else
  Log.notice(F("Switching to RF2 Receiver" CR));
#  endif
#  ifdef ZgatewayPilight
  disablePilightReceive();
#  endif
#  ifdef ZgatewayRTL_433
  disableRTLreceive();
#  endif
#  ifdef ZgatewayRF
  disableRFReceive();
#  endif

  NewRemoteReceiver::init(RF_RECEIVER_GPIO, 2, rf2Callback);
#  ifdef ZradioCC1101
  ELECHOUSE_cc1101.SetRx(receiveMhz); // set Receive on
#  endif
}

#endif
