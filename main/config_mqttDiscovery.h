/*
  OpenMQTTGateway  - ESP8266 or Arduino program for home automation

   Act as a wifi or ethernet gateway between your 433mhz/infrared IR signal  and a MQTT broker
   Send and receiving command by MQTT

   This files enables to set your parameter for the Home assistant mqtt Discovery

    Copyright: (c) Rafal Herok

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
#ifndef config_mqttDiscovery_h
#define config_mqttDiscovery_h

extern String getUniqueId(String name, String sufix);
extern void pubMqttDiscovery();
extern void createDiscoveryFromList(const char* mac, const char* sensorList[][8], int sensorCount,
                                    const char* device_name, const char* device_manufacturer, const char* device_model);
extern void createDiscovery(const char* sensor_type,
                            const char* state_topic, const char* s_name, const char* unique_id,
                            const char* availability_topic, const char* device_class, const char* value_template,
                            const char* payload_on, const char* payload_off, const char* unit_of_meas,
                            int off_delay,
                            const char* payload_available, const char* payload_not_avalaible, bool gateway_entity, const char* command_topic,
                            const char* device_name, const char* device_manufacturer, const char* device_model, const char* device_mac, bool retainCmd);

#define discovery_Topic "homeassistant"

#define DEVICEMANUFACTURER "OMG_community"

/*-------------- Auto discovery macros-----------------*/
// Set the line below to true so as to have autodiscovery working with OpenHAB
#define OpenHABDiscovery false

#if OpenHABDiscovery // OpenHAB autodiscovery value key definition (is defined command is not supported by OpenHAB)
#  define jsonBatt     "{{ value_json.batt }}"
#  define jsonLux      "{{ value_json.lux }}"
#  define jsonPres     "{{ value_json.pres }}"
#  define jsonFer      "{{ value_json.fer }}"
#  define jsonFor      "{{ value_json.for }}"
#  define jsonMoi      "{{ value_json.moi }}"
#  define jsonHum      "{{ value_json.hum }}"
#  define jsonStep     "{{ value_json.steps }}"
#  define jsonWeight   "{{ value_json.weight }}"
#  define jsonPresence "{{ value_json.presence }}"
#  define jsonAltim    "{{ value_json.altim }}"
#  define jsonAltif    "{{ value_json.altift }}"
#  define jsonTempc    "{{ value_json.tempc }}"
#  define jsonTempc2   "{{ value_json.tempc2 }}"
#  define jsonTempc3   "{{ value_json.tempc3 }}"
#  define jsonTempc4   "{{ value_json.tempc4 }}"
#  define jsonTempf    "{{ value_json.tempf }}"
#  define jsonMsg      "{{ value_json.message }}"
#  define jsonVal      "{{ value_json.value }}"
#  define jsonVolt     "{{ value_json.volt }}"
#  define jsonCurrent  "{{ value_json.current }}"
#  define jsonPower    "{{ value_json.power }}"
#  define jsonEnergy   "{{ value_json.energy }}"
#  define jsonGpio     "{{ value_json.gpio }}"
#  define jsonFtcd     "{{ value_json.ftcd }}"
#  define jsonWm2      "{{ value_json.wattsm2 }}"
#  define jsonAdc      "{{ value_json.adc }}"
#  define jsonPa       "{{ float(value_json.pa) * 0.01 }}"
#  define jsonId       "{{ value_json.id }}"
#  define jsonAddress  "{{ value_json.address }}"
#  define jsonOpen     "{{ value_json.open }}"
#  define jsonTime     "{{ value_json.time }}"
#  define jsonCount    "{{ value_json.count }}"
#  define jsonAlarm    "{{ value_json.alarm }}"
#else // Home assistant autodiscovery value key definition
#  define jsonBatt     "{{ value_json.batt | is_defined }}"
#  define jsonLux      "{{ value_json.lux | is_defined }}"
#  define jsonPres     "{{ value_json.pres | is_defined }}"
#  define jsonFer      "{{ value_json.fer | is_defined }}"
#  define jsonFor      "{{ value_json.for | is_defined }}"
#  define jsonMoi      "{{ value_json.moi | is_defined }}"
#  define jsonHum      "{{ value_json.hum | is_defined }}"
#  define jsonStep     "{{ value_json.steps | is_defined }}"
#  define jsonWeight   "{{ value_json.weight | is_defined }}"
#  define jsonPresence "{{ value_json.presence | is_defined }}"
#  define jsonAltim    "{{ value_json.altim | is_defined }}"
#  define jsonAltif    "{{ value_json.altift | is_defined }}"
#  define jsonTempc    "{{ value_json.tempc | is_defined }}"
#  define jsonTempc2   "{{ value_json.tempc2 | is_defined }}"
#  define jsonTempc3   "{{ value_json.tempc3 | is_defined }}"
#  define jsonTempc4   "{{ value_json.tempc4 | is_defined }}"
#  define jsonTempf    "{{ value_json.tempf | is_defined }}"
#  define jsonMsg      "{{ value_json.message | is_defined }}"
#  define jsonVal      "{{ value_json.value | is_defined }}"
#  define jsonVolt     "{{ value_json.volt | is_defined }}"
#  define jsonCurrent  "{{ value_json.current | is_defined }}"
#  define jsonPower    "{{ value_json.power | is_defined }}"
#  define jsonEnergy   "{{ value_json.energy | is_defined }}"
#  define jsonGpio     "{{ value_json.gpio | is_defined }}"
#  define jsonFtcd     "{{ value_json.ftcd | is_defined }}"
#  define jsonWm2      "{{ value_json.wattsm2 | is_defined }}"
#  define jsonAdc      "{{ value_json.adc | is_defined }}"
#  define jsonPa       "{{ float(value_json.pa) * 0.01 | is_defined }}"
#  define jsonId       "{{ value_json.id | is_defined }}"
#  define jsonAddress  "{{ value_json.address | is_defined }}"
#  define jsonOpen     "{{ value_json.open | is_defined }}"
#  define jsonTime     "{{ value_json.time | is_defined }}"
#  define jsonCount    "{{ value_json.count | is_defined }}"
#  define jsonAlarm    "{{ value_json.alarm | is_defined }}"
#endif

#endif
