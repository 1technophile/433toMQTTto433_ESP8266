#ifdef ESP32
#  include "User_config.h"
#  ifdef ZgatewayBT
#    include "ArduinoJson.h"
#    include "ArduinoLog.h"
#    include "ZgatewayBLEConnect.h"

#    define convertTemp_CtoF(c) ((c * 1.8) + 32)

extern bool ProcessLock;
extern std::vector<BLEdevice*> devices;

NimBLERemoteCharacteristic* zBLEConnect::getCharacteristic(const NimBLEUUID& service,
                                                           const NimBLEUUID& characteristic) {
  BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
  if (!m_pClient) {
    Log.error(F("No BLE client" CR));
  } else if (!m_pClient->isConnected() && !m_pClient->connect()) {
    Log.error(F("Connect to: %s failed" CR), m_pClient->getPeerAddress().toString().c_str());
  } else {
    BLERemoteService* pRemoteService = m_pClient->getService(service);
    if (!pRemoteService) {
      Log.notice(F("Failed to find service UUID: %s" CR), service.toString().c_str());
    } else {
      Log.trace(F("Found service: %s" CR), service.toString().c_str());
      Log.trace(F("Client isConnected, freeHeap: %d" CR), ESP.getFreeHeap());
      pRemoteCharacteristic = pRemoteService->getCharacteristic(characteristic);
      if (!pRemoteCharacteristic) {
        Log.notice(F("Failed to find characteristic UUID: %s" CR), characteristic.toString().c_str());
      }
    }
  }

  return pRemoteCharacteristic;
}

bool zBLEConnect::writeData(BLEAction* action) {
  NimBLERemoteCharacteristic* pChar = getCharacteristic(action->service, action->characteristic);
  if (pChar && pChar->canWrite()) {
    switch (action->value_type) {
      case BLE_VAL_HEX: {
        int len = action->value.length();
        if (len % 2) {
          Log.error(F("Invalid HEX value length" CR));
          return false;
        }

        std::vector<uint8_t> buf;
        for (auto i = 0; i < len; i += 2) {
          std::string temp = action->value.substr(i, 2);
          buf.push_back((uint8_t)strtoul(temp.c_str(), nullptr, 16));
        }
        return pChar->writeValue((const uint8_t*)&buf[0], buf.size());
      }
      case BLE_VAL_INT:
        return pChar->writeValue(strtol(action->value.c_str(), nullptr, 0));
      case BLE_VAL_FLOAT:
        return pChar->writeValue(strtod(action->value.c_str(), nullptr));
      default:
        return pChar->writeValue(action->value);
    }
  }
  return false;
}

bool zBLEConnect::readData(BLEAction* action) {
  NimBLERemoteCharacteristic* pChar = getCharacteristic(action->service, action->characteristic);

  if (pChar && pChar->canRead()) {
    action->value = pChar->readValue();
    if (action->value != "") {
      return true;
    }
  }
  return false;
}

bool zBLEConnect::processActions(std::vector<BLEAction>& actions) {
  bool result = false;
  if (actions.size() > 0) {
    for (auto& it : actions) {
      if (NimBLEAddress(it.addr) == m_pClient->getPeerAddress()) {
        JsonObject& BLEresult = getBTJsonObject();
        BLEresult.set("id", it.addr);
        BLEresult.set("service", (char*)it.service.toString().c_str());
        BLEresult.set("characteristic", (char*)it.characteristic.toString().c_str());

        if (it.write) {
          Log.trace(F("processing BLE write" CR));
          BLEresult.set("write", it.value.c_str());
          result = writeData(&it);
        } else {
          Log.trace(F("processing BLE read" CR));
          result = readData(&it);
          if (result) {
            switch (it.value_type) {
              case BLE_VAL_HEX: {
                char* pHex = NimBLEUtils::buildHexData(nullptr, (uint8_t*)it.value.c_str(), it.value.length());
                BLEresult.set("read", pHex);
                free(pHex);
                break;
              }
              case BLE_VAL_INT: {
                int ival = *(int*)it.value.data();
                BLEresult.set<int>("read", ival);
                break;
              }
              case BLE_VAL_FLOAT: {
                float fval = *(double*)it.value.data();
                BLEresult.set<float>("read", fval);
                break;
              }
              default:
                BLEresult.set("read", it.value.c_str());
                break;
            }
          }
        }

        it.complete = true;
        BLEresult.set("success", result);
        pubBT(BLEresult);
      }
    }
  }

  return result;
}

/*-----------------------LYWSD03MMC && MHO_C401 HANDLING-----------------------*/
void LYWSD03MMC_connect::notifyCB(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (m_taskHandle == nullptr) {
    return; // unexpected notification
  }
  if (!ProcessLock) {
    Log.trace(F("Callback from %s characteristic" CR), pChar->getUUID().toString().c_str());

    if (length == 5) {
      Log.trace(F("Device identified creating BLE buffer" CR));
      JsonObject& BLEdata = getBTJsonObject();
      String mac_address = m_pClient->getPeerAddress().toString().c_str();
      mac_address.toUpperCase();
      for (std::vector<BLEdevice*>::iterator it = devices.begin(); it != devices.end(); ++it) {
        BLEdevice* p = *it;
        if ((strcmp(p->macAdr, (char*)mac_address.c_str()) == 0)) {
          if (p->sensorModel == LYWSD03MMC)
            BLEdata.set("model", "LYWSD03MMC");
          else if (p->sensorModel == MHO_C401)
            BLEdata.set("model", "MHO_C401");
        }
      }
      BLEdata.set("id", (char*)mac_address.c_str());
      Log.trace(F("Device identified in CB: %s" CR), (char*)mac_address.c_str());
      BLEdata.set("tempc", (float)((pData[0] | (pData[1] << 8)) * 0.01));
      BLEdata.set("tempf", (float)(convertTemp_CtoF((pData[0] | (pData[1] << 8)) * 0.01)));
      BLEdata.set("hum", (float)(pData[2]));
      BLEdata.set("volt", (float)(((pData[4] * 256) + pData[3]) / 1000.0));
      BLEdata.set("batt", (float)(((((pData[4] * 256) + pData[3]) / 1000.0) - 2.1) * 100));

      pubBT(BLEdata);
    } else {
      Log.notice(F("Invalid notification data" CR));
      return;
    }
  } else {
    Log.trace(F("Callback process canceled by processLock" CR));
  }

  xTaskNotifyGive(m_taskHandle);
}

void LYWSD03MMC_connect::publishData() {
  NimBLEUUID serviceUUID("ebe0ccb0-7a0a-4b0c-8a1a-6ff2997da3a6");
  NimBLEUUID charUUID("ebe0ccc1-7a0a-4b0c-8a1a-6ff2997da3a6");
  NimBLERemoteCharacteristic* pChar = getCharacteristic(serviceUUID, charUUID);

  if (pChar && pChar->canNotify()) {
    Log.trace(F("Registering notification" CR));
    if (pChar->subscribe(true, std::bind(&LYWSD03MMC_connect::notifyCB, this,
                                         std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3, std::placeholders::_4))) {
      m_taskHandle = xTaskGetCurrentTaskHandle();
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BLE_CNCT_TIMEOUT)) == pdFALSE) {
        m_taskHandle = nullptr;
      }
    } else {
      Log.notice(F("Failed registering notification" CR));
    }
  }
}

/*-----------------------DT24 HANDLING-----------------------*/
void DT24_connect::notifyCB(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (m_taskHandle == nullptr) {
    return; // unexpected notification
  }

  if (!ProcessLock) {
    Log.trace(F("Callback from %s characteristic" CR), pChar->getUUID().toString().c_str());
    if (length == 20) {
      m_data.assign(pData, pData + length);
      return;
    } else if (m_data.size() == 20 && length == 16) {
      m_data.insert(m_data.end(), pData, pData + length);

      // DT24-BLE data format
      // https://github.com/NiceLabs/atorch-console/blob/master/docs/protocol-design.md#dc-meter-report
      // Data comes as two packets ( 20 and 16 ), and am only processing first
      Log.trace(F("Device identified creating BLE buffer" CR));
      JsonObject& BLEdata = getBTJsonObject();
      String mac_address = m_pClient->getPeerAddress().toString().c_str();
      mac_address.toUpperCase();
      BLEdata.set("model", "DT24");
      BLEdata.set("id", (char*)mac_address.c_str());
      Log.trace(F("Device identified in CB: %s" CR), (char*)mac_address.c_str());
      BLEdata.set("volt", (float)(((m_data[4] * 256 * 256) + (m_data[5] * 256) + m_data[6]) / 10.0));
      BLEdata.set("current", (float)(((m_data[7] * 256 * 256) + (m_data[8] * 256) + m_data[9]) / 1000.0));
      BLEdata.set("power", (float)(((m_data[10] * 256 * 256) + (m_data[11] * 256) + m_data[12]) / 10.0));
      BLEdata.set("energy", (float)(((m_data[13] * 256 * 256 * 256) + (m_data[14] * 256 * 256) + (m_data[15] * 256) + m_data[16]) / 100.0));
      BLEdata.set("price", (float)(((m_data[17] * 256 * 256) + (m_data[18] * 256) + m_data[19]) / 100.0));
      BLEdata.set("tempc", (float)(m_data[24] * 256) + m_data[25]);
      BLEdata.set("tempf", (float)(convertTemp_CtoF((m_data[24] * 256) + m_data[25])));

      pubBT(BLEdata);
    } else {
      Log.notice(F("Invalid notification data" CR));
      return;
    }
  } else {
    Log.trace(F("Callback process canceled by processLock" CR));
  }

  xTaskNotifyGive(m_taskHandle);
}

void DT24_connect::publishData() {
  NimBLEUUID serviceUUID("ffe0");
  NimBLEUUID charUUID("ffe1");
  NimBLERemoteCharacteristic* pChar = getCharacteristic(serviceUUID, charUUID);

  if (pChar && pChar->canNotify()) {
    Log.trace(F("Registering notification" CR));
    if (pChar->subscribe(true, std::bind(&DT24_connect::notifyCB, this,
                                         std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3, std::placeholders::_4))) {
      m_taskHandle = xTaskGetCurrentTaskHandle();
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BLE_CNCT_TIMEOUT)) == pdFALSE) {
        m_taskHandle = nullptr;
      }
    } else {
      Log.notice(F("Failed registering notification" CR));
    }
  }
}

/*-----------------------SWITCHBOT EXPERIMENTAL-----------------------*/
/* This is experimental and should be used for testing only.
 * At this time no testing has been done so this is only an example of extendability.
 */
void switchbot_connect::notifyCB(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (!ProcessLock) {
    Log.trace(F("Callback from %s characteristic" CR), pChar->getUUID().toString().c_str());
    if (pData[0] == 1) {
      Log.trace(F("Device identified creating BLE buffer" CR));
      JsonObject& BLEdata = getBTJsonObject();
      String mac_adress = m_pClient->getPeerAddress().toString().c_str();
      mac_adress.toUpperCase();

      BLEdata.set("model", "switchbot");
      BLEdata.set("id", (char*)mac_adress.c_str());
      Log.trace(F("Device identified in CB: %s" CR), (char*)mac_adress.c_str());
      BLEdata.set("batt", (int)pData[1]);
      BLEdata.set("version", (float)pData[2] / 10.0);
      BLEdata.set("num_timers", (int)pData[8]);
      BLEdata.set("mode", (pData[9] & 0x08) ? "switch" : "press");
      BLEdata.set("inverted", (bool)(pData[9] & 0x01));
      BLEdata.set("hold_secs", (int)pData[10]);

      pubBT(BLEdata);
    } else {
      Log.notice(F("Device not identified" CR));
    }
  } else {
    Log.trace(F("Callback process canceled by processLock" CR));
  }
  xTaskNotifyGive(m_taskHandle);
}

void switchbot_connect::publishData() {
  NimBLEUUID serviceUUID("cba20d00-224d-11e6-9fb8-0002a5d5c51b");
  NimBLEUUID charUUID("cba20003-224d-11e6-9fb8-0002a5d5c51b");
  NimBLERemoteCharacteristic* pChar = getCharacteristic(serviceUUID, charUUID);

  if (pChar && pChar->canNotify()) {
    Log.trace(F("Registering notification" CR));
    pChar->subscribe(true, std::bind(&switchbot_connect::notifyCB, this,
                                     std::placeholders::_1, std::placeholders::_2,
                                     std::placeholders::_3, std::placeholders::_4));
    m_taskHandle = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BLE_CNCT_TIMEOUT));
  } else {
    Log.notice(F("Failed registering notification" CR));
  }
}

#  endif //ZgatewayBT
#endif //ESP32
