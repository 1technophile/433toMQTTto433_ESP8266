#ifndef zBLEConnect_h
#define zBLEConnect_h

#ifdef ESP32
#  include "ArduinoJson.h"
#  include "NimBLEDevice.h"
#  include "config_BT.h"

extern void pubBT(JsonObject& data);

class zBLEConnect {
public:
  NimBLEClient* m_pClient;
  TaskHandle_t m_taskHandle = nullptr;
  zBLEConnect(NimBLEAddress& addr) { m_pClient = NimBLEDevice::createClient(addr); }
  virtual ~zBLEConnect() { NimBLEDevice::deleteClient(m_pClient); }
  virtual bool writeData(BLEAction* action);
  virtual bool readData(BLEAction* action);
  virtual bool processActions(std::vector<BLEAction>& actions);
  virtual void publishData() {}
  virtual NimBLERemoteCharacteristic* getCharacteristic(const NimBLEUUID& service, const NimBLEUUID& characteristic);
};

class LYWSD03MMC_connect : public zBLEConnect {
  void notifyCB(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);

public:
  LYWSD03MMC_connect(NimBLEAddress& addr) : zBLEConnect(addr) {}
  void publishData() override;
};

class DT24_connect : public zBLEConnect {
  std::vector<uint8_t> m_data;
  void notifyCB(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);

public:
  DT24_connect(NimBLEAddress& addr) : zBLEConnect(addr) {}
  void publishData() override;
};

class GENERIC_connect : public zBLEConnect {
  std::vector<uint8_t> m_data;

public:
  GENERIC_connect(NimBLEAddress& addr) : zBLEConnect(addr) {}
};

/*----------- Experimental ------------*/
class switchbot_connect : public zBLEConnect {
  void notifyCB(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);

public:
  switchbot_connect(NimBLEAddress& addr) : zBLEConnect(addr) {}
  void publishData() override;
};

#endif //ESP32
#endif //zBLEConnect_h
