  /*  
  OpenMQTTGateway Addon  - ESP8266 or Arduino program for home automation 

   Act as a wifi or ethernet gateway between your 433mhz/infrared IR signal  and a MQTT broker 
   Send and receiving command by MQTT
 
   This is the Light Meter Addon based on modules with a TSL2561:
   - Measures ambient Light Intensity in Lux (lx), Foot Candela (ftcd) and Watt/m^2 (wattsm2)
   - Required Hardware Module: TSL2561 (for instance Banggood.com product ID 1129550)
   - Dependencies: Adafruit_TSL2561 and Adafruit_Sensor

   Connection Scheme:
   --------------------

   TSL2561------> Arduino Uno ----------> ESP8266
   ==============================================
   Vcc ---------> 3.3V -----------------> 3.3V
   GND ---------> GND ------------------> GND
   SCL ---------> Pin A5 ---------------> D1
   SDA ---------> Pin A4 ---------------> D2
   ADD ---------> N/C (Not Connected) --> N/C (Not Connected)
 
    Copyright: (c) Chris Broekema
  
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

#ifdef ZsensorTSL2561
#include "math.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);


void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setupZsensorTSL2561()
{
  Wire.begin();
  Wire.beginTransmission(TSL2561_i2c_addr);

  if (!tsl.begin())
  {
    Serial.println("No TSL2561 detected\n");
  }
  
  // enable auto ranging
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  // since we're slowly sampling, enable high resolution but slow mode TSL2561
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);

  Serial.println("TSL2561 Initialized. Printing detials now.");
  displaySensorDetails();  
}

void MeasureLightIntensity()
{
  static uint32_t persisted_lux = 0;
  
  if (millis() > (timetsl2561 + TimeBetweenReadingtsl2561)) {
    timetsl2561 = millis();
    
    sensors_event_t event;
    tsl.getEvent(&event);
    if (event.light)
      // if event.light == 0 the sensor is clipping, do not send
      {
	if (persisted_lux != event.light || tsl2561_always ) {
	  persisted_lux = event.light;
	  char lux[7];
	  char ftcd[7];
	  char wattsm2[7];
	  sprintf(lux, "%s", String(event.light).c_str());
	  sprintf(ftcd, "%s", String(event.light/10.764).c_str());
	  sprintf(wattsm2, "%s", String(event.light/683.0).c_str());
	  	  
	  trc(F("Sending Lux to MQTT"));
	  trc(String(event.light));
	  client.publish(LUX, lux);
	  client.publish(FTCD, ftcd);
	  client.publish(WATTSM2, wattsm2);
	} else {
	  trc(F("Same lux value, do not send"));
	}
      } else {
        trc(F("Failed to read from TSL2561"));
      }
  }
}



#endif 