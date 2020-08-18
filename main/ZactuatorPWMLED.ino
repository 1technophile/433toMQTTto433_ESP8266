/*
  OpenMQTTGateway - ESP8266 or Arduino program for home automation

  This actuator enables LEDs to be directly controlled from the PWM outputs
  of ESP32 or ESP8266 microcontrollers.

  It uses the highest resolution 16-bit duty cycles available, to try to
  give good control over LEDs, particularly at low brightness settings.

  It supports different gamma curves to try to give good perceptually
  linear results, and it supports calibration of the min and max levels
  for each channel.

  In total it supports 5 channels denoted r, g, b, w0 and w1

  Supported MQTT topics...

  ".../commands/MQTTtoPWMLED/set" : Set the state of the LEDs.
  All values support floating point for greater precision.
  {
    "r"  : 0-255,
    "g"  : 0-255,
    "b"  : 0-255,
    "w0" : 0-255,
    "w1" : 0-255,

    "fade" : <fade time in seconds>
  }

  ".../commands/MQTTtoPWMLED/calibrate" : Set calibration data
  All values support floating point for greater precision.
  It can be convenient to use the 'retain' feature of MQTT to
  store calibration data.
  Gamma is a power value that is used help make the intensity
  curves respond in a more perceptually linear way.
  If you would like the values to remain linear, set the gamma
  values to 1.0
  Min and max set the levels that 0 and 255 will correspond to.
  Be aware the the min and max values are in the current gamma space
  before the conversion to linear - so if you change the gamma level
  then you will probably need to tune the min and max values again
  {
    "gamma-r" : 0.5 - 4.0,
    "min-r"   : 0.0 - 255.0,
    "max-r"   : 0.0 - 255.0,

    "gamma-g" : 0.5 - 4.0,
    etc...
  }

    Copyright: (c)

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

#ifdef ZactuatorPWMLED

#include "config_PWMLED.h"

static long previousUpdateTime = 0; // milliseconds
static long currentUpdateTime = 0;  // milliseconds

static const int   kNumChannels = 5;
static const char* channelJsonKeys[] = {"r", "g", "b", "w0", "w1"};
static const int   channelPins[] = {PWMLED_R_PIN, PWMLED_G_PIN, PWMLED_B_PIN, PWMLED_W0_PIN, PWMLED_W1_PIN};

// These are all in a perceptually linear colour space, but scaled 0-1
static float currentValues[kNumChannels] = {};
static float fadeStartValues[kNumChannels] = {};
static float targetValues[kNumChannels] = {};

static long fadeStartUpdateTime = 0; // milliseconds
static long fadeEndUpdateTime = 0; // milliseconds
static bool fadeIsComplete = false;

// Calibration data (initialised during setupPWMLED)
static float calibrationMinLinear[kNumChannels];
static float calibrationMaxLinear[kNumChannels];
static float calibrationGamma[kNumChannels];

void setupPWMLED()
{
  Log.trace(F("ZactuatorPWMLED setup done " CR));

  // Setup the PWM channels at the highest frequency we can for full 16-bit
  // duty cycle control.  These channels will be assigned to the pins
  // for R, G, B, W0 and W1 outputs.

  // PWM outputs vary the light intensity linearly, but our eyes don't
  // perceive actual linear changes as linear.
  // This manifests as a problem when trying to have fine control
  // over LEDs at very low levels.
  // Using an 8-bit duty cycle for example only allows for 256 different
  // linear levels of brightness.  Perceptually, the difference in
  // brightness between levels 1 and 2 is very large - so to get fine
  // control at dark levels, it's important that we use as high 
  // resolution PWM as we can.
  for(int i = 0; i < kNumChannels; ++i)
  {
    // I think this is the fastest frequency that allows for a 16-bit
    // duty cycle on an ESP32
    ledcSetup(i, 625.0, 16);
    ledcAttachPin(channelPins[i], i);
    calibrationMinLinear[i] = 0.f;
    calibrationMaxLinear[i] = 1.f;
    calibrationGamma[i] = 2.2f;
  }
}

// This applies a power curve to the input to try to make the inputs
// be 'perceptually linear', as opposed to actually linear.
static float perceptualToLinear(float perceptual, int channelIdx)
{
  return pow(perceptual, calibrationGamma[channelIdx]);
}

// If we're currently fading between states, then update those states
void PWMLEDLoop()
{
  previousUpdateTime = currentUpdateTime;
  currentUpdateTime = millis();

  if(fadeIsComplete)
  {
    return;
  }

  // Calculate our lerp value through the current fade
  long totalFadeDuration = fadeEndUpdateTime - fadeStartUpdateTime;
  float fadeLerpValue = 1.f;
  if(totalFadeDuration > 0)
  {
    fadeLerpValue = (float) (currentUpdateTime - fadeStartUpdateTime) / (float) totalFadeDuration;
  }
  if(fadeLerpValue >= 1.f)
  {
    fadeIsComplete = true;
    for(int i = 0; i < kNumChannels; ++i)
    {
      currentValues[i] = targetValues[i];
    }
  }
  else
  {
    for(int i = 0; i < kNumChannels; ++i)
    {
      currentValues[i] = ((targetValues[i] - fadeStartValues[i]) * fadeLerpValue) + fadeStartValues[i];
    }
  }

  // Now convert these perceptually linear values into actually linear values
  // and set the appropriate duty cycle for the outputs.
  for(int i = 0; i < kNumChannels; ++i)
  {
    float linear = perceptualToLinear(currentValues[i], i);

    // We always treat zero as zero so that it's truly off, regardless of the calibration data.
    if(linear > 0.f)
    {
      // Remap according to the calibration
      linear = (linear * (calibrationMaxLinear[i] - calibrationMinLinear[i])) + calibrationMinLinear[i];
    }

    long dutyCycle = (long) (linear * 65535.f);
    ledcWrite(i, dutyCycle);
    //Log.notice(F("Setting channel %d : %d" CR),i,dutyCycle);
  }
}

boolean PWMLEDtoMQTT()
{
  return false;
}

#ifdef jsonReceiving
void MQTTtoPWMLED(char *topicOri, JsonObject &jsonData)
{
  if (cmpToMainTopic(topicOri, subjectMQTTtoPWMLEDsetleds))
  {
    Log.trace(F("MQTTtoPWMLED JSON analysis" CR));
    // Parse the target value for each channel
    for(int i = 0; i < kNumChannels; ++i)
    {
      fadeStartValues[i] = currentValues[i];
      JsonVariant value = jsonData[channelJsonKeys[i]];
      if(value.success())
      {
        float targetValue = value.as<float>() * (1.f / 255.f);
        targetValue = std::min(targetValue, 1.f);
        targetValue = std::max(targetValue, 0.f);
        targetValues[i] = targetValue;
      }
    }
    // Configure as an instantaneous change....
    fadeStartUpdateTime = currentUpdateTime;
    fadeEndUpdateTime = currentUpdateTime;
    // ...unless there is a "fade" value in the JSON
    JsonVariant fade = jsonData["fade"];
    if(fade.success())
    {
      // "fade" json value is in seconds. Convert to milliseconds
      fadeEndUpdateTime += (long) (fade.as<float>() * (1000.f));
    }
    fadeIsComplete = false; // The values will start to change during PWMLEDLoop
  }
  else if (cmpToMainTopic(topicOri, subjectMQTTtoPWMLEDcalibrateleds))
  {
    // Read the optional calibration data for each channel
    for(int i = 0; i < kNumChannels; ++i)
    {
      char key[16];
      snprintf(key, 16, "gamma-%s", channelJsonKeys[i]);
      JsonVariant value = jsonData[key];
      if(value.success())
      {
        float gamma = value.as<float>();
        // Sanity check
        gamma = std::min(gamma, 4.f);
        gamma = std::max(gamma, 0.5f);
        calibrationGamma[i] = gamma;
      }
      snprintf(key, 16, "min-%s", channelJsonKeys[i]);
      value = jsonData[key];
      if(value.success())
      {
        calibrationMinLinear[i] = perceptualToLinear(value.as<float>() * (1.f / 255.f), i);
      }
      snprintf(key, 16, "max-%s", channelJsonKeys[i]);
      value = jsonData[key];
      if(value.success())
      {
        calibrationMaxLinear[i] = perceptualToLinear(value.as<float>() * (1.f / 255.f), i);
      }
    }
  }
}
#endif

#ifdef simpleReceiving
void MQTTtoPWMLED(char *topicOri, char *datacallback)
{
  // We currently only support JSON
}
#endif

#endif
