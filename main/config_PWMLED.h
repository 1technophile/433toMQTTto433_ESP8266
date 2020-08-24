/*
  OpenMQTTGateway  - ESP8266 or Arduino program for home automation

   This files enables you to set your parameters for the PWMLED actuator

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

/*-------------------PWMLED topics & parameters----------------------*/

#if defined(ZactuatorPWMLED) && !ESP32
# error The PWM LED actuator only works on ESP32 currently. It should be possible to get working on devices that support a 16-bit PWM duty cycle though.
#endif

// PWMLED MQTT Subjects
#define subjectMQTTtoPWMLED "/commands/MQTTtoPWMLED"
#define subjectMQTTtoPWMLEDsetleds subjectMQTTtoPWMLED "/set"             //set LEDs with JSON struct {"r":0-255,"g":0-255,"b":0-255,"w0":0-255,"w0":0-255,"fade":<fade time in seconds>}
#define subjectMQTTtoPWMLEDcalibrateleds subjectMQTTtoPWMLED "/calibrate" //set LED calibration JSON struct {"gamma-r":0.5-4.0,"min-r":0-255,"max-r":0-255 etc. }

// Edit the following to declare the channel names and corresponding output pins
#define PWM_CHANNEL_NAMES {"r", "g", "b", "w0", "w1"}
#define PWM_CHANNEL_PINS  { 25,  33,  32,   23,   22}

// Gamma defines a power curve that is applied to convert the input values
// to the linear PWM outputs.
// For LED control, the gamma can be used to make the inputs be more
// perceptually linear.
// For controlling other things that require strict linear control, set
// the gamma to 1 and leave it alone.
// This just defines the default gamma to use for all channels.
// The gamma for each channel can be modified using the calibrate MQTT topic.
#define PWM_DEFAULT_GAMMA 2.2f
