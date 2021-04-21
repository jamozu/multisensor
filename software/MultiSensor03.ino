/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *
 * ===============================================================================================
 * MULTI-SENSOR
 * 
 * This project consists of a module that manages different sensors for home use. The idea is to 
 * be able to mix and match different sensors, according to the requirements of each area of the 
 * house, using a single PCB design and a configurable sketch, making it easier to setup and 
 * maintain a sensor network.
 * 
 * ===============================================================================================
 * DESCRIPTION
 *
 * The sketch for this sensor module is based on and Arduino Mini Pro, which has a good number of 
 * interface pins and is suited for running on batteries.
 * 
 * The radio module is the NRF24L01.
 * 
 * It supports the following sensors:
 * - Motion Sensor.
 * - 1 – 3 Door / Window sensors.
 * - Temperature and humidity sensor.
 * - Flame sensor. 
 * - Light sensor.
 * - LUX Sensor (I2C).
 * - Smoke sensor.
 * - Gas sensor.
 * - CO sensor.
 * - 1 – 3 Moisture sensors.
 * - 1 – 3 Leak detectors.
 * 
 * It also includes the features:
 * - Battery operation or battery backup
 * - Audible Alarm
 * - Reset / Test / Calibrate button
 * 
 * Some of the sensors use overlaping pins, so not all of them can be active simultaneously. See 
 * the table below for all the possible sensor/feature combinations.
 * 
 * .------------------------------------------------------------.
 * | Arduino Pins | Sensors               | Features |  Sleep   |
 * .--------------+-----------------------+----------+----------.
 * |  1           |                       | Button   | N/A      |
 * |  2           | Door 1                | Button   |          |
 * |  3           | Door 2, PIR Motion    | Button   |          |
 * |  4           | Door 3                |          | Optional |
 * |  5           |                       | Alarm    |          |
 * |  7           | Temp/Humidity         |          |          |
 * |  8           | Flame                 |          |          |
 * |  6, A1, A7   | MQ-7, Moist 3, Leak 3 |          |          |
 * |  A2          | MQ-2, Moist 2, Leak 1 |          |          |
 * |  A3          | MQ-5, Moist 1         |          |          |
 * |  A4, A5      | Light, Lux            |          |          |
 * |  A6          |                       | Battery  |          |
 * '------------------------------------------------------------'
 *    
 * ===============================================================================================
 * AIR QUALITY SENSORS
 * 
 * The module supports 3 air quality sensors. MQ-2, MQ-5 and MQ-7.
 * It is possible to replace the MQ-2 and MQ-5 sensors with others to detect specific gases, but 
 * the curves for the calculation of PPMs must be adjusted in the global variables section.
 * 
 * These MQ sensors have different sensitivities to different gases, however it is not possible to 
 * differentiate between one specific gas, for this reason the module has been equipped with 
 * a sensor for general air quality detection (MQ-2) and other more specific ones in order to have 
 * an accurate reading of the type of gas present, if required.
 * 
 * These sensors can consume up to 900mA, so it is not recommended for full battery operation, so 
 * it is recommended to have a power source with backup batteries, especially if using more than  
 * one sensor simultaneously.
 * 
 * For a simple, general use sensor and triggering an alarm, a single MQ-2 sensor is enough. If a 
 * more accurate reading of flammable gases is required, the MQ-5 can be used and MQ-7 can be used 
 * to detect Carbon Monoxide more accurately.
 * 
 * The MQ-2 is a general-purpose sensor that can detect:
 * - Smoke
 * - Liquefied Petroleum Gas (LPG)
 * - Natural Gas (CH4)
 * - Carbon Monoxide (CO)
 * 
 * The MQ-5 is sensor focused on flammable gases and can detect:
 * - Liquefied Petroleum Gas (LPG)
 * - Natural Gas (CH4)
 * 
 * The MQ-7 is sensor specific for Carbon Monoxide. This sensor requires the application of two 
 * voltage levels alternately, so it requires constant operation of the microprocessor, so if 
 * this sensor is active, the sleep mode is deactivated.
 * 
 * The air quality sensors can be used to trigger an alarm or exclusively for monitoring.
 * 
 * When any of the air quality sensors is enabled in the module, the calibration button is 
 * automatically enabled. See below on the interrupts used by the button and other sensors.
 * 
 * If the alarm feature is enabled, it will be trigger if certain levels are reached in the 
 * monitored gases. See the "Air quality thresholds" section in the sketch configuration. 
 * 
 * ===============================================================================================
 * MOISTURE SENSORS
 * 
 * The module can be equiped with humidity/moisture sensors. The sensors can be used to detect 
 * soil moisture, water level, etc. Additionally it is possible to use some types of humidity 
 * sensors for leak detection. In that case when the humidity threshold is reached the alarm is 
 * triggered. See the "Leak thresholds" section in the sketch configuration.
 * 
 * It is possible to combine air quality sensors and moisture sensors, but you have to consider 
 * the position of the module, since the air quality sensors are usually placed near the ceiling 
 * and the leak moisture/leak sensors near lower areas or the floor.
 * 
 * Moisture sensors use the same analog inputs as gas sensors.
 * 
 * Moisture sensor 1 uses the same Analog Pin as the MQ-7 Air quality sensor.
 * Moisture sensor 2 uses the same Analog Pin as the MQ-2 Air quality sensor.
 * Moisture sensor 3 uses the same Analog Pin as the MQ-5 Air quality sensor.
 * 
 * ===============================================================================================
 * 
 * BUTTON
 * 
 * The button is used to Reset and Test the alarm, as well as to Calibrate the Air Quality sensors.
 * - Short press; Less than 3 seconds. Resets the alarm (it will stay off for at least 30 seconds; 
 *   see ALARM_RETRIGGER).
 * - Long press; 3-5 seconds. Starts an alarm test for 10 seconds (See ALARM_TEST).
 * - Very long press; Longer than 6 seconds. Starts a calibration for the Air Quality sensors.
 *   The sensors need to be in clean air for the calibration to be effective.
 * 
 * ===============================================================================================
 * INTERRUPTIONS
 * 
 * The Arduino Mini Pro has 2 interrupts that can wake up the module from sleep. This module 
 * supports 4 elements that require these interruptions:
 * - Door sensor 1 (IRQ2)
 * - Door sensor 2 (IRQ3)
 * - Motion sensor (IRQ3)
 * - Reset / Test / Calibrate button (IRQ2)
 * 
 * The button is required to operate the alarm and is also used to calibrate the air quality 
 * sensors.
 * 
 * The Door Sensor 1 and Button share the same interrupt by default, so you have to select 
 * between these two features. Also the Motion Sensor and the Door Sensor 2 use the same pin, so 
 * you have to configure one or the other.
 * 
 * The possible combinations are:
 *    ________________________________________________________________________
 *   |  Door  |  Door  | Motion |        |                                    |
 *   | Sensor | Sensor |        | Alarm  |              Comments              |
 *   |   1    |   2    | Sensor |        |                                    |
 *   |------------------------------------------------------------------------|
 *   |   X    |   X    |        |        |                                    |
 *   |   X    |        |   X    |        |                                    |
 *   |        |   X    |        |   X    | Button uses D2                     |
 *   |        |        |   X    |   X    | Button uses D2                     |
 *   |   X    |        |        |   X    | Button uses D3                     |
 *   |   X    |   X    |        |   X    | Button uses D1, and sleep disabled |
 *   |   X    |        |   X    |   X    | Button uses D1, and sleep disabled |
 *   '------------------------------------------------------------------------'
 * 
 * ===============================================================================================
 *
 * ALARM TRIGGERS
 * 
 * The alarm can be triggered by various gas measurements or sensors. Each gas or sensor has a 
 * binary value assigned to it so that a single integer variable controls all alarms (almTrigger).
 * Each gas / sensor has it's own threshold.
 * 
 * -   1  Flame          On/Off (The threshold is calibrated in the sensor module)
 * -   2  Smoke          300ppm (See AQS_TH_SMOKE and  AQS_TH_SMOKE_OFF)
 * -   4  LPG            300ppm (See AQS_TH_LPG and  AQS_TH_LPG_OFF)
 * -   8  Natural gas    300ppm (See AQS_TH_GAS and  AQS_TH_GAS_OFF)
 * -  16  CO             50ppm (See AQS_TH_CO, AQS_TH_CO_DET and  AQS_TH_CO_OFF)
 * -  32  Leak/Flood 1   10% (See LEAK_ON, LEAK_OFF)
 * -  64  Leak/Flood 2   10% (See LEAK_ON, LEAK_OFF)
 * - 128  Leak/Flood 3   10% (See LEAK_ON, LEAK_OFF)
 * 
 * ===============================================================================================
 *
 * TEMPORARY DEACTIVATION OF THE SLEEP CYCLE
 * 
 * In the same way that the alarm can be triggered by several sensors, various events can prevent 
 * the processor from going to sleep mode; for example if the alarm is active or if the button is 
 * pressed to have an accurate measure of the time it is pressed, etc.
 * 
 * -   1  Alarm is On
 * -   2  The button is pressed
 * -  64  MQ-7 sensor is enabled
 * - 256  Wait for child presentation to occur
 * 
 * ===============================================================================================
 *
 * EEPROM
 * 
 * 0   - 9  -> Reserved positions
 * 10  - 20 -> Battery calibration
 * 30+      -> Air quality sensors
 * 
 * ===============================================================================================
 *
 * CAVEATS
 * 
 * Button
 * ------
 *   The module uses 1 of 3 pins for the "Reset/Test/Calibrate" button. 
 *   
 *   Pin D2 or D3 are normally assigned to the button, depending on the sensors configured.
 *   If both D2 or D3 are required for sensors, D1 can be used, but sleep should be disabled to 
 *   allow consant monitoring of the status of the button.
 *   
 *   D1 is required for serial communication in debug mode (MY_DEBUG or MOD_DEBUG). To still be 
 *   able to debug the module D6 is used instead, but the MQ-7 sensor is then not enabled.
 *   
 *   It is possible to use some other pin of one of the sensors that is not in use, in order to 
 *   debug the CO sensor and the button at the same time, but you must modify the wiring and the 
 *   sketch accordingly.
 * 
 * Motion and Door/Window sensor
 * -----------------------------
 *   The Arduino Pro Mini has 2 pins that can trigger interruptions and wake up the module, 
 *   especially if it is battery operated. For this reason there can only be a combination of:
 *   - Moition Sensor + Door Sensor (1)
 *   - Door Sensor (1) + Door Sensor (2)
 * 
 *  Door sensor 3 can not wake up the module from sleep, therefore it will only report status 
 *  every SLEEP_TIME ms, and will not catch state changes in real time, unless sleep is disabled.
 *  
 * Air Quality Sensors
 * -----------------------------
 * The MQ series of air quality sensors requiere 5.0VCC, therefore is this sensors are used, a
 * 5V Arduino must be used.
 * 
 * CO Sensor
 * -----------------------------
 *   The Carbon monoxide sensor (MQ-7) has a duty cycle that requires alternating between two 
 *   voltage levels and constant monitoring, therefore the microprocessor will not enter sleep
 *   mode. To use this sensor it is recommended have the module connected to the mains and use 
 *   batteries only as backup.
 *   
 * ===============================================================================================
 *
 * ACKNOWLEDGMENTS
 * 
 *  + https://forum.arduino.cc/index.php?topic=41497.0
 *  
 * ===============================================================================================
 * 
 * ===============================================================================================
 * Flashing
 * ===============================================================================================
 * Board: "Arduino Pro or Pro Mini"
 * Procesor: Atmega328P (3.3V, 8MHz)
 * 
 * ===============================================================================================
 * 
  @author Jesus Amozurrutia Elizalde
  @version 0.8.3
  @date 2021/04/10
  Created on: Friday March 12 2020, 11:10:00
  Contact: jamozu@gmail.com
  Device: Arduino Pro Mini 3.3V or Arduino Pro Mini 5.0V
 * 
 */


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration starts here
//              v            v            v            v            v            v            v 

// Module name broadcasted to HomeAssistant
#define MODULE_NAME "MultisensorE902"
// Software version
#define MODULE_VER "0.8.3"
// Debug messages to serial monitor for this sketch
#define MOD_DEBUG

//////////////////////////////////////////////////////////////////////////////////////////////////
// Define MySensors Library parameters
//////////////////////////////////////////////////////////////////////////////////////////////////
// Enable debug messages to serial monitor for My Sensors library.
//#define MY_DEBUG
// Enable and select radio type attached.
#define MY_RADIO_RF24
// Enable non blocking mode. Allows jumping to the loop() without waiting for radio registering.
#define MY_TRANSPORT_WAIT_READY_MS 2000
// Number of retrys for sensor presentation.
#define MY_PRESENTATION_RTR 5
// Number of retrys for messages.
#define MY_MESSAGE_RTR 5
// Enable repater functionality (disables sleep).
//#define MY_REPEATER_FEATURE

//////////////////////////////////////////////////////////////////////////////////////////////////
// Define Module Powering
//////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino VCC 3.3 | 5.0
#define MOD_VCC 3.3
// Battery operated or battery backup
#define MOD_BATTERY
// Module has a power supply
//#define MOD_POWER

//////////////////////////////////////////////////////////////////////////////////////////////////
// Define Sensors
//////////////////////////////////////////////////////////////////////////////////////////////////
#define SENS_DOOR1        // Door/window sensor
//#define SENS_DOOR2      // Second door/window sensor (Same pin as SENS_MOTION)
#define SENS_DOOR3        // Third door/window sensor (No interruption)
#define SENS_MOTION       // Motion sensor (Same pin as SENS_DOOR2)
#define SENS_DHT          // DHT11/DHT22 Sensor (Temperature / humidity)
//#define SENS_LIGHT        // Light sensor (Same pin as SENS_LUX)
#define SENS_LUX          // Lux sensor (Same pin as SENS_LIGHT), Set the Meter type in the 
                          // "Measurements" section with the MOD_LUX parameter
#define SENS_FLAME        // Flame sensor
//#define SENS_MQ2          // MQ2 Air quality sensor (Smoke, LPG, Natural Gas, CO) 
//#define SENS_MQ5          // MQ5 Air quality sensor (LPG, Natural, CO)
//#define SENS_MQ7          // MQ7 Air quality sensor (CO)
#define SENS_MOIST1       // Moisture sensor 1 (Same pins as SENS_MQ7)
#define SENS_MOIST2       // Moisture sensor 2 (Same pins as SENS_MQ2)
#define SENS_MOIST3       // Moisture sensor 3 (Same pins as SENS_MQ5)

//////////////////////////////////////////////////////////////////////////////////////////////////
// Define Measurements
//////////////////////////////////////////////////////////////////////////////////////////////////
#define MOD_TEMPERATURE   // Temperature (Requires SENS_DHT)
#define MOD_HUMIDITY      // Humidity (Requires SENS_DHT)
//#define MOD_SMOKE         // Smoke measurement (requires SENS_MQ2)
//#define MOD_LPG           // LPG measurement (requires SENS_MQ2 and/or SENS_MQ5)
//#define MOD_GAS           // Natural Gas measurement (requires SENS_MQ2 and/or SENS_MQ5)
//#define MOD_CO            // Carbon Monoxide measurement (requires SENS_MQ2 and/or SENS_MQ7)
#define MOD_LEAK1         // Use SENS_MOIST1 as a leak sensor (Requires SENS_MOIST1).
#define MOD_LEAK2         // Use SENS_MOIST2 as a leak sensor (Requires SENS_MOIST2).
#define MOD_LEAK3         // Use SENS_MOIST3 as a leak sensor (Requires SENS_MOIST3).
//#define MOD_MQ7_FINE      // Turn On MQ-7 sensor only to get a fine read of CO levels, based
                            // on raw readings from SENS_MQ2. (Requires SENS_MQ2 and SENS_MQ7).
                            // This setting allows a longer battery life.
#define MOD_LUX TSL2561   // TSL2561, BH1750
                          // Requires the respective libraries:
                          // Adafruit TSL2561
                          // BH1750

//////////////////////////////////////////////////////////////////////////////////////////////////
// Alarm
//////////////////////////////////////////////////////////////////////////////////////////////////
#define MOD_ALARM         // Audible alarm, used by air quality, moisture and leak sensors 

//////////////////////////////////////////////////////////////////////////////////////////////////
// Parameters for the Module sleep / updates
//////////////////////////////////////////////////////////////////////////////////////////////////
// Sleep time between reports (in milliseconds).
static const uint32_t SLEEP_TIME = 60000;
// Minimum time to be awake.
static const uint32_t AWAKE_INTERVAL = 500; 
// Default time between updates.
static const uint32_t UPD_INTERVAL = 25000;
// Force sending an update after N sensor reads, so a controller showing the timestamp of the last 
// update doesn't show something like 3 hours in the case that the value does not change for a 
// while.
// i.e. the sensor would force sending an update every UPD_INTERVAL * UPD_N_READS [ms].
static const uint8_t UPD_N_READS = 5;
// Sleep cycles that force updates on all sensors.
static const uint8_t SLEEP_CYCLES = 5;

//////////////////////////////////////////////////////////////////////////////////////////////////
// Parameters for Temperature and Humidity sensor
//////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor type
#define DHTTYPE DHT11 // [DHT11|DHT22|DHT21]
// Set this offset if the sensor has a permanent small offset to the real temperatures.
// In Celsius degrees (as measured by the device)
#define TMP_OFFSET 0

//////////////////////////////////////////////////////////////////////////////////////////////////
// Parameters for the ligh/lux sensors
//////////////////////////////////////////////////////////////////////////////////////////////////
// Voltage produced by the sensor with 100% light.
#define         LIGHT_MAX               0
// Voltage produced by the sensor with 0% light.
#define         LIGHT_MIN               MOD_VCC
// Max. LUX supported by the sensor.
//#define         LUX_MAX                 120000
// Min. LUX supported by the sensor.
//#define         LUX_MIN                 0

//////////////////////////////////////////////////////////////////////////////////////////////////
// Air quality sensing parameters
//////////////////////////////////////////////////////////////////////////////////////////////////
// Define how many samples you are going to take in the calibration phase.
#define         AQS_CAL_TIMES             (50)
// Define the time interval(in milliseconds) between each samples in the calibration phase.
#define         AQS_CAL_INTERVAL          (500)
// Define how many samples you are going to take in normal operation.
#define         AQS_READ_TIMES            (5)
// Define the time interval(in milliseconds) between each samples in normal operation.
#define         AQS_READ_INTERVAL         (50)

//////////////////////////////////////////////////////////////////////////////////////////////////
// Air quality thresholds
//////////////////////////////////////////////////////////////////////////////////////////////////
// Threshold for trigering the alarm for smoke in PPM.
#define         AQS_TH_SMOKE              500
// Threshold for setting the alarm off for smoke in PPM.
#define         AQS_TH_SMOKE_OFF          450
// Threshold for trigering the alarm for LPG in PPM.
#define         AQS_TH_LPG                300
// Threshold for setting the alarm off for LPG in PPM.
#define         AQS_TH_LPG_OFF            260
// Threshold for trigering the alarm for Natural Gas in PPM.
#define         AQS_TH_GAS                300
// Threshold for setting the alarm off for Natural Gas in PPM.
#define         AQS_TH_GAS_OFF            260
// Threshold for trigering the alarm for Carbon Monoxide in PPM.
#define         AQS_TH_CO                 50
// Threshold for setting the alarm off for Carbon Monoxide in PPM.
#define         AQS_TH_CO_OFF             45
// Threshold for turning On the CO sensor in PPM, if MOD_MQ7_FINE is enabled.
#define         AQS_TH_CO_DET             35
  
//////////////////////////////////////////////////////////////////////////////////////////////////
// Parameters for MQ-2 Smoke detector
//////////////////////////////////////////////////////////////////////////////////////////////////
// Define the load resistance on the board, in Kilo Ohms.
#define         MQ2_RL_VALUE              (18)
// RO_CLEAR_AIR_FACTOR = (Sensor resistance in clean air) / RO, which is derived from the chart 
// in the datasheet.
#define         MQ2_RO_CLEAN_AIR_FACTOR   (9.56)

//////////////////////////////////////////////////////////////////////////////////////////////////
// Parameters for MQ-5 Gas detector
//////////////////////////////////////////////////////////////////////////////////////////////////
// Define the load resistance on the board, in Kilo Ohms.
#define         MQ5_RL_VALUE              (1)
// RO_CLEAR_AIR_FACTOR = (Sensor resistance in clean air) / RO, which is derived from the chart 
// in the datasheet.
#define         MQ5_RO_CLEAN_AIR_FACTOR   (6.33)

//////////////////////////////////////////////////////////////////////////////////////////////////
// Parameters for MQ-7 CO detector
//////////////////////////////////////////////////////////////////////////////////////////////////
// Define the load resistance on the board, in Kilo Ohms
#define         MQ7_RL_VALUE              (1)
// RO_CLEAR_AIR_FACTOR = (Sensor resistance in clean air) / RO, which is derived from the chart 
// in the datasheet.
#define         MQ7_RO_CLEAN_AIR_FACTOR   (11.44)

//////////////////////////////////////////////////////////////////////////////////////////////////
// Parameters for Moisture sensors
//////////////////////////////////////////////////////////////////////////////////////////////////
#define         MOIST_MAX              MOD_VCC  // Voltage produced by the sensor with 100% humidity
#define         MOIST_MIN              0        // Voltage produced by the sensor with 0% humidity

//////////////////////////////////////////////////////////////////////////////////////////////////
// Leak thresholds
//////////////////////////////////////////////////////////////////////////////////////////////////
#define         LEAK_ON               10        // % of sensor value to trigger alarm
#define         LEAK_OFF              6         // % of sensor value turn alarm off

//////////////////////////////////////////////////////////////////////////////////////////////////
// Parameters for the alarm
//////////////////////////////////////////////////////////////////////////////////////////////////
// Enable double tone alarm
#define ALM_DOUBLE true
// Alarm tone frequency (Set to nominal frequency of Piezo Buzzer)
#define ALM_TONE1 4800
#define ALM_TONE2 2500
// Alarm duty cycle (ms)
static const uint64_t ALARM_INTERVAL = 800;
// Alarm test duration (ms)
static const uint64_t ALARM_TEST = 10000;
// Alarm retrigger after a reset if the alarm condition persists (ms)
static const uint64_t ALARM_RETRIGGER = 30000;

//////////////////////////////////////////////////////////////////////////////////////////////////
// Parameters for the button
//////////////////////////////////////////////////////////////////////////////////////////////////
// Duration for a long press that triggers the alarm test
#define BTN_TEST 3000
// Duration for a longer press that triggers the setup/calibration
#define BTN_CALIBRATE 6000

//////////////////////////////////////////////////////////////////////////////////////////////////
// Parameters for battery monitor
//  - Lithium cell: 4.20V (100%) -> 2.40V (0%)
//  - 2 NiMH cells: 2.50V (100%) -> 2.00V (0%)
//  - 3 NiMH cells: 3.75V (100%) -> 3.00V (0%)
//////////////////////////////////////////////////////////////////////////////////////////////////
#define BAT_V_MAX     4.20    // Max Batery voltage (100% charge) 
#define BAT_V_MIN     2.40    // Batery Minimum voltage (0% charge)
#define BAT_R4        1000    // R4 value in K Ohms
#define BAT_R5        330     // R5 value in K Ohms

//////////////////////////////////////////////////////////////////////////////////////////////////
// General parameters for analog sensors
//////////////////////////////////////////////////////////////////////////////////////////////////
// Define the number of samples per read
#define         AG_READ_TIMES           (5) 
// Define the time interval (ms) between samples
#define         AG_READ_INTERVAL        (10)

//////////////////////////////////////////////////////////////////////////////////////////////////
// Refresh timers for each sensor (if not defined, defaults to UPD_INTERVAL)
//////////////////////////////////////////////////////////////////////////////////////////////////
// Time between updates for DHT (Temp / humidity) sensor
static const uint32_t UPD_INTERVAL_DHT = UPD_INTERVAL;
// Time between updates for Light sensor
static const uint32_t UPD_INTERVAL_LGT = 5000;
// Time between updates for MQ2 sensor
static const uint32_t UPD_INTERVAL_MQ2 = UPD_INTERVAL;
// Time between updates for MQ5 sensor
static const uint32_t UPD_INTERVAL_MQ5 = UPD_INTERVAL;

//              ^            ^            ^            ^            ^            ^            ^ 
// Configuration ends here
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
// Module Definitions
//////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef MY_REPEATER_FEATURE
  #warning "Node is a repeater. Sleep turned off."
  #ifndef SLEEP_OFF
    #define SLEEP_OFF
  #endif
#endif
// Define if a DHT measurements are used 
#ifdef SENS_DHT
  #if !defined(MOD_TEMPERATURE) && !defined(MOD_HUMIDITY)
    #warning "MOD_TEMPERATURE and MOD_HUMIDITY measurements disabled. Disabling SENS_DHT sensor."
    #undef SENS_DHT
  #endif
#endif
// Define if a DHT measurements are used 
#if (defined(MOD_TEMPERATURE) || defined(MOD_HUMIDITY)) && !defined(SENS_DHT)
  #error "SENS_DHT disabled, but MOD_TEMPERATURE and MOD_HUMIDITY measurements are enabled."
#endif
// Define if Air Quality sensors are available
#if defined(SENS_MQ2) || defined(SENS_MQ5) || defined(SENS_MQ7)
  // Validate A2 conflicts
  #if defined(SENS_MQ2) && defined(MOD_LEAK2)
    #error "SENS_MOIST2 conflicts with SENS_MQ2."
    #undef SENS_MOIST2
  #endif
  // Validate A3 conflicts
  #if defined(SENS_MQ5) && defined(MOD_LEAK3)
    #error "SENS_MOIST3 conflicts with SENS_MQ5."
    #undef SENS_MOIST3
  #endif
  // If MQ-7 is not set,  MOD_MQ7_FINE is invalid
  #ifdef MOD_MQ7_FINE
    #if !defined(SENS_MQ7)
      #error "MOD_MQ7_FINE without SENS_MQ7 sensor. Disable MOD_MQ7_FINE."
      #undef MOD_MQ7_FINE
    #elif !defined(SENS_MQ2) && !defined(SENS_MQ5)
      #error "MOD_MQ7_FINE without other Air Quality sensors."
      #undef MOD_MQ7_FINE
    #endif
  #endif
  // Validate A1 conflicts
  #ifdef SENS_MQ7
    #warning "CO Sensor (MQ7) enabled."
    #ifdef SENS_MOIST1
      #error "SENS_MOIST1 conflicts with SENS_MQ7."
      #undef SENS_MOIST1
    #endif
    // If MQ-7 CO sensor is present, without the fine sensor, Sleep is off
    #if !defined(MOD_MQ7_FINE)
      #warning "Disable sleep mode: MQ-7 CO sensor without MOD_MQ7_FINE."
      #ifndef SLEEP_OFF
        #define SLEEP_OFF
      #endif
    #endif
  #endif
  // Valiadte sensors vs gas types
  #if defined(MOD_SMOKE) && !defined(SENS_MQ2)
    #error "MOD_SMOKE without SENS_MQ2 smoke sensor."
    #undef MOD_SMOKE
  #endif
  #if defined(MOD_LPG) && !defined(SENS_MQ2) && !defined(SENS_MQ5)
    #error "MOD_LPG without SENS_MQ2 or SENS_MQ5 gas sensor."
    #undef MOD_LPG
  #endif
  #if defined(MOD_GAS) && !defined(SENS_MQ2) && !defined(SENS_MQ5)
    #error "MOD_GAS without SENS_MQ2 or SENS_MQ5 gas sensor."
    #undef MOD_GAS
  #endif
  #if defined(MOD_CO) && !defined(SENS_MQ2) && !defined(SENS_MQ7)
    #error "MOD_CO without SENS_MQ2 or SENS_MQ7 CO sensor."
    #undef MOD_CO
  #endif
  #if defined(SENS_MQ5) && !defined(MOD_GAS) && !defined(MOD_LPG)
    #warning "SENS_MQ5 without Gas measurement (MOD_GAS or MOD_LPG)."
    #undef SENS_MQ5
  #endif
  #if defined(SENS_MQ7) && !defined(MOD_CO)
    #warning "SENS_MQ7 without CO measurement (MOD_CO). Disable SENS_MQ7."
    #undef SENS_MQ7
  #endif
  #ifdef SENS_MQ7
    static const uint64_t MQ7_HEAT = 60000;
    static const uint64_t MQ7_LOWV = 88000;
  #endif
  #define MOD_AQS
  #define AQS_MQ2                   0
  #define AQS_MQ5                   1
  #define AQS_MQ7                   2
  #define AQS_LPG                   (0)
  #define AQS_GAS                   (1)
  #define AQS_SMOKE                 (2)
  #define AQS_CO                    (3)
#endif

// Define if Moisture sensors are available
#if defined(SENS_MOIST1) || defined(SENS_MOIST2) || defined(SENS_MOIST3)
  #define MOD_MOIST
#endif

// Define pin used for the Button
#if defined(MOD_AQS) || defined(MOD_MOIST) || defined(MOD_ALARM)
  #if !defined(SENS_DOOR1)
    #define MOD_BUTTON2
  #elif !defined(SENS_DOOR2) && !defined(SENS_MOTION)
    #define MOD_BUTTON3
  #else
    #define MOD_BUTTON1
    #ifndef SLEEP_OFF
      #define SLEEP_OFF
    #endif
  #endif
#endif

// Check if the Button PIN is available to use
#if defined(MOD_BUTTON1) && (defined(MOD_DEBUG) || defined(MY_DEBUG))
  #undef MOD_BUTTON1
  #ifndef SENS_MQ7
    #define MOD_BUTTON6
  #endif
#endif

// Check elements that use IRQ2
#if defined(SENS_DOOR1) && defined(MOD_BUTTON2)
  #error "IRQ2 conflict. SENS_DOOR1 and MOD_BUTTON2."
#endif

// Check elements that use IRQ3 
#ifdef SENS_MOTION
  #define IRQ3U
#endif
#ifdef SENS_DOOR2
  #ifdef IRQ3U
    #error "IRQ3 conflict. SENS_MOTION and SENS_DOOR2."
  #endif
  #define IRQ3U
#endif
#ifdef MOD_BUTTON3
  #ifdef IRQ3U
    #error "IRQ3 conflict. Button conflicts with SENS_MOTION or SENS_DOOR2."
  #endif
  #define IRQ3U
#endif

// Check elements that use A4
#if defined(SENS_LIGHT) && defined(SENS_LUX)
  #error "Conflict in A4 by SENS_LIGHT and SENS_LUX."
#endif

// Define LUX Sensor
#ifdef SENS_LUX
  #ifdef MOD_LUX
    #if MOD_LUX != TSL2561 && MOD_LUX != BH1750
      #error "Invalid LUX sensor, MOD_LUX "
    #endif
  #else
    #error "No LUX sensor defined. Define MOD_LUX."
  #endif
  #ifndef LUX_MAX
    #define LUX_MAX 120000
  #endif
  #ifndef LUX_MIN
    #define LUX_MIN 0
  #endif
#else
  #ifdef MOD_LUX
    #undef MOD_LUX 
  #endif
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////
// Compilation report
//////////////////////////////////////////////////////////////////////////////////////////////////
#warning "== Compilation report"
#pragma warning "Arduino VCC =  "
#warning "Arduino VCC = " MOD_VCC
#warning MOD_VCC
#ifdef MOD_BATTERY
  #ifdef MOD_POWER
    #warning "Power supply with battery backup."
  #else
    #warning "Battery operated."
  #endif
#else
  #ifdef MOD_POWER
    #warning "Power supply operated."
  #else
    #warning "No power method defined."
  #endif
#endif
#ifdef MY_REPEATER_FEATURE
  #warning "Node is a repeater."
#endif
#ifdef SLEEP_OFF
  #warning "Sleep is disabled."
#else
  #warning "Sleep enabled."
#endif
#if defined(MOD_BUTTON1)
  #warning "Button enabled in pin 1."
#elif defined(MOD_BUTTON2)
  #warning "Button enabled in pin 2."
#elif defined(MOD_BUTTON3)
  #warning "Button enabled in pin 3."
#elif defined(MOD_BUTTON6)
  #warning "Button enabled in pin 6 (Debug Mode)."
#else
  #warning "No Button enabled."
#endif
#ifdef MOD_ALARM
  #warning "Alarm present."
#else
  #warning "No Alarm."
#endif
#warning "====== Sensors ======"
#ifdef SENS_DOOR1
  #warning "Door / Window 1"
#endif
#ifdef SENS_DOOR2
  #warning "Door / Window 2"
#endif
#ifdef SENS_DOOR3
  #warning "Door / Window 3"
#endif
#ifdef SENS_MOTION
  #warning "Motion"
#endif
#ifdef MOD_TEMPERATURE
  #warning "Temperature"
#endif
#ifdef MOD_HUMIDITY
  #warning "Humidity"
#endif
#ifdef SENS_LIGHT
  #warning "Light"
#endif
#ifdef SENS_LUX
  #if MOD_LUX == TSL2561
    #warning "TSL2561 LUX Sensor"
  #elif MOD_LUX == BH1750
    #warning "BH1750 LUX Sensor"
  #endif
#endif
#ifdef SENS_FLAME
  #warning "Flame"
#endif
#ifdef MOD_AQS
  #warning "Air quality sensors enabled."
#endif
#ifdef MOD_SMOKE
  #warning "Smoke"
#endif
#ifdef MOD_LPG
  #warning "LPG"
#endif
#ifdef MOD_GAS
  #warning "Gas"
#endif
#ifdef MOD_CO
  #warning "CO"
#endif
#ifdef SENS_MQ2
  #warning "MQ2"
#endif
#ifdef SENS_MQ5
  #warning "MQ5"
#endif
#if defined(MOD_MQ7_FINE)
  #warning "MQ7 with fine tunning."
#elif defined(SENS_MQ7)
  #warning "MQ7"
#endif
#ifdef MOD_MOIST
  #warning "Moisture sensors enabled."
  #if defined(MOD_LEAK1)
    #warning "Leak 1"
  #elif defined(SENS_MOIST1)
    #warning "Moisture 1"
  #endif
  #if defined(MOD_LEAK2)
    #warning "Leak 2"
  #elif defined(SENS_MOIST2)
    #warning "Moisture 2"
  #endif
  #if defined(MOD_LEAK3)
    #warning "Leak 3"
  #elif defined(SENS_MOIST3)
    #warning "Moisture 3"
  #endif
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////
// Define child IDs for sensors
//////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef SENS_MOTION
  #define CHILD_ID_MOV   0  // Child ID of the motion sensor (With interrupt 3)
#endif
#ifdef SENS_DOOR1
  #define CHILD_ID_DO1   1  // Child ID of the main door/window sensor (With interrupt 2)
#endif
#ifdef SENS_DOOR2
  #define CHILD_ID_DO2   2  // Child ID of the second door/window sensor (With interrupt 3)
#endif
#ifdef SENS_DOOR3
  #define CHILD_ID_DO3   3  // Child ID of the auxiliar door/window sensor (No Interrupt)
#endif
#ifdef MOD_TEMPERATURE
  #define CHILD_ID_TMP   4  // Child ID of the temperature sensor
#endif
#ifdef MOD_HUMIDITY
  #define CHILD_ID_HUM   5  // Child ID of the humidity sensor
#endif
#ifdef SENS_LIGHT
  #define CHILD_ID_LGT   6  // Child ID of the light sensor
#endif
#ifdef SENS_LUX
  #define CHILD_ID_LUX   7  // Child ID of the light sensor
#endif
#ifdef SENS_FLAME
  #define CHILD_ID_FLM   8  // Child ID of the flame sensor
#endif
#ifdef MOD_AQS
  #ifdef MOD_SMOKE
    #define CHILD_ID_SMK   9  // Child ID of the smoke detector
  #endif
  #ifdef MOD_LPG
    #define CHILD_ID_LPG   10  // Child ID of the LPG detector
  #endif
  #ifdef MOD_GAS
    #define CHILD_ID_GAS  11  // Child ID of the LPG detector
  #endif
  #ifdef MOD_CO
    #define CHILD_ID_COS  12  // Child ID of the CO sensor
  #endif
#endif
#ifdef MOD_MOIST
  #ifdef SENS_MOIST1
    #define CHILD_ID_MS1  13  // Child ID of the moist sensor 1
    #ifdef MOD_LEAK1
      #define CHILD_ID_LK1  16  // Child ID of the leak sensor 1
    #endif
  #endif
  #ifdef SENS_MOIST2
    #define CHILD_ID_MS2  14  // Child ID of the moist sensor 1
    #ifdef MOD_LEAK2
      #define CHILD_ID_LK2  17  // Child ID of the leak sensor 1
    #endif
  #endif
  #ifdef SENS_MOIST3
    #define CHILD_ID_MS3  15  // Child ID of the moist sensor 1
    #ifdef MOD_LEAK3
      #define CHILD_ID_LK3  18  // Child ID of the leak sensor 1
    #endif
  #endif
#endif
#ifdef MOD_ALARM
  #define CHILD_ID_ALM  30  // Child ID of the alarm
#endif
#ifdef MOD_BATTERY
  #define CHILD_ID_BAT  90  // Child ID of the battery meter
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////
// Define pins
//////////////////////////////////////////////////////////////////////////////////////////////////
// Button
#if defined(MOD_BUTTON1)
  #define PIN_BUTTON 1
#elif defined(MOD_BUTTON2)
  #define PIN_BUTTON 2
  #define PIN_IRQ2 2
#elif defined(MOD_BUTTON3)
  #define PIN_BUTTON 3
  #define PIN_IRQ3 3
#elif defined(MOD_BUTTON6)
  #define PIN_BUTTON 6
#endif
// Digital I/O pin for PIR sensor (IRQ)
#ifdef CHILD_ID_MOV
  #define PIN_MOVE    3
  #define PIN_IRQ3    3
#endif
// Digital I/O pin for button/reed switch (IRQ)
#ifdef CHILD_ID_DO1
  #define PIN_DOOR1   2
  #define PIN_IRQ2    2
#endif
// Digital I/O pin for second button/reed switch (IRQ)
#ifdef CHILD_ID_DO2
  #define PIN_DOOR2   3
  #define PIN_IRQ3    3
#endif
// Digital I/O pin for button/reed switch
#ifdef CHILD_ID_DO3
  #define PIN_DOOR3   4
#endif
// Digital (Serial) I/O pin for DHT sensor
#ifdef SENS_DHT
  #define PIN_DHT     7
#endif
// Analog Light sensor
#ifdef CHILD_ID_LGT
  #define PIN_LGT     A4
#endif
// IC2 LUX sensor
#ifdef CHILD_ID_LUX
  #define PIN_I2CB    A4
  #define PIN_I2CC    A5
#endif
// Digital I/O pin for flame detector
#ifdef CHILD_ID_FLM
  #define PIN_FLM     8
#endif
// Analog Smoke sensor  (MQ-2)
#ifdef SENS_MQ2
  #define PIN_MQ2     A2
#endif
// Analog Gas sensor (MQ-5)
#ifdef SENS_MQ5
  #define PIN_MQ5     A3
#endif
// Analog Carbon Monoxide sensor (MQ-7)
#ifdef SENS_MQ7
  // Analog pin
  #define PIN_MQ7     A1
  // Analog Voltage monitor for CO sensor
  #define PIN_MQ7_VS  A7
  // PWM pin to control MQ-7 voltage
  #define PIN_MQ7_VC  6
#endif
#ifdef SENS_MOIST1
  #define PIN_MS1     A3
#endif
#ifdef SENS_MOIST2
  #define PIN_MS2     A2
#endif
#ifdef SENS_MOIST3
  #define PIN_MS3     A1
#endif
// Digital I/O pin for the speaker
#if defined(MOD_ALARM) || defined(MOD_BATTERY)
  #define PIN_ALM     5
#endif
// Analog battery monitor
#ifdef MOD_BATTERY
  #define PIN_BAT     A6
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////
// Libraries
//////////////////////////////////////////////////////////////////////////////////////////////////
#include <SPI.h>
#include <MySensors.h>
#include <Bounce2.h>
#ifdef SENS_DHT
  #include <DHT.h>
#endif
#ifdef SENS_LUX
  #include <Wire.h>
  #if MOD_LUX == TSL2561
    #include <Adafruit_Sensor.h>
    #include <Adafruit_TSL2561_U.h>
  #elif MOD_LUX == BH1750
    #include <BH1750.h>
  #endif
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////
// Objects
//////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef SENS_DOOR1
  Bounce door1 = Bounce();
#endif
#ifdef SENS_DOOR2
  Bounce door2 = Bounce();
#endif
#ifdef SENS_DOOR3
  Bounce door3 = Bounce(); 
#endif
#ifdef PIN_BUTTON
  Bounce rstBtn = Bounce();
#endif
#ifdef SENS_DHT
  DHT dht(PIN_DHT, DHTTYPE);
#endif
#ifdef SENS_LUX
  #if MOD_LUX == TSL2561
    Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
  #elif MOD_LUX == BH1750
    BH1750 lightMeter;
  #endif
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////
// Prototypes
//////////////////////////////////////////////////////////////////////////////////////////////////
//void sendMsg(MyMessage &msg, bool echo = false);


//////////////////////////////////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////////////////////////////////
bool initialValueSent = false;
unsigned long tmrAwake = 0;
bool isMetric = true;
bool isPresented = false;
bool noBat = false;
uint8_t sleepCycles = 0;
unsigned long ilTmr = 0;

// Motion sensor
#ifdef SENS_MOTION
  int movLast = -1;
#endif

// Door sensor 1
#ifdef SENS_DOOR1
  int dor1Last = -1;
  uint8_t dor1Update;
#endif

// Door sensor 2
#ifdef SENS_DOOR2
  int dor2Last = -1;
  uint8_t dor2Update;
#endif

// Door sensor 3
#ifdef SENS_DOOR3
  int dor3Last = -1;
  uint8_t dor3Update;
#endif

#ifdef SENS_DHT
  unsigned long dhtTimer = 0;
#endif

// Temperature sensor (DHT)
#ifdef MOD_TEMPERATURE
  float tmpLast = 0;
  uint8_t tmpUpdate;
#endif

// Humidity sensor (DHT)
#ifdef MOD_HUMIDITY
  float humLast = 0;
  uint8_t humUpdate;
#endif

// Light sensor
#if defined(SENS_LIGHT) || defined(SENS_LUX)
  int16_t lgtLast;
  uint8_t lgtUpdate;
  unsigned long lgtTimer = 0;
#endif
// Lux sensor
#ifdef SENS_LUX
  bool luxEnabled = false;
#endif


// Flame detector
#ifdef SENS_FLAME
  int flmLast = -1;
#endif

// Smoke sensor
#ifdef SENS_MQ2
  unsigned long mq2Timer = 0;
  uint8_t mq2Update;
  float mq2Ro = 10.0;       // This has to be tuned 10K Ohm
  float mq2Last = 0;        // Variable to save the raw value coming from the sensor
  // Two points are taken from the curve. With these two points, a line is formed 
  // which is "approximately equivalent" to the original curve.
  // data format:{ x, y, slope};
  /*
   * Calculate the point from the curve
   * log(200)=2.3 (200 is first point from ppm axe)
   * log(1.5)=0.17 (value for 200ppm)
   * log(10000)=4 (Last point from ppm axe )
   * log(0.247)= -0.61 (last point from Rs/R0 axe)
   * Slope = (y2 - y1)/(x2 - x1) = (-0.61 - 0.17) / (4 - 2.3) = -0.46
   */
  // LPG; point1: (lg200, 0.17), point2: (lg10000, -0.61)
  float mq2LPGCurve[3]  =  {2.3,0.17,-0.46};
  // Natural Gas; point1: (lg200, 0.49), point2: (lg10000, -0.15)
  float mq2GasCurve[3]  =  {2.3,0.49,-0.38};
  // CO; point1: (lg200, 0.71), point2: (lg10000,  0.12)
  float mq2COCurve[3]  =  {2.3,0.71,-0.35};
  // Smoke; point1: (lg200, 0.52), point2:(lg10000,-0.21)
  float mq2SmokeCurve[3] = {2.3,0.52,-0.43};
#endif

// Gas sensor
#ifdef SENS_MQ5
  unsigned long mq5Timer = 0;
  uint8_t mq5Update;
  float mq5Ro = 10.0;       // This has to be tuned 10K Ohm
  float mq5Last = 0;        // Variable to save the raw value coming from the sensor
  // LPG; point1: (lg200, -0.15), point2: (lg10000, -0.866)
  float mq5LPGCurve[3]  =  {2.3,-0.15,-0.42};
  // Natural Gas; point1: (lg200, -0.03), point2: (lg10000, -0.67)
  float mq5GasCurve[3]  =  {2.3,-0.03,-0.38};
#endif

// CO sensor
#ifdef SENS_MQ7
  #ifdef MOD_MQ7_FINE
    bool mq7Enable = false;
  #else
    bool mq7Enable = true;
  #endif
  uint8_t mq7Phase = 0;
  int Mq7Pwm;
  unsigned long mq7Timer = 0;
  uint8_t mq7Update;
  float mq7Ro = 10.0;       // This has to be tuned 10K Ohm
  float mq7Last = 0;        // Variable to save the raw value coming from the sensor
  // CO; point1: (lg50, 0.17), point2: (lg4000,  -1.04)
  float mq7COCurve[3]  =  {1.7,0.17,-0.64};
#endif

// Moist sensor 1
#ifdef SENS_MOIST1
  int ms1Last = 0;
  uint8_t ms1Update;
  float ms1Val = 512.0;
#endif

// Moist sensor 2
#ifdef SENS_MOIST2
  int ms2Last = 0;
  uint8_t ms2Update;
  float ms2Val = 512.0;
#endif

// Moist sensor 3
#ifdef SENS_MOIST3
  int ms3Last = 0;
  uint8_t ms3Update;
  float ms3Val = 512.0;
#endif

// Alarm
#ifdef MOD_ALARM
  bool almStatus = false;
  bool almTone = false;
  bool almCtrl = false;
  bool almReset = false;
  int almTrigger = 0;
  unsigned long almTimer = 0;
  unsigned long almTest = 0;
  unsigned long almRetrigger = 0;
#endif

#ifdef MOD_BATTERY
  int batRef11 = true;
  bool batCal = false;
  int batMaxV = 0;
  float batMult = 0.0;
  unsigned long batTimer = 0;
#endif

// Sleep
#ifndef SLEEP_OFF
  bool sleepEnabled = true;
  int slpTrigger = 0;
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////
// MySensor message handlers
//////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef CHILD_ID_MOV
  MyMessage msgMov(CHILD_ID_MOV, V_TRIPPED);
#endif
#ifdef CHILD_ID_DO1
  MyMessage msgDor(CHILD_ID_DO1, V_TRIPPED);
#endif
#ifdef CHILD_ID_DO2
  MyMessage msgDo2(CHILD_ID_DO2, V_TRIPPED);
#endif
#ifdef CHILD_ID_DO3
  MyMessage msgDo3(CHILD_ID_DO3, V_TRIPPED);
#endif
#ifdef CHILD_ID_TMP
  MyMessage msgTmp(CHILD_ID_TMP, V_TEMP);
#endif
#ifdef CHILD_ID_HUM
  MyMessage msgHum(CHILD_ID_HUM, V_HUM);
#endif
#ifdef CHILD_ID_LGT
  MyMessage msgLgt(CHILD_ID_LGT, V_LIGHT_LEVEL);
#endif
#ifdef CHILD_ID_LUX
  MyMessage msgLux(CHILD_ID_LUX, V_LEVEL);
#endif
#ifdef CHILD_ID_FLM
  MyMessage msgFlm(CHILD_ID_FLM, V_TRIPPED);
#endif
#ifdef CHILD_ID_SMK
  MyMessage msgSmk(CHILD_ID_SMK, V_LEVEL);
#endif
#ifdef CHILD_ID_LPG
  MyMessage msgLpg(CHILD_ID_LPG, V_LEVEL);
#endif
#ifdef CHILD_ID_GAS
  MyMessage msgGas(CHILD_ID_GAS, V_LEVEL);
#endif
#ifdef CHILD_ID_COS
  MyMessage msgCos(CHILD_ID_COS, V_LEVEL);
#endif
#ifdef CHILD_ID_MS1
  MyMessage msgMs1(CHILD_ID_MS1, V_LEVEL);
#endif
#ifdef CHILD_ID_MS2
  MyMessage msgMs2(CHILD_ID_MS2, V_LEVEL);
#endif
#ifdef CHILD_ID_MS3
  MyMessage msgMs3(CHILD_ID_MS3, V_LEVEL);
#endif
#ifdef CHILD_ID_LK1
  MyMessage msgLk1(CHILD_ID_LK1, V_TRIPPED);
#endif
#ifdef CHILD_ID_LK2
  MyMessage msgLk2(CHILD_ID_LK2, V_TRIPPED);
#endif
#ifdef CHILD_ID_LK3
  MyMessage msgLk3(CHILD_ID_LK3, V_TRIPPED);
#endif
#ifdef CHILD_ID_ALM
  MyMessage msgAlm(CHILD_ID_ALM, V_STATUS);
#endif
#ifdef CHILD_ID_BAT
  MyMessage msgBat(CHILD_ID_BAT, V_VOLTAGE);
#endif


/** **********************************************************************************************
  Setup pins, objects and do initial calibrations
*********************************************************************************************** */
void setup() {
  // Setup pin mode
  #ifdef PIN_BUTTON
    pinMode(PIN_BUTTON, INPUT_PULLUP);
  #endif
  #ifdef PIN_MOVE
    pinMode(PIN_MOVE, INPUT);
  #endif
  #ifdef PIN_DOOR1
    pinMode(PIN_DOOR1, INPUT);
  #endif
  #ifdef PIN_DOOR2
    pinMode(PIN_DOOR2, INPUT);
  #endif
  #ifdef PIN_DOOR3
    pinMode(PIN_DOOR3, INPUT);
  #endif
  #ifdef PIN_FLM
    pinMode(PIN_FLM, INPUT);
  #endif
  #ifdef PIN_MQ7_VC
    pinMode(PIN_MQ7_VC, OUTPUT);
  #endif
  
  // Activate internal pull-up
  #ifdef PIN_BUTTON
    digitalWrite(PIN_BUTTON, HIGH);
  #endif
  
  // MQ7 Voltage Control Pin initial state
  #ifdef PIN_MQ7_VC
    digitalWrite(PIN_MQ7_VC, LOW);
  #endif
  
  Serial.println(F(" "));
  Serial.println(F(" /===================================================================\\"));
  Serial.println(F(" |="));
  Serial.print(F(" |= "));
  Serial.print(MODULE_NAME);
  Serial.print(F(" | Ver: "));
  Serial.println(MODULE_VER);
  Serial.println(F(" |="));
  Serial.println(F(" \\===================================================================/"));
  Serial.println(F(" "));
  Serial.println(F(" "));

  // Debouncer pins
  #ifdef PIN_DOOR1
    door1.attach(PIN_DOOR1);
    door1.interval(5);
  #endif
  #ifdef PIN_DOOR2
    door2.attach(PIN_DOOR2);
    door2.interval(5);
  #endif
  #ifdef PIN_DOOR3
    door3.attach(PIN_DOOR3);
    door3.interval(5);
  #endif
  #ifdef PIN_BUTTON
    rstBtn.attach(PIN_BUTTON);
    rstBtn.interval(5);
  #endif
  
  // Set standard reference to VCC for analog sensors
  analogReference(DEFAULT);
  delay(100);

  // Initialize DHT sensor
  #ifdef SENS_DHT
    dht.begin();
  #endif

  // Initialize LUX sensor
  #ifdef SENS_LUX
    #if MOD_LUX == TSL2561
      /* Initialise the sensor */
      if(tsl.begin()) {
        luxEnabled = true;
        configureLuxSensor();
      }
      #ifdef MOD_DEBUG
        else {
            /* There was a problem detecting the TSL2561 ... check your connections */
            Serial.print("No TSL2561 LUX Sensor detected ... Check your wiring or I2C ADDR!");
        }
      #endif
      /* Setup the sensor gain and integration time */
    #elif MOD_LUX == BH1750
      Wire.begin();
      if(lightMeter.begin()) {
        luxEnabled = true;
      }
    #endif
  #endif

  // Calibrate Air Quality Sensors for the first time or load calibration values 
  // for subsequent occasions.
  #ifdef MOD_AQS
    #ifdef SENS_MQ7
      MQ7Calibration();
    #endif
    AQSCalibration(false);
  #endif

  #ifdef MOD_BATTERY
    // Get battery multipliers
    int pos = 10;
    // Get the battery voltage reference
    if ((BAT_R5 * BAT_V_MAX / (BAT_R5 + BAT_R4)) > 1.05) {
      batRef11 = false;
    } else {
      batRef11 = true;
    }
    // Get calibration flag
    pos += EEPROM_readAnything(pos, batCal);
    if (batCal) {
      pos += EEPROM_readAnything(pos, batMult);
    } else {
      // Initialize alarm pin to check for the calibration signal
      #ifdef PIN_ALM
        pinMode(PIN_ALM, INPUT_PULLUP);
      #endif
      if (batRef11) {
        batMult = (float)(1.1 * (BAT_R4 + BAT_R5)) / (float)(BAT_R5 * 1023UL);
      } else {
        batMult = (float)(MOD_VCC * (BAT_R4 + BAT_R5)) / (float)(BAT_R5 * 1023UL);
      }
      sleep(500);
      // Check if the Calibrate pin is low
      if (digitalRead(PIN_ALM) == LOW && digitalRead(PIN_ALM) == LOW) {
        sleep(1000);
        // Calibrate battery to max voltage
        batMult = BAT_V_MAX / readRaw(PIN_BAT);
        pos += EEPROM_writeAnything(pos, batMult);
        sleep(2000);
      }
    }
  #endif

  // Alarm Pin definition
  #ifdef PIN_ALM
    pinMode(PIN_ALM, OUTPUT);
    digitalWrite(PIN_ALM, LOW);
  #endif
  
  #ifndef SLEEP_OFF
    setSleep(false, 256);
  #endif
  
  // Sleep for a time to allow all sensors to power up (otherwise, errors might occur 
  //  for the first set of reads)
  sleep(2000);
}

/** **********************************************************************************************
  Present childs to the gateway
*********************************************************************************************** */
void presentation() {
  int ctr;
  sendSketchInfo(F(MODULE_NAME), F(MODULE_VER));
  //
  // Register sensors (they will be created as child devices)
  //
  #ifdef CHILD_ID_MOV
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_MOV, S_MOTION, F("PIR"), false) && ctr <= MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_DO1
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_DO1, S_DOOR, F("Door"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_DO2
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_DO2, S_DOOR, F("Door2"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_DO3
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_DO3, S_DOOR, F("Door3"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_TMP
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_TMP, S_TEMP, F("Temp"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_HUM
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_HUM, S_HUM, F("Humidity"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_LGT
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_LGT, S_LIGHT_LEVEL, F("Light%"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_LUX
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_LUX, S_LIGHT_LEVEL, F("Lux"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_FLM
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_FLM, S_SMOKE, F("Flame"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_SMK
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_SMK, S_AIR_QUALITY, F("Smoke"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_LPG
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_LPG, S_AIR_QUALITY, F("LPG"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_GAS
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_GAS, S_AIR_QUALITY, F("Natural Gas"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_COS
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_COS, S_AIR_QUALITY, F("Carbon Monoxide"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_MS1
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_MS1, S_MOISTURE  , F("Moisture 1"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_MS2
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_MS2, S_MOISTURE  , F("Moisture 2"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_MS3
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_MS3, S_MOISTURE  , F("Moisture 3"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_LK1
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_LK1, S_WATER_LEAK, F("Water leak 1"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_LK2
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_LK2, S_WATER_LEAK, F("Water leak 2"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_LK3
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_LK3, S_WATER_LEAK, F("Water leak 3"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_ALM
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_ALM, S_BINARY, F("Alarm"), false) && ctr < MY_PRESENTATION_RTR);
  #endif
  #ifdef CHILD_ID_BAT
    ctr = 0;
    do {
      wait(50);
      ctr++;
    } while (!present(CHILD_ID_BAT, S_MULTIMETER, F("Battery"), false) && ctr < MY_PRESENTATION_RTR);
  #endif

  isMetric = getControllerConfig().isMetric;
  isPresented = true;
  
  #ifndef SLEEP_OFF
    setSleep(true, 256);
  #endif
}


/** **********************************************************************************************
  Check sensors and send updates
*********************************************************************************************** */
void loop() {
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Loop timer
  ////////////////////////////////////////////////////////////////////////////////////////////////
  unsigned long lTmr = millis();
  bool msst = false;
  if (msst) msst = false;
  if(ilTmr == 0) {
    ilTmr = lTmr;
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Initial value
  ////////////////////////////////////////////////////////////////////////////////////////////////
  if (isPresented && !initialValueSent) {
    #ifdef MOD_DEBUG
      Serial.println(F("Sending initial values"));
    #endif
    #ifdef CHILD_ID_MOV
      msst = sendMsg(msgMov.set(0));
    #endif
    #ifdef CHILD_ID_DO1
      msst = sendMsg(msgDor.set(0));
    #endif
    #ifdef CHILD_ID_DO2
      msst = sendMsg(msgDo2.set(0));
    #endif
    #ifdef CHILD_ID_DO3
      msst = sendMsg(msgDo3.set(0));
    #endif
    #ifdef CHILD_ID_TMP
      msst = sendMsg(msgTmp.set(22));
    #endif
    #ifdef CHILD_ID_HUM
      msst = sendMsg(msgHum.set(0));
    #endif
    #ifdef CHILD_ID_LGT
      msst = sendMsg(msgLgt.set(0));
    #endif
    #ifdef CHILD_ID_LUX
      msst = sendMsg(msgLux.set(0));
    #endif
    #ifdef CHILD_ID_FLM
      msst = sendMsg(msgFlm.set(0));
    #endif
    #ifdef CHILD_ID_SMK
      msst = sendMsg(msgSmk.set(0));
    #endif
    #ifdef CHILD_ID_LPG
      msst = sendMsg(msgLpg.set(0));
    #endif
    #ifdef CHILD_ID_GAS
      msst = sendMsg(msgGas.set(0));
    #endif
    #ifdef CHILD_ID_COS
      msst = sendMsg(msgCos.set(0));
    #endif
    #ifdef CHILD_ID_MS1
      msst = sendMsg(msgMs1.set(0));
    #endif
    #ifdef CHILD_ID_MS2
      msst = sendMsg(msgMs2.set(0));
    #endif
    #ifdef CHILD_ID_MS3
      msst = sendMsg(msgMs3.set(0));
    #endif
    #ifdef CHILD_ID_LK1
      msst = sendMsg(msgLk1.set(0));
    #endif
    #ifdef CHILD_ID_LK2
      msst = sendMsg(msgLk2.set(0));
    #endif
    #ifdef CHILD_ID_LK3
      msst = sendMsg(msgLk3.set(0));
    #endif
    #ifdef CHILD_ID_ALM
      msst = sendMsg(msgAlm.set(0));
    #endif
    #ifdef CHILD_ID_BAT
      msst = sendMsg(msgBat.set(0.0, 2));
    #endif
    // Flag
    initialValueSent = true;
    //Serial.println(F("Requesting initial value from controller"));
    //request(CHILD_ID, V_STATUS);
    //wait(2000, C_SET, V_STATUS);
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Motion sensor
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef SENS_MOTION
    bool vMove = digitalRead(PIN_MOVE) == HIGH;
    if (vMove != movLast || sleepCycles >= SLEEP_CYCLES) {
      // Send the motion value
      if (sendMsg(msgMov.set(vMove ? "1" : "0"))) {
        movLast = vMove;
      }
      //if (vMove == LOW) {
      //  noBat = true;
      //}
      #ifdef MOD_DEBUG
        Serial.print(F("Motion: "));
        Serial.println(vMove);
      #endif
    }
  #endif
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Door 1 sensor
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef SENS_DOOR1
    door1.update();
    int vDoor = door1.read();
    if (vDoor != dor1Last || sleepCycles >= SLEEP_CYCLES) {
      // Send the door value
      if (sendMsg(msgDor.set(vDoor == HIGH ? "1" : "0"))) {
        dor1Last = vDoor;
      }
      #ifdef MOD_DEBUG
        Serial.print(F("Door 1: "));
        Serial.println(vDoor);
      #endif
    }
  #endif
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Door 2 sensor
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef SENS_DOOR2
    door2.update();
    int vDoor2 = door2.read();
    if (vDoor2 != dor2Last || sleepCycles >= SLEEP_CYCLES) {
      // Send the door value
      if (sendMsg(msgDo2.set(vDoor2 == HIGH ? "1" : "0"))) {
        dor2Last = vDoor2;
      }
      #ifdef MOD_DEBUG
        Serial.print(F("Door 2: "));
        Serial.println(vDoor2);
      #endif
    }
  #endif
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Door 3 sensor
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef SENS_DOOR3
    door3.update();
    int vDoor3 = door3.read();
    if (vDoor3 != dor3Last || sleepCycles >= SLEEP_CYCLES) {
      // Send the door value
      if (sendMsg(msgDo3.set(vDoor3 == HIGH ? "1" : "0"))) {
        dor3Last = vDoor3;
      }
      #ifdef MOD_DEBUG
        Serial.print(F("Door 3: "));
        Serial.println(vDoor3);
      #endif
    }
  #endif
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Temperature and Humidity sensor
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef SENS_DHT
    //////////////////////////////////////////////////////////////////////////////////////////////
    // Temp. Sensor
    //////////////////////////////////////////////////////////////////////////////////////////////
    if(dhtTimer > UPD_INTERVAL_DHT) {
      dhtTimer = 0;
      // Force reading sensor, so it works also after sleep()
      dht.read(true);
      #ifdef MOD_TEMPERATURE
        // Get temperature from DHT library
        float vTemperature = dht.readTemperature();
        if (isnan(vTemperature)) {
          #ifdef MOD_DEBUG
            Serial.println(F("Failed reading temperature from DHT!"));
          #endif
          delay(10);
        } else if (vTemperature != tmpLast || tmpUpdate >= UPD_N_READS) {
          // Only send temperature if it changed since the last measurement or if we didn't send an update for N times
          // Apply the offset before converting
          vTemperature += TMP_OFFSET;
          if (!isMetric) {
            vTemperature = dht.convertCtoF(vTemperature);
          }
          if (sendMsg(msgTmp.set(vTemperature, 1))) {
            tmpLast = vTemperature;
          }
          #ifdef MOD_DEBUG
            Serial.print(F("Temperature: "));
            Serial.println(vTemperature);
          #endif
          // Reset no updates counter
          tmpUpdate = 0;
        } else {
          // Increase no update counter if the temperature did not changed
          tmpUpdate++;
        }
      #endif
      
      ////////////////////////////////////////////////////////////////////////////////////////////
      // Humidity Sensor
      ////////////////////////////////////////////////////////////////////////////////////////////
      #ifdef MOD_HUMIDITY
        // Get humidity from DHT library
        float vHumidity = dht.readHumidity();
        if (isnan(vHumidity)) {
          #ifdef MOD_DEBUG
            Serial.println(F("Failed reading humidity from DHT"));
          #endif
          delay(10);
        } else if (vHumidity != humLast || humUpdate >= UPD_N_READS) {
          // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
          if (sendMsg(msgHum.set(vHumidity, 1))) {
            humLast = vHumidity;
          }
          #ifdef MOD_DEBUG
            Serial.print(F("Humidity: "));
            Serial.println(vHumidity);
          #endif
          // Reset no updates counter
          humUpdate = 0;
        } else {
          // Increase no update counter if the humidity did not changed
          humUpdate++;
        }
      #endif
    }
  #endif
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Light sensor
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef SENS_LIGHT
    //LIGHT_MAX
    if(lgtTimer > UPD_INTERVAL_LGT) {
      lgtTimer = 0;
      // Read light sensor in %
      int16_t vLight = (((readRaw(PIN_LGT) / 1023 * MOD_VCC) - LIGHT_MIN) / (LIGHT_MAX - LIGHT_MIN)) * 100;
      if (vLight < 0) {
        vLight = 0;
      } else if (vLight > 100) {
        vLight = 100;
      }
      // Update value
      if (vLight != lgtLast || lgtUpdate >= UPD_N_READS) {
        if (sendMsg(msgLgt.set(vLight))) {
          lgtLast = vLight;
        }
        #ifdef MOD_DEBUG
          Serial.print(F("Light level: "));
          Serial.println(vLight);
        #endif
        // Reset no updates counter
        lgtUpdate = 0;
      } else {
        lgtUpdate++;
      }
    }
  #endif
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // LUX sensor
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef SENS_LUX
    if(lgtTimer > UPD_INTERVAL_LGT) {
      lgtTimer = 0;      
      // Read lux sensor
      int16_t vLight = getLuxValue();
      if (vLight < LUX_MIN) {
        vLight = 0;
      } else if (vLight > LUX_MAX) {
        vLight = LUX_MAX;
      }
      // Update value
      if (vLight != lgtLast || lgtUpdate >= UPD_N_READS) {
        if (sendMsg(msgLux.set(vLight))) {
          lgtLast = vLight;
        }
        #ifdef MOD_DEBUG
          Serial.print(F("Lux level: "));
          Serial.println(vLight);
        #endif
        // Reset no updates counter
        lgtUpdate = 0;
      } else {
        lgtUpdate++;
      }
      lgtUpdate++;
    }
  #endif
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Flame
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef SENS_FLAME
    bool vFlame = digitalRead(PIN_FLM) == HIGH;
    if (vFlame != flmLast || sleepCycles >= SLEEP_CYCLES) {
      // Send the motion value
      if (sendMsg(msgFlm.set(vFlame ? "1" : "0"))) {
        flmLast = vFlame;
      }
      // Trigger alarm
      #ifdef MOD_ALARM
        setAlarm(vFlame, 1, 0);
      #endif
      #ifdef MOD_DEBUG
        Serial.print(F("Flame: "));
        Serial.println(vFlame);
      #endif
    }
  #endif
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // MQ-2 Smoke sensor
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef SENS_MQ2
    if(mq2Timer > UPD_INTERVAL_MQ2) {
      #ifdef MOD_DEBUG
        Serial.print(F("Update MQ2: "));
      #endif
      mq2Timer = 0;
      float mq2RoVal = MQRead(PIN_MQ2, AQS_MQ2);
      #ifdef MOD_DEBUG
        Serial.println(mq2RoVal);
      #endif
      if (mq2RoVal != mq2Last || mq2Update >= UPD_N_READS) {
        mq2Last = mq2RoVal;
        uint16_t mq2Val = 0;
        if (mq2Val == 0) mq2Val = 0;
        // Report Smoke
        #ifdef MOD_SMOKE
          mq2Val = ceil(MQGetGasPercentage(mq2RoVal / mq2Ro, AQS_SMOKE, AQS_MQ2));
          msst = sendMsg(msgSmk.set((int16_t)mq2Val));
          #ifdef MOD_ALARM
            if (mq2Val > AQS_TH_SMOKE) {
              setAlarm(true, 2, 0);
            } else if (mq2Val < AQS_TH_SMOKE_OFF) {
              setAlarm(false, 2, 0);
            }
          #endif
        #endif
        // Report LPG
        #if defined(MOD_LPG) && !defined(SENS_MQ5)
          mq2Val = ceil(MQGetGasPercentage(mq2RoVal / mq2Ro, AQS_LPG, AQS_MQ2));
          msst = sendMsg(msgLpg.set((int16_t)mq2Val));
          #ifdef MOD_ALARM
            if (mq2Val > AQS_TH_LPG) {
              setAlarm(true, 4, 0);
            } else if (mq2Val < AQS_TH_LPG_OFF) {
              setAlarm(false, 4, 0);
            }
          #endif
        #endif
        // Report Natural Gas
        #if defined(MOD_GAS) && !defined(SENS_MQ5)
          mq2Val = ceil(MQGetGasPercentage(mq2RoVal / mq2Ro, AQS_GAS, AQS_MQ2));
          msst = sendMsg(msgGas.set((int16_t)mq2Val));
          #ifdef MOD_ALARM
            if (mq2Val > AQS_TH_GAS) {
              setAlarm(true, 8, 0);
            } else if (mq2Val < AQS_TH_GAS_OFF) {
              setAlarm(false, 8, 0);
            }
          #endif
        #endif
        // Report Carbon Monoxide
        #if defined(MOD_CO) && (defined(MOD_MQ7_FINE) || !defined(SENS_MQ7))
          mq2Val = ceil(MQGetGasPercentage(mq2RoVal / mq2Ro, AQS_CO, AQS_MQ2));
          msst = sendMsg(msgCos.set((int16_t)mq2Val));
          #if defined(MOD_MQ7_FINE)
            if (mq2Val > AQS_TH_CO_DET) {
              mq7Enable = true;
            }
          #elif defined(MOD_ALARM)
            if (mq2Val > AQS_TH_CO) {
              setAlarm(true, 16, 0);
            } else if (mq2Val < AQS_TH_CO_OFF) {
              setAlarm(false, 16, 0);
            }
          #endif
        #endif
        mq2Update = 0;
      } else {
        mq2Update++;
      }
    }
  #endif
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // MQ-5 Gas sensor
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef SENS_MQ5
    if(mq5Timer > UPD_INTERVAL_MQ5) {
      mq5Timer = 0;
      float mq5RoVal = MQRead(PIN_MQ5, AQS_MQ5);
      if (mq5RoVal != mq5Last || mq5Update >= UPD_N_READS) {
        mq5Last = mq5RoVal;
        uint16_t mq5Val = 0;
        // Report LPG
        #if defined(MOD_LPG)
          mq5Val = ceil(MQGetGasPercentage(mq5RoVal / mq5Ro, AQS_LPG, AQS_MQ5));
          msst = sendMsg(msgLpg.set((int16_t)mq5Val));
          #ifdef MOD_ALARM
            if (mq5Val > AQS_TH_LPG) {
              setAlarm(true, 4, 0);
            } else if (mq5Val < AQS_TH_LPG_OFF) {
              setAlarm(false, 4, 0);
            }
          #endif
        #endif
        // Report Natural Gas
        #if defined(MOD_GAS)
          mq5Val = ceil(MQGetGasPercentage(mq5RoVal / mq5Ro, AQS_GAS, AQS_MQ5));
          msst = sendMsg(msgGas.set((int16_t)mq5Val));
          #ifdef MOD_ALARM
            if (mq5Val > AQS_TH_GAS) {
              setAlarm(true, 8, 0);
            } else if (mq5Val < AQS_TH_GAS_OFF) {
              setAlarm(false, 8, 0);
            }
          #endif
        #endif
        mq5Update = 0;
      } else {
        mq5Update++;
      }
    }
  #endif
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // MQ-7 CO sensor
  //
  // Phases:
  //   0 - Start
  //   1 - Heating
  //   2 - Low voltage
  //   3 - Sample
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef SENS_MQ7
    #if !defined(SENS_MQ2) || !defined(MOD_MQ7_FINE)
      mq7Enable = true;
    #endif
    if (mq7Enable) {
      #ifndef SLEEP_OFF
        setSleep(false, 64);
      #endif
      // Heating phase
      if (mq7Phase <= 0) {
        //Start heating phase
        mq7Phase = 1;
        analogWrite(PIN_MQ7_VC, 255);
        mq7Timer = millis();
      } else if (mq7Phase == 1) {
        if((unsigned long)(lTmr - mq7Timer) > MQ7_HEAT) {
          // Start low voltage phase
          mq7Phase = 2;
          analogWrite(PIN_MQ7_VC, Mq7Pwm);
          mq7Timer = millis();
        }
      } else if (mq7Phase == 2) {
        if((unsigned long)(lTmr - mq7Timer) > MQ7_LOWV) {
          // Start sample phase
          mq7Phase = 3;
          mq7Timer = millis();
        }
      } else if (mq7Phase >= 3) {
        float mq7RoVal = MQRead(PIN_MQ7, AQS_MQ7);
        if (mq7RoVal != mq7Last || mq7Update >= UPD_N_READS) {
          mq7Last = mq7RoVal;
          // Report CO
          uint16_t mq7Val = ceil(MQGetGasPercentage(mq7RoVal / mq7Ro, AQS_CO, AQS_MQ7));
          msst = sendMsg(msgCos.set((int16_t)mq7Val));
          #ifdef MOD_ALARM
            if (mq7Val5Val > AQS_TH_CO) {
              setAlarm(true, 16, 0);
            } else if (mq7Val5Val < AQS_TH_CO_OFF) {
              setAlarm(false, 16, 0);
            }
          #endif
          #ifdef MOD_MQ7_FINE
            // Turn Off MQ-7 if CO levels are acceptable
            if (mq7Val < AQS_TH_CO_DET) {
              mq7Enable = false;
              #ifndef SLEEP_OFF
                setSleep(true, 64);
              #endif
            }
          #endif
          mq7Update = 0;
        } else {
          mq7Update++;
        }
        // Back to start
        mq7Phase = 0;
      }
    }
  #endif

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Moisture / leak sensors
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef SENS_MOIST1
    int vMoist1 = moistRead(PIN_MS1);
    if (vMoist1 != ms1Last || ms1Update >= UPD_N_READS) {
      if (sendMsg(msgMs1.set(vMoist1))) {
        ms1Last = vMoist1;
      }
      #ifdef MOD_DEBUG
        Serial.print(F("Moisture 1 level: "));
        Serial.println(vMoist1);
      #endif
      ms1Update = 0;
      #ifdef MOD_LEAK1
        if (vMoist1 >= LEAK_ON) {
          msst = sendMsg(msgLk1.set("1"));  // Leak detected
          #ifdef MOD_ALARM
            // Trigger alarm
            setAlarm(true, 32, 0);
          #endif
          #ifdef MOD_DEBUG
            Serial.print(F("Leak 1 detected."));
          #endif
        } else {
          msst = sendMsg(msgLk1.set("0"));  // Leak dried
          #ifdef MOD_ALARM
            if (vMoist1 < LEAK_OFF) {
              // Alarm off
              setAlarm(false, 32, 0);
            }
          #endif
          #ifdef MOD_DEBUG
            Serial.print(F("Leak 1 cleared."));
          #endif
        }
      #endif
    } else {
      ++ms1Update;
    }
  #endif
  
  #ifdef SENS_MOIST2
    int vMoist2 = moistRead(PIN_MS2);
    if (vMoist2 != ms2Last || ms2Update >= UPD_N_READS) {
      if (sendMsg(msgMs2.set(vMoist2))) {
        ms2Last = vMoist2;
      }
      #ifdef MOD_DEBUG
        Serial.print(F("Moisture 2 level: "));
        Serial.println(vMoist2);
      #endif
      ms2Update = 0;
      #ifdef MOD_LEAK2
        if (vMoist2 >= LEAK_ON) {
          msst = sendMsg(msgLk2.set("1"));  // Leak detected
          #ifdef MOD_ALARM
            // Trigger alarm
            setAlarm(true, 64, 0);
          #endif
          #ifdef MOD_DEBUG
            Serial.print(F("Leak 2 detected."));
          #endif
        } else {
          msst = sendMsg(msgLk2.set("0"));  // Leak dried
          #ifdef MOD_ALARM
            if (vMoist2 < LEAK_OFF) {
              // Alarm off
              setAlarm(false, 64, 0);
            }
          #endif
          #ifdef MOD_DEBUG
            Serial.print(F("Leak 2 cleared."));
          #endif
        }
      #endif
    } else {
      ++ms2Update;
    }
  #endif

  #ifdef SENS_MOIST3
    int vMoist3 = moistRead(PIN_MS3);
    if (vMoist3 != ms3Last || ms3Update >= UPD_N_READS) {
      if (sendMsg(msgMs3.set(vMoist3))) {
        ms3Last = vMoist3;
      }
      #ifdef MOD_DEBUG
        Serial.print(F("Moisture 3 level: "));
        Serial.println(vMoist3);
      #endif
      ms3Update = 0;
      #ifdef MOD_LEAK3
        if (vMoist3 >= LEAK_ON) {
          msst = sendMsg(msgLk3.set("1"));  // Leak detected
          #ifdef MOD_ALARM
            // Trigger alarm
            setAlarm(true, 128, 0);
          #endif
          #ifdef MOD_DEBUG
            Serial.print(F("Leak 3 detected."));
          #endif
        } else {
          msst = sendMsg(msgLk3.set("0"));  // Leak dried
          #ifdef MOD_ALARM
            if (vMoist3 < LEAK_OFF) {
              // Alarm off
              setAlarm(false, 128, 0);
            }
          #endif
          #ifdef MOD_DEBUG
            Serial.print(F("Leak 3 cleared."));
          #endif
        }
      #endif
    } else {
      ++ms3Update;
    }
  #endif

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Alarm
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef MOD_ALARM
    if (almStatus && almTest > 0) {
      if((unsigned long)(lTmr - almTest) > ALARM_TEST) {
        setAlarm(false, 0, 2);
      }
    }
    if (almStatus) {
      if(!almCtrl || (unsigned long)(lTmr - almTimer) > ALARM_INTERVAL) {
        almCtrl = true;
        // Reset timer
        almTimer = millis();
        // Set tone
        if (almTone) {
          noTone(PIN_ALM);
          #ifdef ALM_DOUBLE
            tone(PIN_ALM, ALM_TONE2);
          #endif
        } else {
          noTone(PIN_ALM);
          tone(PIN_ALM, ALM_TONE1);
        }
        // Swap tone
        almTone = !almTone;
      }
      #ifndef SLEEP_OFF
        setSleep(false, 1);
      #endif
    } else if (almCtrl) {
      noTone(PIN_ALM);
      almCtrl = false;
      #ifndef SLEEP_OFF
        setSleep(true, 1);
      #endif
      #ifdef MOD_DEBUG
        Serial.println(F("Turn Off alarm!"));
      #endif
    }
  #endif
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Button
  ////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef PIN_BUTTON
    rstBtn.update();
    if (rstBtn.fell()) {
      // Prevent module from sleeping while button id down
      #ifndef SLEEP_OFF
        setSleep(false, 2);
      #endif
      #ifdef MOD_DEBUG
        Serial.println(F("Button Down."));
      #endif
    } else if (rstBtn.rose()) {
      #ifdef MOD_DEBUG
        Serial.println(F("Button Up."));
      #endif
      #if defined(MOD_AQS) || defined(MOD_MOIST)
        if(rstBtn.previousDuration() > BTN_CALIBRATE) {
          #ifdef MOD_AQS
            // Calibrate Air Quality sensors
            #ifdef SENS_MQ7
              MQ7Calibration();
            #endif
            AQSCalibration(true);
          #endif
        }
      #endif
      #if defined(MOD_ALARM) && (defined(MOD_AQS) || defined(MOD_MOIST))
        else
      #endif
      #ifdef MOD_ALARM
        if(rstBtn.previousDuration() > BTN_TEST) {
          // Test alarm
          setAlarm(true, 0, 2);
          #ifdef MOD_DEBUG
            Serial.println(F("Test Alarm."));
          #endif
        } else {
          // Reset (turn off) alarm
          setAlarm(false, 0, 1);
          #ifdef MOD_DEBUG
            Serial.println(F("Reset Alarm."));
          #endif
        }
      #endif
      #ifndef SLEEP_OFF
        setSleep(true, 2);
      #endif
    }
  #endif
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Sleep!
  ////////////////////////////////////////////////////////////////////////////////////////////////
  unsigned long tElapsed = 0;
  #ifdef SLEEP_OFF
    #ifdef MOD_BATTERY
      // Report battery level
      if(batTimer > AWAKE_INTERVAL) {
        getBat();
      }
    #endif
  #else
    // Start sleep sequence if the alarm is off and we have been awake for a while
    if(sleepEnabled && (unsigned long)(lTmr - tmrAwake) > AWAKE_INTERVAL) {
      #ifdef MOD_DEBUG
        Serial.println(F("Sleep."));
      #endif
      // Sleep until interrupt comes in on motion or door sensors, or after SLEEP_TIME ms (2 min. default).
      #if defined(PIN_IRQ2) && defined(PIN_IRQ3)
        int reason = sleep(digitalPinToInterrupt(PIN_IRQ2), CHANGE, digitalPinToInterrupt(PIN_IRQ3), CHANGE, SLEEP_TIME);
      #elif defined(PIN_IRQ2)
        int reason = sleep(digitalPinToInterrupt(PIN_IRQ2), CHANGE, SLEEP_TIME);
      #elif defined(PIN_IRQ3)
        int reason = sleep(digitalPinToInterrupt(PIN_IRQ3), CHANGE, SLEEP_TIME);
      #endif
      // Wake up
      //
      // Reset awake timer
      tmrAwake = millis();
      sleepCycles++;
      if (sleepCycles > SLEEP_CYCLES) {
        sleepCycles = 0;
      }
      // Update running timers
      if (reason < 0) {
        // Timed wakeup
        tElapsed = AWAKE_INTERVAL + SLEEP_TIME;
      } else {
        // Sensor wakeup (estimate elapsed time)
        tElapsed = SLEEP_TIME / 4;
      }
      #ifdef MOD_DEBUG
        Serial.print(F("WakeUp. "));
        Serial.println(reason);
      #endif
      #ifdef MOD_BATTERY
        // Report battery level
        delay(100);
        getBat();
      #endif
    } else if(batTimer > AWAKE_INTERVAL) {
      // Report battery level
      getBat();
    }
  #endif
  tElapsed = (millis() - ilTmr);
  ilTmr = millis();
  #ifdef SENS_DHT
    dhtTimer += tElapsed;
  #endif
  #ifdef CHILD_ID_LGT
    lgtTimer += tElapsed;
  #endif
  #ifdef MOD_BATTERY
    batTimer += tElapsed;
  #endif
  #ifdef SENS_MQ2
    mq2Timer += tElapsed;
  #endif
  #ifdef SENS_MQ5
    mq5Timer += tElapsed;
  #endif
  #ifdef MOD_ALARM
    if (almReset) {
      almRetrigger += tElapsed;
    }
  #endif
}

/** **********************************************************************************************
  Receive messages
*********************************************************************************************** */
void receive(const MyMessage &message) {
  if (message.isAck()) {
    #ifdef MOD_DEBUG
      Serial.println(F("This is an ack from gateway"));
    #endif
  }
  #ifdef MOD_ALARM
    if (message.sensor == CHILD_ID_ALM) {
      if (message.getType() == V_STATUS) {
        setAlarm(message.getBool(), 0, 2);
        #ifdef MOD_DEBUG
          Serial.print(F("Alarm set remotely: "));
          Serial.println(almStatus);
        #endif
      }
    }
  #endif
}

/** **********************************************************************************************
  Send a message with retry 
  
  @param msg MySensors message
*********************************************************************************************** */
bool sendMsg(MyMessage &msg) {
  if (!isPresented) return false;
  int ctr = 0;
  bool ret = false;
  do {
    ret = send(msg);
    ctr++;
    delay(25);
  } while (!ret && ctr < MY_MESSAGE_RTR);
  return ret;
}

/** **********************************************************************************************
  Read analog raw value
  
  @param pin Analog pin
  @return Returns the analog raw value
*********************************************************************************************** */
int readRaw(int pin) {
  int i;
  int rs = 0;

  for (i = 0; i < AG_READ_TIMES; i++) {
    rs += analogRead(pin);
    delay(AG_READ_INTERVAL);
  }
  return (rs / AG_READ_TIMES);
}

/** **********************************************************************************************
  Write a value to EEPROM

  @param ee MySensors EEPROM position [1 - 255]
  @param value Value variable
  @return Returns the number of bytes written to EEPROM
*********************************************************************************************** */
template <class T> int EEPROM_writeAnything(int ee, const T& value) {
   const byte* p = (const byte*)(const void*)&value;
   unsigned int i;
   for (i = 0; i < sizeof(value); i++)
       saveState(ee++, *p++);
   return i;
}

/** **********************************************************************************************
  Read a value from EEPROM

  @param ee MySensors EEPROM position [1 - 255]
  @param value Value variable
  @return Returns the number of bytes read from EEPROM
*********************************************************************************************** */
template <class T> int EEPROM_readAnything(int ee, T& value) {
   byte* p = (byte*)(void*)&value;
   unsigned int i;
   for (i = 0; i < sizeof(value); i++)
       *p++ = loadState(ee++);
   return i;
}

#ifdef MOD_BATTERY
/** **********************************************************************************************
  Report Battery status
*********************************************************************************************** */
void getBat() {
  bool msst = false;
  if (msst) msst = false;
  batTimer = 0;
  if (noBat) {
    noBat = false;
    return;
  }
  float batVolt;
  // Set the analog references
  if (batRef11) {
    analogReference(INTERNAL);
    delay(50);
    // Dummy read to allow time for the ADC to stabilize
    analogRead(PIN_BAT);
  }
  // Calculate voltage
  batVolt = (float)(readRaw(PIN_BAT) * batMult);
  // Calculate percentage
  int batPr = (float)(batVolt - BAT_V_MIN) / (float)(BAT_V_MAX - BAT_V_MIN) * 100;
  if (batPr < 0) {
    batPr = 0;
  } else if (batPr > 100) {
    batPr = 100;
  }
  // Publish values
  sendBatteryLevel(batPr);
  msst = sendMsg(msgBat.set(batVolt, 2));
  #ifdef MOD_DEBUG
    Serial.print(F("Bat %: "));
    Serial.println(batPr);
    Serial.print(F("Bat Volts: "));
    Serial.println(batVolt);
  #endif
  // Reset analog reference
  if (batRef11) {
    analogReference(DEFAULT);
    delay(100);
  }
}
#endif

#ifndef SLEEP_OFF
/** **********************************************************************************************
  Enable / Disable deep sleep 

  @param stat Sleep On (true) or Off (false)
  @param trigger Element triggering the sleep lock:
    - 1   Alarm
    - 2   Button
    - 4
    - 8
    - 16
    - 32
    - 64  MQ7 Sensor
    - 128
    - 256 Presentation phase
*********************************************************************************************** */
void setSleep(bool stat, int trigger) {
  if (stat) {
    // Clear triggering element
    slpTrigger ^= trigger;
    if (slpTrigger == 0) {
      sleepEnabled = true;
      tmrAwake = millis();
    }
  } else {
    sleepEnabled = false;
    // Set triggering element
    slpTrigger |= trigger;
  }
}
#endif

#ifdef MOD_ALARM
/** **********************************************************************************************
  Set the status of the alarm

  @param stat Alarm status On or Off
  @param trigger Sensor triggering the alarm
    - 0   = Test
    - 1   = Flame sensor
    - 2   = Smoke
    - 4   = LPG
    - 8   = Gas
    - 16  = CO
    - 32  = Moisture 1
    - 64  = Moisture 2
    - 128 = Moisture 3
  @param mode Trigger mode
    - 0 = Normal
    - 1 = Manual Reset
    - 2 = Test
*********************************************************************************************** */
void setAlarm(bool stat, int trigger, int mode) {
  if (almStatus != stat) {
    if (stat) {
      almCtrl = false;
    }
    almTone = false;
  }
  // Turn alarm On
  if (stat) {
    // If the alarm is:
    // - Not on reset mode
    // - On reset mode and the reset timer has elapsed
    if (!almReset || almRetrigger > ALARM_RETRIGGER) {
      // Turn on alarm
      almStatus = true;
      almTimer = millis();
      // Clear retrigger
      almReset = false;
      almRetrigger = 0;
      // Set triggering element
      almTrigger |= trigger;
      // Enable test mode if no sensors have triggered the alarm
      if (mode == 2 && almTrigger == 0) {
        almTest = almTimer;
        if (almTest == 0) {
          almTest = 1;
        }
      } else {
        // Turn off test mode
        almTest = 0;
      }
    }
  } else {
    // Clear triggering element
    almTrigger ^= trigger;
    // If no triggering elements, turn off
    if (almTrigger == 0) {
      almStatus = false;
      almRetrigger = 0;
      almReset = false;
    } else
    // Reset mode 
    if (mode == 1) {
      almStatus = false;
      almReset = true;
      almRetrigger = 0;
    }
  }
}
#endif

#ifdef SENS_LUX
  #if MOD_LUX == TSL2561
    /**************************************************************************/
    /*
        Configures the gain and integration time for the TSL2561
    */
    /**************************************************************************/
    void configureLuxSensor(void) {
      /* You can also manually set the gain or enable auto-gain support */
      // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
      // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
      tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
      
      /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
      tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
      // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
      // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */
      
      #ifdef MOD_DEBUG
        /* Update these values depending on what you've set above! */  
        Serial.print  ("LUX Sensor Gain:  "); Serial.println("Auto");
        Serial.print  ("LUX SensorTiming: "); Serial.println("13 ms");
      #endif
    }
  #endif

/** **********************************************************************************************
  This function Reads the LUX value from the sensor.
   
  @return LuX value
*********************************************************************************************** */
int16_t getLuxValue(void) {
  if (!luxEnabled) return 0;
  #if MOD_LUX == TSL2561
    /* Get a new sensor event */ 
    sensors_event_t event;
    tsl.getEvent(&event);
    /* Display the results (light is measured in lux) */
    if (event.light) {
      return event.light;
    }
    #ifdef MOD_DEBUG
      /* If event.light = 0 lux the sensor is probably saturated
         and no reliable data could be generated! */
      Serial.println("LUX Sensor overload");
    #endif
  #elif MOD_LUX == BH1750
    if (lightMeter.measurementReady()) {
      return (int16_t)lightMeter.readLightLevel();
    }
  #endif
  return 0;
}
#endif

#ifdef MOD_AQS
/** **********************************************************************************************
  This function use MQResistanceCalculation to calculate the sensor resistance (Rs).
  The Rs changes as the sensor is in the different concentration of the target gas.
   
  @param mq_pin Analog channel
  @param sensor Sensor ID
  @return 
*********************************************************************************************** */
float MQRead(int mq_pin, int sensor) {
  float rs=0;

  rs = MQResistanceCalculation(readRaw(mq_pin), sensor);
  return rs;
}

/** **********************************************************************************************
  The sensor and the load resistor forms a voltage divider. Given the voltage across 
  the load resistor and its resistance, the resistance of the sensor could be derived.
   
  @param raw_adc Raw value read from adc, which represents the voltage
  @param sensor Sensor ID
  @return The calculated sensor resistance
*********************************************************************************************** */
float MQResistanceCalculation(int raw_adc, int sensor) {
  #ifdef SENS_MQ7
    if ( sensor == AQS_MQ7 ) {
      return ( ((float)MQ7_RL_VALUE*(1023-raw_adc)/raw_adc));
    }
  #endif
  #ifdef SENS_MQ5
    if( sensor == AQS_MQ5 ) {
      return ( ((float)MQ5_RL_VALUE*(1023-raw_adc)/raw_adc));
    }
  #endif
  #ifdef SENS_MQ2
    return ( ((float)MQ2_RL_VALUE*(1023-raw_adc)/raw_adc));
  #else
    return 200000;
  #endif
}

/** **********************************************************************************************
  This function passes different curves to the MQGetPercentage function which calculates 
  the ppm (parts per million) of the target gas using the MQ-2 sensor.
   
  @param rs_ro_ratio Rs divided by Ro
  @param gas_id Target gas type
  @param sensor Sensor ID
  @return PPM of the target gas
*********************************************************************************************** */
int MQGetGasPercentage(float rs_ro_ratio, int gas_id, int sensor) {
  #ifdef SENS_MQ7
    if (sensor == AQS_MQ7) {
      if ( gas_id == AQS_CO ) {
        return MQGetPercentage(rs_ro_ratio, mq7COCurve);
      }
      return 0;
    }
  #endif
  #ifdef SENS_MQ5
    if (sensor == AQS_MQ5) {
      if ( gas_id == AQS_LPG ) {
        return MQGetPercentage(rs_ro_ratio, mq5LPGCurve);
      } else if ( gas_id == AQS_GAS ) {
        return MQGetPercentage(rs_ro_ratio, mq5GasCurve);
      }
      return 0;
    }
  #endif
  #ifdef SENS_MQ2
    if ( gas_id == AQS_LPG ) {
      return MQGetPercentage(rs_ro_ratio, mq2LPGCurve);
    } else if ( gas_id == AQS_GAS ) {
      return MQGetPercentage(rs_ro_ratio, mq2GasCurve);
    } else if ( gas_id == AQS_CO ) {
      return MQGetPercentage(rs_ro_ratio, mq2COCurve);
    } else if ( gas_id == AQS_SMOKE ) {
      return MQGetPercentage(rs_ro_ratio, mq2SmokeCurve);
    }
  #endif
  return 0;
}

/** **********************************************************************************************
  By using the slope and a point of the line. The x(logarithmic value of ppm) of the 
  line could be derived if y(rs_ro_ratio) is provided. As it is a logarithmic coordinate, 
  power of 10 is used to convert the result to non-logarithmic value.
   
  @param rs_ro_ratio Rs divided by Ro
  @param pcurve Pointer to the curve of the target gas
  @return PPM of the target gas
*********************************************************************************************** */
int  MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

/** **********************************************************************************************
  This function assumes that the sensor is in clean air. It use  
  MQResistanceCalculation to calculates the sensor resistance in clean air.
   
  @param mq_pin Analog channel
  @param sensor Sensor ID
  @return PPM of the target gas
*********************************************************************************************** */
float MQCalibration(int mq_pin, int sensor) {
  int i;
  float val=0;

  for (i=0; i < AQS_CAL_TIMES; i++) {        // Take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin), sensor);
    delay(AQS_CAL_INTERVAL);
  }
  val = val/AQS_CAL_TIMES;                   // Calculate the average value
  return val;
}

/** **********************************************************************************************
  Calibrate Air Quality Sensors and save values to EEPROM
  
  @param force Force re-calibration
*********************************************************************************************** */
void AQSCalibration (bool force) {
  int pos = 30;
  int pos2 = pos;
  bool mqInit = false;
  
  // MQ2 sensor calibration
  #ifdef SENS_MQ2
    if (!force) {
      pos += EEPROM_readAnything(pos, mqInit);
    }
    if (force || !mqInit) {
      #ifdef MOD_DEBUG
        Serial.println(F("MQ-2 sensor calibration."));
      #endif
      mq2Ro = MQCalibration(PIN_MQ2, AQS_MQ2) / MQ2_RO_CLEAN_AIR_FACTOR;
      mqInit = true;
      pos = pos2;
      pos += EEPROM_writeAnything(pos, mqInit);
      pos += EEPROM_writeAnything(pos, mq2Ro);
      #ifdef MOD_DEBUG
        Serial.print(F("MQ-2 sensor re-calibrated to: "));
        Serial.print(mq2Ro);
        Serial.println(F("K."));
      #endif
    } else {
      pos += EEPROM_readAnything(pos, mq2Ro);
      if (mq2Ro < 4) {
        mq2Ro = 10;
      }
      #ifdef MOD_DEBUG
        Serial.print(F("MQ-2 sensor calibrated at: "));
        Serial.print(mq2Ro);
        Serial.println(F("K."));
      #endif
    }
  #endif
  #ifdef SENS_MQ5
    pos2 = pos;
    mqInit = false;
    if (!force) {
      pos += EEPROM_readAnything(pos, mqInit);
    }
    if (force || !mqInit) {
      #ifdef MOD_DEBUG
        Serial.println(F("MQ-5 sensor calibration."));
      #endif
      mq5Ro = MQCalibration(PIN_MQ5, AQS_MQ5) / MQ5_RO_CLEAN_AIR_FACTOR;
      mqInit = true;
      pos = pos2;
      pos += EEPROM_writeAnything(pos, mqInit);
      pos += EEPROM_writeAnything(pos, mq5Ro);
      #ifdef MOD_DEBUG
        Serial.print(F("MQ-5 sensor re-calibrated to: "));
        Serial.print(mq5Ro);
        Serial.println(F("K."));
      #endif
    } else {
      pos += EEPROM_readAnything(pos, mq5Ro);
      if (mq5Ro < 4) {
        mq5Ro = 10;
      }
      #ifdef MOD_DEBUG
        Serial.print(F("MQ-5 sensor calibrated at: "));
        Serial.print(mq5Ro);
        Serial.println(F("K."));
      #endif
    }
  #endif
  #ifdef SENS_MQ7
    pos2 = pos;
    mqInit = false;
    if (!force) {
      pos += EEPROM_readAnything(pos, mqInit);
    }
    if (force || !mqInit) {
      #ifdef MOD_DEBUG
        Serial.println(F("MQ-7 sensor calibration."));
      #endif
      mq7Ro = MQCalibration(PIN_MQ7, AQS_MQ7) / MQ7_RO_CLEAN_AIR_FACTOR;
      mqInit = true;
      pos = pos2;
      pos += EEPROM_writeAnything(pos, mqInit);
      pos += EEPROM_writeAnything(pos, mq7Ro);
      #ifdef MOD_DEBUG
        Serial.print(F("MQ-7 sensor re-calibrated to: "));
        Serial.print(mq7Ro);
        Serial.println(F("K."));
      #endif
    } else {
      pos += EEPROM_readAnything(pos, mq7Ro);
      if (mq7Ro < 4) {
        mq7Ro = 10;
      }
      #ifdef MOD_DEBUG
        Serial.print(F("MQ-7 sensor calibrated at: "));
        Serial.print(mq7Ro);
        Serial.println(F("K."));
      #endif
    }
  #endif
}

#ifdef SENS_MQ7
/** **********************************************************************************************
 Calibrate MQ-7 Sensor
*********************************************************************************************** */
void MQ7Calibration() {
  float lastV = 5.0;            // Voltage at previous attempt
  float raw2v = 5.0 / 1024.0;   // Coefficient to convert Arduino's voltage

  // AnalogRead result into voltage in volts
  for(int w = 0; w < 255; w++) {
    analogWrite(PIN_MQ7_VC, w);
    float sampleV = 0;
    // Wait for the voltage to stabilize
    delay(50);
    // Sample
    for (int s = 0; s < 10; s++) {
      sampleV += analogRead(PIN_MQ7_VS);
      delay(5);
    }
    sampleV *= 0.1;
    #ifdef MOD_DEBUG
      float vlt = sampleV * 5.0 / 1024.0;
      Serial.print("Calibrating MQ7 voltage=");
      Serial.println(vlt);
    #endif
    if(sampleV <= 736 && lastV > 736) {
      // Find the Width that better approaches 1.4V
      float diffS = 736 - sampleV;
      float diffL = lastV - 736;
      if(diffS < diffL) {
        Mq7Pwm = w;
        return;
      } else {
        Mq7Pwm = w - 1;
        return;
      }
    }
    lastV = sampleV;
  }
}
#endif
#endif

#ifdef MOD_MOIST
/** **********************************************************************************************
  This function returns the percentage hmidity value.
   
  @param pin Analog pin
  @return 
*********************************************************************************************** */
int moistRead(int pin) {
  int vl = 0;

  vl = (((readRaw(pin) * MOD_VCC / 1023) - MOIST_MIN) / (MOIST_MAX - MOIST_MIN)) * 100;
  if (vl > 100) vl = 100;
  if (vl < 0) vl = 0;
  return vl;
}
#endif
