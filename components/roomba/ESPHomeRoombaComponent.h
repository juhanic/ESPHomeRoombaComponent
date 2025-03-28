#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
//#include "esphome/components/text_sensor/text_sensor.h"

#define ROOMBA_READ_TIMEOUT 200

namespace esphome {
namespace roomba {

class RoombaComponent : public uart::UARTDevice, public PollingComponent, public sensor::Sensor {
protected:
  InternalGPIOPin * brcPin;
  uint32_t updateInterval;
  sensor::Sensor *distanceSensor;
  sensor::Sensor *voltageSensor;
  sensor::Sensor *currentSensor;
  sensor::Sensor *chargeSensor;
  sensor::Sensor *capacitySensor;
  sensor::Sensor *batteryPercentSensor;
  sensor::Sensor *temperatureSensor;
  //  text_sensor::TextSensor *activitySensor;
 
public:
  void set_distance_sensor(sensor::Sensor *s) { distanceSensor = s; }
  void set_voltage_sensor(sensor::Sensor *s) { voltageSensor = s; }
  void set_current_sensor(sensor::Sensor *s) { currentSensor = s; }
  void set_charge_sensor(sensor::Sensor *s) { chargeSensor = s; }
  void set_capacity_sensor(sensor::Sensor *s) { capacitySensor = s; }
  void set_battery_sensor(sensor::Sensor *s) { batteryPercentSensor = s; }
  void set_temperature_sensor(sensor::Sensor *s) { temperatureSensor = s; }
  //void set_activity_sensor(text_sensor::TextSensor *s) { activitySensor = s; }
  void set_brc_pin(InternalGPIOPin *pin) { brcPin = pin; }
  void brc_wakeup() {
    // Roomba Wakeup
    brcPin->digital_write(false);
    delay(500);
    brcPin->digital_write(true);
    delay(100);
  }

  void flush() {
    while (available()) {
      read();
    }
  }

  void setup() override {
    brcPin->pin_mode(gpio::FLAG_OUTPUT);
    brcPin->digital_write(true);
  }

  void update() override {
    uint8_t charging;
    uint16_t voltage;
    int16_t current;
    uint16_t batteryCharge;
    uint16_t batteryCapacity;
    int16_t distance;
    int8_t temperature;

    flush();

    uint8_t sensors[] = {
      SensorChargingState,
      SensorVoltage,
      SensorCurrent,
      SensorBatteryCharge,
      SensorBatteryCapacity,
      SensorDistance,
      SensorBatteryTemperature
    };
    uint8_t values[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    bool success = getSensorsList(sensors, sizeof(sensors), values, sizeof(values));
    if (!success) {
      ESP_LOGD("custom", "Could not get sensor values from serial");
      return;
    }

    charging = values[0];
    voltage = values[1] * 256 + values[2];
    current = values[3] * 256 + values[4];
    batteryCharge = values[5] * 256 + values[6];
    batteryCapacity = values[7] * 256 + values[8];
    distance = values[9] * 256 + values[10];
    temperature = values[11];
    std::string activity = this->get_activity(charging, current);
    // Only publish new states if there was a change
    if (this->distanceSensor && this->distanceSensor->state != distance)
      this->distanceSensor->publish_state(distance);

    if (this->voltageSensor && this->voltageSensor->state != voltage)
      this->voltageSensor->publish_state(voltage);

    if (this->currentSensor && this->currentSensor->state != current)
      this->currentSensor->publish_state(current);

    if (this->chargeSensor && this->chargeSensor->state != batteryCharge)
      this->chargeSensor->publish_state(batteryCharge);

    if (this->capacitySensor && this->capacitySensor->state != batteryCapacity)
      this->capacitySensor->publish_state(batteryCapacity);

    float battery_level = 100.0 * ((1.0 * batteryCharge) / (1.0 * batteryCapacity));
    if (this->batteryPercentSensor && this->batteryPercentSensor->state != battery_level)
      this->batteryPercentSensor->publish_state(battery_level);

    if (this->temperatureSensor && this->temperatureSensor->state != temperature)
      this->temperatureSensor->publish_state(temperature);

    //if (activity.compare(this->activitySensor->state) == 0)
    //      if (this->activitySensor)
    //      this->activitySensor->publish_state(activity);
  }

private:
  typedef enum {
    Sensors7to26					= 0,  //00
    Sensors7to16					= 1,  //01
    Sensors17to20					= 2,  //02
    Sensors21to26					= 3,  //03
    Sensors27to34					= 4,  //04
    Sensors35to42					= 5,  //05
    Sensors7to42					= 6,  //06
    SensorBumpsAndWheelDrops		= 7,  //07
    SensorWall						= 8,  //08
    SensorCliffLeft					= 9,  //09
    SensorCliffFrontLeft			= 10, //0A
    SensorCliffFrontRight			= 11, //0B
    SensorCliffRight				= 12, //0C
    SensorVirtualWall				= 13, //0D
    SensorOvercurrents				= 14, //0E
    //SensorUnused1					= 15, //0F
    //SensorUnused2					= 16, //10
    SensorIRByte					= 17, //11
    SensorButtons					= 18, //12
    SensorDistance					= 19, //13
    SensorAngle						= 20, //14
    SensorChargingState				= 21, //15
    SensorVoltage					= 22, //16
    SensorCurrent					= 23, //17
    SensorBatteryTemperature		= 24, //18
    SensorBatteryCharge				= 25, //19
    SensorBatteryCapacity			= 26, //1A
    SensorWallSignal				= 27, //1B
    SensoCliffLeftSignal			= 28, //1C
    SensoCliffFrontLeftSignal		= 29, //1D
    SensoCliffFrontRightSignal		= 30, //1E
    SensoCliffRightSignal			= 31, //1F
    SensorUserDigitalInputs			= 32, //20
    SensorUserAnalogInput			= 33, //21
    SensorChargingSourcesAvailable	= 34, //22
    SensorOIMode					= 35, //23
    SensorSongNumber				= 36, //24
    SensorSongPlaying				= 37, //25
    SensorNumberOfStreamPackets		= 38, //26
    SensorVelocity					= 39, //27
    SensorRadius					= 40, //28
    SensorRightVelocity				= 41, //29
    SensorLeftVelocity				= 42, //2A
  } SensorCode;

  typedef enum {
    ChargeStateNotCharging				= 0,
    ChargeStateReconditioningCharging	= 1,
    ChargeStateFullCharging				= 2,
    ChargeStateTrickleCharging			= 3,
    ChargeStateWaiting					= 4,
    ChargeStateFault					= 5,
  } ChargeState;

  typedef enum {
    ResetCmd        = 7,   //07
    StartCmd        = 128, //80
    StopCmd         = 173, //AD
    SafeCmd         = 131, //83
    FullCmd         = 132, //84
    CleanCmd        = 135, //87
    SpotCmd         = 134, //86
    DockCmd         = 143, //8F
    PowerCmd        = 133, //85
    DriveCmd        = 137, //89
    MotorsCmd       = 138, //8A
    LEDAsciiCMD     = 164, //A4
    SongCmd         = 140, //8C
    PlayCmd         = 141, //8D
    SensorsListCmd  = 149, //95
  } Commands;      
            
  void on_command(std::string command) {
    if (command == "clean") {
      clean();
    } else if (command == "dock" || command == "return_to_base") {
      displayString("DOCK");
      dock();
    } else if (command == "locate") {
      locate();
      displayString("LOC ");
    } else if (command == "spot" || command == "clean_spot") {
      spot();
    } else if (command == "wakeup") {
      brc_wakeup();
    } else if (command == "wake_on_dock") {
      wake_on_dock();
    } else if (command == "sleep") {
      sleep();
    } else if (command == "reset") {
      ESP_LOGI("roomba", "reset");
      reset();
    } else {
      ESP_LOGE("roomba", "unrecognized command %s", command.c_str());
    }            
    //ESP_LOGI("roomba", "ACK %s", command.c_str());
  }

  void start_oi() {
    write(StartCmd);
  }

  void reset() {
    write(ResetCmd);
  }

  void locate() {
    uint8_t song[] = {62, 12, 66, 12, 69, 12, 74, 36};
    safeMode();
    delay(500);
    setSong(0, song, sizeof(song));
    playSong(0);
  }
   
  void setSong(uint8_t songNumber, uint8_t data[], uint8_t len){
    write(SongCmd);
    write(songNumber);
    write(len >> 1); // 2 bytes per note
    write_array(data, len);
  }

  void playSong(uint8_t songNumber){
    write(PlayCmd);
    write(songNumber);
  }

  void displayString(std::string mystring){
    write(LEDAsciiCMD);
    uint8_t asciiValue0 = (int)mystring[0];
    write(asciiValue0);
    uint8_t asciiValue1 = (int)mystring[1];
    write(asciiValue1);
    uint8_t asciiValue2 = (int)mystring[2];
    write(asciiValue2);
    uint8_t asciiValue3 = (int)mystring[3];
    write(asciiValue3);
    delay(50);         
  }

  void wake_on_dock() {
    ESP_LOGD("roomba", "wake_on_dock");
    brc_wakeup();
    // Some black magic from @AndiTheBest to keep the Roomba awake on the dock
    // See https://github.com/johnboiles/esp-roomba-mqtt/issues/3#issuecomment-402096638
    delay(10);
    write(CleanCmd); // Clean
    delay(150);
    write(DockCmd); // Dock
  }

  void drive(int16_t velocity, int16_t radius) {
    write(DriveCmd);
    write((velocity & 0xff00) >> 8);
    write(velocity & 0xff);
    write((radius & 0xff00) >> 8);
    write(radius & 0xff);
  }

  void safeMode() {
    write(SafeCmd);
  }

  std::string get_oimode(uint8_t mode) {
    switch(mode) {
    case 0: return "off";
    case 1: return "passive";
    case 2: return "safe";
    case 3: return "full";
    default: return "unknown";
    }
  }

  void clean() {
    write(CleanCmd);
  }

  void dock() {
    write(DockCmd);
  }

  void spot() {
    write(SpotCmd);
  }

  void sleep() {
    write(PowerCmd);
  }

  bool getSensorsList(uint8_t* packetIDs, uint8_t numPacketIDs, uint8_t* dest, uint8_t len){
    write(149);
    write(numPacketIDs);
    write_array(packetIDs, numPacketIDs);
    return getData(dest, len);
  }

  bool getData(uint8_t* dest, uint8_t len) {
    while (len-- > 0) {
      unsigned long startTime = millis();
      while (!available()) {
	      yield();
      	// Look for a timeout
	      if (millis() > startTime + ROOMBA_READ_TIMEOUT) 
	        return false;
      }
      *dest++ = read();
    }
    return true;
  }

  std::string get_activity(uint8_t charging, int16_t current) {
    bool isCharging = charging == ChargeStateReconditioningCharging || charging == ChargeStateFullCharging || charging == ChargeStateTrickleCharging;
			
    if (current > -50)
      return "Docked";
    else if (isCharging)
      return "Charging";
    else if (current < -300)
      return "Cleaning";
    return "Lost";
  }

};

}
}
