#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif

namespace esphome {
namespace radsens {

union Uint32
{
    uint32_t u32;
    uint8_t a8[4];
};

union Uint16
{
    uint16_t u16;
    uint8_t a8[2];
};


class RadSensComponent : public PollingComponent, public i2c::I2CDevice {
#ifdef USE_SWITCH
 SUB_SWITCH(control_led)
 SUB_SWITCH(control_high_voltage)
 SUB_SWITCH(control_low_power)
#endif
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

  void set_sensitivity(uint16_t sensitivity);
  void set_dynamic_intensity_sensor(sensor::Sensor *dynamic_intensity_sensor) { dynamic_intensity_sensor_ = dynamic_intensity_sensor; }
  void set_static_intensity_sensor(sensor::Sensor *static_intensity_sensor) { static_intensity_sensor_ = static_intensity_sensor; }
  void set_counts_per_minute_sensor(sensor::Sensor *counts_per_minute_sensor) { counts_per_minute_sensor_ = counts_per_minute_sensor; }
  void set_firmware_version_sensor(sensor::Sensor *firmware_version_sensor) { firmware_version_sensor_ = firmware_version_sensor; }

  void set_high_voltage(bool enable);
  void set_led(bool enable);
  void set_low_power(bool enable);
  bool get_high_voltage();
  bool get_led();
  bool get_low_power();

 protected:
  void set_control(uint8_t reg, uint8_t val);
  bool get_control(uint8_t reg);
  // Retry-wrapped 16-bit read for the pulse counter. Occasional bus
  // glitches cause read failures; a short delay + retry recovers cleanly
  // without user-visible warning flapping.
  bool read_byte_16_with_retry_(uint8_t reg, uint16_t *data);
  // Read a 24-bit intensity register, retrying on either a bus failure OR
  // the known INTENSITY_CORRUPT_SIGNATURE (0x010001) which the sensor
  // returns when an intensity read follows a bus timeout too quickly.
  // Both failure modes use the same backoff schedule.
  bool read_intensity_(uint8_t reg, Uint32 *out);
  // Retry policy: 3 attempts with backoff (10ms, 20ms between attempts).
  static constexpr uint32_t READ_MAX_ATTEMPTS = 3;
  static constexpr uint32_t READ_RETRY_BASE_DELAY_MS = 10;
  // Known corruption signature: the sensor returns this exact 24-bit raw
  // value when an intensity read follows a bus timeout too quickly.
  static constexpr uint32_t INTENSITY_CORRUPT_SIGNATURE = 0x010001;
  sensor::Sensor *dynamic_intensity_sensor_{nullptr};
  sensor::Sensor *static_intensity_sensor_{nullptr};
  sensor::Sensor *counts_per_minute_sensor_{nullptr};
  sensor::Sensor *firmware_version_sensor_{nullptr};
  uint32_t last_update = 0;
  uint8_t firmware_version = 0;
  uint16_t sensitivity_ = 0;

  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
    ID_REGISTERS,
  } error_code_;

};

}  // namespace radsens
}  // namespace esphome