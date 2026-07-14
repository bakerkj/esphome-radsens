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
  // Retry-wrapped 16-bit read. Occasional bus glitches cause read
  // failures on this sensor; a short delay + retry recovers cleanly
  // without user-visible warning flapping. Empirically the RadSens
  // requires ~15ms of settle time after a bus timeout before it
  // returns valid data — a faster retry can come back with plausible-
  // looking but corrupt payloads.
  bool read_byte_16_with_retry_(uint8_t reg, uint16_t *data);
  // Retry-wrapped multi-byte read used for the 24-bit intensity
  // registers. Same rationale as read_byte_16_with_retry_.
  bool read_bytes_with_retry_(uint8_t reg, uint8_t *buf, size_t len);
  // Retry policy: 3 attempts with a flat 15ms delay before each retry
  // (cumulative 30ms before attempt 3).
  static constexpr uint32_t READ_MAX_ATTEMPTS = 3;
  static constexpr uint32_t READ_RETRY_BASE_DELAY_MS = 15;
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