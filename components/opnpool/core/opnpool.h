/**
 * @file opnpool.h
 * @brief Main OPNpool component interface for ESPHome.
 *
 * @details
 * Declares the OpnPool class, the main ESPHome component that provides
 * bidirectional integration between the pool controller and Home Assistant.
 * It manages climate, switch, sensor, binary sensor, and text sensor entities,
 * spawns the pool_task for RS-485 communication, and handles state updates.
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2026 Coert Vonk
 * @modified 2026 by Dave Fernholz -- LilyGO T-CAN485 (ESP32, 4MB flash) support,
 *           upstream defect fixes, and ESPHome / ESP-IDF 5.x compatibility.
 *           See CHANGES.md in the repository root for the full list.
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once
#ifndef ESPHOME_OPNPOOL_OPNPOOL_H_
#define ESPHOME_OPNPOOL_OPNPOOL_H_
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>
#include <esp_types.h>
#include <esphome/core/component.h>

#include "utils/enum_helpers.h"
#include "opnpool_ids.h"
    // for datalink_corrupt_pkt_count() / datalink_good_pkt_count(): YAML lambdas need them,
    // and the generated main.cpp includes this header but not the pool_task internals.
    // Safe to pull in here -- datalink.h only #includes <esp_system.h> and <esp_types.h>.
#include "pool_task/datalink.h"

#ifdef USE_MATTER
#include "matter/matter_bridge.h"
#endif

namespace esphome {
namespace opnpool {

// Forward declarations (to avoid circular dependencies)
struct ipc_t;
struct poolstate_t;
struct pending_switch_t;
struct pending_climate_t;
class PoolState;
class OpnPoolClimate;
class OpnPoolSwitch;
class OpnPoolSensor;
class OpnPoolBinarySensor;
class OpnPoolTextSensor;

    // Circuit-command outcome counters. Defined in entities/opnpool_switch.cpp and declared
    // here so YAML lambdas can publish them as Home Assistant diagnostics. A log line for a
    // dropped command is easy to miss -- the log stream itself can disconnect -- whereas a
    // monotonic counter cannot be missed and survives until the next reboot.
/// @brief Circuit commands accepted into the transmit path since boot.
uint32_t opnpool_cmd_sent_count();
/// @brief Circuit commands the controller never confirmed within the pending window.
///        A rising value means commands are reaching the bus but not taking effect.
uint32_t opnpool_cmd_unconfirmed_count();
/// @brief Circuit commands that could not even be queued (IPC queue full).
uint32_t opnpool_cmd_dropped_count();

    // Pump diagnostics, defined in core/poolstate_rx.cpp. Both are decoded from the pump's own
    // messages and were previously discarded.
namespace poolstate_rx {
    /// @brief Raw pump control byte: 0xFF means the pump is under automation (remote) control,
    ///        anything else means it is running its own onboard program (local).
    uint8_t pump_ctrl_raw();
    /// @brief Raw bytes 11..12 of the pump status message (hour<<8 | minute as sent).
    ///        Upstream calls this "remaining" but is unsure whether it is a timer or status.
    uint16_t pump_remaining_raw();
}

/// @brief RS-485 GPIO pin configuration.
struct rs485_pins_t {
    uint8_t rx_pin{21};   ///< Receive pin GPIO number.
    uint8_t tx_pin{22};   ///< Transmit pin GPIO number.
    uint8_t rts_pin{23};  ///< Direction control (RTS) pin GPIO number.
};

/**
 * @brief Main OPNpool component for ESPHome.
 *
 * @details
 * Provides bidirectional integration between pool controller hardware and the
 * ESPHome ecosystem. Publishes pool state changes to Home Assistant entities
 * and enacts control requests from Home Assistant on the physical equipment.
 */
class OpnPool : public Component {

  public:
    void setup() override;      ///< Initializes IPC, spawns pool_task, publishes firmware version.
    void loop() override;       ///< Processes messages from pool_task, updates entities.
    void dump_config() override;///< Logs component configuration.
    ~OpnPool();                 ///< Cleans up resources.

    // ========== RS-485 Configuration ==========
    void set_rs485_pins(uint8_t rx_pin, uint8_t tx_pin, uint8_t rts_pin);

    // ========== Climate Setters ==========
    void set_pool_climate(OpnPoolClimate * const climate);
    void set_spa_climate(OpnPoolClimate * const climate);

    // ========== Switch Setters ==========
    void set_pool_switch(OpnPoolSwitch * const sw);
    void set_spa_switch(OpnPoolSwitch * const sw);
    void set_aux1_switch(OpnPoolSwitch * const sw);
    void set_aux2_switch(OpnPoolSwitch * const sw);
    void set_aux3_switch(OpnPoolSwitch * const sw);
    void set_feature1_switch(OpnPoolSwitch * const sw);
    void set_feature2_switch(OpnPoolSwitch * const sw);
    void set_feature3_switch(OpnPoolSwitch * const sw);
    void set_feature4_switch(OpnPoolSwitch * const sw);

    // ========== Sensor Setters ==========
    void set_air_temperature_sensor(OpnPoolSensor * const s);
    void set_water_temperature_sensor(OpnPoolSensor * const s);
    void set_solar1_temperature_sensor(OpnPoolSensor * const s);
    void set_solar2_temperature_sensor(OpnPoolSensor * const s);
    void set_primary_pump_power_sensor(OpnPoolSensor * const s);
    void set_primary_pump_flow_sensor(OpnPoolSensor * const s);
    void set_primary_pump_speed_sensor(OpnPoolSensor * const s);
    void set_primary_pump_error_sensor(OpnPoolSensor * const s);
    void set_chlorinator_level_sensor(OpnPoolSensor * const s);
    void set_chlorinator_salt_sensor(OpnPoolSensor * const s);

    // ========== Binary Sensor Setters ==========
    void set_primary_pump_running_binary_sensor(OpnPoolBinarySensor * const bs);
    void set_mode_service_binary_sensor(OpnPoolBinarySensor * const bs);
    void set_mode_temperature_inc_binary_sensor(OpnPoolBinarySensor * const bs);
    void set_mode_freeze_protection_binary_sensor(OpnPoolBinarySensor * const bs);
    void set_mode_timeout_binary_sensor(OpnPoolBinarySensor * const bs);

    // ========== Text Sensor Setters ==========
    void set_pool_sched_text_sensor(OpnPoolTextSensor * const ts);
    void set_spa_sched_text_sensor(OpnPoolTextSensor * const ts);
    void set_primary_pump_mode_text_sensor(OpnPoolTextSensor * const ts);
    void set_primary_pump_state_text_sensor(OpnPoolTextSensor * const ts);
    void set_chlorinator_name_text_sensor(OpnPoolTextSensor * const ts);
    void set_chlorinator_status_text_sensor(OpnPoolTextSensor * const ts);
    void set_system_time_text_sensor(OpnPoolTextSensor * const ts);
    void set_controller_type_text_sensor(OpnPoolTextSensor * const ts);
    void set_interface_firmware_text_sensor(OpnPoolTextSensor * const ts);

#ifdef USE_MATTER
    // ========== Matter Configuration ==========
    void set_matter_config(uint16_t discriminator, uint32_t passcode);

    /// @brief Check if Matter is enabled and initialized.
    [[nodiscard]] bool is_matter_enabled() const { return matter_bridge_ != nullptr; }

    /// @brief Check if Matter device is commissioned to a fabric.
    [[nodiscard]] bool is_matter_commissioned() const;

    /// @brief Get Matter QR code for commissioning.
    size_t get_matter_qr_code(char * buf, size_t buf_size) const;
#endif

    // ========== Entity Update Methods ==========
    void update_climates(poolstate_t const * const state);
    void update_switches(poolstate_t const * const state);
    void update_text_sensors(poolstate_t const * const state);
    void update_analog_sensors(poolstate_t const * const state);
    void update_binary_sensors(poolstate_t const * const state);
    void update_all(poolstate_t const * const state);

    // ========== Accessors ==========
    ipc_t *         get_ipc()              { return ipc_; }                 ///< Returns IPC structure pointer.
    PoolState *     get_opnpool_state()    { return poolState_; }           ///< Returns pool state pointer.
    OpnPoolSwitch * get_switch(uint8_t id) { return this->switches_[id]; }  ///< Returns switch by ID.

    // ========== Controller Commands ==========

    /**
     * @brief Set the pool controller's real-time clock.
     *
     * Pentair controllers drift (this one gained ~7 minutes), and any schedule programmed on
     * the controller runs off that clock, so it is worth correcting from a known-good source.
     *
     * Byte layout was confirmed against a live CTRL_TIME_RESP capture:
     *   0F 36 20 1F 07 1A 00 01  ->  15:54, Friday, 31 July 2026, clk_speed 0, dst_auto 1
     * Note that dayoftheweek is a BITMASK, not an ordinal: Sunday = bit 0 ... Saturday = bit 6
     * (the captured 0x20 = 1<<5 = Friday).
     *
     * @param hour       0..23
     * @param minute     0..59
     * @param dow_sun0   day of week, 0 = Sunday .. 6 = Saturday (converted to the bitmask here)
     * @param day        1..31
     * @param month      1..12
     * @param year_2000  years since 2000 (e.g. 26 for 2026)
     */
    /**
     * @brief Sets the IntelliBrite light theme (fixed colour or show).
     *
     * @details Sends A5 message type 0x60 with a single theme byte, which is what the wired
     * remote sends. The frame format and the five colour values were verified against live
     * traffic captured from the remote:
     *   `FF 00 FF A5 | 03 10 20 60 02 | <theme> 00 | <checksum>`
     *
     * Verified colours: 193 blue, 194 green, 195 red, 196 white, 197 magenta.
     * Shows (128 colorsync, 144/160 colorswim/colorset, 177-182 named, 190 save, 191 recall)
     * come from nodejs-poolController and are NOT verified here -- note that two files in
     * that project disagree on whether 144 or 160 is colorset.
     *
     * The light circuit must be ON, and configured as an IntelliBrite circuit on the
     * controller, for a theme change to do anything.
     *
     * @param[in] theme Raw theme byte.
     */
    void set_light_theme(uint8_t theme);

    void set_controller_clock(uint8_t hour, uint8_t minute, uint8_t dow_sun0,
                              uint8_t day, uint8_t month, uint8_t year_2000);

    /**
     * @brief Set the controller clock only if it has drifted beyond a tolerance.
     *
     * Writing the clock appears to make the controller re-evaluate its equipment (a pump
     * stop/restart was observed right after a clock write), so this avoids doing it nightly
     * for a few seconds of drift. Returns true if a correction was actually sent.
     *
     * @param tolerance_minutes  Only correct when the controller is off by at least this much.
     */
    bool set_controller_clock_if_drifted(uint8_t hour, uint8_t minute, uint8_t dow_sun0,
                                         uint8_t day, uint8_t month, uint8_t year_2000,
                                         uint8_t tolerance_minutes);

    /**
     * @brief Ask the controller what each circuit is configured as (read-only).
     *
     * Sends CTRL_CIRC_NAMES_REQ for each circuit id; the raw responses are logged. This is
     * how to find out what an unlabelled circuit such as AUX1 or FEATURE2/3/4 is actually
     * assigned to, without toggling it -- toggling an unknown circuit could start equipment
     * (e.g. a booster pump) that should not be run unattended.
     *
     * Response layout, per the observed examples:
     *   req 0x01 -> 01 01 48 00 00   circuit 1, function 0x01 (Spa), name 0x48 (72 = 'SPA')
     *   req 0x02 -> 02 00 03 00 00   circuit 2, function 0x00 (Generic)
     * Function 0 = Generic (i.e. nothing meaningful assigned), 14 = Spillway.
     */
    void request_circuit_config();

  protected:
    rs485_pins_t rs485_pins_;                ///< RS-485 GPIO pin configuration.
    ipc_t * ipc_{nullptr};                   ///< IPC structure for task communication.
    PoolState * poolState_{nullptr};         ///< Pool state manager instance.
    TaskHandle_t pool_task_handle_{nullptr}; ///< FreeRTOS task handle for pool_task.

    // ========== Entity Arrays ==========
    OpnPoolClimate * climates_[enum_count<climate_id_t>()]{nullptr};              ///< Climate entity pointers.
    OpnPoolSwitch * switches_[enum_count<switch_id_t>()]{nullptr};                ///< Switch entity pointers.
    OpnPoolSensor * sensors_[enum_count<sensor_id_t>()]{nullptr};                 ///< Sensor entity pointers.
    OpnPoolBinarySensor * binary_sensors_[enum_count<binary_sensor_id_t>()]{nullptr}; ///< Binary sensor pointers.
    OpnPoolTextSensor * text_sensors_[enum_count<text_sensor_id_t>()]{nullptr};   ///< Text sensor pointers.

#ifdef USE_MATTER
    // ========== Matter Integration ==========
    matter::MatterBridge * matter_bridge_{nullptr};  ///< Matter bridge instance.
    matter::matter_config_t matter_config_{};        ///< Matter configuration.
#endif
};

} // namespace opnpool
} // namespace esphome

#endif // ESPHOME_OPNPOOL_OPNPOOL_H_