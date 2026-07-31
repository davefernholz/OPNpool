/**
 * @file opnpool_switch.cpp
 * @brief Actuates switch settings from Home Assistant on the pool controller.
 *
 * @details
 * This file implements the switch entity interface for the OPNpool component, enabling
 * control and monitoring of pool circuits (such as POOL, SPA, AUX1). It provides methods
 * to handle switch state changes initiated by Home Assistant, constructs and sends
 * protocol messages to the pool controller over RS-485, and updates the switch state
 * based on controller feedback. The implementation ensures that only meaningful state
 * changes are published to Home Assistant, avoiding redundant updates.
 *
 * ESPHome operates in a single-threaded environment, so explicit thread safety measures
 * are not required within the main task context. The maximum number of switches is
 * limited by the use of uint8_t for indexing.
 *
 * restore_mode is ignored because state is always synchronized from the pool controller.
 * The component will always reflect the actual state of the pool circuits as reported by
 * the controller.
 * 
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2026 Coert Vonk
 * @modified 2026 by Dave Fernholz -- LilyGO T-CAN485 (ESP32, 4MB flash) support,
 *           upstream defect fixes, and ESPHome / ESP-IDF 5.x compatibility.
 *           See CHANGES.md in the repository root for the full list.
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esp_system.h>
#include <esp_timer.h>
#include <esphome/core/log.h>

#include "opnpool_switch.h"   // no other #includes that could make a circular dependency
#include "core/opnpool.h"      // no other #includes that could make a circular dependency
#include "ipc/ipc.h"          // no other #includes that could make a circular dependency
#include "pool_task/network_msg.h"  // #includes datalink_pkt.h, that doesn't #include others that could make a circular dependency
#include "core/opnpool_ids.h"      // conversion helper
#include "utils/enum_helpers.h"
#include "core/poolstate.h"
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"
#pragma GCC diagnostic error "-Wunused-parameter"

namespace esphome {
namespace opnpool {

constexpr char TAG[] = "opnpool_switch";

    // How long to trust our own optimistic switch state before deferring back to whatever the
    // controller reports. Long enough to cover the transmit-opportunity wait (~1s) plus the
    // controller acting and re-broadcasting, short enough that a genuinely failed command
    // visibly reverts in Home Assistant rather than lying indefinitely.
constexpr int64_t PENDING_TIMEOUT_US = 8 * 1000000;

    // Command outcome counters, surfaced as Home Assistant diagnostics (see opnpool.h).
    // These exist because a dropped command is otherwise invisible: the only signal was a
    // log line, and the log stream can disconnect without warning.
static uint32_t _cmds_sent        = 0;
static uint32_t _cmds_unconfirmed = 0;
static uint32_t _cmds_dropped     = 0;
uint32_t opnpool_cmd_sent_count()        { return _cmds_sent; }
uint32_t opnpool_cmd_unconfirmed_count() { return _cmds_unconfirmed; }
uint32_t opnpool_cmd_dropped_count()     { return _cmds_dropped; }

/**
 * @brief Dump the configuration and last known state of the switch entity.
 *
 * @details
 * Logs the configuration details for this switch, including its mapped circuit, ID,
 * and last known state (ON/OFF or Unknown). This information is useful for diagnostics
 * and debugging, providing visibility into the entity's state and configuration at
 * runtime.
 */
void
OpnPoolSwitch::dump_config()
{
    network_pool_circuit_t circuit = circuit_;

    LOG_SWITCH("  ", "Switch", this);
    ESP_LOGCONFIG(TAG, "    Circuit: %s", enum_str(circuit));
    ESP_LOGCONFIG(TAG, "    Last state: %s", last_.valid ? (last_.value ? "ON" : "OFF") : "Unknown");
}

/**
 * @brief Handles switch state changes triggered by Home Assistant.
 *
 * @details
 * Constructs and sends a network message to the pool controller to set the state of the
 * specified circuit. The circuit index is mapped to the pool controller's circuit
 * numbering. The message is sent via IPC to the pool_task for RS-485 transmission. This
 * method is called automatically when the switch entity is toggled in Home Assistant or
 * ESPHome.
 *
 * @param[in] state The desired state of the switch (true for ON, false for OFF).
 */
void
OpnPoolSwitch::write_state(bool value)
{
    if (!this->parent_) { ESP_LOGW(TAG, "Parent unknown"); return; }

    PoolState * const state_class_ptr = parent_->get_opnpool_state();
    if (!state_class_ptr) { ESP_LOGW(TAG, "Pool state unknown"); return; }

    poolstate_t state;
    state_class_ptr->get(&state);
    
    datalink_addr_t const controller_addr = state.system.addr.value; // get learned controller address
    if (!controller_addr.is_controller()) {
        ESP_LOGW(TAG, "Controller address still unknown, cannot send control message");
        return;
    }

    network_pool_circuit_t const circuit = circuit_;
    uint8_t const circuit_idx = enum_index(circuit);

    network_msg_t msg;
    msg.src = datalink_addr_t::remote();
    msg.dst = controller_addr;
    msg.typ = network_msg_typ_t::CTRL_CIRCUIT_SET;
    msg.u.a5 = {
        .ctrl_circuit_set = {
            .circuit_plus_1 = static_cast<uint8_t>(circuit_idx + uint8_t(1)),
            .value = static_cast<uint8_t>(value ? 1 : 0)          
        }
    };

    ESP_LOGVV(TAG, "Sending CIRCUIT_SET command: circuit+1=%u to %u", msg.u.a5.ctrl_circuit_set.circuit_plus_1, msg.u.ctrl_circuit_set.value);
    if (ipc_send_network_msg_to_pool_task(&msg, this->parent_->get_ipc()) != ESP_OK) {
            // Do not publish optimistically below: the command never reached the queue, so
            // nothing will ever confirm it. Showing the requested state for PENDING_TIMEOUT_US
            // and then warning about a missing confirmation misreports a send that never
            // happened. Leave the switch showing the controller's actual state.
        _cmds_dropped++;
        ESP_LOGW(TAG, "Failed to send CIRCUIT_SET message to pool task");
        return;
    }

        // Publish the requested value immediately and start suppressing contradicting
        // controller updates until it confirms (or we give up). Waiting for confirmation
        // without doing this is what made the Home Assistant toggle flicker ON -> OFF -> ON:
        // the command sits in the tx queue until the next transmit opportunity while the
        // controller keeps broadcasting the old state, which HA renders as a revert.
    _cmds_sent++;
    pending_valid_    = true;
    pending_value_    = value;
    pending_until_us_ = esp_timer_get_time() + PENDING_TIMEOUT_US;

    this->publish_state(value);
    last_ = { .valid = true, .value = value };
}

/**
 * @brief Publishes the switch state to Home Assistant if it has changed.
 *
 * @details
 * Compares the new switch state with the last published state. If the state has changed
 * or is not yet valid, updates the internal state and publishes the new value to Home
 * Assistant. This avoids redundant updates to Home Assistant.
 *
 * @param[in] value The new state of the switch (true for ON, false for OFF).
 */
void
OpnPoolSwitch::publish_value_if_changed(bool value)
{
        // While a write is awaiting confirmation, ignore controller updates that contradict
        // what we asked for -- those are just stale broadcasts from before the command
        // landed, and publishing them makes the Home Assistant toggle flicker back.
    if (pending_valid_) {
        if (value == pending_value_) {
            pending_valid_ = false;        // controller confirmed; resume normal behaviour
        } else if (esp_timer_get_time() < pending_until_us_) {
            return;                        // still waiting -- suppress the stale value
        } else {
            pending_valid_ = false;        // gave up: let the real state through so a
                                           // genuinely failed command is visible in HA
            _cmds_unconfirmed++;
            ESP_LOGW(TAG, "%s: no confirmation within %llds, reverting to controller state",
                     enum_str(circuit_), (long long)(PENDING_TIMEOUT_US / 1000000));
        }
    }

    if (!last_.valid || last_.value != value) {

        this->publish_state(value);

        last_ = {
            .valid = true,
            .value = value
        };
        
        network_pool_circuit_t const circuit = circuit_;
        ESP_LOGV(TAG, "Published %s: %s", enum_str(circuit), value ? "ON" : "OFF");
    }
}

}  // namespace opnpool
}  // namespace esphome