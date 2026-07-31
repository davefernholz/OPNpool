/**
 * @file poolstate_rx.h
 * @brief Pool state update interface for received network messages
 *
 * @details
 * Declares the function that updates the local pool state from incoming network
 * messages. The implementation in poolstate_rx.cpp dispatches on message type to
 * handler functions for pump, controller, and chlorinator broadcasts.
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2014, 2019, 2022, 2026 Coert Vonk
 * @modified 2026 by Dave Fernholz -- LilyGO T-CAN485 (ESP32, 4MB flash) support,
 *           upstream defect fixes, and ESPHome / ESP-IDF 5.x compatibility.
 *           See CHANGES.md in the repository root for the full list.
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once
#ifndef __cplusplus
# error "Requires C++ compilation"
#endif

#include <esp_system.h>
#include <esp_types.h>

namespace esphome {
namespace opnpool {

    // forward declarations (to avoid circular dependencies)
struct network_msg_t;
struct poolstate_t;

namespace poolstate_rx {

/// Last clk_speed / dst_auto bytes seen in a controller clock message. Cached so that
/// setting the clock can preserve them: clk_speed is a calibration trim and dst_auto is a
/// user preference, and silently overwriting either would change controller behaviour.
/// Defaults match what a real remote was observed to send (00, 01 = auto DST).
/// @brief esp_timer timestamp of the most recent pump status message, 0 if none yet.
int64_t pump_last_seen_us();
uint8_t last_clk_speed();
uint8_t last_dst_auto();

/**
 * @brief Update pool state from a received network message.
 *
 * @param[in]     msg    Pointer to the received network message.
 * @param[in,out] state  Pointer to the pool state to update.
 * @return               ESP_OK on success, ESP_FAIL if the message type is unhandled.
 */
[[nodiscard]] esp_err_t update_state(network_msg_t const * const msg, poolstate_t * const state);

}  // namespace poolstate_rx

}  // namespace opnpool
}  // namespace esphome