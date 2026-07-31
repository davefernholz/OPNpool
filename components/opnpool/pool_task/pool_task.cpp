/**
 * @file pool_task.cpp
 * @brief FreeRTOS task for RS-485 communication with pool controller.
 *
 * This file implements the FreeRTOS task logic for the OPNpool component, responsible for
 * managing RS-485 communication with the pool controller. It handles both receiving and
 * transmitting protocol packets, supporting two distinct message formats:
 *   - A5 protocol: Framed messages with preamble, headers, length, data, and 16-bit checksum.
 *   - IC protocol: Framed with `{0x10, 0x02}` preamble, data, 8-bit checksum, `{0x10, 0x03}` postamble.
 *
 * Core responsibilities:
 * - Continuously reading from the RS-485 bus, packetizing incoming byte streams, and parsing
 *   them into higher-level datalink and network messages.
 * - Relaying received network messages to the main ESPHome task via IPC queues.
 * - Handling requests from the main task, converting them into protocol packets, and transmitting
 *   them to the pool controller.
 * - Managing a transmit queue for outgoing packets, ensuring correct half-duplex operation
 *   (using RTS for direction control) and echoing sent messages back up the protocol stack for
 *   state consistency.
 * - Periodically sending control and status requests (heat, schedule, version queries) to
 *   keep the pool state synchronized.
 *
 * @note The pool controller broadcasts state every ~1 second. The task waits for these broadcasts
 *       to identify transmit opportunities, avoiding bus collisions on the shared RS-485 bus.
 *
 * @note ESPHome's main loop is single-threaded, but this task runs on a separate FreeRTOS task.
 *       Communication with the main task uses thread-safe IPC queues.
 *
 * @see datalink_rx.cpp for packet reception and parsing
 * @see datalink_tx.cpp for packet transmission
 * @see network_rx.cpp for network message decoding
 * @see ipc.h for inter-task communication
 *
 * @author Coert Vonk (@cvonk on GitHub)
 * @copyright Copyright (c) 2014, 2019, 2022, 2026 Coert Vonk
 * @modified 2026 by Dave Fernholz -- LilyGO T-CAN485 (ESP32, 4MB flash) support,
 *           upstream defect fixes, and ESPHome / ESP-IDF 5.x compatibility.
 *           See CHANGES.md in the repository root for the full list.
 * @license SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <esp_system.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_types.h>
#include "esphome/core/log.h"
#include <string.h>

#include "utils/to_str.h"
#include "utils/enum_helpers.h"
#include "skb.h"
#include "rs485.h"
#include "datalink.h"
#include "datalink_pkt.h"
#include "network.h"
#include "network_msg.h"
#include "ipc/ipc.h"
#include "pool_task.h"
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"
#pragma GCC diagnostic error "-Wunused-parameter"

namespace esphome {
namespace opnpool {

constexpr char TAG[] = "pool_task";

    // Upstream default was 100ms. This does NOT drive responsiveness -- the task spends
    // nearly all its time blocked in uart_read_bytes() (100ms timeout), so the delay only
    // applies after a completed packet. What it controls is how often the loop competes with
    // WiFi/API for CPU, and 20ms is the balance: still reacts promptly to the post-broadcast
    // transmit window without spinning 5x more than necessary.
constexpr uint32_t POOL_TASK_DELAY_MS       = 20;         ///< Main loop delay between iterations [ms]
constexpr uint32_t POOL_REQ_INTERVAL_MS     = 30 * 1000;  ///< Interval between periodic controller queries [ms]
constexpr uint32_t POOL_REQ_TASK_STACK_SIZE = 2 * 4096;   ///< Stack size for pool_req_task [bytes]

/// Controller address learned from broadcast messages. Used as destination for outgoing requests.
static datalink_addr_t _controller_addr = datalink_addr_t::unknown();


/**
 * @brief Processes incoming packets from the RS-485 bus and relays messages to the main task.
 *
 * Receives a packet from RS-485, decodes it into a network message, and sends it to the
 * main task via IPC if successful. Also snoops the controller address from broadcast messages
 * for use in outgoing requests.
 *
 * @param[in] rs485 RS-485 handle for bus communication.
 * @param[in] ipc   IPC structure for inter-task communication.
 * @return          True if a transmit opportunity is available (i.e., after receiving a
 *                  controller broadcast, indicating the bus is momentarily idle).
 *
 * @note The packet buffer (pkt.skb) is freed after processing regardless of success/failure.
 */
[[nodiscard]] static bool
_service_pkts_from_rs485(rs485_handle_t const rs485, ipc_t const * const ipc)
{
    datalink_pkt_t pkt = {};
    network_msg_t msg;
    bool txOpportunity = false;

        // datalink_rx_pkt() allocates pkt.skb up front and does NOT free it on its failure
        // paths, so ownership lands here on every path. Since the rx state machine is
        // bounded it returns failure on every timed-out read -- roughly every 2s on a quiet
        // bus -- so freeing only on success would exhaust the heap. Free unconditionally
        // below instead.
    if (datalink_rx_pkt(rs485, &pkt) == ESP_OK) {

        esp_err_t const decode_err = network_rx_msg(&pkt, &msg, &txOpportunity);
        if (decode_err == ESP_OK) {

                // snoop to find the controller address to use as the dst in _queue_req()
            if (msg.src.is_controller()) {
                _controller_addr = msg.src;
                ESP_LOGV(TAG, "learned controller address: 0x%02X", msg.src.addr);
            }

            if( ipc_send_network_msg_to_main_task(&msg, ipc) != ESP_OK) {
                ESP_LOGW(TAG, "Failed to send network message to main task");
            }

        } else if (decode_err == ESP_ERR_NOT_SUPPORTED) {
                // known-benign undecoded type (see _decode_msg_a5_ctrl); already logged there
        } else {
            ESP_LOGW(TAG, "Failed to decode network message from datalink packet");
        }
    } else {
        ESP_LOGVV(TAG, "No packet received from RS-485");
    }
    free(pkt.skb);   // safe on every path: nullptr when allocation itself failed
    return txOpportunity;
}

/**
 * @brief Handles requests from the main task and queues them for RS-485 transmission.
 *
 * Non-blocking receive from the IPC queue. If a message is available, packetizes it and
 * queues it for transmission to the pool controller. The actual transmission occurs later
 * when a transmit opportunity is detected.
 *
 * @param[in] rs485 RS-485 handle for queuing outgoing packets.
 * @param[in] ipc   IPC structure containing the to_pool_q queue.
 *
 * @see _forward_queued_pkt_to_rs485() for actual transmission
 */
static void
_service_requests_from_main(rs485_handle_t rs485, ipc_t const * const ipc)
{
    network_msg_t msg;

    if (xQueueReceive(ipc->to_pool_q, &msg, (TickType_t)0) == pdPASS) {

        datalink_pkt_t * const pkt = static_cast<datalink_pkt_t*>(calloc(1, sizeof(datalink_pkt_t)));
        if (pkt == nullptr) {
                // network_create_pkt() writes through this pointer before any null check.
                // The message is already dequeued, so log it -- a silently dropped user
                // command is worse than a noisy one.
            ESP_LOGE(TAG, "out of memory; dropping outgoing command");
            return;
        }

        if (network_create_pkt(&msg, pkt) == ESP_OK) {

                // urgent: this came from Home Assistant, do not let it wait behind polls
            datalink_tx_pkt_queue(rs485, pkt, /*urgent=*/true);  // pkt and pkt->skb freed by recipient
            return;
        }
        if (pkt->skb) free(pkt->skb);
        free(pkt);
    }
}

/**
 * @brief Queues a request message for transmission to the pool controller.
 *
 * Creates a network message of the specified type with this device as source (pretending
 * to be a remote control) and the learned controller address as destination. The message
 * is packetized and queued for transmission.
 *
 * @param[in] rs485 RS-485 handle for queuing outgoing packets.
 * @param[in] typ   Network message type to send (typically a *_REQ type).
 *
 * @note Requires _controller_addr to be learned from a previous broadcast.
 * @see pool_req_task() for periodic request scheduling
 */
static void
_queue_req(rs485_handle_t const rs485, network_msg_typ_t const typ)
{
    network_msg_t msg = {};
    msg.typ = typ;
    msg.src = datalink_addr_t::remote();               // pretent we're a remote control
    msg.dst = _controller_addr;  // use controller address

    datalink_pkt_t * const pkt = static_cast<datalink_pkt_t*>(calloc(1, sizeof(datalink_pkt_t)));
    if (pkt == nullptr) {
        ESP_LOGE(TAG, "out of memory; dropping periodic request");
        return;
    }

    if (network_create_pkt(&msg, pkt) == ESP_OK) {

        datalink_tx_pkt_queue(rs485, pkt);  // pkt and pkt->skb freed by mailbox recipient

    } else {
        if (pkt->skb) free(pkt->skb);
        free(pkt);
    }
}

/**
 * @brief Forwards a queued packet from the transmit queue to the RS-485 bus.
 *
 * Dequeues a single packet from the RS-485 transmit queue and sends it on the bus.
 * Uses RTS to switch the transceiver to transmit mode. After transmission, the packet
 * is "echoed" back through the network layer to update local state as if the message
 * was received, ensuring consistent state tracking.
 *
 * @param[in] rs485 RS-485 handle for bus communication.
 * @param[in] ipc   IPC structure for relaying the echoed message to main task.
 *
 * @note Both pkt and pkt->skb are freed after processing.
 * @note Only called when a transmit opportunity is available (bus idle after broadcast).
 */
static void
_forward_queued_pkt_to_rs485(rs485_handle_t const rs485, ipc_t const * const ipc)
{
    datalink_pkt_t const * const pkt = rs485->dequeue(rs485);
    if (pkt) {
        ESP_LOGVV(TAG, "forward_queue: pkt typ=%s", enum_str(static_cast<datalink_ctrl_typ_t>(pkt->typ)));

        if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
            size_t const dbg_size = 128;
            char dbg[dbg_size];
            if (pkt->skb == nullptr) {
                ESP_LOGW(TAG, "Packet skb is null");
            } else {
                (void) skb_print(pkt->skb, dbg, dbg_size);
                ESP_LOGVV(TAG, "tx { %s}", dbg);
            }
        }
        rs485->tx_mode(true);
        rs485->write_bytes(pkt->skb->priv.data, pkt->skb->len);
        rs485->tx_mode(false);

            // pretend that we received our own message to ensure consistent state tracking

        bool txOpportunity = false;
        network_msg_t msg;

        ESP_LOGVV(TAG, "pretend rx: pkt typ=%s", enum_str(static_cast<datalink_ctrl_typ_t>(pkt->typ)));

        if (network_rx_msg(pkt, &msg, &txOpportunity) == ESP_OK) {

            if (ipc_send_network_msg_to_main_task(&msg, ipc) != ESP_OK) {
                ESP_LOGW(TAG, "Failed to send network message to main task");
            }
        }
        free(pkt->skb);
        free((void *) pkt);
    }
}

/**
 * @brief FreeRTOS sub-task for periodic pool controller requests.
 *
 * Runs in an infinite loop, sending request messages to the pool controller at
 * POOL_REQ_INTERVAL_MS intervals. Queries version, heat status, and schedules to
 * keep the cached pool state synchronized with the controller.
 *
 * @param[in] rs485_void Pointer to the RS-485 handle (cast to void* for FreeRTOS API).
 *
 * @note Waits for _controller_addr to be learned before sending requests.
 * @note Spawned by pool_task() during initialization.
 */
void
pool_req_task(void * rs485_void) 
{
    rs485_handle_t const rs485 = (rs485_handle_t)rs485_void;

    while (1) {

        vTaskDelay((TickType_t)POOL_REQ_INTERVAL_MS / portTICK_PERIOD_MS);

        if (!_controller_addr.is_controller()) {
            ESP_LOGW(TAG, "Controller address still unknown, skipping periodic requests");
            vTaskDelay((TickType_t)POOL_REQ_INTERVAL_MS / portTICK_PERIOD_MS);
            continue;
        }
        _queue_req(rs485, network_msg_typ_t::CTRL_VERSION_REQ);
        _queue_req(rs485, network_msg_typ_t::CTRL_TIME_REQ);   // needed to read/verify the controller clock

        _queue_req(rs485, network_msg_typ_t::CTRL_HEAT_REQ);
        _queue_req(rs485, network_msg_typ_t::CTRL_SCHED_REQ);

            // Valve status is never requested upstream, so network_ctrl_valve_resp_t has stayed
            // an undecoded `uint8_t unknown[24]`. Ask for it -- CTRL_VALVE_RESP already logs its
            // raw bytes -- so the payload can be correlated against known valve positions
            // (pool mode vs spa mode move the intake/return actuators) and actually decoded.
            // Read-only: requesting status changes nothing on the controller.
        _queue_req(rs485, network_msg_typ_t::CTRL_VALVE_REQ);
    }
}

/**
 * @brief Main FreeRTOS task for RS-485 communication and protocol handling.
 *
 * Entry point for the pool communication task. Runs in an infinite loop with
 * POOL_TASK_DELAY_MS between iterations. Each iteration:
 *   1. Services any pending requests from the main ESPHome task (non-blocking).
 *   2. Attempts to receive and process a packet from the RS-485 bus.
 *   3. If a transmit opportunity is detected (after controller broadcast), forwards
 *      one queued packet to the bus.
 *
 * On startup:
 *   - Initializes the RS-485 interface with pins from the IPC config.
 *   - Spawns the pool_req_task() sub-task for periodic controller queries.
 *
 * @param[in] ipc_void Pointer to the IPC structure (cast to void* for FreeRTOS API).
 *
 * @note This task runs on a separate FreeRTOS task from ESPHome's main loop.
 * @see pool_req_task() for periodic request handling
 * @see ipc_t for the inter-task communication structure
 */
void
pool_task(void * ipc_void)
{
    ESP_LOGI(TAG, "init ..");

    ipc_t * const ipc = static_cast<ipc_t*>(ipc_void);
    rs485_handle_t const rs485 = rs485_init(&ipc->config.rs485_pins);
    if (rs485 == nullptr) {
            // rs485_init() returns nullptr on queue/alloc failure. Every use below loads a
            // function pointer through this handle, so continuing panics on the first loop
            // iteration -- a reboot loop that never gets far enough to accept an OTA.
        ESP_LOGE(TAG, "rs485_init failed; pool_task exiting (no RS-485 comms)");
        vTaskDelete(nullptr);
        return;
    }

        // Subscribe to the ESP-IDF task watchdog; the timeout and the panic-on-expire
        // behaviour come from the ESP-IDF / ESPHome sdkconfig defaults
        // (CONFIG_ESP_TASK_WDT_TIMEOUT_S, CONFIG_ESP_TASK_WDT_PANIC), which this project does
        // not override -- check the generated sdkconfig for the values in force. Without it, a
        // hang in this task leaves the device looking alive (WiFi/API up, pings fine) while
        // pool control is silently dead -- the worst failure mode for equipment left
        // unattended for months. With it, a hang reboots the device instead.
        // The loop feeds it once per iteration; the normal path blocks for well under a
        // second because uart_read_bytes uses a 100ms timeout and the preamble scan is
        // byte-bounded.
    if (esp_task_wdt_add(nullptr) != ESP_OK) {
        ESP_LOGW(TAG, "could not subscribe pool_task to the task watchdog");
    }

        // periodically request information from controller
    if (xTaskCreatePinnedToCore(&pool_req_task, "pool_req_task", POOL_REQ_TASK_STACK_SIZE, rs485, 5, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create pool_req_task");
    }

        // Heartbeat proving THIS task's loop is still running, independent of RS485 activity
        // (pool_req_task's own periodic log is a different task). Time-based rather than
        // iteration-based, since one iteration takes anywhere from ~0 to ~2s.
    int64_t last_heartbeat_us = esp_timer_get_time();

    while (1) {

            // read from ipc->to_pool_q

        _service_requests_from_main(rs485, ipc);

            // read from the rs485 device, until there is a packet,
            // then move the packet up the protocol stack to process it.

        if (_service_pkts_from_rs485(rs485, ipc)) {

                // Drain the IPC queue again before transmitting. _service_pkts_from_rs485()
                // blocks for up to ~2s, and a Home Assistant command that arrives during that
                // window would otherwise still be sitting in to_pool_q when we get here --
                // so tx_q is empty, this transmit opportunity is wasted, and the command waits
                // a whole extra broadcast interval. Measured: this accounted for ~1.9s of a
                // ~2.7s round trip; the bus write plus controller ACK is only ~58ms.
            _service_requests_from_main(rs485, ipc);

                // there is a transmit opportunity after the pool controller
                // send a broadcast.  If there is rs485 transmit queue, then
                // create a network message and transmit it.

            _forward_queued_pkt_to_rs485(rs485, ipc);
        }
        esp_task_wdt_reset();   // we are alive; see esp_task_wdt_add() above

        int64_t const now_us = esp_timer_get_time();
        if (now_us - last_heartbeat_us >= 5000000) {
            last_heartbeat_us = now_us;
            ESP_LOGD(TAG, "pool_task heartbeat: loop alive");
        }
        vTaskDelay((TickType_t)POOL_TASK_DELAY_MS / portTICK_PERIOD_MS);
    }
}

}  // namespace opnpool
}  // namespace esphome