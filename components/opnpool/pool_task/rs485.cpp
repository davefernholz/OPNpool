/**
 * @file rs485.cpp
 * @brief RS485 driver: receive/send bytes to/from the RS485 transceiver
 *
 * @details
 * This file implements the RS485 hardware driver for the OPNpool component, providing
 * low-level functions to initialize, configure, and operate the RS485 transceiver. It
 * handles UART setup for half-duplex communication, GPIO configuration, and manages a
 * transmit queue for outgoing packets. The driver exposes a handle with function pointers
 * for higher-level protocol layers to interact with the RS485 interface, ensuring
 * reliable and efficient communication with pool equipment over the RS485 bus.
 *
 * The driver provides two key functions:
 * 1. Reading bytes from the RS-485 transceiver.
 * 2. Queueing outgoing byte streams, and dequeuing them to write the bytes to the RS-485
 *    transceiver.
 * 
 * ESPHome operates in a single-threaded environment, so explicit thread safety measures
 * are not required within the pool_task context.
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
#include <esp_types.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esphome/core/log.h>
#include <esp_rom_sys.h>
#include <string.h>

#include "rs485.h"
#include "datalink.h"
#include "datalink_pkt.h"
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"
#pragma GCC diagnostic error "-Wunused-parameter"

namespace esphome {
namespace opnpool {

constexpr char TAG[] = "rs485";

constexpr size_t      RX_BUF_SIZE = 127;
constexpr TickType_t  RX_TIMEOUT  = (100 / portTICK_PERIOD_MS);
constexpr TickType_t  TX_TIMEOUT  = (100 / portTICK_PERIOD_MS);

constexpr uart_port_t           UART_PORT = static_cast<uart_port_t>(UART_NUM_1);
constexpr int                   BAUD_RATE = 9600;
constexpr uart_word_length_t    DATA_BITS = UART_DATA_8_BITS;
constexpr uart_parity_t         PARITY    = UART_PARITY_DISABLE;
constexpr uart_stop_bits_t      STOP_BITS = UART_STOP_BITS_1;
constexpr uart_hw_flowcontrol_t FLOW_CTRL = UART_HW_FLOWCTRL_DISABLE; 
constexpr uart_sclk_t           CLOCK_SRC = UART_SCLK_DEFAULT;
constexpr uint8_t               RX_FLOW_CTRL_THRESH = 122;

    // LilyGO T-CAN485 transceiver control. Deliberately not YAML-configurable like the
    // rx/tx/rts pins: these are fixed board wiring, not a user choice.
constexpr gpio_num_t PIN_BOOST_EN  = GPIO_NUM_16;  // 5V boost supply enable
constexpr gpio_num_t PIN_XCVR_RE   = GPIO_NUM_17;  // MAX13487E RE, active low
constexpr gpio_num_t PIN_XCVR_SHDN = GPIO_NUM_19;  // MAX13487E SHDN, HIGH = enabled

static gpio_num_t  _rts_pin;

/**
 * @brief Returns the number of bytes available in the UART RX buffer.
 */
static int
_available()
{
    size_t length = 0;
        // NOT ESP_ERROR_CHECK: see _flush(). Reporting "nothing available" is always a safe
        // degradation; rebooting the chip is not.
    if (uart_get_buffered_data_len(UART_PORT, &length) != ESP_OK) {
        return 0;
    }
    return static_cast<int>(length);
}

/**
 * @brief          Reads bytes from the UART RX buffer with a timeout.
 *
 * @param[out] dst Destination buffer.
 * @param[in]  len Number of bytes to read.
 * @return         Number of bytes read.
 */
[[nodiscard]] static int
_read_bytes(uint8_t * dst, uint32_t len)
{
    int const n = uart_read_bytes(UART_PORT, dst, len, RX_TIMEOUT);

#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
        // Raw byte counter: distinguishes "UART is seeing bytes but framing/checksum fails"
        // from "nothing is arriving at the RX pin at all". Every other log in the receive
        // path sits downstream of framing, so neither case produces output there.
        // #if-guarded rather than relying on the log level alone: _find_preamble() reads one
        // byte per call, so esp_timer_get_time() (an interrupt-disable window) would otherwise
        // run per received byte even in builds where the log body is compiled out.
    static uint32_t total_bytes = 0;
    static int64_t last_report_us = 0;
    if (n > 0) total_bytes += n;
    int64_t const now_us = esp_timer_get_time();
    if (now_us - last_report_us >= 60000000) {  // every 60s
        last_report_us = now_us;
            // Heap visibility is worth keeping permanently: a leak here is silent until the
            // device suddenly reboots, and this codebase has already had one (see git log).
        ESP_LOGD(TAG, "free heap: %lu bytes (min ever %lu)",
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)esp_get_minimum_free_heap_size());
        ESP_LOGD(TAG, "raw bytes received: %lu", (unsigned long)total_bytes);
    }
#endif  // ESPHOME_LOG_LEVEL >= DEBUG

    return n;
}

/**
 * @brief         Writes bytes to the UART TX buffer.
 *
 * @param[in] src Source buffer.
 * @param[in] len Number of bytes to write.
 * @return        Number of bytes written.
 */
[[nodiscard]] static int
_write_bytes(uint8_t * src, size_t len)
{
    return uart_write_bytes(UART_PORT, (char *) src, len);
}

/**
 * @brief Flushes the UART TX and RX buffers, waiting for TX to complete.
 */
static void
_flush(void)
{
        // NOT ESP_ERROR_CHECK: that calls abort() -> panic reboot on any non-OK return, and
        // uart_wait_tx_done() legitimately returns ESP_ERR_TIMEOUT if the FIFO has not drained
        // within TX_TIMEOUT. This runs after EVERY transmit, so a single slow drain during a
        // Home Assistant write would reboot the device. Degrade gracefully instead.
    esp_err_t err = uart_wait_tx_done(UART_PORT, TX_TIMEOUT);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_wait_tx_done: %s (continuing)", esp_err_to_name(err));
    }
        // Blanket input flush, which also drops the echo of our own transmission: the
        // receiver stays enabled while we transmit, because the MAX13487E switches
        // direction in hardware rather than under driver control.
    err = uart_flush_input(UART_PORT);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_flush_input: %s (continuing)", esp_err_to_name(err));
    }
}

/**
 * @brief            Queues a packet for transmission on the RS-485 bus.
 *
 * @param[in] handle RS-485 handle.
 * @param[in] pkt    Packet to queue.
 */
static void
_queue(rs485_handle_t const handle, datalink_pkt_t const * const pkt, bool const urgent)
{
    if (pkt != nullptr) {
        rs485_q_msg_t msg = {
            .pkt = pkt,
        };
            // Only one packet is transmitted per transmit opportunity (~1/sec, gated on a
            // controller broadcast), and the periodic poller enqueues a burst of 5 every 30s.
            // Without prioritisation a user/HA command can sit behind those for several
            // seconds, which feels like a laggy or ignored button press.
            //
            // Both queues are FIFO. An earlier version instead pushed urgent packets to the
            // FRONT of a single queue, which gave the right priority but made user commands
            // LIFO relative to each other: pressing on then off within one transmit interval
            // queued [off, on] and transmitted them in that order, leaving the circuit ON.
            // Nothing was dropped and nothing timed out, so it presented as "the button
            // needed a few tries". Keep urgent traffic in its own FIFO.
        BaseType_t const ok = urgent ? xQueueSendToBack(handle->tx_q_urgent, &msg, 0)
                                     : xQueueSendToBack(handle->tx_q, &msg, 0);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "tx_q full -- dropping %s packet", urgent ? "USER" : "poll");
            free(pkt->skb);
            free((void *) pkt);
        }
    }
}

/**
 * @brief            Dequeues a packet from the RS-485 transmit queue.
 *
 * @param[in] handle RS-485 handle.
 * @return           Pointer to the dequeued packet, or NULL if none available.
 */
[[nodiscard]] static datalink_pkt_t const *
_dequeue(rs485_handle_t const handle)
{
    rs485_q_msg_t msg{};
        // Urgent first, so a user command still beats a queued poll request; FIFO within
        // each queue, so commands keep the order they were issued in.
    if (xQueueReceive(handle->tx_q_urgent, &msg, (TickType_t)0) == pdPASS ||
        xQueueReceive(handle->tx_q, &msg, (TickType_t)0) == pdPASS) {
        if (msg.pkt == nullptr) {
            ESP_LOGE(TAG, "Dequeued packet is null");
        }
        return msg.pkt;
    }
    return nullptr;
}

/**
 * @brief               Sets the RS-485 transceiver to transmit or receive mode.
 *
 * @param[in] tx_enable True to enable transmit mode, false for receive mode.
 */
static void
_tx_mode(bool const tx_enable)
{
    // messages should be sent directly after an A5 packets (and before any IC packets)
    // A note on the DE signal:
    //  - choose a GPIO that doesn't mind being pulled down during reset

    if (tx_enable) {
        gpio_set_level(_rts_pin, 1);  // enable RS485 transmit DE=1 and RE*=1 (DE=driver enable, RE*=inverted receive enable)
    } else {
        _flush();                     // wait until last byte starts transmitting
        esp_rom_delay_us(1500);       // wait until last byte is transmitted (10 bits / 9600 baud =~ 1042 µs)
        gpio_set_level(_rts_pin, 0);  // enable RS485 receive
    }
}

/**
 * @brief Initializes the RS485 hardware interface and driver.
 *
 * @details
 * Configures the specified UART port and GPIO pins for RS485 half-duplex communication,
 * sets up the UART parameters (baud rate, data bits, stop bits, etc.), and initializes
 * the request-to-send (RTS) pin for transmit/receive direction control. Allocates and
 * initializes the RS485 handle structure, sets up the transmit queue, and assigns
 * function pointers for RS485 operations. Returns a handle to the initialized RS485
 * interface for use by higher-level protocol layers.
 *
 * @param[in] rs485_pins Pointer to the structure containing RX, TX, and RTS pin numbers.
 * @return               Handle to the initialized RS485 interface.
 */
[[nodiscard]] rs485_handle_t
rs485_init(rs485_pins_t const * const rs485_pins)
{
    gpio_num_t const rx_pin = static_cast<gpio_num_t>(rs485_pins->rx_pin);
    gpio_num_t const tx_pin = static_cast<gpio_num_t>(rs485_pins->tx_pin);
    _rts_pin = static_cast<gpio_num_t>(rs485_pins->rts_pin);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"    
    uart_config_t const uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = DATA_BITS,
        .parity = PARITY,
        .stop_bits = STOP_BITS,
        .flow_ctrl = FLOW_CTRL,
        .rx_flow_ctrl_thresh = RX_FLOW_CTRL_THRESH,
        .source_clk = CLOCK_SRC,
        // installed ESP-IDF (5.1.6) predates the `flags` member added in later IDF versions
    };
#pragma GCC diagnostic pop

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << static_cast<uint8_t>(_rts_pin)),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK( gpio_config(&io_conf) );
    gpio_set_level(_rts_pin, 0);

    ESP_LOGI(TAG, "Initializing RS485 on UART%u (RX pin %u, TX pin %u, RTS pin %u) ..",
             UART_PORT, rx_pin, tx_pin, _rts_pin);

        // Assert the MAX13487E transceiver enables directly here, immediately before the UART
        // is brought up, rather than via ESPHome `output:` components + on_boot: that ordering
        // did not reliably leave the pins in the required state by this point. The failure is
        // nasty because it is intermittent -- a floating SHDN can work for hours, then stop
        // and stay dead across reboots, which reads exactly like a loose wire.
        // RE=LOW + SHDN=HIGH is the only combination that yields edges on the RX pin.
        // Note LilyGO's own pin_config.h and board README disagree about these assignments.
    {
        uint64_t const en_mask = (1ULL << PIN_BOOST_EN) | (1ULL << PIN_XCVR_RE) |
                                 (1ULL << PIN_XCVR_SHDN);
        gpio_config_t en_conf = {
            .pin_bit_mask = en_mask,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&en_conf) == ESP_OK) {
            gpio_set_level(PIN_BOOST_EN,  1);
            gpio_set_level(PIN_XCVR_SHDN, 1);
            gpio_set_level(PIN_XCVR_RE,   0);
            esp_rom_delay_us(20000);          // let the transceiver settle
            ESP_LOGI(TAG, "transceiver enables asserted (SHDN high, RE low)");
        } else {
            ESP_LOGE(TAG, "failed to configure transceiver enable pins");
        }
    }

        // Upstream discarded all of these return values. A failure in any one of them
        // produces total RX silence on an otherwise healthy-looking device, with nothing
        // logged anywhere -- so check and report them.
    esp_err_t err;
    err = uart_param_config(UART_PORT, &uart_config);
    if (err != ESP_OK) ESP_LOGE(TAG, "uart_param_config FAILED: %s", esp_err_to_name(err));
    err = uart_set_pin(UART_PORT, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) ESP_LOGE(TAG, "uart_set_pin FAILED: %s", esp_err_to_name(err));
    err = uart_driver_install(UART_PORT, RX_BUF_SIZE * 2, 0, 0, NULL, 0);  // no tx buffer
    if (err != ESP_OK) ESP_LOGE(TAG, "uart_driver_install FAILED: %s", esp_err_to_name(err));
        // Plain UART, not ESP-IDF's UART_MODE_RS485_HALF_DUPLEX: the LilyGO's MAX13487E
        // transceiver already does auto-direction switching in hardware, and letting the
        // ESP-IDF driver also manage half-duplex/echo-suppression on top of that caused
        // bus corruption (checksum errors, dropped commands) whenever this device transmitted.
    uart_set_mode(UART_PORT, UART_MODE_UART);

        // Depth 16 (upstream used 5): only one packet is sent per ~1s transmit opportunity,
        // so a burst -- e.g. an HA scene toggling several circuits, landing on top of the
        // periodic 3-request poll -- could overflow and silently drop user commands.
    QueueHandle_t const tx_q = xQueueCreate(16, sizeof(rs485_q_msg_t));
    if (tx_q == nullptr) {
        ESP_LOGE(TAG, "Failed to create TX queue");
        return nullptr;
    }
    QueueHandle_t const tx_q_urgent = xQueueCreate(16, sizeof(rs485_q_msg_t));
    if (tx_q_urgent == nullptr) {
        ESP_LOGE(TAG, "Failed to create urgent TX queue");
        vQueueDelete(tx_q);
        return nullptr;
    }

    rs485_handle_t handle = static_cast<rs485_handle_t>(calloc(1, sizeof(rs485_instance_t)));
    if (handle == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate RS485 handle");
        vQueueDelete(tx_q);
        vQueueDelete(tx_q_urgent);
        return nullptr;
    }

    handle->available = _available;
    handle->read_bytes = _read_bytes;
    handle->write_bytes = _write_bytes;
    handle->flush = _flush;
    handle->tx_mode = _tx_mode;
    handle->queue = _queue;
    handle->dequeue = _dequeue;
    handle->tx_q = tx_q;
    handle->tx_q_urgent = tx_q_urgent;
    
    _tx_mode(false);

    return handle;
}

}  // namespace opnpool
}  // namespace esphome