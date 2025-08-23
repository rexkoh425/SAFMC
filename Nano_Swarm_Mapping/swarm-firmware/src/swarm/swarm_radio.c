/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#include "swarm_radio.h"

#include <string.h>
#include <assert.h>

#include "cfassert.h"
#include "radiolink.h"
#include "debug.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "static_mem.h"

#include "swarm.h"
#include "../util.h"

#define RADIO_SLOT_DURATION     2   // ms

#define RADIO_PKT_HEADER_SIZE   2
#define RADIO_PKT_MAX_DATA_SIZE (P2P_MAX_DATA_SIZE - RADIO_PKT_HEADER_SIZE)

typedef struct {
    uint8_t src: 4;
    uint8_t dst: 4;
    uint8_t ack: 1;
    uint8_t end: 1;
    uint8_t tag: 2;
    uint8_t seq: 4;
    uint8_t data[RADIO_PKT_MAX_DATA_SIZE];
} __attribute__((packed)) comm_packet_t;
static_assert(
        sizeof(comm_packet_t) ==
        RADIO_PKT_HEADER_SIZE + RADIO_PKT_MAX_DATA_SIZE,
        "comm_packet_t fits");

static SemaphoreHandle_t ack_mutex;
static StaticSemaphore_t ack_mutex_buffer;
static QueueHandle_t rx_queue;
static StaticQueue_t rx_queue_static;
static uint8_t rx_queue_buf[2 * SWARM_NUM_DRONES * sizeof(P2PPacket)];

static uint8_t seq_nums[SWARM_NUM_DEVICES];
static uint16_t ack_received;
static uint8_t rx_buffer[SWARM_NUM_DEVICES - 1][1536];
static size_t rx_buffer_offset[SWARM_NUM_DEVICES - 1] = {0};

STATIC_MEM_TASK_ALLOC(rx_task, 256);

static void swarm_radio_rx_task(void *parameters);

void swarm_radio_init() {
    ASSERT(swarm_id != 0xFF);

    // Create synchronisation primitives
    ack_mutex = xSemaphoreCreateMutexStatic(&ack_mutex_buffer);

    // Set up RX
    rx_queue = xQueueCreateStatic(sizeof(rx_queue_buf) / sizeof(P2PPacket),
                                  sizeof(P2PPacket), rx_queue_buf,
                                  &rx_queue_static);
    p2pRegisterCB(swarm_radio_p2p_handler);
    STATIC_MEM_TASK_CREATE(rx_task, swarm_radio_rx_task, "swarm_rx", NULL, 0);
}

static void send_ack(uint8_t dst, uint8_t seq) {
    P2PPacket tx_p2p_packet = {0};
    comm_packet_t *tx_packet = (comm_packet_t *) tx_p2p_packet.data;
    tx_packet->src = swarm_id;
    tx_packet->dst = dst;
    tx_packet->ack = 1;
    tx_packet->seq = seq;
    tx_p2p_packet.size = RADIO_PKT_HEADER_SIZE;
    radiolinkSendP2PPacketBroadcast(&tx_p2p_packet);
}



/*
void swarm_radio_send_msg(uint8_t dst, uint8_t tag, void *data, size_t len) {
    ASSERT(len <= sizeof(rx_buffer[0]));

    P2PPacket tx_p2p_packet = {0};
    comm_packet_t *tx_packet = (comm_packet_t *) tx_p2p_packet.data;
    tx_packet->src = swarm_id;
    tx_packet->dst = dst;
    tx_packet->tag = tag;

    uint8_t seq = (seq_nums[swarm_id] + 1) & 0xF;
    size_t offset = 0;

    while (offset < len) {
        const size_t payload_len = MIN(len - offset, RADIO_PKT_MAX_DATA_SIZE);

        // header fields
        tx_packet->end = (offset + payload_len) >= len;
        tx_p2p_packet.size = RADIO_PKT_HEADER_SIZE + payload_len;

        // copy ONLY the payload
        memcpy(tx_packet->data, ((const uint8_t*)data) + offset, payload_len);

        // ACK expectations
        uint16_t ack_expect;
        int wait_periods;
        if (dst == RADIO_ADR_BROADCAST) {
            wait_periods = SWARM_NUM_DEVICES + 4;
            ack_expect = ((1 << SWARM_NUM_DEVICES) - 1) & ~(1 << swarm_id);
        } else {
            wait_periods = 4;
            ack_expect = 1 << dst;
        }

        xSemaphoreTake(ack_mutex, portMAX_DELAY);
        tx_packet->seq = seq_nums[swarm_id] = seq;
        ack_received = 0;
        xSemaphoreGive(ack_mutex);

        radiolinkSendP2PPacketBroadcast(&tx_p2p_packet);

        bool success = false;
        for (int wp = wait_periods; wp > 0; wp--) {
            vTaskDelay(M2T(RADIO_SLOT_DURATION));
            xSemaphoreTake(ack_mutex, portMAX_DELAY);
            if (ack_received == ack_expect) { xSemaphoreGive(ack_mutex); success = true; break; }
            xSemaphoreGive(ack_mutex);
        }

        if (success) {
            offset += payload_len;               // ✅ advance by what we actually sent
            seq = (seq + 1) & 0xF;
        } else {
            vTaskDelay(swarm_id * RADIO_SLOT_DURATION);
        }
    }
}
*/
/*
void swarm_radio_send_msg(uint8_t dst, uint8_t tag, void *data, size_t len)
{
    ASSERT(len <= sizeof(rx_buffer[0]));

    P2PPacket tx_p2p_packet = (P2PPacket){0};
    comm_packet_t *tx_packet = (comm_packet_t *)tx_p2p_packet.data;
    tx_packet->src = swarm_id;
    tx_packet->dst = dst;
    tx_packet->tag = tag;

    uint8_t seq = (seq_nums[swarm_id] + 1) & 0xF;
    size_t offset = 0;

    const bool no_ack = (dst == RADIO_ADR_BRIDGE);   // <-- bridge never ACKs

    while (offset < len) {
        const size_t payload_len = MIN(len - offset, RADIO_PKT_MAX_DATA_SIZE);

        tx_packet->end = (offset + payload_len) >= len;
        tx_p2p_packet.size = RADIO_PKT_HEADER_SIZE + payload_len;

        // copy ONLY the payload we intend to send in this chunk
        memcpy(tx_packet->data, ((const uint8_t*)data) + offset, payload_len);

        uint16_t ack_expect = 0;
        int wait_periods = 0;

        if (!no_ack) {
            if (dst == RADIO_ADR_BROADCAST) {
                // Expect acks from DRONES only, NOT from the bridge
                // drones have IDs [0 .. SWARM_NUM_DRONES-1], bridge is RADIO_ADR_BRIDGE
                ack_expect = ((1u << SWARM_NUM_DRONES) - 1u) & ~(1u << swarm_id);
                wait_periods = SWARM_NUM_DRONES + 2;
            } else {
                ack_expect = 1u << dst;
                wait_periods = 4;
            }
        }

        xSemaphoreTake(ack_mutex, portMAX_DELAY);
        tx_packet->seq = seq_nums[swarm_id] = seq;
        ack_received = 0;
        xSemaphoreGive(ack_mutex);

        // DEBUG_PRINT("[TX] dst=%u tag=%u pay=%u end=%u seq=%u\n",
        //   (unsigned)tx_packet->dst, (unsigned)tx_packet->tag,
        //   (unsigned)payload_len, (unsigned)tx_packet->end, (unsigned)tx_packet->seq);

        radiolinkSendP2PPacketBroadcast(&tx_p2p_packet);

        if (no_ack) {
            // Fire-and-forget to bridge: advance immediately
            offset += payload_len;
            seq = (seq + 1) & 0xF;
            continue;
        }

        bool success = false;
        for (int wp = wait_periods; wp > 0; wp--) {
            vTaskDelay(M2T(RADIO_SLOT_DURATION));
            xSemaphoreTake(ack_mutex, portMAX_DELAY);
            if (ack_received == ack_expect) { xSemaphoreGive(ack_mutex); success = true; break; }
            xSemaphoreGive(ack_mutex);
        }

        if (success) {
            offset += payload_len;
            seq = (seq + 1) & 0xF;
        } else {
            vTaskDelay(M2T(swarm_id * RADIO_SLOT_DURATION));
        }
    }
}
*/
void swarm_radio_send_msg(uint8_t dst, uint8_t tag, void *data, size_t len) {
    ASSERT(len <= sizeof(rx_buffer[0]));

    P2PPacket tx_p2p_packet = {0};
    comm_packet_t *tx_packet = (comm_packet_t *) tx_p2p_packet.data;
    tx_packet->src = swarm_id;
    tx_packet->dst = dst;
    tx_packet->tag = tag;

    bool expect_ack = (dst != RADIO_ADR_BRIDGE) && (dst != RADIO_ADR_BROADCAST);

    uint8_t seq = (seq_nums[swarm_id] + 1) & 0xF;
    size_t offset = 0;

    while (offset < len) {
        size_t payload_len = MIN(len - offset, RADIO_PKT_MAX_DATA_SIZE);

        tx_packet->end = (offset + payload_len) >= len;
        tx_p2p_packet.size = RADIO_PKT_HEADER_SIZE + payload_len;
        memcpy(tx_packet->data, ((const uint8_t*)data) + offset, payload_len);

        if (expect_ack) {
            uint16_t ack_expect;
            int wait_periods;
            if (dst == RADIO_ADR_BROADCAST) {
                // (You can also choose to NOT expect ACKs for broadcast)
                wait_periods = SWARM_NUM_DEVICES + 4;
                ack_expect = ((1 << SWARM_NUM_DEVICES) - 1) & ~(1 << swarm_id);
            } else {
                wait_periods = 4;
                ack_expect = 1 << dst;
            }

            xSemaphoreTake(ack_mutex, portMAX_DELAY);
            tx_packet->seq = seq_nums[swarm_id] = seq;
            ack_received = 0;
            xSemaphoreGive(ack_mutex);

            radiolinkSendP2PPacketBroadcast(&tx_p2p_packet);

            bool success = false;
            for (int wp = wait_periods; wp > 0; wp--) {
                vTaskDelay(M2T(RADIO_SLOT_DURATION));
                xSemaphoreTake(ack_mutex, portMAX_DELAY);
                if (ack_received == ack_expect) { xSemaphoreGive(ack_mutex); success = true; break; }
                xSemaphoreGive(ack_mutex);
            }

            if (!success) {
                vTaskDelay(swarm_id * RADIO_SLOT_DURATION);
                continue; // retry same chunk
            }
        } else {
            // Fire-and-forget when sending to the bridge
            xSemaphoreTake(ack_mutex, portMAX_DELAY);
            tx_packet->seq = seq_nums[swarm_id] = seq;
            xSemaphoreGive(ack_mutex);

            radiolinkSendP2PPacketBroadcast(&tx_p2p_packet);
        }

        offset += payload_len;
        seq = (seq + 1) & 0xF;
    }
}


static void handle_rx_packet(comm_packet_t *p, size_t len) {

    // Check packet destination
    if (p->dst != swarm_id && p->dst != RADIO_ADR_BROADCAST) return;

    // Check if it is an ACK packet
    if (p->ack) {
        xSemaphoreTake(ack_mutex, portMAX_DELAY);
        if (p->seq == seq_nums[swarm_id]) {
            ack_received |= 1 << p->src;
        }
        xSemaphoreGive(ack_mutex);
    } else {

        // Delay if broadcast to avoid collisions
        if (p->dst == RADIO_ADR_BROADCAST) {
            vTaskDelay(M2T(swarm_id * RADIO_SLOT_DURATION));
        }

        // Acknowledge the packet
        send_ack(p->src, p->seq);

        // Check if this is a new packet
        if (p->seq != seq_nums[p->src]) {
            seq_nums[p->src] = p->seq;

            // Copy packet body to the buffer
            int rx_buffer_index = p->src < swarm_id ? p->src : p->src - 1;
            ASSERT(rx_buffer_offset[rx_buffer_index] + len -
                   RADIO_PKT_HEADER_SIZE <=
                   sizeof(rx_buffer[0]));
            memcpy(rx_buffer[rx_buffer_index] +
                   rx_buffer_offset[rx_buffer_index],
                   p->data, len - RADIO_PKT_HEADER_SIZE);
            rx_buffer_offset[rx_buffer_index] += len - RADIO_PKT_HEADER_SIZE;

            // Check if this is the end of a message
            if (p->end) {
                swarm_radio_handle_msg(p->src, p->tag, rx_buffer[rx_buffer_index],
                                       rx_buffer_offset[rx_buffer_index]);
                rx_buffer_offset[rx_buffer_index] = 0;
            }

        }

    }

}

void swarm_radio_p2p_handler(P2PPacket *p) {
    if (!xQueueSend(rx_queue, p, M2T(10))) {
        DEBUG_PRINT("P2P PACKET DROPPED!\n");
    }
}

static void swarm_radio_rx_task(void *parameters) {

    while (true) {

        // Check if there is a packet to receive
        P2PPacket rx_packet;
        if (xQueueReceive(rx_queue, &rx_packet, M2T(20))) {
            handle_rx_packet((comm_packet_t *) rx_packet.data, rx_packet.size);
        }
    }
}
