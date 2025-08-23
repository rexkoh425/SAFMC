/*
 * Mapping-only self-test
 * - Waits ~1s after boot
 * - Injects a CTRL START to the local control handler (no radio needed)
 * - Seeds a pose in the local graph
 * - Periodically updates + broadcasts poses (and optionally sends a scan)
 */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "static_mem.h"

#include "swarm.h"           // swarm_id
#include "swarm_comm.h"      // ctrl msg + API
#include "swarm_graph.h"     // store/load poses
#include "swarm_radio.h"     // RADIO_ADR_BRIDGE, tags

/* Keep the task small to avoid stealing stack from other modules */
STATIC_MEM_TASK_ALLOC(selftest_task, 256);

static void selftest_task_fn(void *arg) {
    /* Let radios/tasks come up */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* (A) Simulate receiving a START control (no bridge required) */
    swarm_comm_ctrl_msg_t start = {0};
    start.type = CTRL_MSG_TYPE_START;     /* matches swarm_comm.h */
    start.pose.raw = 0;
    printf("[SELFTEST] Inject CTRL START (type=%u)\n", (unsigned)start.type);
    /* Pretend it came from the bridge (ID=1 by convention) */
    swarm_comm_handle_ctrl_msg(1, start);

    /* (B) Seed one pose locally so broadcast has something to send */
    swarm_pose_t pose = {0};
    pose.id.raw = (swarm_id << 12) | 1;   /* e.g., 0/0001 */
    pose.x = 0.0f; pose.y = 0.0f; pose.yaw = 0.0f;
    pose.num_edges = 0;
    swarm_graph_store_pose(&pose);

    /* (C) Periodically update + broadcast */
    uint16_t seq = 1;
    for (;;) {
        /* Tweak pose a bit so logs change */
        uint16_t node = (seq & 0x0FFF);
        if (node == 0) node = 1;
        pose.id.raw = (swarm_id << 12) | node;
        pose.x += 0.10f; pose.y += 0.00f; pose.yaw += 0.01f;
        swarm_graph_store_pose(&pose);

        printf("[SELFTEST] Broadcast POSE id=0x%04X (x=%.2f y=%.2f yaw=%.2f)\n",
               (unsigned)pose.id.raw, pose.x, pose.y, pose.yaw);

        /* Normal path: loads from graph then sends on P2P (no receiver needed here) */
        swarm_comm_broadcast_pose(pose.id);

        /* Optional: exercise scan TX path (receiver not required) */
        // printf("[SELFTEST] TX SCAN_RES pose=0x%04X to %u\n",
        //        (unsigned)pose.id.raw, (unsigned)RADIO_ADR_BRIDGE);
        // swarm_comm_send_scan(RADIO_ADR_BRIDGE, pose.id);

        seq++;
        vTaskDelay(pdMS_TO_TICKS(1000));  /* 1 Hz */
    }
}

void swarm_selftest_start_if_mapping(void) {
    /* Only run on the mapping unit to avoid RF spam if multiple are powered */
    if (swarm_id == 0) {
        printf("[SELFTEST] Starting mapping self-test task (id=%u)\n", (unsigned)swarm_id);
        STATIC_MEM_TASK_CREATE(selftest_task, selftest_task_fn, "selftest", NULL, 0);
    } else {
        /* Silent on non-mapping units */
    }
}

