/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Carl Friess
 */

#ifndef SWARM_COMM_H
#define SWARM_COMM_H

#include <stdint.h>
#include <stddef.h>

#include "swarm_graph.h"

/* Message tags on the P2P link */
#define COMM_MSG_TAG_POSE       0
#define COMM_MSG_TAG_SCAN_REQ   1
#define COMM_MSG_TAG_SCAN_RES   2
#define COMM_MSG_TAG_CTRL       3

/* Control message types */
#define CTRL_MSG_TYPE_START     0
#define CTRL_MSG_TYPE_DONE      1
#define CTRL_MSG_TYPE_POSE      2

/* Matches bridge.py unpack "<BxH": type (1B), pad (1B), pose_id (2B) */
typedef struct __attribute__((packed)) {
    uint8_t         type;      /* CTRL_MSG_TYPE_* */
    uint8_t         reserved;  /* padding to match Python 'x' */
    swarm_pose_id_t pose;      /* 16-bit pose id (pose.raw)    */
} swarm_comm_ctrl_msg_t;

void swarm_comm_init(void);

void swarm_comm_broadcast_pose(swarm_pose_id_t id);

void swarm_comm_fetch_scan(uint8_t addr, swarm_pose_id_t id);

void swarm_comm_send_scan(uint8_t addr, swarm_pose_id_t id);

void swarm_comm_send_ctrl_msg(uint8_t addr, swarm_comm_ctrl_msg_t msg);

void swarm_comm_broadcast_ctrl_msg(swarm_comm_ctrl_msg_t msg);

void swarm_comm_handle_ctrl_msg(uint8_t src, swarm_comm_ctrl_msg_t msg);

#endif // SWARM_COMM_H

