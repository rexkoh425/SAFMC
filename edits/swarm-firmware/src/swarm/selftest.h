/*
 * Mapping-only self-test helper
 * Creates a small RTOS task that simulates START and periodically broadcasts poses.
 */
#ifndef SWARM_SELFTEST_H
#define SWARM_SELFTEST_H

/* Starts the self-test task if this unit is the mapping drone (swarm_id == 0). */
void swarm_selftest_start_if_mapping(void);

#endif /* SWARM_SELFTEST_H */

