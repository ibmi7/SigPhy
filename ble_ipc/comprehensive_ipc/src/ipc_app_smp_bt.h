/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RPC_APP_SMP_BT_H_
#define RPC_APP_SMP_BT_H_

#include <zephyr/types.h>

/**
 * @file
 * @defgroup entropy_ser Entropy driver serialization
 * @{
 * @brief Entropy serialization API.
 */

#ifdef __cplusplus
extern "C" {
#endif

int smp_receive_data(const void *buf, uint16_t len);
void set_smp_mtu(uint16_t len);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* RPC_APP_SMP_BT_H_ */
