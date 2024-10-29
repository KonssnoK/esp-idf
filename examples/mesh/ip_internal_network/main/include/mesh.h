/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once

typedef void (*mesh_connect_cb_t)(bool is_ppp);
typedef void (*mesh_disconnect_cb_t)(bool is_ppp);
typedef void (*mesh_reset_cb_t)(void);
