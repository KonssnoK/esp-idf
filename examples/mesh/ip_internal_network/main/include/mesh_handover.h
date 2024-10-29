/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once

#include "esp_mesh.h"
#include "mesh.h"

typedef void (*timer_callback_t)(void*);
// This file contains definitions for Modem handover functionalities

// Library initialization
void mesh_handover_init(mesh_router_t* router,
    timer_callback_t callback_disconnect,
    timer_callback_t callback_reconnect,
    timer_callback_t callback_wifi_probe
    );

void mesh_handover_set_callbacks(
    mesh_connect_cb_t connect_cb,
    mesh_disconnect_cb_t disconnect_cb);

// Mesh callbacks for timers
void mesh_handover_disconnect_callback();
void mesh_handover_reconnect_callback();
void mesh_handover_wifi_probe_callback();

// Mesh events related to modem handover
void mesh_handover_no_parent_found(bool forced);
void mesh_handover_parent_connected();
void mesh_handover_parent_disconnected(bool forced, bool dont_reconnect);
void mesh_handover_root_fixed(bool fixed);
void mesh_handover_wifi_scan_done(int num);
void mesh_handover_check_lte_only_mode(void);
void mesh_handover_stop();
bool mesh_handover_is_transitioning();
int mesh_handover_get_stats(mesh_addr_t* parent);
