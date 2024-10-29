/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "mesh_handover.h"

#include <string.h>
#include "os.h"
//#include "mesh_driver.h"
#include "esp_event.h"
#include "esp_event_legacy.h"
#include "esp_netif_types.h"
#include "esp_wifi.h"
#include "mesh_netif.h"
#include "esp_log.h"
//#include "hardware.h"
//#include "modem.h"

static const char* TAG = "mesh_hand";
//FAKE MODEM HERE
typedef enum {
    MODEM_UNINITIALIZED       = 0,
    MODEM_OFF                 ,
    MODEM_TURNING_ON          ,
    MODEM_ON_STARTING_IFCS    ,
    MODEM_ON_AT_READY         ,
    MODEM_ON_NO_IFCS          ,
    MODEM_TURNING_OFF         ,
    MODEM_OFF_STOPPING_IFCS   ,
    MODEM_STARTING_PPP        ,
    MODEM_WAITING_FOR_IP      ,
    MODEM_CONNECTED           ,
    MODEM_DISCONNECTING       ,
    MODEM_UPDATING_FW         ,
    MODEM_ON_UPDATE_FW_FAILED ,
} modem_state_t;


modem_state_t modem_get_state(void)
{
    return MODEM_ON_AT_READY;
}

error_t modem_start(void)
{
    esp_err_t err;
    wifi_config_t config = {
        .sta.ssid = "KS",
        .sta.password = "C0ns0lar020136"
    };
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err) {
        ESP_LOGW(TAG, "mode 0x%02X", err);
    }
    err = esp_wifi_set_config(
        WIFI_MODE_STA,
        &config
    );
    if (err) {
        ESP_LOGW(TAG, "config 0x%02X", err);
    } else {
        err = esp_wifi_start();
        if (err) {
            ESP_LOGW(TAG, "connect 0x%02X", err);
        }
    }
    return err;
}

error_t modem_stop(void)
{
    esp_err_t err;
    err = esp_wifi_stop();
    if (err) {
        ESP_LOGW(TAG, "stop 0x%02X", err);
    }
    return ERR_OK;
}

bool hardware_has_modem(void)
{
    return false;
    //uint8_t root[6] = {};
    uint8_t mac[6];
    esp_base_mac_addr_get(mac);

    return mac[5] == 0x04;
    //return memcmp(root, mac, 6);
}
//FAKE MODEM END



// If enabled, we handover to the modem in case of disconnection from wifi
#define MESH_HANDLE_MODEM_HANDOVER

typedef struct mesh_fixed_last_parent_ {
    wifi_ap_record_t parent_record; // Data coming from the AP
    mesh_assoc_t parent_assoc;      // Data coming from the MESH layer
    mesh_type_t type;
    int layer;
    bool found;
    wifi_config_t parent;
} mesh_fixed_last_parent_t;

typedef struct mesh_handover_ {
    bool init;
    mesh_connect_cb_t connect_callback;
    mesh_disconnect_cb_t disconnect_callback;
    bool master_active;                             // Keep track of when to switch to fixed ROOT. On disconnection it gets TRUE once, on connection it gets FALSE once
    bool mesh_connected;                            // Keep track of when the device is connected to a mesh parent
#ifdef MESH_HANDLE_MODEM_HANDOVER
    esp_timer_handle_t timer_wifi_probe;
    esp_timer_handle_t timer_disconnect;
    esp_timer_handle_t timer_reconnect;
    int timer_wifi_probe_root_count;                // After how long the fixed ROOT node should look for wifi. Increased every MESH_MODEM_WIFI_PROBE_INTERVAL
    bool fixed_root_has_parent;
    mesh_router_t mesh_router;
    bool lte_forced_handover;
    // Current layer and current parent
    mesh_fixed_last_parent_t last_scan_result;
    mesh_fixed_last_parent_t active_parent_result;
    uint32_t last_scan_count;
    uint32_t found_idle_devices_and_no_connection;
    bool allow_low_rssi_connections;
#endif
} mesh_handover_t;

static mesh_handover_t priv_self = { 0 };
static mesh_handover_t* self = &priv_self;

#ifdef MESH_HANDLE_MODEM_HANDOVER
#define MESH_MODEM_RECONNECTED_THRESHOLD        (10000)
#define MESH_MODEM_DISCONNECTED_THRESHOLD       (30000)
#define MESH_MODEM_WIFI_PROBE_INTERVAL          (30000) // 30 seconds
#define MESH_MODEM_WIFI_PROBE_THRESHOLD         (3)     // 1.5 minutes
#define MESH_MODEM_WIFI_PROBE_FORCED_THRESHOLD  (120)   // 60 minutes
#define MESH_MODEM_RETRY_THRESHOLD              (10)    // 5 minutes
#define MESH_HANDOVER_TODS_PROPAGATION          (1000)
#define MESH_HANDOVER_5_MIN                     (5)
#define MESH_HANDOVER_30_MIN                    (30)
#endif

void mesh_handover_wifi_probe_callback()
{
#ifdef MESH_HANDLE_MODEM_HANDOVER
    // This function is called periodically to check for the WIFI to be reachable
    // It's used by both LTE and non LTE devices
    // to understand what kind of mesh network enable (dynamic/fixed root)
    struct tm now = { 0 };
    time_t nowt = time(NULL);
    struct tm* tmp = localtime(&nowt);
    memcpy(&now, tmp, sizeof(now));

    if (hardware_has_modem()) {
        self->timer_wifi_probe_root_count++;
        int probe_threshold = MESH_MODEM_WIFI_PROBE_THRESHOLD;
        if (self->lte_forced_handover) {
            // In case the handover was forced by the fact that we were failing
            // to connect to the wifi, we wait 30 minutes before checking for external connectivity
            probe_threshold = MESH_MODEM_WIFI_PROBE_FORCED_THRESHOLD;
        }
        // The scan for the root device is ignored here to avoid
        // shifting from frequency to frequency looking for wifi
        // while the other devices are trying to connect!
        if (self->timer_wifi_probe_root_count < probe_threshold) {
            // Ignore scan request for the first N times
            if (!self->lte_forced_handover || (self->lte_forced_handover && (now.tm_min % MESH_HANDOVER_5_MIN) == 0)) {
                // Print to server only if not forced or every 5 minutes
                ESP_LOGW(TAG, "Skipping wifi connectivity (count %d)", self->lte_forced_handover ? -1 : self->timer_wifi_probe_root_count);
            } else {
                ESP_LOGW(TAG, "Skipping wifi connectivity (count %d)[LOCAL]", self->lte_forced_handover ? -1 : self->timer_wifi_probe_root_count);
            }
            return;
        } else if ((self->timer_wifi_probe_root_count % MESH_MODEM_RETRY_THRESHOLD) == 0) {
            // Mode than the threshold passed, we are supposed to be online with the modem.
            // Double check
            if (modem_get_state() == MODEM_ON_AT_READY) {
                ESP_LOGW(TAG, "Modem is not connected, retry");
                modem_start();
            }
        }
    }

    if ((now.tm_min % MESH_HANDOVER_30_MIN) == 0) {
        // Print to server only every X min
        ESP_LOGI(TAG, "Checking wifi connectivity (count %d)", self->timer_wifi_probe_root_count);
    } else {
        ESP_LOGI(TAG, "Checking wifi connectivity (count %d)[LOCAL]", self->timer_wifi_probe_root_count);
    }

    esp_wifi_scan_stop();
    // Remove any scan results that have been obtained (just in case)
    esp_mesh_flush_scan_result();
    wifi_scan_config_t scan_config = { 0 };
    scan_config.show_hidden = 1;
    scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;

    esp_err_t err = esp_wifi_scan_start(&scan_config, false);
    if ((err == ESP_FAIL) || (err == ESP_ERR_WIFI_STATE)) {
        ESP_LOGW(TAG, "esp_wifi_scan_start %d", err);
        //ongoing = true;
    }
#endif
}

void mesh_handover_disconnect_callback()
{
#ifdef MESH_HANDLE_MODEM_HANDOVER
    ESP_LOGW(TAG, "Triggering FIXED ROOT handover");

    esp_mesh_disconnect();

    if (esp_mesh_get_self_organized()) {

        if (hardware_has_modem()) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mesh_set_type(MESH_ROOT));
            if (modem_get_state() != MODEM_CONNECTED) {
                modem_start();
            } else {
                ESP_LOGW(TAG, "Modem already running");
                self->connect_callback(true);
            }
        } else {
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mesh_set_type(MESH_IDLE));
            esp_mesh_fix_root(true);
        }
        // Switch to fixed root and stop auto-reconnection
        esp_mesh_set_self_organized(false, false);
        self->fixed_root_has_parent = false;

        // Reset fixed layer stats
        memset(&self->last_scan_result, 0, sizeof(mesh_fixed_last_parent_t));
        memset(&self->active_parent_result, 0, sizeof(mesh_fixed_last_parent_t));
        self->allow_low_rssi_connections = false;
        self->found_idle_devices_and_no_connection = 0;
    }
    // Start scanning for Wifi. NOTE: The root will be delayed
    esp_timer_start_periodic(self->timer_wifi_probe, MESH_MODEM_WIFI_PROBE_INTERVAL*1000);
#endif
}

void mesh_handover_reconnect_callback()
{
#ifdef MESH_HANDLE_MODEM_HANDOVER
    if (hardware_has_modem()) {
        if ((modem_get_state() == MODEM_CONNECTED) ||
            (modem_get_state() != MODEM_ON_AT_READY)
            ) {
            // If we were using the LTE or the LTE is any state that is not ready,
            // we stop

            ESP_LOGW(TAG, "Stopping modem");
            modem_stop();

            // We don't have the PPP interface anymore
            self->disconnect_callback(true);
        }
    }
#endif
}

#ifdef MESH_HANDLE_MODEM_HANDOVER
static void mesh_handover_switch_to_dynamic()
{
    ESP_LOGW(TAG, "Triggering DYNAMIC MESH handover");

    // Stop looking for WIFIs
    esp_timer_stop(self->timer_wifi_probe);
    // Reset counter for ROOT
    self->timer_wifi_probe_root_count = 0;

    if (hardware_has_modem()) {
        // we tell children that we don't have anymore access to internet
        // This is done here because the mesh AP interface will be cleared by mesh_netifs_stop
        /*if (self->disconnect_callback) {
            // We call with false because we want the PPP ip to be preserved
            self->disconnect_callback(false);
        }*/
        // Give the system some time to propagate the TODS state
        vTaskDelay(MESH_HANDOVER_TODS_PROPAGATION);
    }

    // In any case, if we find proof of the configured wifi, we switch back to a dynamic mesh network
    // we keep the fixed one only if we are not able to reach any access point, meaning we are on an island.
    // In this case, even using the dynamic mesh network would not improve the shape.

    // Trigger free root selection
    if (!esp_mesh_get_self_organized()) {
        // Espressif:
        // Suggest use esp_mesh_get_self_organized() instead of esp_mesh_is_root_fixed()
        // Because esp_mesh_is_root_fixed() may be affected by MESH_IE change
        esp_wifi_scan_stop();

        mesh_netifs_stop();

        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mesh_set_type(MESH_IDLE));
        esp_mesh_fix_root(false);
        esp_mesh_set_router(&self->mesh_router);
        esp_mesh_set_self_organized(true, true);
    }
}
#endif

static void mesh_handover_store_scan_result(
    mesh_fixed_last_parent_t* out_result,
    wifi_ap_record_t* in_record,
    mesh_assoc_t* in_assoc)
{
    memcpy(&out_result->parent_record, in_record, sizeof(wifi_ap_record_t));
    memcpy(&out_result->parent_assoc, in_assoc, sizeof(mesh_assoc_t));
    if (in_assoc->layer_cap != 1) {
        out_result->type = MESH_NODE;
    } else {
        out_result->type = MESH_LEAF;
    }
    out_result->layer = in_assoc->layer + 1;
    out_result->found = true;
}

void mesh_handover_wifi_scan_done(int num)
{
#ifdef MESH_HANDLE_MODEM_HANDOVER
    int ie_len = 0;
    wifi_ap_record_t record;
    mesh_assoc_t assoc;
    if (!num) {
        // If we have no results we leave
        return;
    }

    // Reset last scan result
    self->last_scan_result.found = false;
    mesh_fixed_last_parent_t* last_result = &self->last_scan_result;
    mesh_fixed_last_parent_t best_result = {
        .parent_assoc.layer = CONFIG_MESH_MAX_LAYER,
        .parent_assoc.rssi = -120,
        .parent_record.rssi = -120,
        .type = MESH_IDLE,
        .layer = -1,
        .found = false
    };

    wifi_ap_record_t ap_rec = { 0 };
    if (esp_wifi_sta_get_ap_info(&ap_rec) == ESP_OK) {
        // If we are connected it means there is a parent with data. Start from it.
        memcpy(&best_result, &self->active_parent_result, sizeof(mesh_fixed_last_parent_t));
        // Update RSSI since it could have changed
        best_result.parent_record.rssi = ap_rec.rssi;
        // If the device disconnects from mesh the fixed_root_has_parent flag gets cleared
        // Since here we are connected, it means we reconnected, so we force it to true
        self->fixed_root_has_parent = true;
    }

    // Get mesh settings we have to check against
    static mesh_cfg_t cfg = { 0 };
    esp_mesh_get_config(&cfg);

    bool found_idles = false;

    for (int i = 0; i < num; i++) {
        // Get each single WiFi record in our cache
        esp_mesh_scan_get_ap_ie_len(&ie_len);
        esp_mesh_scan_get_ap_record(&record, &assoc);

        if (ie_len == sizeof(assoc)) {
            // This is a mesh station
            if (hardware_has_modem()) {
                // Devices with LTE scan only for WIFI stations
                continue;
            }
            if (memcmp(assoc.mesh_id, cfg.mesh_id.addr, MAC_ADDR_LEN)) {
                // This MESH device is not part of our network!
                continue;
            }
            if ((assoc.mesh_type == MESH_IDLE)
                || !assoc.layer_cap
                || (assoc.assoc >= assoc.assoc_cap)
                ) {

                if (assoc.mesh_type == MESH_IDLE) {
                    // Report there are idle stations for further processing
                    found_idles = true;
                    ESP_LOGW(TAG,
                        "<MESH>"MACSTR" is idle",
                        MAC2STR(record.bssid)
                    );
                }
                // Device not connected to a network
                // or not accepting connections
                continue;
            }

            // Check for best device!
            ESP_LOGW(TAG,
                "<MESH>"MACSTR", layer:%d/%d, assoc:%d/%d, %d, ch:%u",
                MAC2STR(record.bssid),
                assoc.layer, assoc.layer_cap,
                assoc.assoc, assoc.assoc_cap,
                assoc.layer2_cap,
                record.primary
            );

            ESP_LOGW(TAG,
                "[%d] rssi:%d, parent_rssi:%d root_rssi:%d Type %d ",
                i,
                record.rssi,
                assoc.rssi,
                assoc.rc_rssi,
                assoc.mesh_type
            );

            uint32_t update = 0;

#define RECORD_GOOD_RSSI (-78)

            // If possible always connect to the ROOT
            update = ((assoc.layer == 1) && (record.rssi > RECORD_GOOD_RSSI || self->allow_low_rssi_connections));
            if (!update) {
                // We are not checking the root or the root strength is low
                // Check for lower layer
                if ((assoc.layer < best_result.parent_assoc.layer) && (record.rssi > RECORD_GOOD_RSSI || self->allow_low_rssi_connections)) {
                    update = 2;
                }
            }
            if (!update) {
                // It's not a lower layer with good signal
                // Check for same layer with better parent RSSI
                if ((assoc.layer == best_result.parent_assoc.layer) && (assoc.rssi > best_result.parent_record.rssi)) {
                    update = 3;
                }
            }

            if (update) {
                ESP_LOGW(TAG, "Possible parent found[%d][%d] L%d %ddB",
                    update, self->allow_low_rssi_connections,
                    assoc.layer,
                    record.rssi
                );
                mesh_handover_store_scan_result(&best_result, &record, &assoc);
            }

        } else {
            // This is a normal WIFI access point
            ESP_LOGI(TAG, "[%d]%s, "MACSTR", channel:%u, rssi:%d", i,
                record.ssid, MAC2STR(record.bssid), record.primary,
                record.rssi);

            // Check if we found the AP and go back to WIFI based network
            if (!memcmp(self->mesh_router.ssid, record.ssid, self->mesh_router.ssid_len)) {
                mesh_handover_switch_to_dynamic();
                // Stop here
                break;
            }
        }
    }

    if (hardware_has_modem()) {
        return;
    }

    ESP_LOGW(TAG, "scan_done: found %d, fixed_root %d, has_parent %d", best_result.found, !esp_mesh_get_self_organized(), self->fixed_root_has_parent);

    if (!best_result.found) {
        if (found_idles) {
            ESP_LOGW(TAG, "Found IDLE devices and no good connection. Wait.");
            self->found_idle_devices_and_no_connection++;
            if (self->found_idle_devices_and_no_connection > MESH_MODEM_WIFI_PROBE_THRESHOLD) {
                ESP_LOGW(TAG, "Allow any RSSI.");
                self->allow_low_rssi_connections = true;
            }
        } else {
            ESP_LOGW(TAG, "All nodes have bad connection, allow any RSSI.");
            self->allow_low_rssi_connections = true;
        }
    }

    // If we found a parent to attach, we do it here, since only the best one was selected
    if (best_result.found && !esp_mesh_get_self_organized()) {

        if (self->fixed_root_has_parent && esp_mesh_get_type() == MESH_IDLE) {
            ESP_LOGW(TAG, "INVALID DEVICE TYPE WITH SET PARENT, recovering");
            self->fixed_root_has_parent = false;
        }

        if (self->fixed_root_has_parent) {

            // Check the currently connected parent, if the new option is consistently better, switch to the new option
            if (memcmp(last_result->parent_record.bssid, best_result.parent_record.bssid, MAC_ADDR_LEN)) {
                // This is a new parent
                self->last_scan_count++;
            } else {
                // Same parent
                self->last_scan_count = 0;
            }

            if (self->last_scan_count >= MESH_MODEM_WIFI_PROBE_THRESHOLD) {
                // The new parent was consistently selected as better than previous parent
                self->fixed_root_has_parent = false;
            }

            ESP_LOGW(TAG, "Old Parent "MACSTR" new "MACSTR" [%d]. %s",
                MAC2STR(last_result->parent_record.bssid),
                MAC2STR(best_result.parent_record.bssid),
                self->last_scan_count,
                self->fixed_root_has_parent ? "" : "Moving.");

            // Update previous result with current best parent
            memcpy(last_result, &best_result, sizeof(mesh_fixed_last_parent_t));
        }
        if (!self->fixed_root_has_parent) {
            /*
             * parent
             * Both channel and SSID of the parent are mandatory.
             */
            best_result.parent.sta.channel = best_result.parent_record.primary;
            memcpy(&best_result.parent.sta.ssid,
                &best_result.parent_record.ssid,
                sizeof(best_result.parent_record.ssid)
            );

            best_result.parent.sta.bssid_set = 1;
            memcpy(&best_result.parent.sta.bssid, best_result.parent_record.bssid, MAC_ADDR_LEN);

            ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(best_result.parent_record.authmode));

            memcpy(&best_result.parent.sta.password,
                cfg.mesh_ap.password,
                strlen((const char*)cfg.mesh_ap.password)
            );

            ESP_LOGW(TAG,
                "<NEWPARENT>"MACSTR", layer:%d/%d, assoc:%d/%d, %d, channel:%u, rssi:%d",
                MAC2STR(best_result.parent_record.bssid),
                best_result.parent_assoc.layer,
                best_result.parent_assoc.layer_cap,
                best_result.parent_assoc.assoc,
                best_result.parent_assoc.assoc_cap,
                best_result.parent_assoc.layer2_cap,
                best_result.parent_record.primary,
                best_result.parent_record.rssi);

            esp_err_t err;
            err = esp_mesh_set_parent(
                &best_result.parent,
                (mesh_addr_t*)&best_result.parent_assoc.mesh_id,
                best_result.type,
                best_result.layer
            );
            if (!err) {
                self->fixed_root_has_parent = true;
                self->last_scan_count = 0;

                // Store current parent data
                memcpy(&self->active_parent_result, &best_result, sizeof(mesh_fixed_last_parent_t));
                // Initialize last result with current parent
                memcpy(last_result, &best_result, sizeof(mesh_fixed_last_parent_t));
            } else {
                ESP_LOGW(TAG, "Set parent err 0x%X", err);
            }
        }
    }
#endif
}

void mesh_handover_no_parent_found(bool forced)
{
#ifdef MESH_HANDLE_MODEM_HANDOVER
    // Stop reconnection timer if any
    esp_timer_stop(self->timer_reconnect);

    if (!self->master_active || forced) {
        // Go directly to LTE
        ESP_LOGW(TAG, "Perform NO PARENT FIXED ROOT handover %d %d", self->master_active, forced);
        self->master_active = true;

        mesh_handover_disconnect_callback();
    }
#endif
}

void mesh_handover_parent_connected()
{
    if (self->connect_callback) {
        self->connect_callback(false);
    }

    if (!self->mesh_connected) {
        self->mesh_connected = true;
    }

    // Trigger delayed reconnection tasks
    if (self->master_active) {
        self->master_active = false;
#ifdef MESH_HANDLE_MODEM_HANDOVER
        ESP_LOGW(TAG, "Start checking DYNAMIC ROOT delayed tasks");

        // In case of fixed root network, we want to update the shape of the network periodically
        if (esp_mesh_get_self_organized()) {
            // Stop looking for WIFIs
            esp_timer_stop(self->timer_wifi_probe);
        }/* else {
            // this function is called when the device connects to a parent.
            // The fact that it found a parent to connect to does not mean that the device has locked the parent from wifi scanning..
            self->fixed_root_has_parent = true;
        }*/

        // We switch off modem PPP after a while to avoid sudden disconnections to impact us
        esp_timer_start_once(self->timer_reconnect, MESH_MODEM_RECONNECTED_THRESHOLD*1000);
#endif
    }
#ifdef MESH_HANDLE_MODEM_HANDOVER
    // Stop disconnection timer if any
    esp_timer_stop(self->timer_disconnect);
#endif
}

void mesh_handover_parent_disconnected(bool forced, bool dont_reconnect)
{
    if (self->disconnect_callback) {
        self->disconnect_callback(false);
    }
#ifdef MESH_HANDLE_MODEM_HANDOVER
    // Stop reconnection timer if any
    esp_timer_stop(self->timer_reconnect);

    if (!esp_mesh_get_self_organized() && dont_reconnect) {
        // When in fixed root, if we receive some strange error, we stop to retry until next scan
        ESP_LOGW(TAG, "Stop mesh auto-reconnect");
        esp_mesh_set_self_organized(false, false);
        self->fixed_root_has_parent = false;
    }
#endif
    if ((!self->master_active && self->mesh_connected) || forced) {
        // We stop once
        mesh_netifs_stop();
        // We are (probably) in the scenario where the master LTE device is ROOT.
        self->master_active = true;
        self->mesh_connected = false;
#ifdef MESH_HANDLE_MODEM_HANDOVER
        ESP_LOGW(TAG, "Start checking for FIXED ROOT handover");

        // The scenario will become definitive if we won't have any reconnection before the timer expires
        esp_timer_start_once(self->timer_disconnect, MESH_MODEM_DISCONNECTED_THRESHOLD*1000);

        // Reset the parent state, this function is called each time the device disconnects
        self->fixed_root_has_parent = false;
        if (forced) {
            self->lte_forced_handover = true;
        } else {
            self->lte_forced_handover = false;
        }
#endif
    }
}

void mesh_handover_root_fixed(bool fixed)
{
#ifdef MESH_HANDLE_MODEM_HANDOVER
    if (fixed) {
        //ensure all devices can change to fixed-root network ASAP
        self->master_active = true;
        mesh_handover_disconnect_callback();
    } else if (!fixed && !hardware_has_modem()) {
        mesh_handover_switch_to_dynamic();
    }
#endif
}

void mesh_handover_init(mesh_router_t* router,
    timer_callback_t callback_disconnect,
    timer_callback_t callback_reconnect,
    timer_callback_t callback_wifi_probe
)
{
    if (self->init) {
        ESP_LOGW(TAG, "Init called without stop");
        return;
    }

    // Init settings, do not do a memset because the mesh callbacks are set once
    self->master_active = false;
    self->mesh_connected = false;
#ifdef MESH_HANDLE_MODEM_HANDOVER
    // Continue reset
    self->timer_disconnect = NULL;
    self->timer_reconnect = NULL;
    self->timer_wifi_probe = NULL;
    self->timer_wifi_probe_root_count = 0;
    self->fixed_root_has_parent = false;
    self->lte_forced_handover = false;

    // Set default values to scan results
    self->last_scan_result.parent_assoc.layer = CONFIG_MESH_MAX_LAYER;
    self->last_scan_result.parent_assoc.rssi = -120;
    self->last_scan_result.parent_record.rssi = -120;
    self->last_scan_result.type = MESH_IDLE;
    self->last_scan_result.layer = -1;
    self->last_scan_result.found = false;

    // Reset any active fixed root parent
    memset(&self->active_parent_result, 0, sizeof(mesh_fixed_last_parent_t));

    // Store the router info for hybrid fixed root meshing
    memcpy(&self->mesh_router, router, sizeof(mesh_router_t));

    const esp_timer_create_args_t args = {
        .callback = callback_disconnect,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "dc"
    };

    esp_timer_create(
        &args,
        &self->timer_disconnect
    );

    const esp_timer_create_args_t args2 = {
        .callback = callback_reconnect,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "rc"
    };

    esp_timer_create(
        &args2,
        &self->timer_reconnect
    );

    const esp_timer_create_args_t args3 = {
        .callback = callback_wifi_probe,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pr"
    };

    esp_timer_create(
        &args3,
        &self->timer_wifi_probe
    );
#endif
    self->init = true;
}

void mesh_handover_set_callbacks(
    mesh_connect_cb_t connect_cb,
    mesh_disconnect_cb_t disconnect_cb
)
{
    self->connect_callback = connect_cb;
    self->disconnect_callback = disconnect_cb;
}

void mesh_handover_check_lte_only_mode(void)
{
#ifdef MESH_HANDLE_MODEM_HANDOVER
    return;
    //if (mesh_is_lte_dummy_credentials((const char*)&self->mesh_router.ssid, (const char*)&self->mesh_router.password)) {
    //    ESP_LOGI(TAG, "LTE-only mode configured -> directly connect to LTE");
    //    mesh_handover_disconnect_callback();
    //}
#endif
}

void mesh_handover_stop()
{
    if (!self->init) {
        ESP_LOGW(TAG, "Stop called before init");
        return;
    }

#ifdef MESH_HANDLE_MODEM_HANDOVER
    // Stop any ongoing timer
    esp_timer_stop(self->timer_disconnect);
    esp_timer_stop(self->timer_reconnect);
    esp_timer_stop(self->timer_wifi_probe);

    // Clear timers
    esp_timer_delete(self->timer_disconnect);
    esp_timer_delete(self->timer_reconnect);
    esp_timer_delete(self->timer_wifi_probe);
#endif

    self->init = false;
}

bool mesh_handover_is_transitioning()
{
    return
        esp_timer_is_active(self->timer_disconnect) ||
        esp_timer_is_active(self->timer_reconnect);
}

int mesh_handover_get_stats(mesh_addr_t* parent)
{
    if (parent != NULL) {
        memcpy(parent, self->active_parent_result.parent_assoc.mesh_id, MAC_ADDR_LEN);
    }
    if (hardware_has_modem()) {
        // LTE devices are always ROOT in fixed root
        return 1;
    }
    return self->active_parent_result.layer;
}
