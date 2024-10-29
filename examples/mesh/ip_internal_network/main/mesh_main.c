/* Mesh Internal Communication Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <inttypes.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "nvs_flash.h"
#include "mesh_netif.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include <ping/ping_sock.h>
#include "mesh_handover.h"

/*******************************************************
 *                Macros
 *******************************************************/
#define EXAMPLE_BUTTON_GPIO     0

// commands for internal mesh communication:
// <CMD> <PAYLOAD>, where CMD is one character, payload is variable dep. on command
#define CMD_KEYPRESSED 0x55
// CMD_KEYPRESSED: payload is always 6 bytes identifying address of node sending keypress event
#define CMD_ROUTE_TABLE 0x56
// CMD_KEYPRESSED: payload is a multiple of 6 listing addresses in a routing table
/*******************************************************
 *                Constants
 *******************************************************/
static const char *MESH_TAG = "mesh_main";
static const char *TAG = "ping";
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x76};


#define MACSTR_COMPACT "%02X%02X%02X%02X%02X%02X"
#define MESH_THRESHOLD_3    (3)
#define MESH_THRESHOLD_10   (10)
#define array_size(x)	(sizeof(x) / sizeof(x[0]))


/*******************************************************
 *                Variable Definitions
 *******************************************************/
static bool is_running = true;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;
static esp_ip4_addr_t s_current_ip;
static mesh_addr_t s_route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
static int s_route_table_size = 0;
static SemaphoreHandle_t s_route_table_lock = NULL;
static uint8_t s_mesh_tx_payload[CONFIG_MESH_ROUTE_TABLE_SIZE*6+1];

typedef struct config_mesh_ {
    int max_layer;          // CONFIG_MESH_MAX_LAYER
    float vote_percentage;  //
    int ap_assoc_expire;    //
    uint8_t ap_auth_mode;
    // Mesh AP  settings
    uint8_t ap_max_connections;
    uint8_t ap_nonmesh_max_connections;
} config_mesh_t;

typedef struct mesh_child_ {
    int8_t connection_count;
    uint32_t mac;
} mesh_child_t;

typedef struct mesh_self_ {
    int mesh_layer;
    mesh_addr_t mesh_parent_addr;
    bool is_initialized;
    config_mesh_t config_mesh;
    bool had_internet;
    bool has_internet;

    bool task_started;

    uint32_t counter_no_parent;
    uint32_t wifi_empty_scan;

    mesh_child_t children[CONFIG_MESH_AP_CONNECTIONS];
    mesh_reset_cb_t reset_callback;
} mesh_self_t;

static mesh_self_t priv_self = {
    .mesh_layer = -1,
        .config_mesh.max_layer = CONFIG_MESH_MAX_LAYER,
        .config_mesh.vote_percentage = 1,
        .config_mesh.ap_assoc_expire = 10,
        .config_mesh.ap_auth_mode = CONFIG_MESH_AP_AUTHMODE,
        .config_mesh.ap_max_connections = CONFIG_MESH_AP_CONNECTIONS,
        .config_mesh.ap_nonmesh_max_connections = CONFIG_MESH_NON_MESH_AP_CONNECTIONS,
};
static mesh_self_t* self = &priv_self;




/*******************************************************
 *                Function Declarations
 *******************************************************/
// interaction with public mqtt broker
void mqtt_app_start(void);
void mqtt_app_publish(char* topic, char *publish_string);

/*******************************************************
 *                Function Definitions
 *******************************************************/


/** 'tasks' command prints the list of tasks and related information */
#define WITH_TASKS_INFO 1
#if WITH_TASKS_INFO

static int tasks_info()
{
    const size_t bytes_per_task = 40; /* see vTaskList description */
    char *task_list_buffer = malloc(uxTaskGetNumberOfTasks() * bytes_per_task);
    if (task_list_buffer == NULL) {
        ESP_LOGE(TAG, "failed to allocate buffer for vTaskList output");
        return 1;
    }
    ESP_LOGW(TAG, "Task Name\tStatus\tPrio\tHWM\tTask#\tAffinity");
    vTaskList(task_list_buffer);
    ESP_LOGW(TAG, "%s", task_list_buffer);
    free(task_list_buffer);
    return 0;
}

/*static void register_tasks(void)
{
    const esp_console_cmd_t cmd = {
    .command = "tasks",
    .help = "Get information about running tasks",
    .hint = NULL,
    .func = &tasks_info,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}*/

#endif // WITH_TASKS_INFO



static void initialise_button(void)
{
    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = BIT64(EXAMPLE_BUTTON_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
}

void static recv_cb(mesh_addr_t *from, mesh_data_t *data)
{
    if (data->data[0] == CMD_ROUTE_TABLE) {
        int size =  data->size - 1;
        if (s_route_table_lock == NULL || size%6 != 0) {
            ESP_LOGE(MESH_TAG, "CMD_ROUTE_TABLE: Unexpected size %d", size);
            return;
        }
        xSemaphoreTake(s_route_table_lock, portMAX_DELAY);
        s_route_table_size = size / 6;
        for (int i=0; i < s_route_table_size; ++i) {
            ESP_LOGD(MESH_TAG, "Received Routing table [%d] "
                    MACSTR, i, MAC2STR(data->data + 6*i + 1));
        }
        memcpy(&s_route_table, data->data + 1, size);
        xSemaphoreGive(s_route_table_lock);
    } else if (data->data[0] == CMD_KEYPRESSED) {
        if (data->size != 7) {
            ESP_LOGE(MESH_TAG, "CMD_KEYPRESSED: Unexpected size %d", data->size);
            return;
        }
        ESP_LOGW(MESH_TAG, "Keypressed detected on node: "
                MACSTR, MAC2STR(data->data + 1));
    } else {
        ESP_LOGE(MESH_TAG, "Error in receiving raw mesh data: Unknown command 0x%02X", data->data[0]);
    }
}

static void check_button(void* args)
{
    static bool old_level = true;
    bool new_level;
    bool run_check_button = true;
    initialise_button();
    while (run_check_button) {
        new_level = gpio_get_level(EXAMPLE_BUTTON_GPIO);
        if (!new_level && old_level) {
            if (s_route_table_size && !esp_mesh_is_root()) {
                ESP_LOGW(MESH_TAG, "Key pressed!");
                mesh_data_t data;
                uint8_t *my_mac = mesh_netif_get_station_mac();
                uint8_t data_to_send[6+1] = { CMD_KEYPRESSED, };
                esp_err_t err;
                char print[6*3+1]; // MAC addr size + terminator
                memcpy(data_to_send + 1, my_mac, 6);
                data.size = 7;
                data.proto = MESH_PROTO_BIN;
                data.tos = MESH_TOS_P2P;
                data.data = data_to_send;
                snprintf(print, sizeof(print),MACSTR, MAC2STR(my_mac));
                mqtt_app_publish("/topic/ip_mesh/key_pressed", print);
                xSemaphoreTake(s_route_table_lock, portMAX_DELAY);
                for (int i = 0; i < s_route_table_size; i++) {
                    if (MAC_ADDR_EQUAL(s_route_table[i].addr, my_mac)) {
                        continue;
                    }
                    err = esp_mesh_send(&s_route_table[i], &data, MESH_DATA_P2P, NULL, 0);
                    ESP_LOGI(MESH_TAG, "Sending to [%d] "
                            MACSTR ": sent with err code: %d", i, MAC2STR(s_route_table[i].addr), err);
                }
                xSemaphoreGive(s_route_table_lock);
            }
        }
        old_level = new_level;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

}


void esp_mesh_mqtt_task(void *arg)
{
    is_running = true;
    char *print;
    mesh_data_t data;
    esp_err_t err;
    mqtt_app_start();
    while (is_running) {
        asprintf(&print, "layer:%d IP:" IPSTR, esp_mesh_get_layer(), IP2STR(&s_current_ip));
        ESP_LOGI(MESH_TAG, "Tried to publish %s", print);
        mqtt_app_publish("/topic/ip_mesh", print);
        free(print);
        if (esp_mesh_is_root()) {
            esp_mesh_get_routing_table((mesh_addr_t *) &s_route_table,
                                       CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &s_route_table_size);
            data.size = s_route_table_size * 6 + 1;
            data.proto = MESH_PROTO_BIN;
            data.tos = MESH_TOS_P2P;
            s_mesh_tx_payload[0] = CMD_ROUTE_TABLE;
            memcpy(s_mesh_tx_payload + 1, s_route_table, s_route_table_size*6);
            data.data = s_mesh_tx_payload;
            for (int i = 0; i < s_route_table_size; i++) {
                err = esp_mesh_send(&s_route_table[i], &data, MESH_DATA_P2P, NULL, 0);
                ESP_LOGD(MESH_TAG, "Sending routing table to [%d] "
                        MACSTR ": sent with err code: %d", i, MAC2STR(s_route_table[i].addr), err);
            }
        }
        vTaskDelay(2 * 1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

esp_err_t esp_mesh_comm_mqtt_task_start(void)
{
    static bool is_comm_mqtt_task_started = false;

    if (!is_comm_mqtt_task_started) {
        xTaskCreate(esp_mesh_mqtt_task, "mqtt task", 3072, NULL, 5, NULL);
        xTaskCreate(check_button, "check button task", 3072, NULL, 5, NULL);
        is_comm_mqtt_task_started = true;
    }
    return ESP_OK;
}


static void mesh_track_child_connect(mesh_event_child_connected_t* child_connected)
{
    // Keep track of connected children
    bool found = false;
    int8_t firstfreeslot = -1;
    uint32_t child_mac = ((child_connected->mac[2] << 24) | (child_connected->mac[3] << 16) | (child_connected->mac[4] << 8) | (child_connected->mac[5]));

    for (int i = 0; i < CONFIG_MESH_AP_CONNECTIONS; ++i) {
        if (self->children[i].mac == child_mac) {
            found = true;
            self->children[i].connection_count++;
            break;
        } else if (!self->children[i].mac && firstfreeslot < 0) {
            firstfreeslot = i;
        }
    }
    if (!found) {
        // Add
        if (firstfreeslot >= 0) {
            self->children[firstfreeslot].mac = child_mac;
            self->children[firstfreeslot].connection_count = 1;
        } else {
            ESP_LOGE(TAG, "Child connection with no free slots! "MACSTR_COMPACT, MAC2STR(child_connected->mac));
            for (int i = 0; i < CONFIG_MESH_AP_CONNECTIONS; ++i) {
                ESP_LOGW(TAG, "%04lX %d", self->children[i].mac, self->children[i].connection_count);
            }
        }
    }
}

static void mesh_track_child_disconnect(mesh_event_child_disconnected_t* child_disconnected)
{
    // We keep track of connected children and check for strange behavior
    bool found = false;
    int8_t firstfreeslot = -1;
    uint32_t child_mac = ((child_disconnected->mac[2] << 24) | (child_disconnected->mac[3] << 16) | (child_disconnected->mac[4] << 8) | (child_disconnected->mac[5]));

    for (int i = 0; i < CONFIG_MESH_AP_CONNECTIONS; ++i) {
        if (self->children[i].mac == child_mac) {
            found = true;
            self->children[i].connection_count--;
            if (!self->children[i].connection_count) {
                // Remove from list
                self->children[i].mac = 0;
            } else if (self->children[i].connection_count < -2) {
                // We saw this happening when the ROOT stops accepting children for no reason.
                // A reboot solved the issue -> workaround
                ESP_LOGE(TAG, "Negative children count! Reset mesh...");

                // The mesh task cannot reset itself. Someone else has to do it.
                self->reset_callback();
                // Clean the state of the children to allow time for reset
                memset(self->children, 0, sizeof(mesh_child_t) * CONFIG_MESH_AP_CONNECTIONS);
            }
            break;
        } else if (!self->children[i].mac && firstfreeslot < 0) {
            firstfreeslot = i;
        }
    }
    if (!found) {
        ESP_LOGW(TAG, "Disconnection from non registered child! "MACSTR_COMPACT, MAC2STR(child_disconnected->mac));
        if (firstfreeslot >= 0) {
            self->children[firstfreeslot].mac = child_mac;
            self->children[firstfreeslot].connection_count = -1;
        }
    }
}


// Run from esp_events
static void callback_handover_disconnect(void* args)
{
    mesh_handover_disconnect_callback();
}

// Run from esp_events
static void callback_handover_reconnect(void* args)
{
    mesh_handover_reconnect_callback();
}

// Run from esp_events
static void callback_handover_wifi_probe(void* args)
{
    mesh_handover_wifi_probe_callback();
}

static bool mesh_check_forced_handover(int reason)
{
    typedef struct {
        int reason;         // wifi_err_reason_t or mesh_disconnect_reason_t
        uint32_t counter;
        uint32_t reset_at;
    } error_map_entry_t;
    static error_map_entry_t error_mapping[] = {
        {.reason = WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT, .counter = 0, .reset_at = MESH_THRESHOLD_10 },
        {.reason = WIFI_REASON_AUTH_EXPIRE,            .counter = 0, .reset_at = MESH_THRESHOLD_10 },
        {.reason = WIFI_REASON_CONNECTION_FAIL,        .counter = 0, .reset_at = MESH_THRESHOLD_10 },
        {.reason = MESH_REASON_PARENT_IDLE,            .counter = 0, .reset_at = MESH_THRESHOLD_10 },
    };

    // When the device has the correct SSID but for some reason
    // cannot connect with the given password (e.g. due to changed
    // settings in the access point) we get this error. Also,
    // the device seems not to connect to the mesh if it is in this
    // state, so it isolates itself. By explicly forcing the fixed
    // root handover the device might recover.

    bool force = false;

    if (mesh_handover_is_transitioning()) {
        // In case we already triggered a forced handover we ignore counters
        ESP_LOGW(TAG, "Ignore counters while transitioning. error %d", reason);
        return force;
    }

    for (int i = 0; i < array_size(error_mapping); i++) {
        error_map_entry_t* entry = &error_mapping[i];
        if (reason == entry->reason) {
            entry->counter++;

            ESP_LOGW(TAG, "Counting wifi error %d, new count %ld", reason, entry->counter);

            if (entry->counter >= entry->reset_at) {
                ESP_LOGW(TAG, "Counter exceeds max value, force transition");
                force = true;
            }

            break;
        }
    }

    if (force) {
        for (int i = 0; i < array_size(error_mapping); i++) {
            error_map_entry_t* entry = &error_mapping[i];
            entry->counter = 0;
        }
    }

    return force;
}

void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint8_t last_layer = 0;

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_MESH_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
                 child_connected->aid,
                 MAC2STR(child_connected->mac));

        mesh_track_child_connect(child_connected);

    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 child_disconnected->aid,
                 MAC2STR(child_disconnected->mac));

        mesh_track_child_disconnect(child_disconnected);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new);
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);

        // This counter gets normally reset by is_rootless being called right after
        // If this does not happen it means we are switching between LTE and WIFI without
        // managing to connect to the WIFI -> force a fixed handover to continue on LTE
        self->counter_no_parent++;
        bool force = self->counter_no_parent > MESH_THRESHOLD_3;
        if (force) {
            self->counter_no_parent = 0;
        }
        mesh_handover_no_parent_found(force);
    }
    /* TODO handler for the failure */
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        memcpy(&mesh_parent_addr.addr, connected->connected.bssid, 6);
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:"MACSTR"%s, ID:"MACSTR"",
                 last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr));
        last_layer = mesh_layer;
        mesh_netifs_start(esp_mesh_is_root());


        mesh_handover_parent_connected();

    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d %s",
                 disconnected->reason,
                 esp_err_to_name(disconnected->reason));

        mesh_layer = esp_mesh_get_layer();

        bool force = mesh_check_forced_handover(disconnected->reason);

        bool dont_reconnect = (disconnected->reason == MESH_REASON_CYCLIC) ||
            (disconnected->reason == MESH_REASON_PARENT_IDLE) ||
            (disconnected->reason == WIFI_REASON_ASSOC_TOOMANY) ||
            (disconnected->reason == MESH_REASON_SCAN_FAIL);

        mesh_handover_parent_disconnected(force, dont_reconnect);

    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
                 MAC2STR(root_addr->addr));
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_VOTE_STOPPED>");
        break;
    }
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(mesh_parent_addr.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_TODS_REACHABLE>state:%d", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");



        mesh_handover_root_fixed(root_fixed->is_fixed);
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_SCAN_DONE>number:%d",
                 scan_done->number);

        mesh_handover_wifi_scan_done(scan_done->number);

        if (!scan_done->number) {
            self->wifi_empty_scan++;
            if (self->wifi_empty_scan > MESH_THRESHOLD_10) {
                ESP_LOGE(TAG, "Wifi cannot scan. Reset.");
                esp_restart();
            }
        } else {
            self->wifi_empty_scan = 0;
        }
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
                 network_state->is_rootless);
        if (network_state->is_rootless) {
                // This happens after a MESH_EVENT_NO_PARENT_FOUND unless we are stuck
                self->counter_no_parent = 0;
        }
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:"MACSTR"",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, "MACSTR"",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
    }
    break;
    default:
        ESP_LOGI(MESH_TAG, "unknown id:%" PRId32 "", event_id);
        break;
    }
}

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));
    s_current_ip.addr = event->ip_info.ip.addr;
#if !CONFIG_MESH_USE_GLOBAL_DNS_IP
    esp_netif_t *netif = event->esp_netif;
    esp_netif_dns_info_t dns;
    ESP_ERROR_CHECK(esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns));
    mesh_netif_start_root_ap(esp_mesh_is_root(), dns.ip.u_addr.ip4.addr);
#endif
    esp_mesh_comm_mqtt_task_start();
}

static void test_on_ping_success(esp_ping_handle_t hdl, void *args)
{
    // optionally, get callback arguments
    // const char* str = (const char*) args;
    // printf("%s\r\n", str); // "foo"
    uint8_t ttl;
    uint16_t seqno;
    uint32_t elapsed_time, recv_len;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recv_len, sizeof(recv_len));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
    ESP_LOGW(TAG, "%ld bytes from %s icmp_seq=%d ttl=%d time=%ld ms",
           recv_len, ipaddr_ntoa(&target_addr), seqno, ttl, elapsed_time);
}

static void test_on_ping_timeout(esp_ping_handle_t hdl, void *args)
{
    uint16_t seqno;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    ESP_LOGW(TAG, "From %s icmp_seq=%d timeout", ipaddr_ntoa(&target_addr), seqno);
}

static void test_on_ping_end(esp_ping_handle_t hdl, void *args)
{
    uint32_t transmitted;
    uint32_t received;
    uint32_t total_time_ms;

    esp_ping_get_profile(hdl, ESP_PING_PROF_REQUEST, &transmitted, sizeof(transmitted));
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &received, sizeof(received));
    esp_ping_get_profile(hdl, ESP_PING_PROF_DURATION, &total_time_ms, sizeof(total_time_ms));
    ESP_LOGW(TAG, "%ld packets transmitted, %ld received, time %ldms", transmitted, received, total_time_ms);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    /*  tcpip initialization */
    ESP_ERROR_CHECK(esp_netif_init());
    /*  event initialization */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /*  crete network interfaces for mesh (only station instance saved for further manipulation, soft AP instance ignored */
    ESP_ERROR_CHECK(mesh_netifs_init(recv_cb));

    esp_log_level_set("mesh", ESP_LOG_DEBUG);


    /*  wifi initialization */
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_wifi_start());
#define WIFI_POWER_2DB  (8)
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(WIFI_POWER_2DB));

    /*  mesh initialization */
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));
    /* set blocking time of esp_mesh_send() to 30s, to prevent the esp_mesh_send() from permanently for some reason */
    ESP_ERROR_CHECK(esp_mesh_send_block_time(30000));
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
#if !MESH_IE_ENCRYPTED
    cfg.crypto_funcs = NULL;
#endif
    /* mesh ID */
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);
    /* router */
    cfg.channel = CONFIG_MESH_CHANNEL;
    cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
    memcpy((uint8_t *) &cfg.router.ssid, CONFIG_MESH_ROUTER_SSID, cfg.router.ssid_len);
    memcpy((uint8_t *) &cfg.router.password, CONFIG_MESH_ROUTER_PASSWD,
           strlen(CONFIG_MESH_ROUTER_PASSWD));
    /* mesh softAP */
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,
           strlen(CONFIG_MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    ESP_ERROR_CHECK(esp_mesh_allow_root_conflicts(false));
    ESP_ERROR_CHECK(esp_mesh_send_block_time(30000));

    mesh_handover_init(&cfg.router,
        callback_handover_disconnect,
        callback_handover_reconnect,
        callback_handover_wifi_probe
    );

    /* mesh start */
    ESP_ERROR_CHECK(esp_mesh_start());
    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%" PRId32 ", %s",  esp_get_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed");

    s_route_table_lock = xSemaphoreCreateMutex();


    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    IP_ADDR4(&ping_config.target_addr, 8,8,8,8);          // target IP address
    ping_config.count = ESP_PING_COUNT_INFINITE;    // ping in infinite mode, esp_ping_stop can stop it
    ping_config.timeout_ms = 3000;

    /* set callback functions */
    esp_ping_callbacks_t cbs;
    cbs.on_ping_success = test_on_ping_success;
    cbs.on_ping_timeout = test_on_ping_timeout;
    cbs.on_ping_end = test_on_ping_end;
    cbs.cb_args = "foo";  // arguments that will feed to all callback functions, can be NULL

    esp_ping_handle_t ping;
    esp_ping_new_session(&ping_config, &cbs, &ping);
    esp_ping_start(ping);

#define DUMP_INTERVAL_MS (1000*60*2 / portTICK_PERIOD_MS)
#define DUMP_TASK_INTERVAL_MS (1000*60 / portTICK_PERIOD_MS)
    //int64_t last_dump = 0;
    //int64_t task_dump = 0;
    /*while (true) {
        if((xTaskGetTickCount() - last_dump) >= DUMP_INTERVAL_MS) {
            last_dump = xTaskGetTickCount();
            esp_wifi_statis_dump(0xFFFF);
        }
        if((xTaskGetTickCount() - task_dump) >= DUMP_TASK_INTERVAL_MS) {
            task_dump = xTaskGetTickCount();
            tasks_info();
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }*/
}
