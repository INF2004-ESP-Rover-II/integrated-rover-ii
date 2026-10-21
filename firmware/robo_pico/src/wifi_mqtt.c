#include "wifi_mqtt.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/ip4_addr.h"

// MQTT configuration
// Increased to allow LIDAR + command messages to coexist without blocking
#define MQTT_VAR_HEADER_BUFFER_LEN 128
#define MQTT_REQ_MAX_IN_FLIGHT 2

// =================== PRIVATE STATE ===================
static mqtt_client_t *mqtt_client = NULL;
static ip_addr_t mqtt_broker_ip;
static bool wifi_initialized = false;
static bool wifi_connected_flag = false;
static bool mqtt_connected_flag = false;
static mqtt_command_callback_t command_callback = NULL;

// Buffer for incoming MQTT messages
typedef enum {
    IN_TOPIC_OTHER = 0,
    IN_TOPIC_COMMAND,
    IN_TOPIC_HEARTBEAT
} in_topic_t;
static in_topic_t current_in_topic = IN_TOPIC_OTHER;

static char message_buffer[256];
static size_t buffer_pos = 0;

// Time of the last message from the laptop (command or heartbeat).
// 0 = nothing received yet. Read by main.c for the FR-08 link timeout.
static volatile uint32_t last_link_rx_ms = 0;

// =================== MQTT CALLBACKS ==================

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    printf("[MQTT] ===== CONNECTION CALLBACK FIRED =====\n");
    printf("[MQTT] Status code: %d\n", status);
    
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Connected to broker successfully!\n");
        mqtt_connected_flag = true;
    } else if (status == MQTT_CONNECT_REFUSED_PROTOCOL_VERSION) {
        printf("[MQTT] Connection refused: protocol version\n");
        mqtt_connected_flag = false;
    } else if (status == MQTT_CONNECT_REFUSED_IDENTIFIER) {
        printf("[MQTT] Connection refused: identifier rejected\n");
        mqtt_connected_flag = false;
    } else if (status == MQTT_CONNECT_REFUSED_SERVER) {
        printf("[MQTT] Connection refused: server unavailable\n");
        mqtt_connected_flag = false;
    } else if (status == MQTT_CONNECT_REFUSED_USERNAME_PASS) {
        printf("[MQTT] Connection refused: bad username/password\n");
        mqtt_connected_flag = false;
    } else if (status == MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_) {
        printf("[MQTT] Connection refused: not authorized\n");
        mqtt_connected_flag = false;
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        printf("[MQTT] Disconnected from broker\n");
        mqtt_connected_flag = false;
    } else if (status == MQTT_CONNECT_TIMEOUT) {
        printf("[MQTT] Connection timeout!\n");
        mqtt_connected_flag = false;
    } else {
        printf("[MQTT] Unknown status: %d\n", status);
        mqtt_connected_flag = false;
    }
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    // Copy incoming data to buffer
    if (buffer_pos + len < sizeof(message_buffer)) {
        memcpy(message_buffer + buffer_pos, data, len);
        buffer_pos += len;
    }

    // Check if this is the last fragment
    if (flags & MQTT_DATA_FLAG_LAST) {
        message_buffer[buffer_pos] = '\0';
        
        // 
        switch (current_in_topic) {
            case IN_TOPIC_COMMAND:
                last_link_rx_ms = to_ms_since_boot(get_absolute_time());
                if (command_callback != NULL) {
                    printf("[MQTT] Received command: %s\n", message_buffer);
                    command_callback(message_buffer);
                }
                break;
            case IN_TOPIC_HEARTBEAT:
                // Payload is just a counter; arriving at all is what counts.
                // Not printed: it comes twice a second.
                last_link_rx_ms = to_ms_since_boot(get_absolute_time());
                break;
            default:
                // A topic we did not ask for: never treat it as a command
                printf("[MQTT] Ignoring message on unexpected topic\n");
                break;
        }
        
        // Reset buffer
        buffer_pos = 0;
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    // New message starts: work out where it should go and reset the buffer
    if (strcmp(topic, MQTT_TOPIC_COMMAND) == 0) {
        current_in_topic = IN_TOPIC_COMMAND;
    } else if (strcmp(topic, MQTT_TOPIC_HEARTBEAT) == 0) {
        current_in_topic = IN_TOPIC_HEARTBEAT;
    } else {
        current_in_topic = IN_TOPIC_OTHER;
    }
    buffer_pos = 0;

    if (current_in_topic != IN_TOPIC_HEARTBEAT) {
        printf("[MQTT] Incoming publish on topic: %s (length: %lu)\n", topic, tot_len);
    }
}

static void mqtt_subscribe_cb(void *arg, err_t result) {
    if (result == ERR_OK) {
        printf("[MQTT] Successfully subscribed to command topic\n");
    } else {
        printf("[MQTT] Subscribe failed (error: %d)\n", result);
    }
}

static void mqtt_publish_cb(void *arg, err_t result) {
    if (result != ERR_OK) {
        printf("[MQTT] Publish failed (error: %d)\n", result);
    }
}

// ================= PUBLIC FUNCTIONS ==================

wifi_mqtt_error_t wifi_init(void) {
    if (wifi_initialized) {
        return WIFI_OK;
    }

    if (cyw43_arch_init()) {
        printf("[WIFI] Initialization failed\n");
        return WIFI_ERR_INIT_FAILED;
    }

    cyw43_arch_enable_sta_mode();
    wifi_initialized = true;
    
    printf("[WIFI] Initialized\n");
    return WIFI_OK;
}

wifi_mqtt_error_t wifi_connect(const char *ssid, const char *password, uint32_t timeout_ms) {
    if (!wifi_initialized) {
        return WIFI_ERR_INIT_FAILED;
    }

    printf("[WIFI] Connecting to '%s'...\n", ssid);

    int result = cyw43_arch_wifi_connect_timeout_ms(
        ssid, 
        password,
        CYW43_AUTH_WPA2_AES_PSK, 
        timeout_ms
    );

    if (result != 0) {
        printf("[WIFI] Connection failed (code: %d)\n", result);
        wifi_connected_flag = false;
        return (result == -2) ? WIFI_ERR_AUTH : 
               (result == -3) ? WIFI_ERR_TIMEOUT : 
               WIFI_ERR_NOT_CONNECTED;
    }

    wifi_connected_flag = true;
    
    extern struct netif *netif_list;
    if (netif_list) {
        printf("[WIFI] Connected! IP: %s\n", 
               ip4addr_ntoa(netif_ip4_addr(netif_list)));
    }

    return WIFI_OK;
}

bool wifi_is_connected(void) {
    return wifi_connected_flag;
}

const char* wifi_get_ip_address(void) {
    if (!wifi_connected_flag) {
        return NULL;
    }

    extern struct netif *netif_list;
    if (netif_list) {
        return ip4addr_ntoa(netif_ip4_addr(netif_list));
    }

    return NULL;
}

wifi_mqtt_error_t mqtt_init(const char *broker_ip, uint16_t broker_port, const char *client_id) {
    // Parse broker IP address
    if (!ip4addr_aton(broker_ip, &mqtt_broker_ip)) {
        printf("[MQTT] Invalid broker IP: %s\n", broker_ip);
        return MQTT_ERR_CONNECT_FAILED;
    }

    // Create MQTT client
    mqtt_client = mqtt_client_new();
    if (mqtt_client == NULL) {
        printf("[MQTT] Failed to create client\n");
        return MQTT_ERR_CONNECT_FAILED;
    }

    printf("[MQTT] Initialized (broker: %s:%d)\n", broker_ip, broker_port);
    return WIFI_OK;
}

wifi_mqtt_error_t mqtt_connect(void) {
    if (!wifi_connected_flag) {
        printf("[MQTT] ERROR: WiFi not connected\n");
        return WIFI_ERR_NOT_CONNECTED;
    }

    if (mqtt_client == NULL) {
        printf("[MQTT] ERROR: Client is NULL\n");
        return MQTT_ERR_CONNECT_FAILED;
    }

    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = MQTT_CLIENT_ID;
    ci.keep_alive = 60;

    printf("[MQTT] ===== MQTT CONNECTION DEBUG =====\n");
    printf("[MQTT] Client ID: %s\n", MQTT_CLIENT_ID);
    printf("[MQTT] Broker IP: %s\n", ip4addr_ntoa(&mqtt_broker_ip));
    printf("[MQTT] Broker Port: %d\n", MQTT_BROKER_PORT);
    printf("[MQTT] Keep-alive: %d\n", ci.keep_alive);
    printf("[MQTT] Client pointer: %p\n", (void*)mqtt_client);
    printf("[MQTT] Calling mqtt_client_connect()...\n");
    
    err_t err = mqtt_client_connect(
        mqtt_client,
        &mqtt_broker_ip,
        MQTT_BROKER_PORT,
        mqtt_connection_cb,
        NULL,
        &ci
    );

    printf("[MQTT] mqtt_client_connect() returned: %d\n", (int)err);
    
    if (err == ERR_OK) {
        printf("[MQTT] Connection initiated successfully\n");
    } else if (err == ERR_VAL) {
        printf("[MQTT] ERROR: Invalid value (ERR_VAL = -6)\n");
    } else if (err == ERR_MEM) {
        printf("[MQTT] ERROR: Out of memory (ERR_MEM = -1)\n");
    } else if (err == ERR_ISCONN) {
        printf("[MQTT] ERROR: Already connected (ERR_ISCONN = -8)\n");
    } else {
        printf("[MQTT] ERROR: Unknown error code %d\n", (int)err);
    }

    return (err == ERR_OK) ? WIFI_OK : MQTT_ERR_CONNECT_FAILED;
}

wifi_mqtt_error_t mqtt_subscribe_commands(mqtt_command_callback_t callback) {
    if (!mqtt_connected_flag || mqtt_client == NULL) {
        return MQTT_ERR_CONNECT_FAILED;
    }

    command_callback = callback;

    // Set incoming callbacks
    mqtt_set_inpub_callback(
        mqtt_client,
        mqtt_incoming_publish_cb,
        mqtt_incoming_data_cb,
        NULL
    );

    // Subscribe to command topic
    err_t err = mqtt_subscribe(
        mqtt_client,
        MQTT_TOPIC_COMMAND,
        0,  // QoS 0
        mqtt_subscribe_cb,
        NULL
    );

    if (err != ERR_OK) {
        printf("[MQTT] Subscribe failed (error: %d)\n", err);
        return MQTT_ERR_SUBSCRIBE_FAILED;
    }

    // Subscribe to the laptop heartbeat (FR-08 link timeout)
    err = mqtt_subscribe(
        mqtt_client,
        MQTT_TOPIC_HEARTBEAT,
        0,  // QoS 0
        mqtt_subscribe_cb,
        NULL
    );

    if (err != ERR_OK) {
        printf("[MQTT] Heartbeat subscribe failed (error: %d)\n", err);
        return MQTT_ERR_SUBSCRIBE_FAILED;
    }

    return WIFI_OK;
}

uint32_t mqtt_last_link_rx_ms(void) {
    return last_link_rx_ms;
}

wifi_mqtt_error_t mqtt_publish_status(const char *status) {
    if (!mqtt_connected_flag || mqtt_client == NULL) {
        return MQTT_ERR_CONNECT_FAILED;
    }

    err_t err = mqtt_publish(
        mqtt_client,
        MQTT_TOPIC_STATUS,
        status,
        strlen(status),
        0,  // QoS 0
        0,  // Not retained
        mqtt_publish_cb,
        NULL
    );

    return (err == ERR_OK) ? WIFI_OK : MQTT_ERR_PUBLISH_FAILED;
}

wifi_mqtt_error_t mqtt_publish_telemetry(const char *data) {
    if (!mqtt_connected_flag || mqtt_client == NULL) {
        return MQTT_ERR_CONNECT_FAILED;
    }

    // Check if MQTT client is ready (not busy)
    if (!mqtt_client_is_connected(mqtt_client)) {
        printf("[WARN] MQTT client not ready\n");
        return MQTT_ERR_PUBLISH_FAILED;
    }

    err_t err = mqtt_publish(
        mqtt_client,
        MQTT_TOPIC_TELEMETRY,
        data,
        strlen(data),
        0,  // QoS 0
        0,  // Not retained
        mqtt_publish_cb,
        NULL
    );

    if (err != ERR_OK) {
        printf("[MQTT] Publish failed with lwIP error: %d\n", err);
    }

    return (err == ERR_OK) ? WIFI_OK : MQTT_ERR_PUBLISH_FAILED;
}

bool mqtt_is_connected(void) {
    return mqtt_connected_flag && mqtt_client_is_connected(mqtt_client);
}

void wifi_mqtt_poll(void) {
    if (wifi_initialized) {
        cyw43_arch_poll();
    }
}

// Expose mqtt_client for LIDAR publishing
mqtt_client_t* get_mqtt_client(void) {
    return mqtt_client;
}