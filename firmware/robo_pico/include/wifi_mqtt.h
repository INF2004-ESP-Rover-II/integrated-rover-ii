#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include <stdint.h>
#include <stdbool.h>
#include "lwip/apps/mqtt.h"  // <--- ADD THIS LINE

uint32_t mqtt_last_link_rx_ms(void);

// Error codes
typedef enum {
    WIFI_OK = 0,
    WIFI_ERR_INIT_FAILED = -1,
    WIFI_ERR_AUTH = -2,
    WIFI_ERR_TIMEOUT = -3,
    WIFI_ERR_NOT_CONNECTED = -4,
    MQTT_ERR_CONNECT_FAILED = -5,
    MQTT_ERR_PUBLISH_FAILED = -6,
    MQTT_ERR_SUBSCRIBE_FAILED = -7
} wifi_mqtt_error_t;

// Command callback type
typedef void (*mqtt_command_callback_t)(const char *command);

/**
 * @brief Initialize WiFi subsystem
 */
wifi_mqtt_error_t wifi_init(void);

/**
 * @brief Connect to WiFi network
 */
wifi_mqtt_error_t wifi_connect(const char *ssid, const char *password, uint32_t timeout_ms);

/**
 * @brief Check WiFi connection status
 */
bool wifi_is_connected(void);

/**
 * @brief Get IP address
 */
const char* wifi_get_ip_address(void);

/**
 * @brief Initialize MQTT client
 */
wifi_mqtt_error_t mqtt_init(const char *broker_ip, uint16_t broker_port, const char *client_id);

/**
 * @brief Connect to MQTT broker
 */
wifi_mqtt_error_t mqtt_connect(void);

/**
 * @brief Subscribe to command topic
 */
wifi_mqtt_error_t mqtt_subscribe_commands(mqtt_command_callback_t callback);

/**
 * @brief Publish status message
 */
wifi_mqtt_error_t mqtt_publish_status(const char *status);

/**
 * @brief Publish telemetry data
 */
wifi_mqtt_error_t mqtt_publish_telemetry(const char *data);

/**
 * @brief Check MQTT connection status
 */
bool mqtt_is_connected(void);

/**
 * @brief Poll WiFi/MQTT (call in main loop)
 */
void wifi_mqtt_poll(void);

/**
 * @brief Get MQTT client pointer for direct publishing
 * @return Pointer to mqtt_client_t
 */
mqtt_client_t* get_mqtt_client(void);  // <--- ADD THIS FUNCTION DECLARATION

#endif // WIFI_MQTT_H