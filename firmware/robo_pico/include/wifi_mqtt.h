/**
 * @file wifi_mqtt.h
 * @brief Wifi mqtt firmware module.
 */
#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include <stdint.h>
#include <stdbool.h>
#include "lwip/apps/mqtt.h" // <--- ADD THIS LINE


uint32_t mqtt_last_link_rx_ms (void);


#ifndef ROVER_MQTT_DIAGNOSTICS
#define ROVER_MQTT_DIAGNOSTICS 0
#endif
#if ROVER_MQTT_DIAGNOSTICS
// Measurements only; these do not alter publication or motor control decisions.
void mqtt_diagnostics_loop_begin (void);
void mqtt_diagnostics_poll_begin (void);
void mqtt_diagnostics_poll_end (void);
void mqtt_diagnostics_report (uint32_t sensor_age_ms);
#endif

// Error codes
typedef enum
{
    WIFI_OK                   = 0,
    WIFI_ERR_INIT_FAILED      = -1,
    WIFI_ERR_AUTH             = -2,
    WIFI_ERR_TIMEOUT          = -3,
    WIFI_ERR_NOT_CONNECTED    = -4,
    MQTT_ERR_CONNECT_FAILED   = -5,
    MQTT_ERR_PUBLISH_FAILED   = -6,
    MQTT_ERR_SUBSCRIBE_FAILED = -7,
    MQTT_ERR_BUSY = -8 // Try the latest telemetry again after network progress.
} wifi_mqtt_error_t;

// Command callback type
typedef void (*mqtt_command_callback_t)(const char * p_command);

/**
 * @brief Initialize WiFi subsystem
 */

wifi_mqtt_error_t wifi_init (void);


/**
 * @brief Connect to WiFi network
 */

wifi_mqtt_error_t wifi_connect (const char * p_ssid, const char * p_password,
                                uint32_t timeout_ms);


/**
 * @brief Check WiFi connection status
 */

bool wifi_is_connected (void);


/**
 * @brief Get IP address
 */

const char * wifi_get_ip_address (void);


/**
 * @brief Initialize MQTT client
 */

wifi_mqtt_error_t mqtt_init (const char * p_broker_ip, uint16_t broker_port,
                             const char * p_client_id);


/**
 * @brief Connect to MQTT broker
 */

wifi_mqtt_error_t mqtt_connect (void);


/**
 * @brief Subscribe to command topic
 */

wifi_mqtt_error_t mqtt_subscribe_commands (mqtt_command_callback_t p_callback);


/**
 * @brief Publish a command reply/event, or accept it into a bounded FIFO.
 * Replies must fit in 255 bytes. WIFI_OK means accepted locally, not delivered.
 * Four may be in flight and eight may wait; overflow is explicitly refused.
 */

wifi_mqtt_error_t mqtt_publish_status (const char * p_status);

// Routine reports share robot/status; one pending slot per type, 200 ms
// retries. BUSY does not queue an old report: the caller retries its latest
// state later.
typedef enum
{
    MQTT_REPORT_MISSION,
    MQTT_REPORT_DFS,
    MQTT_REPORT_TRACKING,
    MQTT_REPORT_TARGET
} mqtt_report_t;

wifi_mqtt_error_t mqtt_publish_report (mqtt_report_t report,
                                       const char *  p_status);

// Drain a bounded command-reply/event queue; call after due telemetry, before
// reports.

void mqtt_service_status_queue (void);


/**
 * @brief Publish the latest telemetry, or return MQTT_ERR_BUSY without queueing
 * it. At most one telemetry publish is awaiting completion; poll the network
 * before retrying.
 */

wifi_mqtt_error_t mqtt_publish_telemetry (const char * p_data);

// robot/camera, QoS 0, not retained; one camera publish in flight at most.

wifi_mqtt_error_t mqtt_publish_camera (const char * p_data);

// Full authoritative map, QoS 0/not retained; one snapshot in flight.

wifi_mqtt_error_t mqtt_publish_map (const char * p_data);


/**
 * @brief Check MQTT connection status
 */

bool mqtt_is_connected (void);


/**
 * @brief Poll WiFi/MQTT (call in main loop)
 */

void wifi_mqtt_poll (void);


/**
 * @brief Get MQTT client pointer for direct publishing
 * @return Pointer to mqtt_client_t
 */

mqtt_client_t * get_mqtt_client (void);
// <--- ADD THIS FUNCTION DECLARATION

/**
 * @brief Time (ms since boot) of the last message from the laptop, 0 if none
 * yet (FR-08)
 */

uint32_t mqtt_last_link_rx_ms (void);


#endif // WIFI_MQTT_H

/* End of file. */
