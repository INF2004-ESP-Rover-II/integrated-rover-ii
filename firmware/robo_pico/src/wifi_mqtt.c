/**
 * @file wifi_mqtt.c
 * @brief Wifi mqtt firmware module.
 */
#include <inttypes.h>
#include "wifi_mqtt.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/ip4_addr.h"
#include "mqtt_binary.h"
#include "mqtt_publish_policy.h"
// Read-only adapter to this SDK's MQTT ring/request lists. Admission must
// include subscription requests and actual ring occupancy, not only application
// counts.
#include "lwip/apps/mqtt_priv.h"
#if ROVER_MQTT_DIAGNOSTICS
#include "pico/stdio_usb.h"
#endif

// MQTT limits belong in lwipopts.h so the application and SDK use the same
// values.

// =================== PRIVATE STATE ===================
static mqtt_client_t *         gp_mqtt_client = NULL;
static ip_addr_t               g_mqtt_broker_ip;
static bool                    g_b_wifi_initialized    = false;
static bool                    g_b_wifi_connected_flag = false;
static bool                    g_b_mqtt_connected_flag = false;
static mqtt_command_callback_t gp_command_callback     = NULL;
static unsigned                g_publishes_in_flight   = 0;
static bool                    g_b_telemetry_in_flight = false;
static bool                    g_b_camera_in_flight    = false;
static bool                    g_b_map_in_flight       = false;
static mqtt_publish_policy_t   g_publish_policy;
static char g_reply_queue[MQTT_REPLY_QUEUE_CAPACITY][MQTT_REPLY_CAPACITY];
static unsigned int g_reply_head;
static unsigned int g_reply_count;
static bool         g_b_reply_attempted;
static uint32_t     g_reply_last_attempt_ms;
static unsigned     g_reply_dropped;
#if ROVER_MQTT_DIAGNOSTICS
typedef struct
{
    uint32_t queued;
    uint32_t completed;
    uint32_t busy;
    uint32_t errors;
    unsigned pending;
} publish_diagnostics_t;
static publish_diagnostics_t g_publication_stats[PUBLISH_COUNT];
static uint32_t              g_loop_last_us;
static uint32_t              g_loop_gap_max_us;
static uint32_t              g_poll_last_us;
static uint32_t              g_poll_start_us;
static uint32_t              g_poll_gap_max_us;
static uint32_t              g_poll_duration_max_us;
static uint32_t              g_last_log_duration_us;
static uint32_t              g_telemetry_completion_max_ms;
static uint32_t              g_telemetry_completion_last_ms;
static unsigned int          g_connection_events;
static unsigned int          g_pending_peak;
static int                   g_last_publish_error;
static bool                  g_b_loop_seen;
static bool                  g_b_poll_seen;
#define DIAG_BUSY(kind) (++g_publication_stats[kind].busy)
#define DIAG_ERROR(kind) (++g_publication_stats[kind].errors)
#else
#define DIAG_BUSY(kind) ((void) 0)
#define DIAG_ERROR(kind) ((void) 0)
#endif
static uint32_t g_telemetry_queued_ms         = 0;
static uint32_t g_last_publish_warning_ms     = 0;
static bool     g_b_publish_warning_logged    = false;
static bool     g_b_first_telemetry_queued    = false;
static bool     g_b_first_telemetry_completed = false;

// FR-08: which topic the incoming data belongs to, and when we last heard the
// laptop
typedef enum
{
    IN_TOPIC_OTHER,
    IN_TOPIC_COMMAND,
    IN_TOPIC_HEARTBEAT
} in_topic_t;
static in_topic_t        g_current_in_topic = IN_TOPIC_OTHER;
static volatile uint32_t g_last_link_rx_ms  = 0;

static void
reset_publish_state (void)
{
#if ROVER_MQTT_DIAGNOSTICS
    for (unsigned i = 0; i < PUBLISH_COUNT; ++i)
    {
        g_publication_stats[i].pending = 0;
    }
    g_connection_events++;
#endif
    g_publishes_in_flight = 0;
    mqtt_publish_policy_init(&g_publish_policy);
    // Old acknowledgements/events must not replay into a new
    // connection/session.
    g_reply_dropped += g_reply_count;
    g_reply_head = g_reply_count  = 0;
    g_b_reply_attempted           = false;
    g_b_telemetry_in_flight       = false;
    g_b_camera_in_flight          = false;
    g_b_map_in_flight             = false;
    g_b_publish_warning_logged    = false;
    g_b_first_telemetry_queued    = false;
    g_b_first_telemetry_completed = false;
}

static bool
publish_warning_due (void)
{
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (!g_b_publish_warning_logged ||
        (uint32_t) (now - g_last_publish_warning_ms) >= 1000u)
    {
        g_last_publish_warning_ms  = now;
        g_b_publish_warning_logged = true;
        return true;
    }
    return false;
}

// =================== MQTT CALLBACKS ==================

static void
mqtt_connection_cb (mqtt_client_t * p_client, void * p_arg,
                    mqtt_connection_status_t status)
{
    (void) p_client;
    (void) p_arg;

    reset_publish_state();
    printf("[MQTT] ===== CONNECTION CALLBACK FIRED =====\n");
    printf("[MQTT] Status code: %d\n", status);

    if (status == MQTT_CONNECT_ACCEPTED)
    {
        printf("[MQTT] Connected to broker successfully!\n");
        g_b_mqtt_connected_flag = true;
    }
    else if (status == MQTT_CONNECT_REFUSED_PROTOCOL_VERSION)
    {
        printf("[MQTT] Connection refused: protocol version\n");
        g_b_mqtt_connected_flag = false;
    }
    else if (status == MQTT_CONNECT_REFUSED_IDENTIFIER)
    {
        printf("[MQTT] Connection refused: identifier rejected\n");
        g_b_mqtt_connected_flag = false;
    }
    else if (status == MQTT_CONNECT_REFUSED_SERVER)
    {
        printf("[MQTT] Connection refused: server unavailable\n");
        g_b_mqtt_connected_flag = false;
    }
    else if (status == MQTT_CONNECT_REFUSED_USERNAME_PASS)
    {
        printf("[MQTT] Connection refused: bad username/password\n");
        g_b_mqtt_connected_flag = false;
    }
    else if (status == MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_)
    {
        printf("[MQTT] Connection refused: not authorized\n");
        g_b_mqtt_connected_flag = false;
    }
    else if (status == MQTT_CONNECT_DISCONNECTED)
    {
        printf("[MQTT] Disconnected from broker\n");
        g_b_mqtt_connected_flag = false;
    }
    else if (status == MQTT_CONNECT_TIMEOUT)
    {
        printf("[MQTT] Connection timeout!\n");
        g_b_mqtt_connected_flag = false;
    }
    else
    {
        printf("[MQTT] Unknown status: %d\n", status);
        g_b_mqtt_connected_flag = false;
    }
}

static void
mqtt_incoming_data_cb (void * p_arg, const u8_t * p_data, u16_t len, u8_t flags)
{
    (void) p_arg;

    // Accumulate message data
    static char   message_buffer[256];
    static size_t buffer_pos = 0;

    // Copy incoming data to buffer
    if (buffer_pos + len < sizeof(message_buffer))
    {
        memcpy(message_buffer + buffer_pos, p_data, len);
        buffer_pos += len;
    }

    // Check if this is the last fragment
    if ((uint32_t) (flags) & (uint32_t) (MQTT_DATA_FLAG_LAST))
    {
        message_buffer[buffer_pos] = '\0';

        // Process command
        uint32_t time_value = to_ms_since_boot(get_absolute_time());
        switch (g_current_in_topic)
        {
            case IN_TOPIC_COMMAND:
                g_last_link_rx_ms = time_value;
                if (gp_command_callback != NULL)
                {
                    char           text[24];
                    const char *   p_command = message_buffer;
                    mqtt_command_t binary;
                    if (mqtt_is_binary_command(buffer_pos))
                    {
                        if (mqtt_decode_command(
                                (const uint8_t *) message_buffer, buffer_pos,
                                &binary) &&
                            mqtt_command_text(&binary, text, sizeof text))
                        {
                            p_command = text;
                        }
                        else
                        {
                            printf("[MQTT] Ignored unknown binary command (%u "
                                   "bytes)\n",
                                   (unsigned) buffer_pos);
                            break;
                        }
                    }
                    printf("[MQTT] Received command: %s\n", p_command);
                    gp_command_callback(p_command);
                }
                break;
            case IN_TOPIC_HEARTBEAT:
                g_last_link_rx_ms =
                    time_value; // heartbeat only proves the link is alive
                break;
            default:
                break; // not ours, ignore
        }

        // Reset buffer
        buffer_pos = 0;
    }
}

static void
mqtt_incoming_publish_cb (void * p_arg, const char * p_topic, u32_t tot_len)
{
    (void) p_arg;
    (void) tot_len;
    if (strcmp(p_topic, MQTT_TOPIC_COMMAND) == 0)
    {
        g_current_in_topic = IN_TOPIC_COMMAND;
    }
    else if (strcmp(p_topic, MQTT_TOPIC_HEARTBEAT) == 0)
    {
        g_current_in_topic = IN_TOPIC_HEARTBEAT;
    }
    else
    {
        g_current_in_topic = IN_TOPIC_OTHER;
    }
}

static void
mqtt_subscribe_cb (void * p_arg, err_t result)
{
    (void) p_arg;

    if (result == ERR_OK)
    {
        printf("[MQTT] Successfully subscribed to command topic\n");
    }
    else
    {
        printf("[MQTT] Subscribe failed (error: %d)\n", result);
    }
}

static void
mqtt_publish_cb (void * p_arg, err_t result)
{
    publish_slot_t * p_slot = p_arg;
    publish_kind_t   kind   = p_slot->kind;
    mqtt_publish_policy_release(p_slot);
#if ROVER_MQTT_DIAGNOSTICS
    if (g_publication_stats[kind].pending)
    {
        g_publication_stats[kind].pending--;
    }
    if (result == ERR_OK)
    {
        g_publication_stats[kind].completed++;
    }
    else
    {
        DIAG_ERROR(kind);
        g_last_publish_error = result;
    }
    if (kind == PUBLISH_SENSOR && result == ERR_OK)
    {
        g_telemetry_completion_last_ms =
            to_ms_since_boot(get_absolute_time()) - g_telemetry_queued_ms;
        if (g_telemetry_completion_last_ms > g_telemetry_completion_max_ms)
        {
            g_telemetry_completion_max_ms = g_telemetry_completion_last_ms;
        }
    }
#endif
    if (g_publishes_in_flight != 0)
    {
        --g_publishes_in_flight;
    }
    if (kind == PUBLISH_CAMERA)
    {
        g_b_camera_in_flight = false;
    }
    if (kind == PUBLISH_MAP)
    {
        g_b_map_in_flight = false;
    }
    if (kind == PUBLISH_SENSOR)
    {
        g_b_telemetry_in_flight = false;
        if (result == ERR_OK && !g_b_first_telemetry_completed)
        {
            g_b_first_telemetry_completed = true;
            printf(
                "[MQTT] First telemetry send completed (TCP acknowledged)\n");
        }
    }
    if (result != ERR_OK)
    {
        printf("[MQTT] Publish failed (error: %d)\n", result);
    }
}

static unsigned
mqtt_free_requests (void)
{
    unsigned free = 0;
    for (unsigned index = 0; index < MQTT_REQ_MAX_IN_FLIGHT; ++index)
    {
        if (gp_mqtt_client->req_list[index].next ==
            &gp_mqtt_client->req_list[index])
        {
            free++;
        }
    }
    return free;
}

static size_t
mqtt_free_output_bytes (void)
{

    int32_t used = (int32_t) gp_mqtt_client->output.put -
                   (int32_t) gp_mqtt_client->output.get;

    if (used < 0)
    {
        used += MQTT_OUTPUT_RINGBUF_SIZE;
    }
    return MQTT_OUTPUT_RINGBUF_SIZE - (size_t) used;
}

typedef struct
{
    _Bool             b_telemetry;
    _Bool             b_camera;
    _Bool             b_map;
    size_t            length;
    size_t            wire_bytes;
    uint32_t          now;
    err_t             err;
    wifi_mqtt_error_t return_value;
} publish_message_context_t;

/** @brief Execute the prepare stage of publish_message. */
static bool
publish_message_prepare (const char * p_topic, const char * p_data,
                         publish_kind_t              kind,
                         publish_message_context_t * p_context)
{


    p_context->b_telemetry = kind == PUBLISH_SENSOR;

    p_context->b_camera = kind == PUBLISH_CAMERA;

    p_context->b_map = kind == PUBLISH_MAP;
    if (!g_b_mqtt_connected_flag || gp_mqtt_client == NULL ||
        !mqtt_client_is_connected(gp_mqtt_client))
    {
        DIAG_ERROR(kind);
        p_context->return_value = MQTT_ERR_CONNECT_FAILED;
        return true;
    }


    p_context->length = strlen(p_data);
    // Conservative QoS 0 wire size: topic prefix and up to five fixed-header
    // bytes.

    p_context->wire_bytes = p_context->length + strlen(p_topic) + 7u;
    if (p_context->length > UINT16_MAX ||
        p_context->wire_bytes > MQTT_OUTPUT_RINGBUF_SIZE)
    {
        DIAG_ERROR(kind);
        if (publish_warning_due())
        {
            printf("[MQTT] Payload too large: %u bytes\n",
                   (unsigned) p_context->length);
        }
        p_context->return_value = MQTT_ERR_PUBLISH_FAILED;
        return true;
    }

    p_context->now = to_ms_since_boot(get_absolute_time());
    return false;
}

/** @brief Execute the inputs stage of publish_message. */
static bool
publish_message_inputs (const char * p_topic, const char * p_data,
                        publish_kind_t              kind,
                        publish_message_context_t * p_context)
{

    if (!mqtt_publish_policy_begin(
            &g_publish_policy, kind, p_context->now, p_context->wire_bytes,
            mqtt_free_output_bytes(), mqtt_free_requests(), g_reply_count != 0))
    {
        DIAG_BUSY(kind);

        uint32_t age =
            to_ms_since_boot(get_absolute_time()) - g_telemetry_queued_ms;

        if (p_context->b_telemetry && g_b_telemetry_in_flight && age >= 1000u &&
            publish_warning_due())
        {
            printf("[MQTT] Telemetry awaiting TCP completion for %" PRIu32
                   "ms; pending=%u/%u\n",
                   (uint32_t) age, g_publishes_in_flight,
                   (unsigned) MQTT_REQ_MAX_IN_FLIGHT);
        }
        p_context->return_value = MQTT_ERR_BUSY;
        return true;
    }

    // Reserve before calling lwIP so a completion callback cannot underflow
    // counts.
    ++g_publishes_in_flight;
#if ROVER_MQTT_DIAGNOSTICS
    g_publication_stats[kind].pending++;
#endif
    if (p_context->b_camera)
    {
        g_b_camera_in_flight = true;
    }
    if (p_context->b_map)
    {
        g_b_map_in_flight = true;
    }
    if (p_context->b_telemetry)
    {
        g_b_telemetry_in_flight = true;
        g_telemetry_queued_ms   = to_ms_since_boot(get_absolute_time());
    }


    p_context->err =
        mqtt_publish(gp_mqtt_client, p_topic, p_data, (u16_t) p_context->length,
                     0, 0, mqtt_publish_cb, &g_publish_policy.slots[kind]);

    return false;
}

/** @brief Execute the motion stage of publish_message. */
static bool
publish_message_motion (publish_kind_t              kind,
                        publish_message_context_t * p_context)
{

    if (p_context->err != ERR_OK)
    {
        mqtt_publish_policy_release(&g_publish_policy.slots[kind]);
#if ROVER_MQTT_DIAGNOSTICS
        if (g_publication_stats[kind].pending)
        {
            g_publication_stats[kind].pending--;
        }
        g_last_publish_error = p_context->err;
#endif
        DIAG_ERROR(kind);
        if (g_publishes_in_flight != 0)
        {
            --g_publishes_in_flight;
        }
        if (p_context->b_telemetry)
        {
            g_b_telemetry_in_flight = false;
        }
        if (p_context->b_camera)
        {
            g_b_camera_in_flight = false;
        }
        if (p_context->b_map)
        {
            g_b_map_in_flight = false;
        }
        if (publish_warning_due())
        {
            printf("[MQTT] Publish rejected: err=%d payload=%u pending=%u/%u "
                   "TX ring=%u\n",
                   (int) p_context->err, (unsigned) p_context->length,
                   g_publishes_in_flight, (unsigned) MQTT_REQ_MAX_IN_FLIGHT,
                   (unsigned) MQTT_OUTPUT_RINGBUF_SIZE);
        }

        p_context->return_value =
            p_context->err == ERR_MEM ? MQTT_ERR_BUSY : MQTT_ERR_PUBLISH_FAILED;

        return true;
    }
    return false;
}

/** @brief Execute the camera stage of publish_message. */
static bool
publish_message_camera (const char * p_topic, publish_kind_t kind,
                        publish_message_context_t * p_context)
{
    (void) kind; /* Used by diagnostic builds. */

#if ROVER_MQTT_DIAGNOSTICS
    g_publication_stats[kind].queued++;
    if (g_publishes_in_flight > g_pending_peak)
    {
        g_pending_peak = g_publishes_in_flight;
    }
#endif
    if (p_context->b_telemetry && !g_b_first_telemetry_queued)
    {
        g_b_first_telemetry_queued = true;
        printf("[MQTT] First telemetry queued: %u bytes on %s\n",
               (unsigned) p_context->length, p_topic);
    }
    p_context->return_value = WIFI_OK;
    return true;
}

static wifi_mqtt_error_t
publish_message (const char * p_topic, const char * p_data, publish_kind_t kind)
{
    publish_message_context_t context = { 0 };
    if (publish_message_prepare(p_topic, p_data, kind, &context))
    {
        return context.return_value;
    }
    if (publish_message_inputs(p_topic, p_data, kind, &context))
    {
        return context.return_value;
    }
    if (publish_message_motion(kind, &context))
    {
        return context.return_value;
    }
    if (publish_message_camera(p_topic, kind, &context))
    {
        return context.return_value;
    }
    return context.return_value;
}


#if ROVER_MQTT_DIAGNOSTICS
void
mqtt_diagnostics_loop_begin (void)
{
    uint32_t now = time_us_32();
    uint32_t gap = now - g_loop_last_us;
    if (g_b_loop_seen && gap > g_loop_gap_max_us)
    {
        g_loop_gap_max_us = gap;
    }
    g_b_loop_seen  = true;
    g_loop_last_us = now;
}

void
mqtt_diagnostics_poll_begin (void)
{
    uint32_t now = time_us_32();
    uint32_t gap = now - g_poll_last_us;
    if (g_b_poll_seen && gap > g_poll_gap_max_us)
    {
        g_poll_gap_max_us = gap;
    }
    g_b_poll_seen  = true;
    g_poll_last_us = g_poll_start_us = now;
}

void
mqtt_diagnostics_poll_end (void)
{
    uint32_t duration = time_us_32() - g_poll_start_us;
    if (duration > g_poll_duration_max_us)
    {
        g_poll_duration_max_us = duration;
    }
}

void
mqtt_diagnostics_report (uint32_t sensor_age_ms)
{
    static uint32_t last_report_ms;
    uint32_t        now = to_ms_since_boot(get_absolute_time());
    if ((uint32_t) (now - last_report_ms) < 2000u || !stdio_usb_connected())
    {
        return;
    }
    last_report_ms    = now;
    uint32_t start_us = time_us_32();
    printf(
        "[NETDIAG] t_ms=%" PRIu32 " mqtt=%s loop_gap_max_us=%" PRIu32
        " poll_gap_max_us=%" PRIu32 " poll_max_us=%" PRIu32
        " prev_diag_print_us=%" PRIu32 " sensor_age_ms=%" PRIu32 " "
        "rx_age_ms=%" PRIu32 " conn_events=%u\n",
        (uint32_t) now, mqtt_is_connected() ? "UP" : "DOWN",
        (uint32_t) g_loop_gap_max_us, (uint32_t) g_poll_gap_max_us,
        (uint32_t) g_poll_duration_max_us, (uint32_t) g_last_log_duration_us,
        (uint32_t) sensor_age_ms,
        (uint32_t) (g_last_link_rx_ms ? now - g_last_link_rx_ms : UINT32_MAX),
        g_connection_events);
    printf(
        "[NETDIAG] pending=%u/%u pending_peak=%u telemetry_wait_ms=%" PRIu32
        " completion_last_ms=%" PRIu32 " completion_max_ms=%" PRIu32
        " last_lwip_error=%d\n",
        g_publishes_in_flight, (unsigned) MQTT_REQ_MAX_IN_FLIGHT,
        g_pending_peak,
        (uint32_t) (g_b_telemetry_in_flight ? now - g_telemetry_queued_ms : 0u),
        (uint32_t) g_telemetry_completion_last_ms,
        (uint32_t) g_telemetry_completion_max_ms, g_last_publish_error);
    printf("[NETDIAG] free_requests=%u free_output_bytes=%u reply_queue=%u/%u "
           "reply_dropped=%u\n",
           gp_mqtt_client ? mqtt_free_requests() : 0u,
           gp_mqtt_client ? (unsigned) mqtt_free_output_bytes() : 0u,
           g_reply_count, MQTT_REPLY_QUEUE_CAPACITY, g_reply_dropped);
    static const char * names[] = {
        "REPLIES", "TELEMETRY", "CAMERA",   "MAP",
        "MISSION", "DFS",       "TRACKING", "TARGET"
    };
    for (unsigned i = 0; i < PUBLISH_COUNT; ++i)
    {
        const publish_diagnostics_t * s = &g_publication_stats[i];
        printf("[NETDIAG] %s queued=%" PRIu32 " completed=%" PRIu32
               " busy=%" PRIu32 " errors=%" PRIu32 " pending=%u\n",
               names[i], (uint32_t) s->queued, (uint32_t) s->completed,
               (uint32_t) s->busy, (uint32_t) s->errors, s->pending);
    }
    // Maxima cover the reporting interval. Per-topic counts remain cumulative.
    g_loop_gap_max_us = g_poll_gap_max_us = g_poll_duration_max_us =
        g_telemetry_completion_max_ms     = 0;
    g_pending_peak                        = g_publishes_in_flight;
    g_last_log_duration_us                = time_us_32() - start_us;
}
#endif

// ================= PUBLIC FUNCTIONS ==================

wifi_mqtt_error_t
wifi_init (void)
{
    if (g_b_wifi_initialized)
    {
        return WIFI_OK;
    }

    if (cyw43_arch_init())
    {
        printf("[WIFI] Initialization failed\n");
        return WIFI_ERR_INIT_FAILED;
    }

    cyw43_arch_enable_sta_mode();
    g_b_wifi_initialized = true;

    printf("[WIFI] Initialized\n");
    return WIFI_OK;
}

wifi_mqtt_error_t
wifi_connect (const char * p_ssid, const char * p_password, uint32_t timeout_ms)
{
    if (!g_b_wifi_initialized)
    {
        return WIFI_ERR_INIT_FAILED;
    }

    printf("[WIFI] Connecting to '%s'...\n", p_ssid);


    int result = cyw43_arch_wifi_connect_timeout_ms(
        p_ssid, p_password, CYW43_AUTH_WPA2_AES_PSK, timeout_ms);


    if (result != 0)
    {
        printf("[WIFI] Connection failed (code: %d)\n", result);
        g_b_wifi_connected_flag = false;
        return (result == -2)   ? WIFI_ERR_AUTH
               : (result == -3) ? WIFI_ERR_TIMEOUT
                                : WIFI_ERR_NOT_CONNECTED;
    }

    g_b_wifi_connected_flag = true;

    // Disable power save mode to improve latency for MQTT messages
    cyw43_wifi_pm(&cyw43_state,
                  cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, 20, 1, 1, 1));

    extern struct netif * netif_list;
    if (netif_list)
    {
        printf("[WIFI] Connected! IP: %s\n",
               ip4addr_ntoa(netif_ip4_addr(netif_list)));
    }

    return WIFI_OK;
}

bool
wifi_is_connected (void)
{
    return g_b_wifi_connected_flag;
}

const char *
wifi_get_ip_address (void)
{
    if (!g_b_wifi_connected_flag)
    {
        return NULL;
    }

    extern struct netif * netif_list;
    if (netif_list)
    {
        return ip4addr_ntoa(netif_ip4_addr(netif_list));
    }

    return NULL;
}

wifi_mqtt_error_t
mqtt_init (const char * p_broker_ip, uint16_t broker_port,
           const char * p_client_id)
{
    (void) p_client_id;

    // Parse broker IP address
    if (!ip4addr_aton(p_broker_ip, &g_mqtt_broker_ip))
    {
        printf("[MQTT] Invalid broker IP: %s\n", p_broker_ip);
        return MQTT_ERR_CONNECT_FAILED;
    }

    // Create MQTT client
    gp_mqtt_client = mqtt_client_new();
    if (gp_mqtt_client == NULL)
    {
        printf("[MQTT] Failed to create client\n");
        return MQTT_ERR_CONNECT_FAILED;
    }

    printf("[MQTT] Initialized (broker: %s:%d)\n", p_broker_ip, broker_port);
    printf("[MQTT] Limits: TX ring=%u bytes; requests=%u; telemetry pending "
           "limit=1\n",
           (unsigned) MQTT_OUTPUT_RINGBUF_SIZE,
           (unsigned) MQTT_REQ_MAX_IN_FLIGHT);
    return WIFI_OK;
}

wifi_mqtt_error_t
mqtt_connect (void)
{
    if (!g_b_wifi_connected_flag)
    {
        printf("[MQTT] ERROR: WiFi not connected\n");
        return WIFI_ERR_NOT_CONNECTED;
    }

    if (gp_mqtt_client == NULL)
    {
        printf("[MQTT] ERROR: Client is NULL\n");
        return MQTT_ERR_CONNECT_FAILED;
    }

    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id  = MQTT_CLIENT_ID;
    ci.keep_alive = 5;

    printf("[MQTT] ===== MQTT CONNECTION DEBUG =====\n");
    printf("[MQTT] Client ID: %s\n", MQTT_CLIENT_ID);
    printf("[MQTT] Broker IP: %s\n", ip4addr_ntoa(&g_mqtt_broker_ip));
    printf("[MQTT] Broker Port: %d\n", MQTT_BROKER_PORT);
    printf("[MQTT] Keep-alive: %d\n", ci.keep_alive);
    printf("[MQTT] Client pointer: %p\n", (void *) gp_mqtt_client);
    printf("[MQTT] Calling mqtt_client_connect()...\n");


    err_t err =
        mqtt_client_connect(gp_mqtt_client, &g_mqtt_broker_ip, MQTT_BROKER_PORT,
                            mqtt_connection_cb, NULL, &ci);


    printf("[MQTT] mqtt_client_connect() returned: %d\n", (int) err);

    if (err == ERR_OK)
    {
        printf("[MQTT] Connection initiated successfully\n");
    }
    else if (err == ERR_VAL)
    {
        printf("[MQTT] ERROR: Invalid value (ERR_VAL = -6)\n");
    }
    else if (err == ERR_MEM)
    {
        printf("[MQTT] ERROR: Out of memory (ERR_MEM = -1)\n");
    }
    else if (err == ERR_ISCONN)
    {
        printf("[MQTT] ERROR: Already connected (ERR_ISCONN = -8)\n");
    }
    else
    {
        printf("[MQTT] ERROR: Unknown error code %d\n", (int) err);
    }

    return (err == ERR_OK) ? WIFI_OK : MQTT_ERR_CONNECT_FAILED;
}

wifi_mqtt_error_t
mqtt_subscribe_commands (mqtt_command_callback_t p_callback)
{
    if (!g_b_mqtt_connected_flag || gp_mqtt_client == NULL)
    {
        return MQTT_ERR_CONNECT_FAILED;
    }

    gp_command_callback = p_callback;

    // Set incoming callbacks
    mqtt_set_inpub_callback(gp_mqtt_client, mqtt_incoming_publish_cb,
                            mqtt_incoming_data_cb, NULL);

    // Subscribe to command topic

    err_t err = mqtt_subscribe(gp_mqtt_client, MQTT_TOPIC_COMMAND,
                               0, // QoS 0
                               mqtt_subscribe_cb, NULL);


    // FR-08: also listen for the dashboard heartbeat

    err = mqtt_subscribe(gp_mqtt_client, MQTT_TOPIC_HEARTBEAT, 0,
                         mqtt_subscribe_cb, NULL);

    if (err != ERR_OK)
    {
        printf("[MQTT] Heartbeat subscribe failed (error: %d)\n", err);
        return MQTT_ERR_SUBSCRIBE_FAILED;
    }

    return WIFI_OK;
}

wifi_mqtt_error_t
mqtt_publish_status (const char * p_status)
{
    if (!mqtt_is_connected())
    {
        return MQTT_ERR_CONNECT_FAILED;
    }
    size_t length = strlen(p_status);
    if (length >= MQTT_REPLY_CAPACITY)
    {
        g_reply_dropped++;
        if (publish_warning_due())
        {
            printf("[MQTT] Reply too large; not truncated (%u bytes)\n",
                   (unsigned) length);
        }
        return MQTT_ERR_PUBLISH_FAILED;
    }
    if (!g_reply_count)
    {

        wifi_mqtt_error_t result =
            publish_message(MQTT_TOPIC_STATUS, p_status, PUBLISH_STATUS);

        if (result != MQTT_ERR_BUSY)
        {
            return result;
        }
        g_b_reply_attempted     = true;
        g_reply_last_attempt_ms = to_ms_since_boot(get_absolute_time());
    }
    if (g_reply_count == MQTT_REPLY_QUEUE_CAPACITY)
    {
        g_reply_dropped++;
        if (publish_warning_due())
        {
            printf("[MQTT] Reply queue full; new reply refused\n");
        }
        return MQTT_ERR_BUSY;
    }
    memcpy(g_reply_queue[(g_reply_head + g_reply_count) %
                         MQTT_REPLY_QUEUE_CAPACITY],
           p_status, length + 1u);
    g_reply_count++;
    return WIFI_OK; // Accepted locally; no claim of broker/dashboard delivery.
}

void
mqtt_service_status_queue (void)
{
    if (!g_reply_count || !mqtt_is_connected())
    {
        return;
    }
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (g_b_reply_attempted &&
        (uint32_t) (now - g_reply_last_attempt_ms) < MQTT_REPORT_RETRY_MS)
    {
        return;
    }
    g_b_reply_attempted     = true;
    g_reply_last_attempt_ms = now;
    if (publish_message(MQTT_TOPIC_STATUS, g_reply_queue[g_reply_head],
                        PUBLISH_STATUS) == WIFI_OK)
    {
        g_reply_head = (g_reply_head + 1u) % MQTT_REPLY_QUEUE_CAPACITY;
        g_reply_count--;
    }
}

wifi_mqtt_error_t
mqtt_publish_report (mqtt_report_t report, const char * p_status)
{
    if ((unsigned) report > MQTT_REPORT_TARGET)
    {
        return MQTT_ERR_PUBLISH_FAILED;
    }
    return publish_message(MQTT_TOPIC_STATUS, p_status,
                           (publish_kind_t) (PUBLISH_MISSION + report));
}

wifi_mqtt_error_t
mqtt_publish_telemetry (const char * p_data)
{
    return publish_message(MQTT_TOPIC_TELEMETRY, p_data, PUBLISH_SENSOR);
}

wifi_mqtt_error_t
mqtt_publish_camera (const char * p_data)
{
    return publish_message("robot/camera", p_data, PUBLISH_CAMERA);
}

wifi_mqtt_error_t
mqtt_publish_map (const char * p_data)
{
    return publish_message("robot/map", p_data, PUBLISH_MAP);
}

bool
mqtt_is_connected (void)
{
    return g_b_mqtt_connected_flag && mqtt_client_is_connected(gp_mqtt_client);
}

void
wifi_mqtt_poll (void)
{
    if (g_b_wifi_initialized)
    {
        cyw43_arch_poll();
    }
}

// Expose mqtt_client for LIDAR publishing
mqtt_client_t *
get_mqtt_client (void)
{
    return gp_mqtt_client;
}

uint32_t
mqtt_last_link_rx_ms (void)
{
    return g_last_link_rx_ms;
}

/* End of file. */
